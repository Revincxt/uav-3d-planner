"""Geometry/provenance checks for the committed official Manhattan extract."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import unittest

from uav3d.collision import point_is_free, segment_is_free
from uav3d.manhattan_city import (
    DATA_DIRECTORY,
    ORIGIN_WGS84,
    ROI_WGS84,
    build_manhattan_city,
    lonlat_to_enu,
    make_city_scene,
    preprocess_city,
)


class ManhattanGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.city = build_manhattan_city()
        cls.source = json.loads((DATA_DIRECTORY / "building-footprints.geojson").read_text())

    def test_source_digest_and_complete_feature_preservation(self) -> None:
        source_bytes = (DATA_DIRECTORY / "building-footprints.geojson").read_bytes()
        self.assertEqual(
            hashlib.sha256(source_bytes).hexdigest(), self.city.metadata["sourceSha256"]
        )
        expected_parts = sum(
            1
            if feature["geometry"]["type"] == "Polygon"
            else len(feature["geometry"]["coordinates"])
            for feature in self.source["features"]
        )
        self.assertEqual(len(self.city.buildings), expected_parts)
        self.assertEqual(self.city.metadata["sourceFeatureCount"], len(self.source["features"]))
        self.assertGreater(len(self.city.buildings), 4000)
        self.assertFalse(self.city.metadata["geometrySimplified"])

    def test_every_real_polygon_is_enclosed_by_its_physical_obstacle_and_map(self) -> None:
        self.assertEqual(
            len({box.obstacle_id for box in self.city.buildings}), len(self.city.buildings)
        )
        for box, footprint in zip(self.city.buildings, self.city.footprints, strict=True):
            self.assertEqual(box.obstacle_id, footprint.building_id)
            for ring in footprint.rings:
                self.assertEqual(ring[0], ring[-1])
                for east, north in ring:
                    self.assertLessEqual(box.minimum[0], east)
                    self.assertLessEqual(east, box.maximum[0])
                    self.assertLessEqual(box.minimum[1], north)
                    self.assertLessEqual(north, box.maximum[1])
            for axis in range(3):
                self.assertLessEqual(self.city.bounds.minimum[axis], box.minimum[axis])
                self.assertLessEqual(box.maximum[axis], self.city.bounds.maximum[axis])
            self.assertGreater(footprint.height_m, 0)
            if not footprint.height_assumed:
                self.assertTrue(
                    math.isclose(
                        footprint.height_m, footprint.source_height_ft * 0.3048, abs_tol=0.00001
                    )
                )

    def test_web_polygons_use_matching_physical_ids_and_retain_courtyards(self) -> None:
        web = self.city.web_buildings()
        self.assertEqual(web, self.city.to_web_buildings())
        for record, box, footprint in zip(
            web, self.city.buildings, self.city.footprints, strict=True
        ):
            self.assertEqual(record["id"], box.obstacle_id)
            self.assertEqual(record["min"], list(box.minimum))
            self.assertEqual(record["max"], list(box.maximum))
            self.assertEqual(len(record["footprint"]), len(footprint.rings))
        self.assertEqual(self.city.to_web_metadata()["collisionModel"], "conservative-aabb")
        self.assertNotIn("footprints", self.city.to_web_metadata())

    def test_projection_keeps_genuine_east_north_axes(self) -> None:
        longitude, latitude = ORIGIN_WGS84
        self.assertEqual(lonlat_to_enu(longitude, latitude), (0.0, 0.0, 0.0))
        east = lonlat_to_enu(longitude + 0.001, latitude)
        north = lonlat_to_enu(longitude, latitude + 0.001)
        self.assertGreater(east[0], 80)
        self.assertLess(abs(east[1]), 0.001)
        self.assertGreater(north[1], 100)
        self.assertLess(abs(north[0]), 0.001)

    def test_southern_expansion_covers_battery_park_and_preserves_projection_origin(self) -> None:
        self.assertEqual(ORIGIN_WGS84, (-74.005, 40.742))
        self.assertEqual(self.city.metadata["requestedBoundsWgs84"], list(ROI_WGS84))
        self.assertEqual(self.city.metadata["planningRegion"]["id"], "manhattan-south-expanded-v4")
        self.assertEqual(ROI_WGS84, (-74.0200, 40.7000, -73.9670, 40.7676))
        width = self.city.bounds.maximum[0] - self.city.bounds.minimum[0]
        depth = self.city.bounds.maximum[1] - self.city.bounds.minimum[1]
        self.assertGreater(width, 4400)
        self.assertLess(width, 4800)
        self.assertGreater(depth, 7400)
        self.assertLess(depth, 8000)
        self.assertGreater(len(self.city.buildings), 20000)
        self.assertTrue(
            all(str(f["properties"]["BASE_BBL"]).startswith("1") for f in self.source["features"])
        )

    def test_real_city_is_shared_in_scenes_and_simulated_endpoints_are_free(self) -> None:
        start = self.city.rooftop_point(-73.9935, 40.7506)
        goal = self.city.rooftop_point(-73.9842, 40.7537)
        scene = make_city_scene(
            self.city, "test-penn-bryant", name="Test transfer", start=start, goal=goal
        )
        self.assertIs(scene.buildings, self.city.buildings)
        self.assertEqual(
            scene.metadata["city"]["sourceSha256"], self.city.to_web_metadata()["sourceSha256"]
        )
        self.assertEqual(scene.metadata["city"]["sourceSha256"], self.city.metadata["sourceSha256"])
        self.assertTrue(point_is_free(scene, start))
        self.assertTrue(point_is_free(scene, goal))
        self.assertFalse(segment_is_free(scene, start, goal))

    def test_source_roof_units_and_explicit_unknown_height_policy(self) -> None:
        empire = next(
            footprint for footprint in self.city.footprints if footprint.source_bin == 1015862
        )
        self.assertGreater(empire.height_m, 350)
        self.assertLess(empire.height_m, 400)
        self.assertFalse(empire.height_assumed)
        assumed = [footprint for footprint in self.city.footprints if footprint.height_assumed]
        self.assertEqual(
            len(assumed), self.city.metadata["heightAssumptions"]["missingHeightCount"]
        )
        for footprint in assumed:
            self.assertEqual(
                footprint.height_m, self.city.metadata["heightAssumptions"]["fallbackHeightM"]
            )

    def test_multipart_source_features_split_without_dropping_rings(self) -> None:
        first = copy.deepcopy(self.source["features"][0])
        second = self.source["features"][1]
        self.assertEqual(first["geometry"]["type"], "Polygon")
        self.assertEqual(second["geometry"]["type"], "Polygon")
        first["geometry"] = {
            "type": "MultiPolygon",
            "coordinates": [first["geometry"]["coordinates"], second["geometry"]["coordinates"]],
        }
        first["properties"]["HEIGHT_ROOF"] = None
        processed = preprocess_city({"type": "FeatureCollection", "features": [first]}, {})
        self.assertEqual(len(processed["buildings"]), 2)
        self.assertEqual(len({record["id"] for record in processed["buildings"]}), 2)
        self.assertEqual(processed["metadata"]["heightAssumptions"]["missingHeightCount"], 2)


if __name__ == "__main__":
    unittest.main()
