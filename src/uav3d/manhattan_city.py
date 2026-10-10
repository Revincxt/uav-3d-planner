"""Shared real Manhattan geometry from the official NYC OTI building layer.

The original polygon rings are retained for rendering. Existing planners use conservative
axis-aligned envelopes of those polygons, with source roof heights converted from feet to metres.
No decorative or generated buildings are added to this physical city.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

from uav3d.geometry import Point3, as_point
from uav3d.scene import AABB, Bounds3D, Cylinder, Scene

SERVICE_URL = "https://services6.arcgis.com/yG5s3afENB5iO9fj/arcgis/rest/services/BUILDING_view/FeatureServer/0"
METADATA_URL = "https://github.com/CityOfNewYork/nyc-geo-metadata/blob/main/Metadata/Metadata_BuildingFootprints.md"
# Preserve the northern district and extend south across the Manhattan peninsula.
# The projection origin stays fixed so buildings, maps and missions remain aligned.
ROI_WGS84 = (-74.0200, 40.7000, -73.9670, 40.7676)
ORIGIN_WGS84 = (-74.005, 40.742)
DATA_DIRECTORY = Path(__file__).resolve().parents[2] / "data" / "manhattan"
FEET_TO_METRES = 0.3048
UNKNOWN_HEIGHT_M = 40.0

Point2 = tuple[float, float]
Ring = tuple[Point2, ...]


def _ecef(longitude: float, latitude: float) -> Point3:
    longitude_rad = math.radians(longitude)
    latitude_rad = math.radians(latitude)
    eccentricity_squared = 6.6943799901413165e-3
    prime_vertical_radius = 6378137.0 / math.sqrt(
        1 - eccentricity_squared * math.sin(latitude_rad) ** 2
    )
    return (
        prime_vertical_radius * math.cos(latitude_rad) * math.cos(longitude_rad),
        prime_vertical_radius * math.cos(latitude_rad) * math.sin(longitude_rad),
        prime_vertical_radius * (1 - eccentricity_squared) * math.sin(latitude_rad),
    )


def lonlat_to_enu(longitude: float, latitude: float) -> Point3:
    """Project WGS84 coordinates onto genuine local east/north axes; flatten ground to z=0."""
    origin = _ecef(*ORIGIN_WGS84)
    point = _ecef(longitude, latitude)
    dx, dy, dz = (point[index] - origin[index] for index in range(3))
    longitude_rad, latitude_rad = (math.radians(value) for value in ORIGIN_WGS84)
    east = -math.sin(longitude_rad) * dx + math.cos(longitude_rad) * dy
    north = (
        -math.sin(latitude_rad) * math.cos(longitude_rad) * dx
        - math.sin(latitude_rad) * math.sin(longitude_rad) * dy
        + math.cos(latitude_rad) * dz
    )
    return (east, north, 0.0)


@dataclass(frozen=True, slots=True)
class ManhattanFootprint:
    building_id: str
    rings: tuple[Ring, ...]
    height_m: float
    source_doitt_id: int
    source_bin: int | None
    name: str | None
    source_height_ft: float | None
    height_assumed: bool


@dataclass(frozen=True, slots=True)
class ManhattanCity:
    city_id: str
    bounds: Bounds3D
    buildings: tuple[AABB, ...]
    footprints: tuple[ManhattanFootprint, ...]
    metadata: dict[str, Any]

    def project(self, longitude: float, latitude: float) -> Point3:
        return lonlat_to_enu(longitude, latitude)

    def to_web_metadata(self) -> dict[str, Any]:
        return dict(self.metadata)

    def web_buildings(self) -> list[dict[str, Any]]:
        return [
            {
                "id": box.obstacle_id,
                "min": list(box.minimum),
                "max": list(box.maximum),
                "footprint": [[list(point) for point in ring] for ring in footprint.rings],
                "sourceBin": footprint.source_bin,
                "sourceDoittId": footprint.source_doitt_id,
                "heightAssumed": footprint.height_assumed,
            }
            for box, footprint in zip(self.buildings, self.footprints, strict=True)
        ]

    def to_web_buildings(self) -> list[dict[str, Any]]:
        return self.web_buildings()

    def rooftop_point(self, longitude: float, latitude: float, clearance_m: float = 12.0) -> Point3:
        """A simulated launch point above every conservative envelope at this location."""
        east, north, _ = self.project(longitude, latitude)
        nearby = [
            box
            for box in self.buildings
            if box.minimum[0] - 3 <= east <= box.maximum[0] + 3
            and box.minimum[1] - 3 <= north <= box.maximum[1] + 3
        ]
        if not nearby:
            nearest = min(
                self.buildings,
                key=lambda box: math.hypot(
                    (box.minimum[0] + box.maximum[0]) / 2 - east,
                    (box.minimum[1] + box.maximum[1]) / 2 - north,
                ),
            )
            east = (nearest.minimum[0] + nearest.maximum[0]) / 2
            north = (nearest.minimum[1] + nearest.maximum[1]) / 2
            nearby = [
                box
                for box in self.buildings
                if box.minimum[0] - 3 <= east <= box.maximum[0] + 3
                and box.minimum[1] - 3 <= north <= box.maximum[1] + 3
            ]
        return (east, north, max(box.maximum[2] for box in nearby) + clearance_m)


def preprocess_city(source: dict[str, Any], provenance: dict[str, Any]) -> dict[str, Any]:
    """Convert unmodified official source rings to metres, keeping all selected features."""
    buildings: list[dict[str, Any]] = []
    assumed: list[str] = []
    source_placeholders = 0
    for feature in source["features"]:
        properties = feature["properties"]
        geometry = feature["geometry"]
        if geometry is None or geometry["type"] not in {"Polygon", "MultiPolygon"}:
            raise ValueError("Every retained official feature must have polygon geometry")
        polygons = (
            [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
        )
        doitt_id = int(properties["DOITT_ID"])
        raw_height = properties.get("HEIGHT_ROOF")
        valid_height = (
            isinstance(raw_height, (int, float)) and math.isfinite(raw_height) and raw_height > 0
        )
        height = float(raw_height) * FEET_TO_METRES if valid_height else UNKNOWN_HEIGHT_M
        source_placeholders += int(properties.get("FEATURE_CODE") == 1003)
        for part_index, polygon in enumerate(polygons):
            obstacle_id = f"nyc-{doitt_id}-part-{part_index + 1}"
            rings = []
            for ring in polygon:
                projected = [lonlat_to_enu(float(point[0]), float(point[1]))[:2] for point in ring]
                if len(projected) < 4 or projected[0] != projected[-1]:
                    raise ValueError(f"Invalid or unclosed official footprint ring {obstacle_id}")
                rings.append([[round(value, 5) for value in point] for point in projected])
            all_points = [point for ring in rings for point in ring]
            minimum = [
                min(point[0] for point in all_points),
                min(point[1] for point in all_points),
                0.0,
            ]
            maximum = [
                max(point[0] for point in all_points),
                max(point[1] for point in all_points),
                round(height, 5),
            ]
            if minimum[0] >= maximum[0] or minimum[1] >= maximum[1]:
                raise ValueError(f"Degenerate official footprint {obstacle_id}")
            if not valid_height:
                assumed.append(obstacle_id)
            buildings.append(
                {
                    "id": obstacle_id,
                    "min": minimum,
                    "max": maximum,
                    "footprint": rings,
                    "sourceDoittId": doitt_id,
                    "sourceBin": properties.get("BIN"),
                    "sourceName": properties.get("NAME"),
                    "sourceHeightFt": raw_height,
                    "heightAssumed": not valid_height,
                    "sourceFeatureCode": properties.get("FEATURE_CODE"),
                    "groundElevationFt": properties.get("GROUND_ELEVATION"),
                }
            )
    buildings.sort(key=lambda building: building["id"])
    if not buildings:
        raise ValueError("The official Manhattan extract must not be empty")
    if len({building["id"] for building in buildings}) != len(buildings):
        raise ValueError("Duplicate source polygon identifiers; refetch a stable complete extract")
    padding = 30.0
    min_east = min(building["min"][0] for building in buildings) - padding
    min_north = min(building["min"][1] for building in buildings) - padding
    max_east = max(building["max"][0] for building in buildings) + padding
    max_north = max(building["max"][1] for building in buildings) + padding
    max_height = max(building["max"][2] for building in buildings)
    bounds = {
        "min": [round(min_east, 5), round(min_north, 5), 0.0],
        "max": [
            round(max_east, 5),
            round(max_north, 5),
            max(240.0, math.ceil((max_height + 40) / 20) * 20),
        ],
    }
    metadata = {
        **provenance,
        "id": "nyc-manhattan-midtown-official",
        "name": "Manhattan · Midtown to Battery Park",
        "sourceKind": "nyc-open-data",
        "collisionModel": "conservative-aabb",
        "buildingCount": len(buildings),
        "coordinateSystem": {
            "frame": "ENU",
            "unit": "m",
            "originWgs84": list(ORIGIN_WGS84),
            "projection": (
                "WGS84 ECEF to local east/north tangent plane; "
                "horizontal geometry retained; no axis rotation."
            ),
            "groundModel": (
                "Flattened ground at z=0; HEIGHT_ROOF is height above each "
                "source building's local ground."
            ),
        },
        "heightAssumptions": {
            "missingHeightCount": len(assumed),
            "fallbackHeightM": UNKNOWN_HEIGHT_M,
            "rule": (
                "Source HEIGHT_ROOF zero, null or non-finite is unknown; "
                "retained obstacles use an explicitly assumed 40 m height."
            ),
            "buildingIds": assumed,
        },
        "sourcePlaceholderCount": source_placeholders,
        "collisionApproximation": (
            "Each physical polygon part uses its enclosing axis-aligned box; "
            "concavities and courtyards are conservatively blocked. "
            "All buildings participate in planning."
        ),
        "bounds": bounds,
        "geometrySimplified": False,
        "maxSourceRoofHeightM": max_height,
        "planningRegion": {
            "id": "manhattan-south-expanded-v4",
            "requestedBoundsWgs84": list(ROI_WGS84),
            "selectionPurpose": (
                "Extend the existing northern district south to Battery Park, including "
                "Financial District, Tribeca, SoHo and Greenwich Village; retain complete "
                "Manhattan polygons without clipping or simplification."
            ),
            "fixedProjectionOrigin": True,
            "previousRegionWgs84": [-74.0078, 40.7368, -73.9670, 40.7676],
        },
    }
    return {"schemaVersion": 1, "metadata": metadata, "bounds": bounds, "buildings": buildings}


@lru_cache(maxsize=1)
def build_manhattan_city(path: Path | None = None) -> ManhattanCity:
    path = path if path is not None else DATA_DIRECTORY / "city.json"
    if not path.is_file():
        raise FileNotFoundError("Official NYC extract missing; run data/manhattan/fetch_source.py")
    data = json.loads(path.read_text(encoding="utf-8"))
    boxes = tuple(
        AABB(record["id"], as_point(record["min"]), as_point(record["max"]))
        for record in data["buildings"]
    )
    footprints = tuple(
        ManhattanFootprint(
            record["id"],
            tuple(
                tuple((float(point[0]), float(point[1])) for point in ring)
                for ring in record["footprint"]
            ),
            float(record["max"][2]),
            int(record["sourceDoittId"]),
            record["sourceBin"],
            record.get("sourceName"),
            record.get("sourceHeightFt"),
            bool(record["heightAssumed"]),
        )
        for record in data["buildings"]
    )
    return ManhattanCity(
        data["metadata"]["id"],
        Bounds3D(as_point(data["bounds"]["min"]), as_point(data["bounds"]["max"])),
        boxes,
        footprints,
        data["metadata"],
    )


def make_city_scene(
    city: ManhattanCity,
    mission_id: str,
    *,
    name: str,
    start: Point3,
    goal: Point3,
    no_fly_zones: tuple[Cylinder, ...] = (),
    metadata: dict[str, Any] | None = None,
    drone_radius: float = 1.0,
    safety_margin: float = 2.0,
) -> Scene:
    return Scene(
        mission_id,
        name,
        city.bounds,
        start,
        goal,
        city.buildings,
        no_fly_zones,
        drone_radius,
        safety_margin,
        {"family": "real-manhattan-city", "city": city.to_web_metadata(), **(metadata or {})},
    )


__all__ = [
    "METADATA_URL",
    "ORIGIN_WGS84",
    "ROI_WGS84",
    "SERVICE_URL",
    "ManhattanCity",
    "ManhattanFootprint",
    "build_manhattan_city",
    "lonlat_to_enu",
    "make_city_scene",
    "preprocess_city",
]
