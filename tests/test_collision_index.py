"""Differential checks for the city-scale broad phase against exhaustive queries."""

import math
import random
import unittest

from uav3d.collision import (
    _distance_to_aabb,
    _distance_to_cylinder,
    _inside_inset_bounds,
    _point_in_aabb,
    _point_in_cylinder,
    _segment_intersects_aabb,
    _segment_intersects_cylinder,
    point_clearance,
    point_is_free,
    segment_is_free,
)
from uav3d.scene import AABB, Bounds3D, Cylinder, Scene


class CityCollisionIndexTests(unittest.TestCase):
    def setUp(self):
        randomizer = random.Random(60261005)
        self.scene = Scene(
            "index-audit",
            "Index audit",
            Bounds3D((-400, -400, 0), (1600, 1600, 500)),
            (-350, -350, 20),
            (1550, 1550, 60),
            tuple(
                AABB(f"b{index}", (x, y, 0), (x + 24, y + 35, randomizer.uniform(12, 380)))
                for index, (x, y) in enumerate(
                    (-320 + i * 100, -300 + j * 100) for i in range(19) for j in range(19)
                )
            ),
            (Cylinder("restricted", (800, 720), 85, 0, 360),),
            drone_radius=1.5,
            safety_margin=5,
        )

    def test_points_and_edges_match_exhaustive_exact_queries(self):
        randomizer = random.Random(77)
        for _ in range(2000):
            a = tuple(
                randomizer.uniform(low - 10, high + 10)
                for low, high in zip(
                    self.scene.bounds.minimum, self.scene.bounds.maximum, strict=True
                )
            )
            b = tuple(a[axis] + randomizer.uniform(-140, 140) for axis in range(3))
            padding = randomizer.choice((0, 1, 6.5, 20, 120))
            expected_point = (
                _inside_inset_bounds(self.scene, a, padding)
                and not any(_point_in_aabb(a, box, padding) for box in self.scene.buildings)
                and not any(
                    _point_in_cylinder(a, zone, padding) for zone in self.scene.no_fly_zones
                )
            )
            expected_edge = (
                _inside_inset_bounds(self.scene, a, padding)
                and _inside_inset_bounds(self.scene, b, padding)
                and not any(
                    _segment_intersects_aabb(a, b, box, padding) for box in self.scene.buildings
                )
                and not any(
                    _segment_intersects_cylinder(a, b, zone, padding)
                    for zone in self.scene.no_fly_zones
                )
            )
            self.assertEqual(point_is_free(self.scene, a, padding), expected_point)
            self.assertEqual(segment_is_free(self.scene, a, b, padding), expected_edge)

    def test_nearest_clearance_is_exact_not_neighborhood_approximation(self):
        randomizer = random.Random(94)
        for _ in range(1500):
            point = tuple(
                randomizer.uniform(low - 5, high + 5)
                for low, high in zip(
                    self.scene.bounds.minimum, self.scene.bounds.maximum, strict=True
                )
            )
            expected = (
                min(
                    *(point[axis] - self.scene.bounds.minimum[axis] for axis in range(3)),
                    *(self.scene.bounds.maximum[axis] - point[axis] for axis in range(3)),
                    *(_distance_to_aabb(point, box) for box in self.scene.buildings),
                    *(_distance_to_cylinder(point, zone) for zone in self.scene.no_fly_zones),
                )
                - self.scene.drone_radius
            )
            self.assertTrue(
                math.isclose(point_clearance(self.scene, point), expected, abs_tol=1e-12)
            )

    def test_cell_boundaries_large_padding_and_long_cross_city_segments(self):
        for padding in (0, 6.5, 90, 300):
            for a, b in (
                ((-350, -350, 30), (1550, 1550, 30)),
                ((0, -350, 390), (0, 1550, 390)),
                ((80 - 1e-10, 80, 60), (80 + 1e-10, 80, 60)),
                ((1550, -350, 300), (-350, 1550, 300)),
            ):
                expected = (
                    _inside_inset_bounds(self.scene, a, padding)
                    and _inside_inset_bounds(self.scene, b, padding)
                    and not any(
                        _segment_intersects_aabb(a, b, box, padding) for box in self.scene.buildings
                    )
                    and not any(
                        _segment_intersects_cylinder(a, b, zone, padding)
                        for zone in self.scene.no_fly_zones
                    )
                )
                self.assertEqual(segment_is_free(self.scene, a, b, padding), expected)


if __name__ == "__main__":
    unittest.main()
