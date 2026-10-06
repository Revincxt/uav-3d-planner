from __future__ import annotations

import itertools
import unittest
from unittest.mock import patch

import uav3d.planners.grid as grid_module
from uav3d.planners.astar import AStar3D, AStarConfig
from uav3d.planners.grid import VoxelGrid
from uav3d.planners.lazy_theta import LazyThetaStar, LazyThetaStarConfig
from uav3d.scene import AABB, Bounds3D, Scene


def scene(buildings=()) -> Scene:
    return Scene(
        "grid",
        "Grid",
        Bounds3D((0, 0, 0), (12, 12, 12)),
        (2, 2, 2),
        (10, 10, 10),
        buildings,
        drone_radius=0,
        safety_margin=0,
    )


class GridCacheTests(unittest.TestCase):
    def test_cache_preserves_search_path_and_work_exactly(self) -> None:
        blocked = scene((AABB("barrier", (4, 3, 0), (8, 9, 8)),))
        for planner in (
            AStar3D(AStarConfig(resolution=2)),
            LazyThetaStar(LazyThetaStarConfig(resolution=2)),
        ):
            with self.subTest(planner=planner.algorithm_id):
                cached = planner.plan(blocked)
                with (
                    patch.object(
                        VoxelGrid,
                        "is_free",
                        lambda self, index: grid_module.point_is_free(
                            self.scene, self.point(index)
                        ),
                    ),
                    patch.object(
                        VoxelGrid,
                        "edge_is_free",
                        lambda self, first, second: grid_module.segment_is_free(
                            self.scene, self.point(first), self.point(second)
                        ),
                    ),
                    patch.object(
                        VoxelGrid,
                        "visible_from",
                        lambda self, point, index: grid_module.segment_is_free(
                            self.scene, point, self.point(index)
                        ),
                    ),
                ):
                    uncached = planner.plan(blocked)
                self.assertTrue(cached.success)
                self.assertEqual(cached.path, uncached.path)
                self.assertEqual(cached.expanded_nodes, uncached.expanded_nodes)
                self.assertEqual(cached.generated_nodes, uncached.generated_nodes)

    def test_cache_capacity_never_changes_collision_results(self) -> None:
        grid = VoxelGrid(scene((AABB("wall", (4, 3, 0), (8, 9, 8)),)), 2)
        with patch.object(grid_module, "_CACHE_LIMIT", 2):
            for index in itertools.product(range(7), repeat=3):
                self.assertEqual(
                    grid.is_free(index), grid_module.point_is_free(grid.scene, grid.point(index))
                )
            self.assertEqual(len(grid._point_cache), 2)

    def test_neighbor_point_and_symmetric_edge_queries_are_cached(self) -> None:
        grid = VoxelGrid(scene(), 2)
        with (
            patch.object(grid_module, "point_is_free", wraps=grid_module.point_is_free) as point,
            patch.object(grid_module, "segment_is_free", wraps=grid_module.segment_is_free) as edge,
        ):
            expected = grid.neighbors((2, 2, 2))
            calls = point.call_count, edge.call_count
            self.assertEqual(len(expected), 26)
            self.assertEqual(grid.neighbors((2, 2, 2)), expected)
            self.assertEqual((point.call_count, edge.call_count), calls)
            self.assertTrue(grid.edge_is_free((3, 2, 2), (2, 2, 2)))
            self.assertEqual(edge.call_count, calls[1])

    def test_anchor_visibility_cache_is_local_to_a_grid_and_a_static_snapshot(self) -> None:
        first = VoxelGrid(scene(), 2)
        with patch.object(
            grid_module, "segment_is_free", wraps=grid_module.segment_is_free
        ) as edge:
            expected = first.anchor_indices((3, 3, 3))
            calls = edge.call_count
            self.assertEqual(first.anchor_indices((3, 3, 3)), expected)
            self.assertEqual(edge.call_count, calls)
            self.assertEqual(VoxelGrid(scene(), 2).anchor_indices((3, 3, 3)), expected)
            self.assertGreater(edge.call_count, calls)
        blocked = VoxelGrid(scene((AABB("new-snapshot", (3, 3, 3), (5, 5, 5)),)), 2)
        self.assertTrue(first.is_free((2, 2, 2)))
        self.assertFalse(blocked.is_free((2, 2, 2)))

    def test_nearest_enumeration_matches_full_sort_including_ties_and_boundaries(self) -> None:
        grid = VoxelGrid(scene(), 2)
        for point in ((5.1, 4.9, 7.2), (0, 0, 0), (12, 12, 12)):
            with self.subTest(point=point):
                origin = grid.nearest_index(point)
                expected = sorted(
                    itertools.product(*(range(size) for size in grid.shape)),
                    key=lambda item: (sum((item[i] - origin[i]) ** 2 for i in range(3)), item),
                )
                queried = []
                with patch.object(
                    VoxelGrid,
                    "is_free",
                    lambda self, index, queried=queried: queried.append(index) or False,
                ):
                    self.assertIsNone(grid.nearest_free_index(point))
                self.assertEqual(queried, expected)

    def test_nearest_visible_anchor_preserves_obstacle_semantics(self) -> None:
        grid = VoxelGrid(scene((AABB("barrier", (4, 3, 0), (8, 9, 8)),)), 2)
        point = (3, 6, 3)
        origin = grid.nearest_index(point)
        expected = next(
            (
                index
                for index in sorted(
                    itertools.product(*(range(size) for size in grid.shape)),
                    key=lambda item: (sum((item[i] - origin[i]) ** 2 for i in range(3)), item),
                )
                if grid_module.point_is_free(grid.scene, grid.point(index))
                and grid_module.segment_is_free(grid.scene, point, grid.point(index))
            ),
            None,
        )
        self.assertEqual(grid.nearest_free_index(point), expected)

    def test_nearest_free_index_checks_nearby_node_without_building_entire_grid(self) -> None:
        large = Scene(
            "large",
            "Large",
            Bounds3D((0, 0, 0), (10000, 10000, 10000)),
            (2, 2, 2),
            (9998, 9998, 9998),
        )
        grid = VoxelGrid(large, 2)
        with patch.object(grid_module, "point_is_free", wraps=grid_module.point_is_free) as point:
            self.assertEqual(grid.nearest_free_index((10, 10, 10)), (5, 5, 5))
            self.assertEqual(point.call_count, 1)
