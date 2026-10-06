from __future__ import annotations

import math
import random
import unittest
from dataclasses import replace
from itertools import pairwise
from unittest.mock import patch

import uav3d.planners.rrt_star as rrt_module
from uav3d.geometry import distance, polyline_length
from uav3d.planners.rrt_star import RRTStar, RRTStarConfig
from uav3d.scene import AABB, Bounds3D, Scene
from uav3d.validation import audit_path


def detour_scene() -> Scene:
    return Scene(
        "informed-detour",
        "Informed detour",
        Bounds3D((0, 0, 0), (30, 30, 20)),
        (3, 15, 5),
        (27, 15, 5),
        (AABB("barrier", (12, 8, 0), (18, 22, 12)),),
        drone_radius=0.5,
        safety_margin=0.5,
    )


class RRTInformedTests(unittest.TestCase):
    def test_cost_prefilter_matches_eager_los_tree_and_quality_with_less_collision_work(
        self,
    ) -> None:
        class CapturedRRT(RRTStar):
            def _best_goal_parent(self, scene, nodes, costs, *args):
                self.costs = list(costs)
                return super()._best_goal_parent(scene, nodes, costs, *args)

            def _reconstruct(self, nodes, parents, current):
                self.nodes, self.parents = list(nodes), list(parents)
                return super()._reconstruct(nodes, parents, current)

        class EagerRRT(CapturedRRT):
            def _near_vertices(self, scene, nodes, candidate, nearest, radius):
                near, visibility = super()._near_vertices(scene, nodes, candidate, nearest, radius)
                for index in near:
                    visibility[index] = rrt_module.segment_is_free(scene, nodes[index], candidate)
                return [index for index in near if visibility[index]], visibility

        for informed in (False, True):
            with self.subTest(informed=informed):
                config = RRTStarConfig(
                    max_samples=500,
                    step_size=4,
                    goal_bias=0.18,
                    goal_tolerance=5,
                    neighbor_radius=12,
                    rewire_gamma=35,
                    quality_checkpoints=(40, 120, 500),
                    global_goal_connection=informed,
                    informed_sampling=informed,
                )
                lazy, eager = CapturedRRT(config), EagerRRT(config)
                with patch.object(
                    rrt_module, "segment_is_free", wraps=rrt_module.segment_is_free
                ) as los:
                    lazy_result = lazy.plan(detour_scene(), 12)
                    lazy_queries = los.call_count
                    los.reset_mock()
                    eager_result = eager.plan(detour_scene(), 12)
                    eager_queries = los.call_count
                self.assertTrue(lazy_result.success)
                self.assertEqual(lazy_result.path, eager_result.path)
                self.assertEqual(lazy.nodes, eager.nodes)
                self.assertEqual(lazy.parents, eager.parents)
                self.assertEqual(lazy.costs, eager.costs)
                self.assertEqual(lazy_result.iterations, eager_result.iterations)
                self.assertEqual(lazy_result.generated_nodes, eager_result.generated_nodes)
                self.assertEqual(
                    [point.best_path_length_m for point in lazy_result.quality_trace],
                    [point.best_path_length_m for point in eager_result.quality_trace],
                )
                self.assertLess(lazy_queries, eager_queries)

    def test_direct_lower_bound_is_opt_in_and_uses_no_sample_attempts(self) -> None:
        scene = Scene(
            "open",
            "Open",
            Bounds3D((0, 0, 0), (100, 100, 100)),
            (10, 50, 50),
            (90, 50, 50),
        )
        legacy = RRTStarConfig(max_samples=1, goal_bias=0)
        self.assertFalse(RRTStar(legacy).plan(scene).success)
        result = RRTStar(replace(legacy, direct_path_check=True)).plan(scene)
        self.assertTrue(result.success)
        self.assertEqual(result.path, (scene.start, scene.goal))
        self.assertEqual(result.iterations, 0)
        self.assertEqual(result.budget_usage.work_used, 0)
        self.assertEqual(result.budget_usage.termination, "goal-reached")
        self.assertEqual(result.quality_trace, ())
        expired = RRTStar(replace(legacy, direct_path_check=True, max_wall_time_ms=1e-9)).plan(
            scene
        )
        self.assertFalse(expired.success)
        self.assertEqual(expired.failure_reason, "wall-time-budget-exhausted")

    def test_global_goal_connection_checks_cheapest_bound_and_reuses_only_visibility(self) -> None:
        scene = Scene(
            "goal-edge",
            "Goal edge",
            Bounds3D((0, 0, 0), (100, 100, 100)),
            (10, 50, 50),
            (90, 50, 50),
            (AABB("wall", (45, 20, 0), (55, 80, 80)),),
        )
        nodes = [scene.start, (70, 90, 50), (85, 50, 50)]
        costs = [0.0, 90.0, 200.0]
        self.assertEqual(
            RRTStar(RRTStarConfig(goal_tolerance=9))._best_goal_parent(scene, nodes, costs), 2
        )
        planner = RRTStar(RRTStarConfig(goal_tolerance=9, global_goal_connection=True))
        visibility: dict[int, bool] = {}
        with patch.object(rrt_module, "segment_is_free", wraps=rrt_module.segment_is_free) as los:
            self.assertEqual(planner._best_goal_parent(scene, nodes, costs, visibility), 1)
            self.assertEqual(los.call_count, 2)  # blocked cheapest edge, then the winner
            self.assertEqual(planner._best_goal_parent(scene, nodes, costs, visibility), 1)
            self.assertEqual(los.call_count, 2)
            costs[2] = 100.0  # a rewire changes cost, so the previous winner is not cached
            self.assertEqual(planner._best_goal_parent(scene, nodes, costs, visibility), 2)
            self.assertEqual(los.call_count, 3)

    def test_informed_samples_stay_in_ellipsoid_and_are_seed_reproducible(self) -> None:
        config = RRTStarConfig(informed_sampling=True, informed_uniform_ratio=0)
        planner = RRTStar(config)
        for start, goal in (
            ((60, 100, 100), (140, 100, 100)),
            ((100, 100, 60), (100, 100, 140)),
            ((60, 60, 60), (140, 140, 140)),
        ):
            with self.subTest(start=start):
                scene = Scene(
                    "ellipse", "Ellipse", Bounds3D((0, 0, 0), (200, 200, 200)), start, goal
                )
                best = distance(start, goal) * 1.3
                first_rng = random.Random(47)
                second_rng = random.Random(47)
                points = [planner._sample_informed(scene, first_rng, best) for _ in range(250)]
                self.assertEqual(
                    points, [planner._sample_informed(scene, second_rng, best) for _ in points]
                )
                for point in points:
                    self.assertLessEqual(
                        distance(start, point) + distance(point, goal), best + 1e-9
                    )
                    self.assertTrue(all(2 <= value <= 198 for value in point))

    def test_degenerate_ellipsoid_and_uniform_fallback_terminate(self) -> None:
        scene = Scene(
            "line", "Line", Bounds3D((0, 0, 0), (100, 100, 100)), (10, 50, 50), (90, 50, 50)
        )
        planner = RRTStar(RRTStarConfig(informed_sampling=True, informed_uniform_ratio=0))
        for _ in range(20):
            point = planner._sample_informed(scene, random.Random(_), 80)
            self.assertEqual(point[1:], (50.0, 50.0))
            self.assertLessEqual(
                distance(scene.start, point) + distance(point, scene.goal), 80 + 1e-9
            )
        fallback = planner._sample_informed(scene, random.Random(1), math.inf)
        self.assertTrue(all(2 <= value <= 98 for value in fallback))

    def test_informed_tree_costs_paths_and_budget_checkpoints(self) -> None:
        class AuditedRRT(RRTStar):
            rewires = 0

            def _propagate_cost_delta(self, root, improvement, parents, costs):
                self.rewires += 1
                return super()._propagate_cost_delta(root, improvement, parents, costs)

            def _best_goal_parent(self, scene, nodes, costs, *args):
                self.costs = list(costs)
                return super()._best_goal_parent(scene, nodes, costs, *args)

            def _reconstruct(self, nodes, parents, current):
                self.nodes, self.parents = list(nodes), list(parents)
                return super()._reconstruct(nodes, parents, current)

        config = RRTStarConfig(
            max_samples=400,
            step_size=4,
            goal_bias=0.18,
            goal_tolerance=5,
            neighbor_radius=12,
            rewire_gamma=35,
            global_goal_connection=True,
            informed_sampling=True,
            quality_checkpoints=(40, 120, 400),
        )
        scene = detour_scene()
        planner = AuditedRRT(config)
        result = planner.plan(scene, seed=12)
        self.assertTrue(result.success)
        self.assertTrue(audit_path(scene, result.path).valid)
        self.assertGreater(planner.rewires, 0)
        self.assertEqual(result.iterations, 400)
        self.assertEqual(result.budget_usage.work_used, 400)
        self.assertEqual([point.work for point in result.quality_trace], [40, 120, 400])
        for index, expected in enumerate(planner.costs):
            length = 0.0
            seen = set()
            current = index
            while planner.parents[current] >= 0:
                self.assertNotIn(current, seen)
                seen.add(current)
                parent = planner.parents[current]
                length += distance(planner.nodes[current], planner.nodes[parent])
                current = parent
            self.assertAlmostEqual(length, expected, places=9)
        values = [
            point.best_path_length_m
            for point in result.quality_trace
            if point.best_path_length_m is not None
        ]
        self.assertTrue(all(later <= earlier + 1e-9 for earlier, later in pairwise(values)))
        prefix = RRTStar(replace(config, max_samples=120, quality_checkpoints=())).plan(
            scene, seed=12
        )
        self.assertAlmostEqual(
            result.quality_trace[1].best_path_length_m, polyline_length(prefix.path), places=9
        )
        repeated = RRTStar(config).plan(scene, seed=12)
        self.assertEqual(result.path, repeated.path)

    def test_new_parameters_are_rejected_when_invalid(self) -> None:
        for ratio in (-0.1, 1.1, math.nan, math.inf):
            with self.subTest(ratio=ratio), self.assertRaises(ValueError):
                RRTStarConfig(informed_uniform_ratio=ratio)
        with self.assertRaises(ValueError):
            RRTStarConfig(informed_refresh_interval=0)

    def test_all_search_parameters_require_finite_values_and_integer_work(self) -> None:
        for name in (
            "step_size",
            "goal_tolerance",
            "neighbor_radius",
            "rewire_gamma",
            "max_wall_time_ms",
        ):
            for value in (math.nan, math.inf, -math.inf, 0, -1, True):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    RRTStarConfig(**{name: value})
        for name in ("goal_bias", "informed_uniform_ratio"):
            for value in (math.nan, math.inf, -math.inf, -0.1, 1.1, True):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    RRTStarConfig(**{name: value})
        for name in ("max_samples", "informed_refresh_interval"):
            for value in (math.nan, math.inf, 1.5, 0, -1, True):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    RRTStarConfig(**{name: value})
        for checkpoint in (math.nan, math.inf, 1.5, 0, -1, True):
            with self.subTest(checkpoint=checkpoint), self.assertRaises(ValueError):
                RRTStarConfig(quality_checkpoints=(checkpoint,))
