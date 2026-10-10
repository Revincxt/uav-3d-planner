"""Flight-aware search must improve routes, not just alter their visualization."""

from __future__ import annotations

import math
import random
from dataclasses import replace
from itertools import pairwise

import pytest

from uav3d.collision import path_is_free
from uav3d.dynamic import DynamicScenario
from uav3d.flight_cost import (
    flight_distance,
    mission_altitude_levels,
    vertical_travel,
    with_turn_clearance,
)
from uav3d.horizontal_curves import smooth_horizontal_curves
from uav3d.planners import (
    AStar3D,
    AStarConfig,
    DStarLite3D,
    DStarLiteConfig,
    LazyThetaStar,
    LazyThetaStarConfig,
)
from uav3d.planners.grid import VoxelGrid
from uav3d.planners.rrt_star import RRTStar, RRTStarConfig
from uav3d.planners.space_time_astar import SpaceTimeAStar3D, SpaceTimeAStarConfig
from uav3d.replanning import _candidate_traversal, _horizontal_escape, simulate_replanning
from uav3d.scene import AABB, Bounds3D, Scene


def barrier_scene() -> Scene:
    return Scene(
        "flight-cost",
        "Flight cost",
        Bounds3D((0, 0, 0), (120, 100, 60)),
        (10, 50, 13),
        (110, 50, 13),
        (AABB("roof", (50, 35, 0), (70, 65, 25)),),
        drone_radius=0.5,
        safety_margin=0.5,
    )


@pytest.mark.parametrize("kind", ["astar", "theta", "dstar"])
def test_slow_climb_prefers_a_level_horizontal_detour(kind: str) -> None:
    scene = barrier_scene()
    levels = mission_altitude_levels(scene, 5)
    if kind == "astar":
        old = AStar3D(AStarConfig(resolution=5)).plan(scene)
        new = AStar3D(
            AStarConfig(resolution=5, vertical_cost_scale=5, altitude_levels=levels)
        ).plan(scene)
    elif kind == "theta":
        old = LazyThetaStar(LazyThetaStarConfig(resolution=5)).plan(scene)
        new = LazyThetaStar(
            LazyThetaStarConfig(resolution=5, vertical_cost_scale=5, altitude_levels=levels)
        ).plan(scene)
    else:
        old = DStarLite3D(DStarLiteConfig(resolution=5)).plan(scene)
        new = DStarLite3D(
            DStarLiteConfig(resolution=5, vertical_cost_scale=5, altitude_levels=levels)
        ).plan(scene)
    assert old.success and new.success
    assert path_is_free(scene, new.path)
    assert new.path[0] == scene.start and new.path[-1] == scene.goal
    assert vertical_travel(new.path) < vertical_travel(old.path)
    assert vertical_travel(new.path) == pytest.approx(0)
    assert max(abs(p[1] - 50) for p in new.path) >= 16


def test_flight_metric_is_a_norm_and_has_a_consistent_heuristic() -> None:
    a, b, goal = (1, 2, 3), (4, 6, 8), (9, 2, 20)
    assert flight_distance(a, goal, 5) <= flight_distance(a, b, 5) + flight_distance(b, goal, 5)
    assert flight_distance(a, b, 5) == flight_distance(b, a, 5)
    assert flight_distance(a, b) == math.dist(a, b)


@pytest.mark.parametrize("scale", [0, -1, math.nan, math.inf])
def test_invalid_vertical_cost_is_rejected(scale: float) -> None:
    with pytest.raises(ValueError):
        AStarConfig(vertical_cost_scale=scale)
    with pytest.raises(ValueError):
        RRTStarConfig(vertical_cost_scale=scale)


def test_nonuniform_height_grid_contains_exact_required_anchor_layers() -> None:
    scene = replace(
        barrier_scene(), metadata={"missionTaskPoints": [{"position": [20, 40, 17.25]}]}
    )
    levels = mission_altitude_levels(scene, 5)
    grid = VoxelGrid(scene, 5, levels)
    assert grid.point(grid.nearest_index(scene.start))[2] == 13
    assert grid.point(grid.nearest_index((20, 40, 17.25)))[2] == 17.25
    assert len(levels) == grid.shape[2]
    with pytest.raises(ValueError):
        VoxelGrid(scene, 5, (13, 13))


def test_optional_curve_clearance_never_invalidates_a_valid_task() -> None:
    scene = replace(barrier_scene(), start=(48.9, 50, 13))
    reserved = with_turn_clearance(scene)
    assert reserved is scene
    assert reserved.safety_margin >= scene.safety_margin
    with pytest.raises(ValueError):
        with_turn_clearance(scene, -1)


def test_weighted_rrt_informed_samples_are_in_the_weighted_ellipsoid() -> None:
    scene = replace(barrier_scene(), bounds=Bounds3D((-1000, -1000, -1000), (1000, 1000, 1000)))
    planner = RRTStar(RRTStarConfig(vertical_cost_scale=5, informed_uniform_ratio=0))
    rng = random.Random(42)
    for _ in range(100):
        point = planner._sample_informed(scene, rng, 140)
        assert (
            flight_distance(scene.start, point, 5) + flight_distance(point, scene.goal, 5)
            <= 140 + 1e-7
        )


def test_space_time_connectors_obey_climb_rate_before_search() -> None:
    scene = replace(barrier_scene(), buildings=(), goal=(10, 50, 43))
    scenario = DynamicScenario("climb", "Climb", scene)
    config = SpaceTimeAStarConfig(
        resolution=5,
        time_step=0.5,
        cruise_speed=10,
        max_climb_rate=2,
        time_horizon=40,
        connectivity=26,
    )
    result = SpaceTimeAStar3D(config).plan(scenario)
    assert result.success and result.timed_path is not None
    assert result.timed_path.duration_s == pytest.approx(15)
    assert result.timed_path.is_safe(scenario)


def test_emergency_horizontal_escape_preserves_the_existing_heading_and_full_step_clock() -> None:
    scene = Scene(
        "escape",
        "Escape",
        Bounds3D((-200, -200, 0), (200, 200, 100)),
        (0, 0, 30),
        (100, 0, 30),
        drone_radius=0,
        safety_margin=0,
    )
    scenario = DynamicScenario("escape", "Escape", scene)
    path = _horizontal_escape(scenario, scene.start, 0, 2, 15, start_heading=(0, 1))
    assert len(path) > 2 and path_is_free(scene, path)
    first = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    assert first[1] / math.hypot(*first) > 0.99
    assert all(p[2] == 30 for p in path)
    length = sum(math.dist(a, b) for a, b in pairwise(path))
    traversals, _, elapsed = _candidate_traversal(path, 0, 2, min(15, length / 2))
    assert elapsed == pytest.approx(2)
    assert traversals[-1].end_time == pytest.approx(2)


def test_reactive_controller_exports_exact_variable_speed_knots_without_a_false_wait() -> None:
    scene = replace(barrier_scene(), buildings=(), goal=(70, 50, 43))
    scenario = DynamicScenario("climb", "Climb", scene)
    run = simulate_replanning(
        scenario,
        "repeated-astar-3d",
        cruise_speed=10,
        max_climb_rate=2,
        max_time=40,
        vertical_cost_scale=5,
        time_step=2,
    )
    trace = run.execution_timed_path
    assert run.metrics.success and trace is not None
    assert trace.duration_s == pytest.approx(15)
    assert trace.wait_time_s == 0 and trace.is_safe(scenario)
    assert max(trace.segment_speeds()) <= 10
    for a, b in pairwise(trace.waypoints):
        assert abs(b.position[2] - a.position[2]) / (b.time_s - a.time_s) <= 2 + 1e-9
    traversals, remaining, elapsed = _candidate_traversal(
        ((0, 0, 0), (0, 0, 20), (20, 0, 20)), 0, 5, 10, 2
    )
    assert elapsed == pytest.approx(5)
    assert traversals[-1].end == (0, 0, 10)
    assert remaining == ((0, 0, 10), (0, 0, 20), (20, 0, 20))


@pytest.mark.parametrize("protected", [frozenset(), frozenset({1.0})])
def test_hairpin_is_curved_with_nonzero_lateral_tangent(protected: frozenset[float]) -> None:
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0), (0.0, 0.0, 20.0))
    result = smooth_horizontal_curves(
        points, (0, 1, 2), lambda *_: True, round_reversals=True, protected=protected
    )
    assert result.rounded_corners == 1
    assert result.points[0] == points[0] and result.points[-1] == points[-1]
    assert max(abs(p[1]) for p in result.points) > 1
    if protected:
        assert result.points[result.parameters.index(1.0)] == points[1]
    turns = []
    for a, b, c in zip(result.points, result.points[1:], result.points[2:], strict=False):
        u, v = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
        lengths = math.hypot(*u) * math.hypot(*v)
        if lengths > 1e-12:
            turns.append(
                math.degrees(
                    math.acos(
                        max(-1, min(1, sum(x * y for x, y in zip(u, v, strict=True)) / lengths))
                    )
                )
            )
    assert max(turns) < 6
