"""Snapshot shortcuts never bypass the simulator's exact dynamic safety gate."""

from __future__ import annotations

import math
from itertools import pairwise

import pytest

import uav3d.replanning as replanning
from uav3d.collision import path_is_free
from uav3d.dynamic import (
    DynamicScenario,
    MovingSphere,
    TemporaryCylinder,
    load_builtin_dynamic_scenario,
)
from uav3d.planners.base import PlanningResult
from uav3d.predictive_study import _reactive_timed_path
from uav3d.replanning import REPLANNING_ALGORITHMS, simulate_replanning
from uav3d.scene import AABB, Bounds3D, Scene


def barrier_scenario() -> DynamicScenario:
    return DynamicScenario(
        "shortcut-barrier",
        "Snapshot shortcut barrier",
        Scene(
            "shortcut-static",
            "Static barrier",
            Bounds3D((0.0, 0.0, 0.0), (40.0, 40.0, 20.0)),
            (4.0, 20.0, 6.0),
            (36.0, 20.0, 6.0),
            buildings=(AABB("barrier", (16.0, 14.0, 0.0), (24.0, 26.0, 16.0)),),
            drone_radius=0.5,
            safety_margin=0.5,
        ),
    )


@pytest.mark.parametrize("algorithm", REPLANNING_ALGORITHMS)
def test_snapshot_shortcut_preserves_safety_and_improves_grid_route(algorithm: str) -> None:
    scenario = barrier_scenario()
    options = {"resolution": 4.0, "max_time": 100.0, "replan_interval": 100.0}
    original = simulate_replanning(scenario, algorithm, **options)
    explicit_default = simulate_replanning(scenario, algorithm, shortcut_paths=False, **options)
    optimized = simulate_replanning(scenario, algorithm, shortcut_paths=True, **options)
    assert original.to_dict() == explicit_default.to_dict()
    assert optimized.metrics.success
    assert optimized.metrics.collision_count == 0
    assert optimized.metrics.executed_path_length_m <= original.metrics.executed_path_length_m
    assert _reactive_timed_path(optimized).is_safe(scenario)
    assert optimized.parameters["path_shortcut"] == 1
    assert "path_shortcut" not in original.parameters
    assert (
        optimized.to_dict()
        == simulate_replanning(scenario, algorithm, shortcut_paths=True, **options).to_dict()
    )


@pytest.mark.parametrize("algorithm", REPLANNING_ALGORITHMS)
@pytest.mark.parametrize("scenario_id", ["crossing-traffic", "pop-up-nfz"])
def test_snapshot_shortcut_remains_safe_under_unknown_future_events(
    algorithm: str, scenario_id: str
) -> None:
    scenario = load_builtin_dynamic_scenario(scenario_id)
    optimized = simulate_replanning(scenario, algorithm, shortcut_paths=True, max_time=100.0)
    assert optimized.metrics.success
    assert optimized.metrics.collision_count == 0
    # This independently audits every movement subsegment and wait, not just frame chords.
    assert _reactive_timed_path(optimized).is_safe(scenario)


@pytest.mark.parametrize("replacement", ("equivalent-detour", "search-budget", "new-hazard"))
def test_smooth_replans_keep_a_safe_continuation_but_replace_it_when_blocked(
    monkeypatch: pytest.MonkeyPatch, replacement: str
) -> None:
    base = barrier_scenario()
    scenario = DynamicScenario(
        base.scenario_id,
        base.name,
        base.static_scene,
        temporary_cylinders=(TemporaryCylinder("closure", (20.0, 30.0), 3.0, 0.0, 20.0, 1.0, 30.0),)
        if replacement == "new-hazard"
        else (),
    )
    calls = 0

    class AlternatingPlanner:
        def plan(self, snapshot: Scene, seed: int = 0) -> PlanningResult:
            nonlocal calls
            calls += 1
            if calls > 1 and replacement == "search-budget":
                return PlanningResult("astar-3d", False, (), 0.0, failure_reason="budget")
            y = 30.0 if calls == 1 else 10.0
            path = (snapshot.start, (12.0, y, 6.0), (28.0, y, 6.0), snapshot.goal)
            if not path_is_free(snapshot, path):
                return PlanningResult("astar-3d", False, (), 0.0, failure_reason="no-path")
            return PlanningResult("astar-3d", True, path, 0.0)

    monkeypatch.setattr(replanning, "_planner", lambda *_: AlternatingPlanner())
    run = simulate_replanning(
        scenario,
        "repeated-astar-3d",
        time_step=0.25,
        replan_interval=0.5,
        cruise_speed=4.0,
        max_time=100.0,
        smooth_turns=True,
        spatial_curves=True,
        preserve_altitude=True,
        max_climb_rate=2.0,
    )
    assert run.metrics.success and run.metrics.collision_count == 0
    assert run.execution_timed_path is not None and run.execution_timed_path.is_safe(scenario)
    ys = [point[1] for point in run.execution_timed_path.positions]
    if replacement == "new-hazard":
        assert min(ys) < 15.0
    else:
        assert min(ys) >= 20.0 - 1e-9
        assert run.metrics.holds == 0
    if replacement == "search-budget":
        assert run.metrics.failed_replans > 0


def test_partial_emergency_escape_is_not_retained_as_a_complete_mission_plan(monkeypatch):
    scene = Scene(
        "escape-tail",
        "Escape tail",
        Bounds3D((0, 0, 0), (40, 40, 20)),
        (2, 20, 6),
        (36, 20, 6),
        drone_radius=0.1,
        safety_margin=0.1,
    )
    scenario = DynamicScenario(
        "escape-tail",
        "Escape tail",
        scene,
        moving_spheres=(
            MovingSphere(
                "crossing", 0.5, ((0.0, (2, 22, 6)), (1.0, (2, 20, 6)), (20.0, (2, -18, 6)))
            ),
        ),
    )
    calls = 0

    class Planner:
        def plan(self, snapshot, seed=0):
            nonlocal calls
            calls += 1
            if calls == 1:
                return PlanningResult("astar-3d", False, (), 0.0, failure_reason="no-path")
            return PlanningResult("astar-3d", True, (snapshot.start, snapshot.goal), 0.0)

    monkeypatch.setattr(replanning, "_planner", lambda *_: Planner())
    monkeypatch.setattr(
        replanning, "_horizontal_escape", lambda *a, **k: ((2, 20, 6), (10, 20, 6), (10.001, 20, 6))
    )
    run = simulate_replanning(
        scenario,
        "repeated-astar-3d",
        time_step=2.0,
        replan_interval=2.0,
        cruise_speed=4.0,
        max_time=20.0,
        smooth_turns=True,
        spatial_curves=True,
        preserve_altitude=True,
        allow_horizontal_escape=True,
        planning_guard_s=2.0,
        vertical_cost_scale=5,
        max_climb_rate=2.0,
    )
    assert run.metrics.success and run.metrics.holds == 0
    assert run.metrics.completion_time_s == pytest.approx(8.5)
    for frame in run.frames:
        assert any(
            abs(w.time_s - frame.time_s) < 1e-10 and w.position == frame.position
            for w in run.execution_timed_path.waypoints
        )
    assert run.execution_timed_path.is_safe(scenario)


def test_emergency_escape_keeps_a_descending_spatial_tangent():
    scene = Scene(
        "spatial-escape",
        "Spatial escape",
        Bounds3D((0, 0, 0), (100, 100, 40)),
        (20, 20, 20),
        (80, 20, 20),
        drone_radius=0.1,
        safety_margin=0.1,
    )
    scenario = DynamicScenario("spatial-escape", "Spatial escape", scene)
    path = replanning._horizontal_escape(
        scenario, scene.start, 0, 2, 4, start_heading=(0.0, 0.0, -1.0), max_climb_rate=2
    )
    assert len(path) > 2
    first = path[1]
    assert first[2] < scene.start[2]
    assert abs(first[0] - scene.start[0]) < (scene.start[2] - first[2]) * 0.01
    traversals, _, _ = replanning._candidate_traversal(path, 0, 2, 4, 2)
    assert replanning._safe_traversals(scenario, traversals)


@pytest.mark.parametrize("new_closure", (False, True))
def test_committed_avoidance_passes_head_on_traffic_before_returning(new_closure: bool) -> None:
    scenario = DynamicScenario(
        "head-on-traffic",
        "Head-on traffic",
        Scene(
            "open-corridor",
            "Open corridor",
            Bounds3D((0, 0, 0), (500, 300, 100)),
            (20, 150, 40),
            (480, 150, 40),
            drone_radius=1,
            safety_margin=2,
        ),
        temporary_cylinders=(TemporaryCylinder("new-closure", (152, 103), 6, 0, 100, 9, 100),)
        if new_closure
        else (),
        moving_spheres=(
            MovingSphere(
                "cargo",
                24,
                ((0, (250, 150, 40)), (10, (150, 150, 40)), (25, (0, 150, 40)), (100, (0, 0, 40))),
            ),
        ),
    )
    run = simulate_replanning(
        scenario,
        "repeated-astar-3d",
        time_step=1,
        replan_interval=4,
        planning_guard_s=6,
        cruise_speed=14,
        resolution=10,
        max_time=100,
        shortcut_paths=True,
        preserve_altitude=True,
        smooth_turns=True,
        spatial_curves=True,
        allow_horizontal_escape=True,
        vertical_cost_scale=5,
        max_climb_rate=3,
        altitude_levels=(40,),
    )
    assert run.metrics.success and run.metrics.collision_count == 0 and run.metrics.holds == 0
    assert run.execution_timed_path is not None and run.execution_timed_path.is_safe(scenario)
    assert run.metrics.safety_gate_activations <= 2
    # One continuous pass on one side, followed by one return. A collision-safe
    # zigzag can still fail this check even when its dense sample angles are tiny.
    ys = [f.position[1] for f in run.frames]
    lowest = min(range(len(ys)), key=ys.__getitem__)
    assert all(b <= a + 1e-6 for a, b in pairwise(ys[: lowest + 1]))
    assert all(b >= a - 1e-6 for a, b in pairwise(ys[lowest:]))
    # The local escape is retained across several control steps rather than
    # discarding it and steering back at the very next search.
    local_frames = [f for f in run.frames if f.replan_reason == "safety-gate"]
    assert len(local_frames) >= 3
    for frame in run.frames:
        assert any(
            abs(w.time_s - frame.time_s) < 1e-8 and w.position == frame.position
            for w in run.execution_timed_path.waypoints
        )


def test_overtaking_a_patrolling_aircraft_keeps_one_passing_side():
    scenario = DynamicScenario(
        "overtaking-patrol",
        "Overtaking patrol",
        Scene(
            "corridor",
            "Corridor",
            Bounds3D((0, 0, 0), (1000, 400, 100)),
            (20, 150, 40),
            (700, 150, 40),
            drone_radius=1,
            safety_margin=2,
        ),
        moving_spheres=(
            MovingSphere(
                "cargo",
                26,
                tuple(
                    (t, (160 + 8 * t, 150 + 40 * math.sin(t * math.tau / 8), 40))
                    for t in range(0, 91, 2)
                ),
            ),
        ),
    )
    run = simulate_replanning(
        scenario,
        "repeated-astar-3d",
        time_step=1,
        replan_interval=4,
        planning_guard_s=6,
        cruise_speed=14,
        resolution=10,
        max_time=100,
        shortcut_paths=True,
        preserve_altitude=True,
        smooth_turns=True,
        spatial_curves=True,
        allow_horizontal_escape=True,
        vertical_cost_scale=5,
        max_climb_rate=3,
        altitude_levels=(40,),
    )
    assert run.metrics.success and run.metrics.holds == 0 and run.metrics.collision_count == 0
    assert run.execution_timed_path is not None and run.execution_timed_path.is_safe(scenario)
    assert run.metrics.executed_path_length_m < 780
    ys = [f.position[1] for f in run.frames]
    assert max(abs(y - 150) for y in ys) > 30
    changes = [b - a for a, b in pairwise(ys) if abs(b - a) > 0.3]
    assert sum(a * b < 0 for a, b in pairwise(changes)) <= 1
