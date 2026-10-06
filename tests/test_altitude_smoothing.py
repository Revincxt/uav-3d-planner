"""XY-only optimization keeps the original altitude profile as a hard constraint."""

from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

import pytest

import uav3d.replanning as replanning
import uav3d.smoothing as smoothing
from uav3d.collision import path_is_free
from uav3d.dynamic import DynamicScenario, load_builtin_dynamic_scenario
from uav3d.geometry import Point3, distance
from uav3d.planners.base import PlanningResult
from uav3d.predictive_study import _reactive_timed_path
from uav3d.replanning import REPLANNING_ALGORITHMS, simulate_replanning
from uav3d.scene import AABB, Bounds3D, Scene
from uav3d.smoothing import farthest_visible_shortcut, shortest_visible_shortcut, smooth_path


def scene_for_path(path: Sequence[Point3], buildings: tuple[AABB, ...] = ()) -> Scene:
    return Scene(
        "altitude-profile",
        "Preserved altitude profile",
        Bounds3D((0.0, 0.0, 0.0), (20.0, 20.0, 20.0)),
        path[0],
        path[-1],
        buildings=buildings,
        drone_radius=0.1,
        safety_margin=0.1,
    )


def vertical_travel(path: Sequence[Point3]) -> float:
    return math.fsum(abs(following[2] - current[2]) for current, following in pairwise(path))


def horizontal_length(path: Sequence[Point3]) -> float:
    return math.fsum(
        math.hypot(following[0] - current[0], following[1] - current[1])
        for current, following in pairwise(path)
    )


def raw_progress(path: Sequence[Point3]) -> list[float]:
    cumulative = [0.0]
    for current, following in pairwise(path):
        cumulative.append(cumulative[-1] + distance(current, following))
    return [value / cumulative[-1] for value in cumulative]


def assert_preserved_profile(path: Sequence[Point3], result: smoothing.SmoothingResult) -> None:
    assert result.collision_free
    assert result.altitude_policy == "preserve-raw-altitude-profile-v1"
    assert result.altitude_profile_max_error == 0.0
    assert len(result.altitude_progress) == len(result.path)
    assert result.path[0] == path[0]
    assert result.path[-1] == path[-1]
    # This compares the declared original-progress parameter, not the changed curve's arc length.
    for point, parameter in zip(path, raw_progress(path), strict=True):
        index = result.altitude_progress.index(parameter)
        assert result.path[index][2] == point[2]
    assert min(point[2] for point in result.path) == min(point[2] for point in path)
    assert max(point[2] for point in result.path) == max(point[2] for point in path)
    assert vertical_travel(result.path) == pytest.approx(vertical_travel(path), abs=1e-10)


@pytest.mark.parametrize("optimize", [False, True])
def test_shortcuts_reduce_horizontal_detour_without_removing_altitude_peaks(optimize: bool) -> None:
    raw = [
        (2.0, 2.0, 4.0),
        (2.0, 14.0, 12.0),
        (6.0, 17.0, 7.0),
        (14.0, 14.0, 15.0),
        (18.0, 2.0, 4.0),
    ]
    scene = scene_for_path(raw)
    result = smooth_path(scene, raw, optimize_shortcuts=optimize, preserve_altitude=True)
    assert_preserved_profile(raw, result)
    assert horizontal_length(result.path) == pytest.approx(16.0)
    assert horizontal_length(result.path) < horizontal_length(raw)
    assert [point[2] for point in result.path] == [point[2] for point in raw]
    shortcut = shortest_visible_shortcut if optimize else farthest_visible_shortcut
    assert shortcut(scene, raw, preserve_altitude=True) == list(result.path)
    # The compatibility default is intentionally unchanged and remains fully 3D.
    default = smooth_path(scene, raw)
    assert default.path == (raw[0], raw[-1])
    assert default.to_dict() == smooth_path(scene, raw, preserve_altitude=False).to_dict()
    assert "altitude_policy" not in default.to_dict()


def test_pure_vertical_segments_and_tiny_nonzero_height_changes_survive() -> None:
    raw = [
        (4.0, 4.0, 3.0),
        (4.0, 4.0, 3.0000000001),
        (4.0, 4.0, 11.0),
        (4.0, 4.0, 5.0),
        (4.0, 4.0, 16.0),
    ]
    result = smooth_path(scene_for_path(raw), raw, preserve_altitude=True, optimize_shortcuts=True)
    assert_preserved_profile(raw, result)
    assert result.path == tuple(raw)
    assert horizontal_length(result.path) == 0.0


def test_xy_spline_inserts_every_raw_height_breakpoint_and_never_overshoots() -> None:
    raw = [(2.0, 10.0, 5.0), (6.0, 6.0, 8.0), (14.0, 6.0, 4.0), (18.0, 10.0, 5.0)]
    scene = scene_for_path(raw, (AABB("barrier", (8.0, 8.0, 0.0), (12.0, 12.0, 9.0)),))
    result = smooth_path(scene, raw, sample_spacing=0.4, preserve_altitude=True)
    assert result.method == "bspline"
    assert len(result.path) > len(raw)
    assert_preserved_profile(raw, result)
    assert path_is_free(scene, result.path)
    assert result.to_dict()["altitude_profile_max_error"] == 0.0


def test_preserved_low_altitude_forces_xy_detour_instead_of_flying_over_building() -> None:
    raw = [(2.0, 10.0, 10.0), (6.0, 4.0, 2.0), (14.0, 4.0, 2.0), (18.0, 10.0, 10.0)]
    scene = scene_for_path(raw, (AABB("low-building", (8.0, 8.0, 0.0), (12.0, 12.0, 6.0)),))
    assert path_is_free(scene, raw)
    assert smooth_path(scene, raw).path == (raw[0], raw[-1])
    result = smooth_path(scene, raw, preserve_altitude=True, optimize_shortcuts=True)
    assert_preserved_profile(raw, result)
    assert path_is_free(scene, result.path)
    assert any(point[1] < 8.0 and point[2] == 2.0 for point in result.path)
    assert horizontal_length(result.path) > 16.0


def test_unsafe_xy_spline_falls_back_without_changing_height(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    raw = [(2.0, 10.0, 5.0), (6.0, 6.0, 8.0), (14.0, 6.0, 4.0), (18.0, 10.0, 5.0)]
    scene = scene_for_path(raw, (AABB("barrier", (8.0, 8.0, 0.0), (12.0, 12.0, 9.0)),))
    original_audit = smoothing.path_is_free
    # Force every dense spline candidate to fail certification, but allow exact chord auditing.
    monkeypatch.setattr(
        smoothing,
        "path_is_free",
        lambda candidate_scene, candidate: (
            len(candidate) <= len(raw) and original_audit(candidate_scene, candidate)
        ),
    )
    result = smooth_path(scene, raw, preserve_altitude=True)
    assert result.method == "shortcut-fallback"
    assert_preserved_profile(raw, result)
    assert original_audit(scene, result.path)


def test_replanning_shortcut_keeps_each_plans_altitude_breakpoints(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    raw = [(2.0, 2.0, 4.0), (2.0, 14.0, 12.0), (14.0, 14.0, 6.0), (18.0, 2.0, 4.0)]
    scenario = DynamicScenario("profile-replanning", "Profile replanning", scene_for_path(raw))

    class FixedPlanner:
        def plan(self, snapshot: Scene, seed: int = 0) -> PlanningResult:
            assert snapshot.start == raw[0]
            return PlanningResult("repeated-astar-3d", True, tuple(raw), 0.0)

    monkeypatch.setattr(replanning, "_planner", lambda *_: FixedPlanner())
    run = simulate_replanning(
        scenario,
        "repeated-astar-3d",
        shortcut_paths=True,
        preserve_altitude=True,
        replan_interval=100.0,
        time_step=0.25,
        cruise_speed=4.0,
        max_time=100.0,
    )
    assert run.metrics.success
    assert run.metrics.collision_count == 0
    assert run.parameters["preserve_altitude"] == 1
    plan = run.frames[0].planned_path
    assert [point[2] for point in plan] == [point[2] for point in raw]
    assert horizontal_length(plan) == pytest.approx(16.0)
    assert _reactive_timed_path(run).is_safe(scenario)


@pytest.mark.parametrize("algorithm", REPLANNING_ALGORITHMS)
@pytest.mark.parametrize("scenario_id", ["crossing-traffic", "pop-up-nfz"])
def test_xy_only_replanning_still_checks_unknown_future_collisions(
    algorithm: str, scenario_id: str
) -> None:
    scenario = load_builtin_dynamic_scenario(scenario_id)
    run = simulate_replanning(
        scenario, algorithm, shortcut_paths=True, preserve_altitude=True, max_time=100.0
    )
    assert run.metrics.success
    assert run.metrics.collision_count == 0
    assert _reactive_timed_path(run).is_safe(scenario)


def test_invalid_raw_path_remains_fail_closed_with_altitude_constraint() -> None:
    raw = [(2.0, 10.0, 5.0), (18.0, 10.0, 5.0)]
    scene = scene_for_path(raw, (AABB("barrier", (8.0, 8.0, 0.0), (12.0, 12.0, 9.0)),))
    result = smooth_path(scene, raw, preserve_altitude=True)
    assert result.method == "not-run"
    assert result.path == ()
    assert not result.collision_free
