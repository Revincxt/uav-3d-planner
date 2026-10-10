"""XYZ spline geometry, hard mission anchors, and dense space-time certification."""

from __future__ import annotations

import math
from dataclasses import replace
from itertools import pairwise

import pytest

from uav3d.collision import segment_is_free
from uav3d.curve_timing import smooth_timed_curves
from uav3d.dynamic import DynamicScenario, MovingSphere
from uav3d.horizontal_curves import _span, smooth_horizontal_curves, smooth_spatial_curves
from uav3d.kinematics import DiscreteExecutionEnvelope
from uav3d.mission_refinement import refine_mission_trajectory
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.predictive_smoothing import shortcut_timed_path
from uav3d.scene import AABB, Bounds3D, Scene


def max_turn(points):
    angles = []
    for a, b, c in zip(points, points[1:], points[2:], strict=False):
        u, v = tuple(b[i] - a[i] for i in range(3)), tuple(c[i] - b[i] for i in range(3))
        denominator = math.hypot(*u) * math.hypot(*v)
        if denominator > 1e-10:
            cosine = sum(x * y for x, y in zip(u, v, strict=True)) / denominator
            angles.append(math.degrees(math.acos(max(-1, min(1, cosine)))))
    return max(angles, default=0)


def make_scenario(points, *, buildings=(), tasks=()):
    return DynamicScenario(
        "spatial",
        "Spatial",
        Scene(
            "spatial",
            "Spatial",
            Bounds3D((-200, -200, 0), (400, 400, 300)),
            points[0],
            points[-1],
            buildings=buildings,
            drone_radius=0,
            safety_margin=0,
            metadata={"missionTaskPoints": list(tasks)},
        ),
    )


@pytest.mark.parametrize(
    "points",
    [
        ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0), (100.0, 0.0, 100.0)),
        ((0.0, 0.0, 20.0), (0.0, 0.0, 100.0), (100.0, 0.0, 100.0)),
        ((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (100.0, 0.0, 20.0)),
        ((0.0, 0.0, 20.0), (100.0, 0.0, 70.0), (200.0, 0.0, 20.0)),
        ((0.0, 0.0, 20.0), (100.0, 0.0, 70.0), (100.0, 100.0, 100.0)),
    ],
)
@pytest.mark.parametrize("protected", [False, True])
def test_xyz_rounds_climb_descent_and_heading_together(points, protected):
    result = smooth_spatial_curves(
        points,
        (0.0, 10.0, 20.0),
        lambda *_: True,
        round_reversals=True,
        protected=frozenset({10.0}) if protected else frozenset(),
    )
    assert result.rounded_corners == 1
    assert max_turn(result.points) < 4
    assert result.points[0] == points[0] and result.points[-1] == points[-1]
    assert 0 < result.max_altitude_deviation_m <= 12
    assert min(p[2] for p in result.points) >= min(p[2] for p in points) - 1e-8
    assert max(p[2] for p in result.points) <= max(p[2] for p in points) + 1e-8
    if protected:
        assert result.points[result.parameters.index(10.0)] == points[1]


def test_all_three_axes_match_tangents_and_zero_second_derivatives():
    initial, final = (4.0, 1.0, 2.0), (1.0, 4.0, -1.0)
    span = _span((0.0, 0.0, 20.0), (30.0, 20.0, 30.0), 2.0, 12.0, initial, final, 2, math.pi / 2)
    controls = span.controls
    for axis in range(3):
        assert 5 * (controls[1][axis] - controls[0][axis]) / 10 == pytest.approx(initial[axis])
        assert 5 * (controls[5][axis] - controls[4][axis]) / 10 == pytest.approx(final[axis])
        assert controls[0][axis] - 2 * controls[1][axis] + controls[2][axis] == pytest.approx(0)
        assert controls[3][axis] - 2 * controls[4][axis] + controls[5][axis] == pytest.approx(0)


def test_constant_height_and_straight_vertical_flight_remain_exact():
    flat = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0), (100.0, 100.0, 20.0))
    result = smooth_spatial_curves(flat, (0.0, 10.0, 20.0), lambda *_: True)
    assert all(p[2] == 20 for p in result.points)
    assert result.max_altitude_deviation_m == 0
    vertical = ((0.0, 0.0, 20.0), (0.0, 0.0, 40.0), (0.0, 0.0, 100.0))
    assert smooth_spatial_curves(vertical, (0.0, 10.0, 20.0), lambda *_: True).points == vertical


def test_height_budget_shrinks_a_vertical_bend_without_switching_back_to_xy():
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0), (100.0, 0.0, 100.0))
    result = smooth_spatial_curves(
        points,
        (0.0, 10.0, 20.0),
        lambda *_: True,
        max_altitude_deviation_m=0.5,
    )
    assert result.rounded_corners == 1
    assert 0 < result.max_altitude_deviation_m <= 0.5
    assert max_turn(result.points) < 4
    xy = smooth_horizontal_curves(points, (0.0, 10.0, 20.0), lambda *_: True)
    assert xy.points == points


def test_rooftop_collision_blocks_only_its_own_spatial_curve():
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0), (100.0, 0.0, 100.0), (200.0, 0.0, 100.0))
    building = AABB("low-roof", (60.0, -5.0, 23.0), (99.999, 5.0, 70.0))
    scenario = make_scenario(points, buildings=(building,))
    result = smooth_spatial_curves(
        points,
        (0.0, 10.0, 20.0, 30.0),
        lambda a, b, *_: segment_is_free(scenario.static_scene, a, b),
    )
    assert result.rounded_corners >= 1
    assert all(segment_is_free(scenario.static_scene, a, b) for a, b in pairwise(result.points))
    assert result.points[-1] == points[-1]


def test_xyz_boundary_heading_includes_the_vertical_tangent():
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 100.0))
    result = smooth_spatial_curves(
        points, (0.0, 10.0), lambda *_: True, start_direction=(1.0, 0.0, 0.0)
    )
    first = result.points[1]
    assert abs(first[2] - 20) < abs(first[0]) * 0.01
    assert max_turn(result.points) < 4


def test_online_boundary_keeps_climb_direction_when_next_plan_is_level():
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0))
    locked = smooth_spatial_curves(
        points, (0.0, 10.0), lambda *_: True, start_direction=(1.0, 0.0, 0.2)
    )
    assert locked.points == points
    online = smooth_spatial_curves(
        points,
        (0.0, 10.0),
        lambda *_: True,
        start_direction=(1.0, 0.0, 0.2),
        boundary_height_excursion=True,
    )
    first = online.points[1]
    assert (first[2] - 20) / first[0] == pytest.approx(0.2, abs=0.003)
    assert 0 < online.max_altitude_deviation_m <= 12
    assert online.points[0] == points[0] and online.points[-1] == points[-1]
    assert max_turn(online.points) < 4


def test_protected_height_peak_and_xy_reversal_have_a_nonzero_spatial_tangent():
    points = ((0.0, 0.0, 60.0), (100.0, 0.0, 70.0), (0.0, 0.0, 20.0))
    result = smooth_spatial_curves(
        points,
        (0.0, 10.0, 20.0),
        lambda *_: True,
        protected=frozenset({10.0}),
        round_reversals=True,
    )
    assert result.rounded_corners == 1
    assert result.points[result.parameters.index(10.0)] == points[1]
    assert max_turn(result.points) < 4


def test_timed_spatial_curve_keeps_task_position_and_complete_real_hold():
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 60.0), (100.0, 100.0, 20.0))
    raw = TimedPath(
        (
            TimedWaypoint(0.0, points[0], "start"),
            TimedWaypoint(20.0, points[1], "move"),
            TimedWaypoint(24.0, points[1], "wait"),
            TimedWaypoint(44.0, points[2], "move"),
        )
    )
    scenario = make_scenario(points, tasks=({"position": points[1]},))
    result, corners, _ = smooth_timed_curves(scenario, raw, 60, 2, round_reversals=True)
    assert result.is_safe(scenario) and corners == 1
    holds = [(a, b) for a, b in pairwise(result.waypoints) if b.action == "wait"]
    assert len(holds) == 1
    assert holds[0][0].time_s == 20 and holds[0][1].time_s == 24
    assert holds[0][0].position == holds[0][1].position == points[1]
    assert result.wait_time_s == 4
    assert max_turn(result.positions) < 4
    assert any(p.position[2] != 20 for p in result.waypoints)


def test_timed_spatial_curve_audits_moving_obstacles_not_only_sample_points():
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0), (100.0, 0.0, 100.0))
    raw = TimedPath(
        tuple(
            TimedWaypoint(t, p, "start" if i == 0 else "move")
            for i, (t, p) in enumerate(zip((0.0, 20.0, 60.0), points, strict=True))
        )
    )
    scenario = make_scenario(points)
    curve, _, _ = smooth_timed_curves(scenario, raw, 60, 2)
    sample = next(w for w in curve.waypoints if 5 < w.time_s < 20 and w.position[2] > 22)
    traffic = MovingSphere(
        "crossing",
        0.5,
        (
            (0.0, (sample.position[0], -100.0, sample.position[2])),
            (sample.time_s, sample.position),
            (80.0, (sample.position[0], 100.0, sample.position[2])),
        ),
    )
    scenario = replace(scenario, moving_spheres=(traffic,))
    assert raw.is_safe(scenario) and not curve.is_safe(scenario)
    audited, _, _ = smooth_timed_curves(scenario, raw, 60, 2)
    assert audited.is_safe(scenario)


def test_mission_execution_has_xyz_metadata_and_independent_qualification():
    points = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0), (100.0, 0.0, 100.0), (200.0, 0.0, 100.0))
    raw = TimedPath(
        tuple(
            TimedWaypoint(t, p, "start" if i == 0 else "move")
            for i, (t, p) in enumerate(zip((0.0, 20.0, 60.0, 80.0), points, strict=True))
        )
    )
    scenario = make_scenario(points)
    result = refine_mission_trajectory(
        scenario,
        raw,
        DiscreteExecutionEnvelope(
            max_speed_mps=8.0,
            max_abs_climb_rate_mps=3.0,
            max_execution_time_s=180.0,
        ),
    )
    assert result.execution_qualified and result.execution_candidate.is_safe(scenario)
    assert max_turn(result.timed_path.positions) < 4
    assert result.to_dict()["optimization_axes"] == ["x", "y", "z"]
    assert result.to_dict()["altitude_policy"] == "bounded-spatial-spline-v1"
    assert result.raw_waypoint_count == len(raw.waypoints)
    assert result.execution_candidate.wait_time_s == 0


def test_spatial_shortcut_retains_mandatory_gate_and_checks_removed_height_knots():
    points = ((0.0, 0.0, 20.0), (50.0, 20.0, 30.0), (100.0, 0.0, 20.0))
    raw = TimedPath(
        tuple(
            TimedWaypoint(i * 20.0, p, "start" if i == 0 else "move") for i, p in enumerate(points)
        )
    )
    plain = make_scenario(points)
    bounded = shortcut_timed_path(plain, raw, max_altitude_deviation_m=6)
    assert bounded == raw
    direct = shortcut_timed_path(plain, raw, max_altitude_deviation_m=12)
    assert len(direct.waypoints) == 2 and direct.is_safe(plain)
    tasks = (
        {"id": "gate", "position": points[1], "visitMode": "fly-through", "serviceDurationS": 0},
    )
    scenario = make_scenario(points, tasks=tasks)
    assert shortcut_timed_path(scenario, raw, max_altitude_deviation_m=12) == raw


@pytest.mark.parametrize("budget", [0, -1, math.nan, math.inf])
def test_rejects_invalid_altitude_budget(budget):
    with pytest.raises(ValueError, match="altitude deviation"):
        smooth_spatial_curves(
            ((0.0, 0.0, 20.0), (100.0, 0.0, 100.0)),
            (0.0, 10.0),
            lambda *_: True,
            max_altitude_deviation_m=budget,
        )
