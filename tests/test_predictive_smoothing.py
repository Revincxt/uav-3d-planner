from __future__ import annotations

import math
from itertools import pairwise

import pytest

from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.dynamic_collision import minimum_dynamic_separation
from uav3d.kinematics import diagnose_timed_path_kinematics
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.predictive_smoothing import smooth_predictive_timed_path
from uav3d.scene import AABB, Bounds3D, Scene


def _scenario(
    *,
    start: tuple[float, float, float] = (4.0, 4.0, 5.0),
    goal: tuple[float, float, float] = (12.0, 12.0, 5.0),
    buildings: tuple[AABB, ...] = (),
    moving_spheres: tuple[MovingSphere, ...] = (),
    temporary_cylinders: tuple[TemporaryCylinder, ...] = (),
) -> DynamicScenario:
    scene = Scene(
        "smoothing-scene",
        "Smoothing scene",
        Bounds3D((0.0, 0.0, 0.0), (24.0, 24.0, 12.0)),
        start,
        goal,
        buildings,
        drone_radius=0.0,
        safety_margin=0.0,
    )
    return DynamicScenario(
        "smoothing-dynamic",
        "Smoothing dynamic",
        scene,
        temporary_cylinders=temporary_cylinders,
        moving_spheres=moving_spheres,
    )


def _right_angle_path() -> TimedPath:
    return TimedPath(
        (
            TimedWaypoint(0.0, (4.0, 4.0, 5.0), "start"),
            TimedWaypoint(2.0, (12.0, 4.0, 5.0), "move"),
            TimedWaypoint(4.0, (12.0, 12.0, 5.0), "move"),
        )
    )


def test_open_right_angle_is_rounded_and_certified() -> None:
    scenario = _scenario()
    raw = _right_angle_path()

    result = smooth_predictive_timed_path(
        scenario,
        raw,
        requested_radius_m=3.0,
        sample_spacing_m=0.25,
        max_speed_mps=4.0,
    )

    assert result.applied
    assert result.certified
    assert result.method == "sampled-circular-fillet"
    assert result.rounded_corners == 1
    assert result.applied_radius_m == pytest.approx(3.0)
    assert result.raw_waypoint_count == 3
    assert result.output_waypoint_count > result.raw_waypoint_count
    assert result.max_turn_before_deg == pytest.approx(90.0)
    assert result.max_turn_after_deg < result.max_turn_before_deg
    assert result.timed_path.start == raw.start
    assert result.timed_path.goal == raw.goal
    assert result.timed_path.departure_time_s == raw.departure_time_s
    assert result.timed_path.arrival_time_s == raw.arrival_time_s
    assert max(result.timed_path.segment_speeds()) <= 4.0 + 1e-9
    assert result.timed_path.is_safe(scenario)
    assert all(
        current.time_s > previous.time_s
        for previous, current in pairwise(result.timed_path.waypoints)
    )
    assert result.to_dict()["certified"] is True


def test_narrow_inner_obstacle_causes_explicit_raw_fallback() -> None:
    obstacle = AABB("inner-block", (8.0, 4.2, 0.0), (11.8, 11.8, 10.0))
    scenario = _scenario(buildings=(obstacle,))
    raw = _right_angle_path()
    assert raw.is_safe(scenario)

    result = smooth_predictive_timed_path(
        scenario,
        raw,
        requested_radius_m=4.0,
        sample_spacing_m=0.1,
        max_speed_mps=4.0,
    )

    assert not result.applied
    assert result.certified
    assert result.method == "raw-fallback"
    assert result.timed_path == raw
    assert result.applied_radius_m is None
    assert result.rounded_corners == 0
    assert result.output_waypoint_count == result.raw_waypoint_count == 3
    assert result.timed_path.is_safe(scenario)


def test_wait_segment_is_a_hard_boundary_with_exact_position_and_time() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (2.0, 2.0, 5.0), "start"),
            TimedWaypoint(2.0, (10.0, 2.0, 5.0), "move"),
            TimedWaypoint(4.0, (10.0, 10.0, 5.0), "move"),
            TimedWaypoint(6.0, (10.0, 10.0, 5.0), "wait"),
            TimedWaypoint(8.0, (18.0, 10.0, 5.0), "move"),
            TimedWaypoint(10.0, (18.0, 18.0, 5.0), "move"),
        )
    )
    scenario = _scenario(start=raw.start, goal=raw.goal)

    result = smooth_predictive_timed_path(
        scenario,
        raw,
        requested_radius_m=3.0,
        sample_spacing_m=0.25,
        max_speed_mps=4.0,
    )

    assert result.applied
    assert result.rounded_corners == 2
    waits = tuple(item for item in result.timed_path.waypoints if item.action == "wait")
    assert waits == (raw.waypoints[3],)
    assert result.timed_path.wait_time_s == pytest.approx(2.0)
    assert any(
        item.time_s == 4.0 and item.position == (10.0, 10.0, 5.0)
        for item in result.timed_path.waypoints
    )
    assert not any(4.0 < item.time_s < 6.0 for item in result.timed_path.waypoints)
    assert result.timed_path.is_safe(scenario)


def test_moving_sphere_conflict_rejects_every_rounded_candidate() -> None:
    # The stationary sphere is 0.18 m from each leg, so the raw right-angle path clears its
    # 0.17 m radius. Every attempted inner fillet crosses it around t=2 s.
    position = (11.82, 4.18, 5.0)
    sphere = MovingSphere("corner-traffic", 0.17, ((0.0, position), (4.0, position)))
    scenario = _scenario(moving_spheres=(sphere,))
    raw = _right_angle_path()
    assert raw.is_safe(scenario)

    result = smooth_predictive_timed_path(
        scenario,
        raw,
        requested_radius_m=1.0,
        sample_spacing_m=0.05,
        max_speed_mps=4.0,
    )

    assert result.method == "raw-fallback"
    assert not result.applied
    assert result.certified
    assert result.timed_path == raw
    assert result.timed_path.is_safe(scenario)


def test_smoothing_is_deterministic() -> None:
    scenario = _scenario()
    raw = _right_angle_path()
    parameters = {
        "requested_radius_m": 2.5,
        "sample_spacing_m": 0.2,
        "max_speed_mps": 4.0,
    }

    first = smooth_predictive_timed_path(scenario, raw, **parameters)
    second = smooth_predictive_timed_path(scenario, raw, **parameters)

    assert first == second
    assert first.to_dict() == second.to_dict()


def test_u_turn_is_retained_but_reported_as_a_discrete_reversal() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (4.0, 4.0, 5.0), "start"),
            TimedWaypoint(2.0, (12.0, 4.0, 5.0), "move"),
            TimedWaypoint(4.0, (6.0, 4.0, 5.0), "move"),
        )
    )
    scenario = _scenario(start=raw.start, goal=raw.goal)

    result = smooth_predictive_timed_path(
        scenario,
        raw,
        requested_radius_m=3.0,
        sample_spacing_m=0.25,
        max_speed_mps=4.0,
    )

    assert result.method == "raw-no-roundable-corners"
    assert result.collision_certified
    assert result.timed_path == raw
    assert result.raw_kinematics.reversal_count == 1
    assert result.output_kinematics.reversal_count == 1
    assert result.output_kinematics.max_discrete_velocity_change_mps == pytest.approx(7.0)
    assert result.output_kinematics.max_discrete_acceleration_proxy_mps2 == pytest.approx(3.5)
    assert result.output_kinematics.max_abs_climb_rate_mps == 0.0
    payload = result.to_dict()
    assert payload["collision_certified"] is True
    kinematics = payload["kinematic_diagnostics"]
    assert isinstance(kinematics, dict)
    assert kinematics["continuous_dynamics_certified"] is False


def test_discrete_kinematic_diagnostics_report_climb_and_velocity_change() -> None:
    path = TimedPath(
        (
            TimedWaypoint(0.0, (4.0, 4.0, 4.0), "start"),
            TimedWaypoint(2.0, (8.0, 4.0, 8.0), "move"),
            TimedWaypoint(4.0, (12.0, 4.0, 8.0), "move"),
        )
    )

    diagnostics = diagnose_timed_path_kinematics(path)

    assert diagnostics.reversal_count == 0
    assert diagnostics.max_abs_climb_rate_mps == pytest.approx(2.0)
    assert diagnostics.max_discrete_velocity_change_mps == pytest.approx(2.0)
    assert diagnostics.max_discrete_acceleration_proxy_mps2 == pytest.approx(1.0)
    assert diagnostics.to_dict()["continuous_dynamics_certified"] is False


def test_dynamic_separation_reports_exact_moving_sphere_witness() -> None:
    position = (8.0, 8.0, 5.0)
    sphere = MovingSphere("parallel-traffic", 1.0, ((0.0, position), (2.0, position)))
    path = TimedPath(
        (
            TimedWaypoint(0.0, (4.0, 4.0, 5.0), "start"),
            TimedWaypoint(2.0, (12.0, 4.0, 5.0), "move"),
        )
    )
    scenario = _scenario(start=path.start, goal=path.goal, moving_spheres=(sphere,))

    witness = minimum_dynamic_separation(scenario, path.timed_points)

    assert witness is not None
    assert witness.exact
    assert witness.obstacle_kind == "moving-sphere"
    assert witness.time_s == pytest.approx(1.0)
    assert witness.vehicle_position == pytest.approx((8.0, 4.0, 5.0))
    assert witness.separation_m == pytest.approx(3.0)


def test_dynamic_separation_labels_temporary_cylinder_search_as_approximate() -> None:
    zone = TemporaryCylinder("temporary", (8.0, 8.0), 1.0, 0.0, 10.0, 0.0, 2.0)
    path = TimedPath(
        (
            TimedWaypoint(0.0, (4.0, 4.0, 5.0), "start"),
            TimedWaypoint(2.0, (12.0, 4.0, 5.0), "move"),
        )
    )
    scenario = _scenario(start=path.start, goal=path.goal, temporary_cylinders=(zone,))

    witness = minimum_dynamic_separation(scenario, path.timed_points)

    assert witness is not None
    assert not witness.exact
    assert witness.obstacle_kind == "temporary-cylinder"
    assert witness.time_s == pytest.approx(1.0, abs=1e-6)
    assert witness.separation_m == pytest.approx(3.0, abs=1e-9)


@pytest.mark.parametrize(
    ("parameter", "value"),
    (
        ("requested_radius_m", 0.0),
        ("requested_radius_m", math.nan),
        ("sample_spacing_m", 0.0),
        ("sample_spacing_m", math.inf),
        ("max_speed_mps", -1.0),
    ),
)
def test_invalid_parameters_are_rejected(parameter: str, value: float) -> None:
    scenario = _scenario()
    raw = _right_angle_path()
    parameters = {
        "requested_radius_m": 2.0,
        "sample_spacing_m": 0.25,
        "max_speed_mps": 4.0,
    }
    parameters[parameter] = value

    with pytest.raises(ValueError, match="finite and positive"):
        smooth_predictive_timed_path(scenario, raw, **parameters)
