from __future__ import annotations

from itertools import pairwise

import pytest

from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.geometry import lerp
from uav3d.kinematics import DiscreteExecutionEnvelope
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.predictive_smoothing import (
    _horizontal_geometry_to_timed_block,
    shortcut_timed_path,
    smooth_predictive_timed_path,
)
from uav3d.scene import AABB, Bounds3D, Scene


def _path(rows: list[tuple[float, tuple[float, float, float]]]) -> TimedPath:
    return TimedPath(
        tuple(
            TimedWaypoint(
                time_s,
                position,
                "start" if index == 0 else "wait" if position == rows[index - 1][1] else "move",
            )
            for index, (time_s, position) in enumerate(rows)
        )
    )


def _scenario(
    path: TimedPath,
    *,
    buildings: tuple[AABB, ...] = (),
    zones: tuple[TemporaryCylinder, ...] = (),
    traffic: tuple[MovingSphere, ...] = (),
) -> DynamicScenario:
    return DynamicScenario(
        "altitude-scenario",
        "Altitude scenario",
        Scene(
            "altitude-scene",
            "Altitude scene",
            Bounds3D((0.0, 0.0, 0.0), (200.0, 200.0, 200.0)),
            path.start,
            path.goal,
            buildings,
            drone_radius=0.0,
            safety_margin=0.0,
        ),
        temporary_cylinders=zones,
        moving_spheres=traffic,
    )


def _position_at(path: TimedPath, time_s: float) -> tuple[float, float, float]:
    if time_s == path.arrival_time_s:
        return path.goal
    for previous, current in pairwise(path.waypoints):
        if previous.time_s <= time_s < current.time_s:
            return lerp(
                previous.position,
                current.position,
                (time_s - previous.time_s) / (current.time_s - previous.time_s),
            )
    raise AssertionError("time must lie in path range")


def _assert_exact_height_profile(raw: TimedPath, output: TimedPath) -> None:
    times = {item.time_s for item in raw.waypoints} | {item.time_s for item in output.waypoints}
    # All breakpoints plus each open linear interval establish equality of the complete z(t).
    ordered = sorted(times)
    samples = ordered + [0.5 * (left + right) for left, right in pairwise(ordered)]
    for time_s in samples:
        assert _position_at(output, time_s)[2] == pytest.approx(
            _position_at(raw, time_s)[2], abs=1e-12
        )
    by_time = {item.time_s: item.position[2] for item in output.waypoints}
    for item in raw.waypoints:
        assert by_time[item.time_s] == item.position[2]


@pytest.mark.parametrize("shortcut", [False, True])
def test_horizontal_smoothing_keeps_height_peaks_troughs_and_unequal_endpoints(
    shortcut: bool,
) -> None:
    raw = _path(
        [
            (0.0, (10.0, 10.0, 10.0)),
            (4.0, (50.0, 10.0, 30.0)),
            (8.0, (50.0, 50.0, 10.0)),
            (12.0, (90.0, 50.0, 25.0)),
        ]
    )
    scenario = _scenario(raw)
    result = smooth_predictive_timed_path(
        scenario,
        raw,
        requested_radius_m=10.0,
        sample_spacing_m=2.0,
        max_speed_mps=15.0,
        shortcut=shortcut,
        preserve_altitude=True,
    )
    assert result.applied
    assert result.geometry_candidate.is_safe(scenario)
    assert result.geometry_candidate.start == raw.start
    assert result.geometry_candidate.goal == raw.goal
    assert max(result.geometry_candidate.segment_speeds()) <= max(raw.segment_speeds()) + 1e-9
    _assert_exact_height_profile(raw, result.geometry_candidate)
    assert _position_at(result.geometry_candidate, 4.0)[:2] != raw.waypoints[1].position[:2]
    assert result.execution_candidate is not None
    assert result.execution_candidate.positions == result.geometry_candidate.positions
    assert result.to_dict()["altitude_policy"] == "preserve-raw-z-time-profile"
    assert result.to_dict()["optimization_axes"] == ["x", "y"]


@pytest.mark.parametrize("shortcut", [False, True])
def test_original_vertical_intervals_have_no_horizontal_drift_or_added_speed(
    shortcut: bool,
) -> None:
    raw = _path(
        [
            (0.0, (10.0, 10.0, 5.0)),
            (1.0, (10.0, 10.0, 20.0)),
            (5.0, (50.0, 10.0, 20.0)),
            (7.0, (50.0, 30.0, 20.0)),
            (8.0, (50.0, 30.0, 35.0)),
        ]
    )
    result = smooth_predictive_timed_path(
        _scenario(raw),
        raw,
        requested_radius_m=6.0,
        sample_spacing_m=1.0,
        max_speed_mps=15.0,
        execution_envelope=DiscreteExecutionEnvelope(max_speed_mps=15.0),
        shortcut=shortcut,
        preserve_altitude=True,
    )
    candidate = result.geometry_candidate
    _assert_exact_height_profile(raw, candidate)
    assert _position_at(candidate, 0.5)[:2] == raw.start[:2]
    assert _position_at(candidate, 7.5)[:2] == raw.goal[:2]
    assert max(candidate.segment_speeds()) <= 15.0 + 1e-9


def test_pure_vertical_path_and_waits_are_not_rewritten() -> None:
    raw = _path(
        [
            (0.0, (10.0, 10.0, 5.0)),
            (5.0, (10.0, 10.0, 20.0)),
            (8.0, (10.0, 10.0, 20.0)),
            (12.0, (10.0, 10.0, 10.0)),
        ]
    )
    result = smooth_predictive_timed_path(
        _scenario(raw), raw, preserve_altitude=True, shortcut=True
    )
    assert result.geometry_candidate == raw
    assert result.geometry_candidate.wait_time_s == 3.0


def test_wait_absolute_boundaries_are_preserved_between_horizontal_blocks() -> None:
    raw = _path(
        [
            (0.0, (10.0, 10.0, 10.0)),
            (4.0, (50.0, 10.0, 15.0)),
            (8.0, (50.0, 50.0, 10.0)),
            (11.0, (50.0, 50.0, 10.0)),
            (15.0, (90.0, 50.0, 20.0)),
            (19.0, (90.0, 90.0, 25.0)),
        ]
    )
    result = smooth_predictive_timed_path(
        _scenario(raw), raw, max_speed_mps=15.0, shortcut=True, preserve_altitude=True
    )
    candidate = result.geometry_candidate
    _assert_exact_height_profile(raw, candidate)
    wait = next(item for item in candidate.waypoints if item.time_s == 11.0)
    assert wait.action == "wait"
    assert wait.position == raw.waypoints[2].position
    assert _position_at(candidate, 9.5) == wait.position
    assert candidate.wait_time_s == raw.wait_time_s == 3.0


def test_shortcut_checks_low_intermediate_altitude_not_just_high_endpoints() -> None:
    raw = _path(
        [
            (0.0, (10.0, 10.0, 50.0)),
            (4.0, (10.0, 50.0, 5.0)),
            (12.0, (90.0, 50.0, 50.0)),
        ]
    )
    building = AABB("low-gate", (30.0, 15.0, 0.0), (50.0, 35.0, 35.0))
    scenario = _scenario(raw, buildings=(building,))
    assert raw.is_safe(scenario)
    legacy_chord = _path([(0.0, raw.start), (12.0, raw.goal)])
    assert legacy_chord.is_safe(scenario)  # Both endpoints stay above the roof: not our policy.
    candidate = shortcut_timed_path(scenario, raw, preserve_altitude=True)
    assert candidate == raw
    _assert_exact_height_profile(raw, candidate)


def test_horizontal_shortcut_checks_the_actual_traffic_schedule() -> None:
    raw = _path([(0.0, (10.0, 10.0, 10.0)), (4.0, (50.0, 10.0, 30.0)), (8.0, (50.0, 50.0, 10.0))])
    traffic = MovingSphere(
        "chord-blocker", 1.0, ((0.0, (30.0, 30.0, 30.0)), (8.0, (30.0, 30.0, 30.0)))
    )
    scenario = _scenario(raw, traffic=(traffic,))
    assert raw.is_safe(scenario)
    assert shortcut_timed_path(scenario, raw, preserve_altitude=True) == raw


def test_collision_unsafe_planar_fillets_fall_back_without_altering_height() -> None:
    raw = _path([(0.0, (4.0, 4.0, 5.0)), (2.0, (12.0, 4.0, 10.0)), (4.0, (12.0, 12.0, 5.0))])
    obstacle = AABB("inner-block", (8.0, 4.2, 0.0), (11.8, 11.8, 100.0))
    result = smooth_predictive_timed_path(
        _scenario(raw, buildings=(obstacle,)),
        raw,
        requested_radius_m=4.0,
        sample_spacing_m=0.1,
        max_speed_mps=8.0,
        preserve_altitude=True,
    )
    assert result.geometry_candidate == raw
    assert not result.applied
    assert result.method == "raw-fallback"


def test_dynamic_collision_after_height_preserving_retiming_is_not_exposed() -> None:
    raw = _path([(0.0, (10.0, 10.0, 10.0)), (1.0, (18.0, 10.0, 12.0))])
    zone = TemporaryCylinder("late-gate", (14.0, 10.0), 0.5, 0.0, 30.0, 1.5, 3.0)
    scenario = _scenario(raw, zones=(zone,))
    assert raw.is_safe(scenario)
    result = smooth_predictive_timed_path(
        scenario,
        raw,
        max_speed_mps=10.0,
        execution_envelope=DiscreteExecutionEnvelope(max_discrete_acceleration_proxy_mps2=1.0),
        preserve_altitude=True,
    )
    assert result.geometry_candidate == raw
    assert result.execution_candidate is None
    assert result.execution_status == "dynamic-collision-after-retiming"


def test_retiming_collision_tries_another_horizontal_radius_with_the_same_height_profile() -> None:
    raw = _path([(0.0, (4.0, 4.0, 5.0)), (2.0, (12.0, 4.0, 7.0)), (4.0, (12.0, 12.0, 6.0))])
    options = {
        "requested_radius_m": 3.0,
        "sample_spacing_m": 0.25,
        "max_speed_mps": 8.0,
        "preserve_altitude": True,
    }
    first = smooth_predictive_timed_path(_scenario(raw), raw, **options)
    assert first.execution_candidate is not None
    conflict_time = 4.2  # Only the retimed execution remains in flight at this time.
    conflict_position = _position_at(first.execution_candidate, conflict_time)
    zone = TemporaryCylinder(
        "execution-only-gate", conflict_position[:2], 0.08, 0.0, 30.0, 4.19, 4.21
    )
    scenario = _scenario(raw, zones=(zone,))
    assert first.geometry_candidate.is_safe(scenario)
    assert not first.execution_candidate.is_safe(scenario)
    result = smooth_predictive_timed_path(scenario, raw, **options)
    assert result.applied_radius_m == pytest.approx(2.25)
    assert result.execution_candidate is not None
    assert result.execution_candidate.is_safe(scenario)
    _assert_exact_height_profile(raw, result.geometry_candidate)
    assert result.execution_candidate.positions == result.geometry_candidate.positions


def test_legacy_default_does_not_advertise_height_preservation() -> None:
    raw = _path([(0.0, (10.0, 10.0, 5.0)), (2.0, (20.0, 10.0, 5.0))])
    result = smooth_predictive_timed_path(_scenario(raw), raw)
    assert "altitude_policy" not in result.to_dict()
    assert "optimization_axes" not in result.to_dict()


def test_inverse_horizontal_progress_does_not_invent_an_almost_zero_duration_wait() -> None:
    raw = _path(
        [
            (0.0, (12.657654590558113, 0.8939165831421103, 1.0)),
            (1.0, (6.559139244108101, 15.160658643100872, 2.0)),
            (2.0, (0.7960790905159087, 5.965129520599454, 3.0)),
            (3.0, (19.496533133385697, 16.3482444180965, 4.0)),
        ]
    )
    output = TimedPath(
        tuple(_horizontal_geometry_to_timed_block(list(raw.positions), raw.waypoints))
    )
    assert [item.time_s for item in output.waypoints] == [0.0, 1.0, 2.0, 3.0]
    assert all(item.action == "move" for item in output.waypoints[1:])
    _assert_exact_height_profile(raw, output)


def test_public_planar_fillet_has_no_roundoff_wait_at_the_original_height_peak() -> None:
    raw = _path([(0.0, (10.0, 10.0, 5.0)), (4.0, (50.0, 10.0, 10.0)), (8.0, (50.0, 50.0, 5.0))])
    result = smooth_predictive_timed_path(
        _scenario(raw),
        raw,
        requested_radius_m=1.0,
        sample_spacing_m=0.1,
        max_speed_mps=15.0,
        preserve_altitude=True,
        shortcut=False,
    )
    assert result.applied
    assert result.geometry_candidate.wait_time_s == 0.0
    assert all(item.action == "move" for item in result.geometry_candidate.waypoints[1:])
    _assert_exact_height_profile(raw, result.geometry_candidate)
    assert result.execution_candidate is not None
    assert result.execution_candidate.wait_time_s == 0.0


def test_original_nearby_height_knots_and_short_wait_are_never_coalesced() -> None:
    raw = _path(
        [
            (0.0, (10.0, 10.0, 5.0)),
            (1.0, (20.0, 10.0, 5.0)),
            (1.0 + 1e-13, (20.0, 10.0, 5.000000002)),
            (1.0 + 2e-13, (20.0, 10.0, 5.000000002)),
            (2.0, (20.0, 20.0, 5.0)),
        ]
    )
    output = TimedPath(
        tuple(_horizontal_geometry_to_timed_block(list(raw.positions), raw.waypoints))
    )
    times = {item.time_s for item in output.waypoints}
    assert times.issuperset(item.time_s for item in raw.waypoints)
    _assert_exact_height_profile(raw, output)
    assert output.wait_time_s == raw.wait_time_s
