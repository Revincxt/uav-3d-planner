"""Independent decimal round-trip qualification, with unchanged published constraints."""

from __future__ import annotations

import math
from itertools import pairwise

import pytest

from uav3d.kinematics import DiscreteExecutionEnvelope, qualify_timed_path_execution
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.trajectory_timing import retime_timed_path


def _round_trip(path: TimedPath, decimal_places: int = 11) -> TimedPath:
    return TimedPath(
        tuple(
            TimedWaypoint(
                round(waypoint.time_s, decimal_places),
                tuple(round(value, decimal_places) for value in waypoint.position),
                waypoint.action,
            )
            for waypoint in path.waypoints
        )
    )


def test_already_qualified_near_limit_input_is_checked_after_serialization_too() -> None:
    start = (10.0, 10.0, 5.0)
    end = (10.0002999999951, 10.0, 5.0)
    duration = math.sqrt(2.0 * (end[0] - start[0]) / 4.0)
    raw = TimedPath((TimedWaypoint(0.0, start, "start"), TimedWaypoint(duration, end, "move")))
    declared = DiscreteExecutionEnvelope()
    assert qualify_timed_path_execution(raw, declared).qualified
    rounded_unsafe = qualify_timed_path_execution(_round_trip(raw), declared)
    assert rounded_unsafe.violations == ("acceleration-proxy-limit-exceeded",)
    assert rounded_unsafe.boundary_aware_max_discrete_acceleration_proxy_mps2 > 4.0 + 4e-9

    result = retime_timed_path(raw, declared, serialization_decimal_places=11)

    assert result.timed_path is not None
    assert result.timed_path.positions == raw.positions
    assert result.timed_path.duration_s >= raw.duration_s
    assert result.qualification.envelope == declared  # No relaxed limit or diagnostic clamp.
    assert qualify_timed_path_execution(_round_trip(result.timed_path), declared).qualified
    # The tiny reserve is not a new global slowdown policy.
    assert result.timed_path.duration_s < raw.duration_s * (1.0 + 2e-6)


def test_default_preserves_historical_timing_without_decimal_export_guard() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (10.0, 10.0, 5.0), "start"),
            TimedWaypoint(4.0, (18.0, 10.0, 5.0), "move"),
        )
    )
    assert retime_timed_path(raw).timed_path == raw
    assert retime_timed_path(raw, serialization_decimal_places=None).timed_path == raw


def test_round_trip_guard_keeps_height_sequence_real_wait_and_duration_lower_bounds() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (10.0, 10.0, 5.0), "start"),
            TimedWaypoint(0.1, (10.0003, 10.0, 6.0), "move"),
            TimedWaypoint(1.23, (10.0003, 10.0, 6.0), "wait"),
            TimedWaypoint(1.33, (10.0005, 10.0, 5.0), "move"),
        )
    )
    result = retime_timed_path(raw, serialization_decimal_places=11)
    assert result.timed_path is not None
    assert result.timed_path.positions == raw.positions
    for (old_a, old_b), (new_a, new_b) in zip(
        pairwise(raw.waypoints), pairwise(result.timed_path.waypoints), strict=True
    ):
        assert new_b.action == old_b.action
        assert new_b.time_s - new_a.time_s >= old_b.time_s - old_a.time_s - 1e-12
        if old_b.action == "wait":
            assert new_b.time_s - new_a.time_s == pytest.approx(old_b.time_s - old_a.time_s)
    assert qualify_timed_path_execution(
        _round_trip(result.timed_path), DiscreteExecutionEnvelope()
    ).qualified


def test_unrepresentable_real_wait_fails_closed_instead_of_discarding_it() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (10.0, 10.0, 5.0), "start"),
            TimedWaypoint(1.0, (18.0, 10.0, 5.0), "move"),
            TimedWaypoint(1.0001, (18.0, 10.0, 5.0), "wait"),
            TimedWaypoint(2.0001, (26.0, 10.0, 5.0), "move"),
        )
    )
    result = retime_timed_path(raw, serialization_decimal_places=0)
    assert result.timed_path is None
    assert result.status == "time-parameterization-did-not-converge"


@pytest.mark.parametrize("decimal_places", [-1, 16, 11.5, True])
def test_invalid_serialization_precision_is_rejected(decimal_places: object) -> None:
    raw = TimedPath((TimedWaypoint(0.0, (10.0, 10.0, 5.0), "start"),))
    with pytest.raises(ValueError, match="integer between 0 and 15"):
        retime_timed_path(raw, serialization_decimal_places=decimal_places)
