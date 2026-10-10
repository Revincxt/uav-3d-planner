from __future__ import annotations

from dataclasses import replace
from itertools import pairwise

import pytest

import uav3d.trajectory_timing as timing_module
from uav3d.kinematics import (
    DiscreteExecutionEnvelope,
    qualify_timed_path_execution,
)
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.trajectory_timing import retime_timed_path


def _path(points: tuple[tuple[float, float, float], ...], duration_s: float = 1.0) -> TimedPath:
    return TimedPath(
        tuple(
            TimedWaypoint(
                index * duration_s,
                point,
                "start" if index == 0 else "move",
            )
            for index, point in enumerate(points)
        )
    )


def test_default_execution_envelope_is_explicit_and_not_a_continuous_certificate() -> None:
    envelope = DiscreteExecutionEnvelope()

    assert envelope.max_speed_mps == 8.0
    assert envelope.max_abs_climb_rate_mps == 3.0
    assert envelope.max_discrete_acceleration_proxy_mps2 == 4.0
    assert envelope.reversal_threshold_deg == 150.0
    assert not envelope.allow_reversals
    assert envelope.max_execution_time_s == 90.0
    assert envelope.to_dict()["continuous_dynamics_certified"] is False


def test_execution_qualification_includes_virtual_zero_speed_boundaries() -> None:
    path = _path(((0.0, 0.0, 0.0), (8.0, 0.0, 0.0)))

    qualification = qualify_timed_path_execution(path)

    # The legacy interior-only diagnostic has no adjacent pair, while the execution contract sees
    # 0 -> 8 m/s over the first half-segment and the matching terminal stop.
    assert qualification.diagnostics.max_discrete_acceleration_proxy_mps2 == 0.0
    assert qualification.boundary_aware_max_discrete_acceleration_proxy_mps2 == 16.0
    assert qualification.violations == ("acceleration-proxy-limit-exceeded",)
    assert not qualification.qualified


def test_straight_path_is_delayed_without_changing_geometry_or_actions() -> None:
    original = _path(((0.0, 0.0, 0.0), (8.0, 0.0, 0.0)))

    result = retime_timed_path(original)

    assert result.status == "qualified"
    assert result.timed_path is not None
    assert result.timed_path.positions == original.positions
    assert tuple(item.action for item in result.timed_path.waypoints) == ("start", "move")
    assert result.original_duration_s == 1.0
    assert result.candidate_duration_s == pytest.approx(2.0)
    assert result.added_duration_s == pytest.approx(1.0)
    assert result.qualification.qualified
    assert (
        result.qualification.boundary_aware_max_discrete_acceleration_proxy_mps2
        == pytest.approx(4.0)
    )


def test_local_retiming_preserves_wait_duration_and_zero_speed_boundaries() -> None:
    original = TimedPath(
        (
            TimedWaypoint(0.0, (0.0, 0.0, 0.0), "start"),
            TimedWaypoint(1.0, (8.0, 0.0, 0.0), "move"),
            TimedWaypoint(3.0, (8.0, 0.0, 0.0), "wait"),
            TimedWaypoint(4.0, (16.0, 0.0, 0.0), "move"),
        )
    )

    result = retime_timed_path(original)

    assert result.status == "qualified"
    assert result.timed_path is not None
    assert tuple(item.time_s for item in result.timed_path.waypoints) == pytest.approx(
        (0.0, 2.0, 4.0, 6.0)
    )
    waits = [
        current.time_s - previous.time_s
        for previous, current in pairwise(result.timed_path.waypoints)
        if current.action == "wait"
    ]
    assert waits == pytest.approx([2.0])
    assert result.qualification.qualified


def test_climb_rate_lower_bound_is_applied_before_acceleration_relaxation() -> None:
    original = _path(((0.0, 0.0, 0.0), (0.0, 0.0, 8.0)))

    result = retime_timed_path(original)

    assert result.status == "qualified"
    assert result.timed_path is not None
    assert result.timed_path.duration_s == pytest.approx(8.0 / 3.0)
    assert result.qualification.diagnostics.max_abs_climb_rate_mps == pytest.approx(3.0)


def test_nonzero_speed_u_turn_has_an_explicit_failure_and_no_candidate() -> None:
    original = _path(((0.0, 0.0, 0.0), (8.0, 0.0, 0.0), (0.0, 0.0, 0.0)))

    result = retime_timed_path(original)

    assert result.status == "reversal-not-allowed"
    assert result.timed_path is None
    assert "reversal-not-allowed" in result.qualification.violations


def test_execution_time_limit_failure_does_not_expose_a_candidate() -> None:
    original = _path(((0.0, 0.0, 0.0), (8.0, 0.0, 0.0)))
    envelope = DiscreteExecutionEnvelope(
        max_discrete_acceleration_proxy_mps2=1.0,
        max_execution_time_s=2.0,
    )

    result = retime_timed_path(original, envelope)

    assert result.status == "execution-time-limit-exceeded"
    assert result.timed_path is None
    assert result.candidate_duration_s == pytest.approx(4.0)
    assert "execution-time-limit-exceeded" in result.qualification.violations


def test_time_parameterization_is_deterministic_and_never_shortens_a_segment() -> None:
    original = _path(((0.0, 0.0, 0.0), (8.0, 0.0, 0.0), (8.0, 8.0, 0.0)))

    first = retime_timed_path(original)
    second = retime_timed_path(original)

    assert first == second
    assert first.to_dict() == second.to_dict()
    assert first.timed_path is not None
    original_durations = [
        current.time_s - previous.time_s for previous, current in pairwise(original.waypoints)
    ]
    candidate_durations = [
        current.time_s - previous.time_s
        for previous, current in pairwise(first.timed_path.waypoints)
    ]
    assert all(
        candidate + 1e-12 >= source
        for source, candidate in zip(original_durations, candidate_durations, strict=True)
    )


def test_converged_solver_retries_decimal_guard_without_relaxing_limits_or_waits(
    monkeypatch,
) -> None:
    path = TimedPath(
        (
            TimedWaypoint(0, (0, 0, 20), "start"),
            TimedWaypoint(1, (0, 0, 20), "wait"),
            TimedWaypoint(3, (1, 0, 20), "move"),
            TimedWaypoint(5, (2, 0, 20), "move"),
        )
    )
    original = timing_module._qualified_after_decimal_serialization

    def guard(candidate, envelope, decimal):
        return candidate.duration_s > 5.001 and original(candidate, envelope, decimal)

    monkeypatch.setattr(timing_module, "_qualified_after_decimal_serialization", guard)
    result = retime_timed_path(path, serialization_decimal_places=11)
    assert result.status == "qualified" and result.timed_path is not None
    assert result.timed_path.positions == path.positions
    assert result.timed_path.wait_time_s == 1
    assert result.timed_path.duration_s == pytest.approx(5.004)
    assert qualify_timed_path_execution(result.timed_path).qualified
    assert original(result.timed_path, DiscreteExecutionEnvelope(), 11)


def test_unrecoverable_decimal_guard_still_fails_closed(monkeypatch) -> None:
    monkeypatch.setattr(timing_module, "_qualified_after_decimal_serialization", lambda *_: False)
    result = retime_timed_path(_path(((0, 0, 20), (1, 0, 20)), 2), serialization_decimal_places=11)
    assert result.status == "time-parameterization-did-not-converge"
    assert result.timed_path is None
    assert result.iterations < 128


def test_converged_local_durations_retry_cumulative_clock_roundoff(monkeypatch) -> None:
    """Checking the rebuilt absolute clock can expose error absent from local durations."""
    path = _path(((0, 0, 20), (1, 0, 20), (2, 0, 20)), 2)
    qualify = timing_module.qualify_timed_path_execution

    def absolute_clock_check(candidate, envelope):
        result = qualify(candidate, envelope)
        if candidate.duration_s <= 4.001:
            return replace(
                result,
                qualified=False,
                boundary_aware_max_discrete_acceleration_proxy_mps2=4.000003,
                violations=("acceleration-proxy-limit-exceeded",),
            )
        return result

    monkeypatch.setattr(timing_module, "qualify_timed_path_execution", absolute_clock_check)
    result = retime_timed_path(path, serialization_decimal_places=11)
    assert result.status == "qualified" and result.timed_path is not None
    assert result.timed_path.positions == path.positions
    assert result.timed_path.duration_s == pytest.approx(4.004)
    assert qualify(result.timed_path).qualified


@pytest.mark.parametrize(
    ("parameter", "value"),
    (
        ("max_speed_mps", 0.0),
        ("max_abs_climb_rate_mps", -1.0),
        ("max_discrete_acceleration_proxy_mps2", float("nan")),
        ("reversal_threshold_deg", 181.0),
        ("max_execution_time_s", 0.0),
    ),
)
def test_execution_envelope_rejects_invalid_limits(parameter: str, value: float) -> None:
    with pytest.raises(ValueError):
        DiscreteExecutionEnvelope(
            max_speed_mps=value if parameter == "max_speed_mps" else 8.0,
            max_abs_climb_rate_mps=(value if parameter == "max_abs_climb_rate_mps" else 3.0),
            max_discrete_acceleration_proxy_mps2=(
                value if parameter == "max_discrete_acceleration_proxy_mps2" else 4.0
            ),
            reversal_threshold_deg=(value if parameter == "reversal_threshold_deg" else 150.0),
            max_execution_time_s=(value if parameter == "max_execution_time_s" else 90.0),
        )
