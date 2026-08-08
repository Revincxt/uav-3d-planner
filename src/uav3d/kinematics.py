"""Honest discrete kinematic diagnostics for timestamped waypoint paths.

These measurements use segment-average velocities and finite differences between adjacent
segments.  They are useful for detecting reversals and abrupt waypoint motion, but they do not
certify continuous acceleration, attitude, curvature, or jerk bounds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise
from typing import Literal

from uav3d.geometry import Point3, dot, norm, scale, subtract
from uav3d.predictive import TimedAction, TimedPath

DEFAULT_REVERSAL_THRESHOLD_DEG = 150.0
_VELOCITY_EPSILON = 1e-12

ExecutionEnvelopeViolation = Literal[
    "speed-limit-exceeded",
    "climb-rate-limit-exceeded",
    "acceleration-proxy-limit-exceeded",
    "reversal-not-allowed",
    "execution-time-limit-exceeded",
]


@dataclass(frozen=True, slots=True)
class DiscreteExecutionEnvelope:
    """Declared limits for a waypoint-level execution model.

    The envelope constrains segment-average velocities and finite differences.  It does not model
    continuous thrust, attitude, curvature, or jerk and therefore is never a flight certificate.
    """

    max_speed_mps: float = 8.0
    max_abs_climb_rate_mps: float = 3.0
    max_discrete_acceleration_proxy_mps2: float = 4.0
    reversal_threshold_deg: float = DEFAULT_REVERSAL_THRESHOLD_DEG
    allow_reversals: bool = False
    max_execution_time_s: float = 90.0

    def __post_init__(self) -> None:
        positive = (
            self.max_speed_mps,
            self.max_abs_climb_rate_mps,
            self.max_discrete_acceleration_proxy_mps2,
            self.max_execution_time_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("execution-envelope limits must be finite and positive")
        if (
            not math.isfinite(self.reversal_threshold_deg)
            or self.reversal_threshold_deg <= 0.0
            or self.reversal_threshold_deg > 180.0
        ):
            raise ValueError("reversal_threshold_deg must be finite and in (0, 180]")

    def to_dict(self) -> dict[str, object]:
        return {
            "model": "discrete-segment-average-envelope-v1",
            "max_speed_mps": self.max_speed_mps,
            "max_abs_climb_rate_mps": self.max_abs_climb_rate_mps,
            "max_discrete_acceleration_proxy_mps2": (self.max_discrete_acceleration_proxy_mps2),
            "reversal_threshold_deg": self.reversal_threshold_deg,
            "allow_reversals": self.allow_reversals,
            "max_execution_time_s": self.max_execution_time_s,
            "continuous_dynamics_certified": False,
        }


@dataclass(frozen=True, slots=True)
class DiscreteKinematicDiagnostics:
    """Finite-difference descriptors; never a continuous-dynamics certificate."""

    segment_count: int
    movement_segment_count: int
    reversal_count: int
    reversal_threshold_deg: float
    max_speed_mps: float
    max_discrete_velocity_change_mps: float
    max_discrete_acceleration_proxy_mps2: float
    max_abs_climb_rate_mps: float

    def to_dict(self) -> dict[str, object]:
        return {
            "status": "discrete-diagnostic-only",
            "continuous_dynamics_certified": False,
            "segment_count": self.segment_count,
            "movement_segment_count": self.movement_segment_count,
            "reversal_count": self.reversal_count,
            "reversal_threshold_deg": self.reversal_threshold_deg,
            "max_speed_mps": self.max_speed_mps,
            "max_discrete_velocity_change_mps": self.max_discrete_velocity_change_mps,
            "max_discrete_acceleration_proxy_mps2": (self.max_discrete_acceleration_proxy_mps2),
            "max_abs_climb_rate_mps": self.max_abs_climb_rate_mps,
        }


@dataclass(frozen=True, slots=True)
class DiscreteExecutionQualification:
    """Independent verdict for one path under a declared discrete envelope."""

    qualified: bool
    envelope: DiscreteExecutionEnvelope
    diagnostics: DiscreteKinematicDiagnostics
    boundary_aware_max_discrete_acceleration_proxy_mps2: float
    violations: tuple[ExecutionEnvelopeViolation, ...]

    def __post_init__(self) -> None:
        if self.qualified != (not self.violations):
            raise ValueError("qualified must be true exactly when violations is empty")
        if (
            not math.isfinite(self.boundary_aware_max_discrete_acceleration_proxy_mps2)
            or self.boundary_aware_max_discrete_acceleration_proxy_mps2 < 0.0
        ):
            raise ValueError("boundary-aware acceleration proxy must be finite and non-negative")

    @property
    def continuous_dynamics_certified(self) -> bool:
        return False

    def to_dict(self) -> dict[str, object]:
        return {
            "status": "qualified" if self.qualified else "not-qualified",
            "qualified": self.qualified,
            "continuous_dynamics_certified": False,
            "envelope": self.envelope.to_dict(),
            "diagnostics": self.diagnostics.to_dict(),
            "boundary_aware_max_discrete_acceleration_proxy_mps2": (
                self.boundary_aware_max_discrete_acceleration_proxy_mps2
            ),
            "violations": list(self.violations),
        }


@dataclass(frozen=True, slots=True)
class _TimedVelocity:
    duration_s: float
    velocity_mps: Point3
    action: TimedAction


def _segment_velocities(path: TimedPath) -> tuple[_TimedVelocity, ...]:
    segments: list[_TimedVelocity] = []
    for previous, current in pairwise(path.waypoints):
        duration = current.time_s - previous.time_s
        displacement = subtract(current.position, previous.position)
        segments.append(
            _TimedVelocity(duration, scale(displacement, 1.0 / duration), current.action)
        )
    return tuple(segments)


def execution_acceleration_proxies(path: TimedPath) -> tuple[float, ...]:
    """Return acceleration proxies with zero speed at mission and wait boundaries.

    Consecutive ``move`` segments use the time between their segment midpoints.  Every maximal move
    block starts and ends at virtual zero velocity; a wait's duration is deliberately not treated as
    extra braking time.  This makes an instantaneous move-to-wait transition visible instead of
    diluting it with a long stationary interval.
    """

    segments = _segment_velocities(path)
    proxies: list[float] = []
    index = 0
    while index < len(segments):
        if segments[index].action != "move":
            index += 1
            continue
        block_start = index
        while index + 1 < len(segments) and segments[index + 1].action == "move":
            index += 1
        block_end = index

        first = segments[block_start]
        proxies.append(2.0 * norm(first.velocity_mps) / first.duration_s)
        for previous, current in pairwise(segments[block_start : block_end + 1]):
            change = norm(subtract(current.velocity_mps, previous.velocity_mps))
            proxies.append(change / (0.5 * (previous.duration_s + current.duration_s)))
        last = segments[block_end]
        proxies.append(2.0 * norm(last.velocity_mps) / last.duration_s)
        index += 1
    return tuple(proxies)


def diagnose_timed_path_kinematics(
    path: TimedPath,
    *,
    reversal_threshold_deg: float = DEFAULT_REVERSAL_THRESHOLD_DEG,
) -> DiscreteKinematicDiagnostics:
    """Describe waypoint-level motion without asserting continuous dynamic feasibility.

    The acceleration proxy divides the change between adjacent segment-average velocity vectors by
    the time between their segment midpoints, namely half the sum of their durations.  Reversals are
    counted only across adjacent non-zero movement segments whose direction change meets the
    declared threshold; waits are hard boundaries.
    """

    if (
        not math.isfinite(reversal_threshold_deg)
        or reversal_threshold_deg <= 0.0
        or reversal_threshold_deg > 180.0
    ):
        raise ValueError("reversal_threshold_deg must be finite and in (0, 180]")

    segments = _segment_velocities(path)
    speeds = tuple(norm(segment.velocity_mps) for segment in segments)
    velocity_changes: list[float] = []
    acceleration_proxies: list[float] = []
    reversals = 0

    for previous, current in pairwise(segments):
        change = norm(subtract(current.velocity_mps, previous.velocity_mps))
        velocity_changes.append(change)
        midpoint_delta = 0.5 * (previous.duration_s + current.duration_s)
        acceleration_proxies.append(change / midpoint_delta)

        previous_speed = norm(previous.velocity_mps)
        current_speed = norm(current.velocity_mps)
        if (
            previous.action != "move"
            or current.action != "move"
            or previous_speed <= _VELOCITY_EPSILON
            or current_speed <= _VELOCITY_EPSILON
        ):
            continue
        cosine = dot(previous.velocity_mps, current.velocity_mps) / (previous_speed * current_speed)
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        if angle_deg + 1e-9 >= reversal_threshold_deg:
            reversals += 1

    return DiscreteKinematicDiagnostics(
        segment_count=len(segments),
        movement_segment_count=sum(segment.action == "move" for segment in segments),
        reversal_count=reversals,
        reversal_threshold_deg=reversal_threshold_deg,
        max_speed_mps=max(speeds, default=0.0),
        max_discrete_velocity_change_mps=max(velocity_changes, default=0.0),
        max_discrete_acceleration_proxy_mps2=max(acceleration_proxies, default=0.0),
        max_abs_climb_rate_mps=max(
            (abs(segment.velocity_mps[2]) for segment in segments), default=0.0
        ),
    )


def _exceeds(value: float, limit: float) -> bool:
    tolerance = max(1e-9, limit * 1e-9)
    return value > limit + tolerance


def qualify_timed_path_execution(
    path: TimedPath,
    envelope: DiscreteExecutionEnvelope | None = None,
) -> DiscreteExecutionQualification:
    """Check a path against a declared finite-difference execution envelope."""

    declared = envelope or DiscreteExecutionEnvelope()
    diagnostics = diagnose_timed_path_kinematics(
        path, reversal_threshold_deg=declared.reversal_threshold_deg
    )
    boundary_acceleration = max(execution_acceleration_proxies(path), default=0.0)
    violations: list[ExecutionEnvelopeViolation] = []
    if _exceeds(diagnostics.max_speed_mps, declared.max_speed_mps):
        violations.append("speed-limit-exceeded")
    if _exceeds(diagnostics.max_abs_climb_rate_mps, declared.max_abs_climb_rate_mps):
        violations.append("climb-rate-limit-exceeded")
    if _exceeds(
        boundary_acceleration,
        declared.max_discrete_acceleration_proxy_mps2,
    ):
        violations.append("acceleration-proxy-limit-exceeded")
    if diagnostics.reversal_count and not declared.allow_reversals:
        violations.append("reversal-not-allowed")
    if _exceeds(path.duration_s, declared.max_execution_time_s):
        violations.append("execution-time-limit-exceeded")
    return DiscreteExecutionQualification(
        qualified=not violations,
        envelope=declared,
        diagnostics=diagnostics,
        boundary_aware_max_discrete_acceleration_proxy_mps2=boundary_acceleration,
        violations=tuple(violations),
    )


__all__ = [
    "DEFAULT_REVERSAL_THRESHOLD_DEG",
    "DiscreteExecutionEnvelope",
    "DiscreteExecutionQualification",
    "DiscreteKinematicDiagnostics",
    "ExecutionEnvelopeViolation",
    "diagnose_timed_path_kinematics",
    "execution_acceleration_proxies",
    "qualify_timed_path_execution",
]
