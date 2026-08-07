"""Honest discrete kinematic diagnostics for timestamped waypoint paths.

These measurements use segment-average velocities and finite differences between adjacent
segments.  They are useful for detecting reversals and abrupt waypoint motion, but they do not
certify continuous acceleration, attitude, curvature, or jerk bounds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from uav3d.geometry import Point3, dot, norm, scale, subtract
from uav3d.predictive import TimedAction, TimedPath

DEFAULT_REVERSAL_THRESHOLD_DEG = 150.0
_VELOCITY_EPSILON = 1e-12


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
            "max_discrete_acceleration_proxy_mps2": (
                self.max_discrete_acceleration_proxy_mps2
            ),
            "max_abs_climb_rate_mps": self.max_abs_climb_rate_mps,
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
        cosine = dot(previous.velocity_mps, current.velocity_mps) / (
            previous_speed * current_speed
        )
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


__all__ = [
    "DEFAULT_REVERSAL_THRESHOLD_DEG",
    "DiscreteKinematicDiagnostics",
    "diagnose_timed_path_kinematics",
]
