"""Deterministic time dilation for a discrete waypoint execution envelope.

The geometry and action sequence remain fixed.  Movement durations may only grow, waits keep their
original durations, and each move block is reviewed as starting and ending at zero segment-average
velocity.  The result is a finite-difference qualification, not a continuous-dynamics certificate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise
from typing import Literal

from uav3d.geometry import Point3, norm, scale, subtract
from uav3d.kinematics import (
    DiscreteExecutionEnvelope,
    DiscreteExecutionQualification,
    qualify_timed_path_execution,
)
from uav3d.predictive import TimedPath, TimedWaypoint

TrajectoryTimingStatus = Literal[
    "qualified",
    "reversal-not-allowed",
    "execution-time-limit-exceeded",
    "time-parameterization-did-not-converge",
]

_NUMERIC_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class TrajectoryTimingResult:
    """Outcome of deterministic duration-only time parameterization."""

    status: TrajectoryTimingStatus
    timed_path: TimedPath | None
    qualification: DiscreteExecutionQualification
    iterations: int
    original_duration_s: float
    candidate_duration_s: float

    def __post_init__(self) -> None:
        if self.iterations < 0:
            raise ValueError("timing iterations must be non-negative")
        durations = (self.original_duration_s, self.candidate_duration_s)
        if not all(math.isfinite(value) and value >= 0.0 for value in durations):
            raise ValueError("timing durations must be finite and non-negative")
        if self.candidate_duration_s + _NUMERIC_EPSILON < self.original_duration_s:
            raise ValueError("time parameterization must not shorten the original path")
        if self.status == "qualified":
            if self.timed_path is None or not self.qualification.qualified:
                raise ValueError("qualified timing requires a qualified timed path")
        elif self.timed_path is not None:
            raise ValueError("failed timing must not expose an execution candidate")

    @property
    def added_duration_s(self) -> float:
        return self.candidate_duration_s - self.original_duration_s

    @property
    def continuous_dynamics_certified(self) -> bool:
        return False

    def to_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "timed_path": self.timed_path.to_dict() if self.timed_path is not None else None,
            "qualification": self.qualification.to_dict(),
            "iterations": self.iterations,
            "original_duration_s": self.original_duration_s,
            "candidate_duration_s": self.candidate_duration_s,
            "added_duration_s": self.added_duration_s,
            "continuous_dynamics_certified": False,
        }


def _segment_displacements(path: TimedPath) -> list[Point3]:
    return [
        subtract(current.position, previous.position)
        for previous, current in pairwise(path.waypoints)
    ]


def _segment_durations(path: TimedPath) -> list[float]:
    return [current.time_s - previous.time_s for previous, current in pairwise(path.waypoints)]


def _build_timed_path(path: TimedPath, durations: list[float]) -> TimedPath:
    if len(durations) != len(path.waypoints) - 1:
        raise ValueError("one duration is required for every path segment")
    output = [path.waypoints[0]]
    time_s = path.departure_time_s
    for waypoint, duration in zip(path.waypoints[1:], durations, strict=True):
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("retimed segment durations must be finite and positive")
        time_s += duration
        output.append(TimedWaypoint(time_s, waypoint.position, waypoint.action))
    return TimedPath(tuple(output))


def _movement_blocks(path: TimedPath) -> tuple[tuple[int, int], ...]:
    """Return inclusive segment-index ranges for maximal consecutive move blocks."""

    actions = tuple(waypoint.action for waypoint in path.waypoints[1:])
    blocks: list[tuple[int, int]] = []
    index = 0
    while index < len(actions):
        if actions[index] != "move":
            index += 1
            continue
        start = index
        while index + 1 < len(actions) and actions[index + 1] == "move":
            index += 1
        blocks.append((start, index))
        index += 1
    return tuple(blocks)


def _velocity(displacement: Point3, duration: float) -> Point3:
    return scale(displacement, 1.0 / duration)


def _boundary_duration(displacement: Point3, acceleration_limit: float) -> float:
    # 0 -> segment-average velocity over half a segment duration:
    # a_proxy = 2 * |delta| / duration**2.
    return math.sqrt(2.0 * norm(displacement) / acceleration_limit)


def _transition_proxy(
    left_displacement: Point3,
    left_duration: float,
    right_displacement: Point3,
    right_duration: float,
) -> float:
    change = norm(
        subtract(
            _velocity(right_displacement, right_duration),
            _velocity(left_displacement, left_duration),
        )
    )
    return change / (0.5 * (left_duration + right_duration))


def _grow_duration(durations: list[float], index: int, required: float) -> bool:
    if required <= durations[index] * (1.0 + _NUMERIC_EPSILON):
        return False
    durations[index] = required
    return True


def _relax_acceleration_constraints(
    displacements: list[Point3],
    durations: list[float],
    blocks: tuple[tuple[int, int], ...],
    acceleration_limit: float,
) -> bool:
    """Perform one deterministic forward/backward local duration-growth sweep."""

    changed = False
    for start, end in blocks:
        changed |= _grow_duration(
            durations,
            start,
            _boundary_duration(displacements[start], acceleration_limit),
        )
        changed |= _grow_duration(
            durations,
            end,
            _boundary_duration(displacements[end], acceleration_limit),
        )

        for indices in (range(start + 1, end + 1), range(end, start, -1)):
            for right in indices:
                left = right - 1
                proxy = _transition_proxy(
                    displacements[left],
                    durations[left],
                    displacements[right],
                    durations[right],
                )
                if proxy <= acceleration_limit * (1.0 + 1e-9):
                    continue
                # Scaling both adjacent movement durations by s divides this transition's proxy by
                # s**2 exactly. Later neighboring updates only grow durations, and another sweep
                # independently verifies the full block.
                factor = math.sqrt(proxy / acceleration_limit)
                durations[left] *= factor
                durations[right] *= factor
                changed = True
    return changed


def retime_timed_path(
    path: TimedPath,
    envelope: DiscreteExecutionEnvelope | None = None,
    *,
    max_iterations: int = 128,
) -> TrajectoryTimingResult:
    """Delay a fixed path locally until it satisfies the discrete execution envelope.

    No segment is made faster than the input. Wait durations are copied exactly. A failed result
    intentionally returns ``timed_path=None`` so callers cannot mistake a collision-only geometry
    path for a qualified execution candidate.
    """

    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    declared = envelope or DiscreteExecutionEnvelope()
    original_qualification = qualify_timed_path_execution(path, declared)
    if "reversal-not-allowed" in original_qualification.violations:
        return TrajectoryTimingResult(
            "reversal-not-allowed",
            None,
            original_qualification,
            0,
            path.duration_s,
            path.duration_s,
        )

    displacements = _segment_displacements(path)
    original_durations = _segment_durations(path)
    durations = list(original_durations)
    for index, (waypoint, displacement) in enumerate(
        zip(path.waypoints[1:], displacements, strict=True)
    ):
        if waypoint.action != "move":
            continue
        durations[index] = max(
            durations[index],
            norm(displacement) / declared.max_speed_mps,
            abs(displacement[2]) / declared.max_abs_climb_rate_mps,
        )

    blocks = _movement_blocks(path)
    candidate = _build_timed_path(path, durations)
    qualification = qualify_timed_path_execution(candidate, declared)
    if "execution-time-limit-exceeded" in qualification.violations:
        return TrajectoryTimingResult(
            "execution-time-limit-exceeded",
            None,
            qualification,
            0,
            path.duration_s,
            candidate.duration_s,
        )
    if qualification.qualified:
        return TrajectoryTimingResult(
            "qualified",
            candidate,
            qualification,
            0,
            path.duration_s,
            candidate.duration_s,
        )

    last_iteration = 0
    for iteration in range(1, max_iterations + 1):
        last_iteration = iteration
        changed = _relax_acceleration_constraints(
            displacements,
            durations,
            blocks,
            declared.max_discrete_acceleration_proxy_mps2,
        )
        candidate = _build_timed_path(path, durations)
        qualification = qualify_timed_path_execution(candidate, declared)
        if "execution-time-limit-exceeded" in qualification.violations:
            return TrajectoryTimingResult(
                "execution-time-limit-exceeded",
                None,
                qualification,
                iteration,
                path.duration_s,
                candidate.duration_s,
            )
        if qualification.qualified:
            return TrajectoryTimingResult(
                "qualified",
                candidate,
                qualification,
                iteration,
                path.duration_s,
                candidate.duration_s,
            )
        if not changed:
            break

    return TrajectoryTimingResult(
        "time-parameterization-did-not-converge",
        None,
        qualification,
        last_iteration,
        path.duration_s,
        candidate.duration_s,
    )


__all__ = [
    "TrajectoryTimingResult",
    "TrajectoryTimingStatus",
    "retime_timed_path",
]
