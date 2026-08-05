"""Contracts for deterministic predictive space-time planning."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import pairwise
from typing import TYPE_CHECKING, Literal

from uav3d.geometry import Point3, almost_equal, distance

if TYPE_CHECKING:
    from uav3d.dynamic import DynamicScenario

TimedAction = Literal["start", "move", "wait"]
PredictiveScalar = str | int | float | bool | None


@dataclass(frozen=True, slots=True)
class TimedWaypoint:
    """One timestamped position and the action that ends at it.

    ``action`` describes the segment from the preceding waypoint. The first waypoint must use
    ``"start"``; later equal-position segments must use ``"wait"`` and moving segments must use
    ``"move"``. This makes waiting explicit instead of encoding it as an undocumented duplicate.
    """

    time_s: float
    position: Point3
    action: TimedAction

    def __post_init__(self) -> None:
        if not math.isfinite(self.time_s) or self.time_s < 0:
            raise ValueError("timed-waypoint time must be finite and non-negative")
        if not all(math.isfinite(value) for value in self.position):
            raise ValueError("timed-waypoint coordinates must be finite")
        if self.action not in {"start", "move", "wait"}:
            raise ValueError("timed-waypoint action must be start, move, or wait")

    def to_dict(self) -> dict[str, object]:
        return {
            "time_s": self.time_s,
            "position": list(self.position),
            "action": self.action,
        }


@dataclass(frozen=True, slots=True)
class TimedPath:
    """A non-empty trajectory with strictly increasing timestamps and explicit waits."""

    waypoints: tuple[TimedWaypoint, ...]

    def __post_init__(self) -> None:
        if not self.waypoints:
            raise ValueError("a timed path requires at least one waypoint")
        if self.waypoints[0].action != "start":
            raise ValueError("the first timed waypoint must use the start action")
        for previous, current in pairwise(self.waypoints):
            if current.time_s <= previous.time_s:
                raise ValueError("timed-path timestamps must be strictly increasing")
            stationary = almost_equal(previous.position, current.position)
            expected: TimedAction = "wait" if stationary else "move"
            if current.action != expected:
                raise ValueError(
                    f"a {'stationary' if stationary else 'moving'} segment must use {expected!r}"
                )

    @property
    def start(self) -> Point3:
        return self.waypoints[0].position

    @property
    def goal(self) -> Point3:
        return self.waypoints[-1].position

    @property
    def departure_time_s(self) -> float:
        return self.waypoints[0].time_s

    @property
    def arrival_time_s(self) -> float:
        return self.waypoints[-1].time_s

    @property
    def duration_s(self) -> float:
        return self.arrival_time_s - self.departure_time_s

    @property
    def wait_time_s(self) -> float:
        return math.fsum(
            current.time_s - previous.time_s
            for previous, current in pairwise(self.waypoints)
            if current.action == "wait"
        )

    @property
    def positions(self) -> tuple[Point3, ...]:
        return tuple(waypoint.position for waypoint in self.waypoints)

    @property
    def timed_points(self) -> tuple[tuple[float, Point3], ...]:
        return tuple((waypoint.time_s, waypoint.position) for waypoint in self.waypoints)

    def segment_speeds(self) -> tuple[float, ...]:
        return tuple(
            distance(previous.position, current.position) / (current.time_s - previous.time_s)
            for previous, current in pairwise(self.waypoints)
        )

    def is_safe(self, scenario: DynamicScenario) -> bool:
        """Audit every wait and movement with the exact continuous space-time predicate."""

        # Kept local so ``uav3d.planners`` can export predictive planners while ``dynamic`` imports
        # the static benchmark fingerprint implementation during package initialization.
        from uav3d.dynamic_collision import timed_path_is_free

        return timed_path_is_free(scenario, self.timed_points)

    def to_dict(self) -> dict[str, object]:
        return {
            "waypoints": [waypoint.to_dict() for waypoint in self.waypoints],
            "departure_time_s": self.departure_time_s,
            "arrival_time_s": self.arrival_time_s,
            "duration_s": self.duration_s,
            "wait_time_s": self.wait_time_s,
        }


@dataclass(frozen=True, slots=True)
class PredictivePlanningResult:
    """Serializable outcome for a finite-horizon predictive search."""

    algorithm: str
    success: bool
    timed_path: TimedPath | None
    expanded_spacetime_states: int
    generated_spacetime_states: int
    failure_reason: str | None = None
    parameters: dict[str, PredictiveScalar] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.algorithm:
            raise ValueError("predictive result algorithm must not be empty")
        if self.expanded_spacetime_states < 0 or self.generated_spacetime_states < 0:
            raise ValueError("predictive state counters must be non-negative")
        if self.generated_spacetime_states < self.expanded_spacetime_states:
            raise ValueError("generated space-time states must cover every expanded state")
        if self.success:
            if self.timed_path is None or self.failure_reason is not None:
                raise ValueError(
                    "successful predictive results require a path and no failure reason"
                )
        elif self.timed_path is not None or not self.failure_reason:
            raise ValueError("failed predictive results require a failure reason and no path")

    @property
    def path(self) -> tuple[Point3, ...]:
        return self.timed_path.positions if self.timed_path is not None else ()

    @property
    def arrival_time_s(self) -> float | None:
        return self.timed_path.arrival_time_s if self.timed_path is not None else None

    def to_dict(self) -> dict[str, object]:
        return {
            "algorithm": self.algorithm,
            "success": self.success,
            "timed_path": self.timed_path.to_dict() if self.timed_path is not None else None,
            "expanded_spacetime_states": self.expanded_spacetime_states,
            "generated_spacetime_states": self.generated_spacetime_states,
            "failure_reason": self.failure_reason,
            "parameters": self.parameters,
        }


__all__ = [
    "PredictivePlanningResult",
    "PredictiveScalar",
    "TimedAction",
    "TimedPath",
    "TimedWaypoint",
]
