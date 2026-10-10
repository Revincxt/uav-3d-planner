"""Lift local horizontal curves onto the unchanged motion/hold clock."""

from __future__ import annotations

from bisect import bisect_left, bisect_right

from uav3d.dynamic import DynamicScenario
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import Point3, almost_equal
from uav3d.horizontal_curves import smooth_horizontal_curves
from uav3d.predictive import TimedAction, TimedPath, TimedWaypoint


def smooth_timed_horizontal_curves(
    scenario: DynamicScenario,
    path: TimedPath,
    turn_scale_m: float,
    sample_spacing_m: float,
    *,
    round_reversals: bool = False,
) -> tuple[TimedPath, int, float | None]:
    """Interpolate service positions and preserve every original Z/time and hold boundary."""
    clocks: list[float] = []
    holds = 0.0
    for index, waypoint in enumerate(path.waypoints):
        if index and waypoint.action == "wait":
            # Cancellation may otherwise turn an exact hold into +/- one ulp of motion.
            clock = clocks[-1]
            holds = waypoint.time_s - clock
        else:
            clock = waypoint.time_s - holds
        clocks.append(clock)
    times = [item.time_s for item in path.waypoints]
    points: list[Point3] = []
    parameters: list[float] = []
    protected: set[float] = set()
    task_positions = [
        tuple(task["position"])
        for task in scenario.static_scene.metadata.get("missionTaskPoints", [])
    ]
    for waypoint, clock in zip(path.waypoints, clocks, strict=True):
        if any(almost_equal(waypoint.position, anchor) for anchor in task_positions):
            protected.add(clock)
        if parameters and clock == parameters[-1]:
            protected.add(clock)
            if waypoint.position != points[-1]:
                raise ValueError("a service hold must preserve its exact position")
            continue
        points.append(waypoint.position)
        parameters.append(clock)

    def time_at(clock: float, *, departure: bool) -> float:
        left, right = bisect_left(clocks, clock), bisect_right(clocks, clock)
        if left != right:
            return times[right - 1 if departure else left]
        index = max(0, right - 1)
        if index >= len(clocks) - 1:
            return times[-1]
        fraction = (clock - clocks[index]) / (clocks[index + 1] - clocks[index])
        return times[index] + fraction * (times[index + 1] - times[index])

    def check(a: Point3, b: Point3, u: float, v: float) -> bool:
        return spacetime_segment_is_free(
            scenario, a, b, time_at(u, departure=True), time_at(v, departure=False)
        )

    curves = smooth_horizontal_curves(
        points,
        parameters,
        check,
        protected=frozenset(protected),
        turn_scale_m=turn_scale_m,
        sample_spacing_m=sample_spacing_m,
        round_reversals=round_reversals,
    )
    output: list[TimedWaypoint] = []
    for point, clock in zip(curves.points, curves.parameters, strict=True):
        left, right = bisect_left(clocks, clock), bisect_right(clocks, clock)
        emitted_times = times[left:right] if left != right else [time_at(clock, departure=True)]
        for time_s in emitted_times:
            action: TimedAction = (
                "start"
                if not output
                else "wait"
                if almost_equal(output[-1].position, point)
                else "move"
            )
            output.append(TimedWaypoint(time_s, point, action))
    candidate = TimedPath(tuple(output))
    if not candidate.is_safe(scenario):
        raise ValueError("horizontal curve failed its complete time-domain collision audit")
    return candidate, curves.rounded_corners, curves.turn_scale_m
