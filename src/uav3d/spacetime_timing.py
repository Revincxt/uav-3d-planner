"""Reconcile execution timing with dynamic occupancy, without changing geometry.

Only whole move blocks may be delayed. Waiting occurs at an already-stationary
block boundary, never by inventing a stop in the middle of a high-speed segment.
Both the additional hold and every subsequent traversal are continuously checked.
Failure exposes no execution candidate.
"""

from __future__ import annotations

import math
from itertools import pairwise

from uav3d.dynamic import DynamicScenario
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import dot, norm, subtract
from uav3d.kinematics import DiscreteExecutionEnvelope, qualify_timed_path_execution
from uav3d.predictive import TimedPath, TimedWaypoint


def insert_braking_holds(path: TimedPath, envelope: DiscreteExecutionEnvelope) -> TimedPath:
    """A retained sharp altitude extremum requires braking, not an instant reversal.

    XY-only smoothing cannot round away a hard Z knot. Insert an explicit hover;
    the duration solver must independently qualify stopping and restarting there.
    """
    output = [path.waypoints[0]]
    delay = 0.0
    for index, point in enumerate(path.waypoints[1:], 1):
        output.append(TimedWaypoint(point.time_s + delay, point.position, point.action))
        if envelope.allow_reversals or index + 1 == len(path.waypoints):
            continue
        previous, following = path.waypoints[index - 1], path.waypoints[index + 1]
        if point.action != "move" or following.action != "move":
            continue
        incoming, outgoing = (
            subtract(point.position, previous.position),
            subtract(following.position, point.position),
        )
        product = norm(incoming) * norm(outgoing)
        if product and dot(incoming, outgoing) / product <= math.cos(
            math.radians(envelope.reversal_threshold_deg)
        ):
            delay += 2.0
            output.append(TimedWaypoint(point.time_s + delay, point.position, "wait"))
    return TimedPath(tuple(output))


def schedule_safe_departures(
    scenario: DynamicScenario,
    path: TimedPath,
    envelope: DiscreteExecutionEnvelope,
    *,
    time_step_s: float = 2.0,
) -> TimedPath | None:
    """Find the earliest certified departure on a declared discrete delay grid.

    Original move durations, all altitude knots, ordered stops and service durations
    are retained. This is a conservative scheduling repair, not a global optimizer.
    """
    if not math.isfinite(time_step_s) or time_step_s <= 0:
        raise ValueError("Scheduling time step must be finite and positive")
    output = [path.waypoints[0]]
    index = 1
    limit = path.departure_time_s + envelope.max_execution_time_s
    tasks = scenario.static_scene.metadata.get("missionTaskPoints", [])
    while index < len(path.waypoints):
        end = index
        # When a mission declares service roofs, schedule an entire leg at its
        # source roof. Intermediate 4D waits/braking holds stay within that leg;
        # delaying at an arbitrary mid-leg point could strand it in closing airspace.
        while end < len(path.waypoints):
            point = path.waypoints[end]
            boundary = point.action == "wait" and (
                not tasks
                or any(
                    norm(subtract(point.position, tuple(task["position"]))) < 1e-5 for task in tasks
                )
            )
            end += 1
            if boundary:
                while end < len(path.waypoints) and path.waypoints[end].action == "wait":
                    end += 1
                break
        original_start = path.waypoints[index - 1].time_s
        clock, position = output[-1].time_s, output[-1].position
        duration = path.waypoints[end - 1].time_s - original_start
        found = False
        for step in range(max(0, math.floor((limit - clock - duration) / time_step_s)) + 1):
            delay = step * time_step_s
            departure = clock + delay
            if delay and not spacetime_segment_is_free(
                scenario, position, position, clock, departure
            ):
                # All longer waits contain this unsafe prefix as well.
                break
            block = [TimedWaypoint(departure, position, "start")]
            block.extend(
                TimedWaypoint(
                    departure + point.time_s - original_start, point.position, point.action
                )
                for point in path.waypoints[index:end]
            )
            if block[-1].time_s > limit + 1e-9:
                break
            if not all(
                spacetime_segment_is_free(scenario, a.position, b.position, a.time_s, b.time_s)
                for a, b in pairwise(block)
            ):
                continue
            if delay:
                output.append(TimedWaypoint(departure, position, "wait"))
            output.extend(block[1:])
            found = True
            break
        if not found:
            return None
        index = end
    candidate = TimedPath(tuple(output))
    if (
        not candidate.is_safe(scenario)
        or not qualify_timed_path_execution(candidate, envelope).qualified
    ):
        return None
    return candidate
