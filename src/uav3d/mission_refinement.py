"""Flight-aware demo refinements with independently audited collision/kinematic gates."""

from __future__ import annotations

import math
from dataclasses import replace
from itertools import pairwise

from uav3d.dynamic import DynamicScenario
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import distance
from uav3d.kinematics import DiscreteExecutionEnvelope
from uav3d.planners.grid import GridIndex, VoxelGrid
from uav3d.planners.space_time_astar import SpaceTimeAStar3D, SpaceTimeState
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.predictive_smoothing import PredictiveSmoothingResult, smooth_predictive_timed_path


class ContinuousAnchorSpaceTimeAStar(SpaceTimeAStar3D):
    """Reach a lattice anchor continuously instead of braking to align its clock.

    State arrival times and the search graph are unchanged. The exact connector
    moves slightly more slowly to the same aligned timestamp, but only when its
    entire space-time segment is safe. If slower motion would hit traffic, retain
    the original safe connector and hover. Real forecast waits are never removed.
    """

    def _initial_states(
        self,
        scenario: DynamicScenario,
        grid: VoxelGrid,
        anchors: list[GridIndex],
        start_time: float,
        horizon_time: float,
    ) -> list[tuple[SpaceTimeState, tuple[TimedWaypoint, ...]]]:
        states = super()._initial_states(scenario, grid, anchors, start_time, horizon_time)
        output = []
        for state, prefix in states:
            if len(prefix) >= 3 and prefix[-1].action == "wait" and prefix[-2].action == "move":
                previous, arrival, aligned = prefix[-3:]
                if spacetime_segment_is_free(
                    scenario, previous.position, arrival.position, previous.time_s, aligned.time_s
                ):
                    prefix = (*prefix[:-2], TimedWaypoint(aligned.time_s, arrival.position, "move"))
            output.append((state, prefix))
        return output


def horizontal_reversals(path: TimedPath) -> int:
    """Count meaningful XY reversals, not centimetre-scale noise on a vertical leg."""
    count = 0
    for a, b, c in zip(path.waypoints, path.waypoints[1:], path.waypoints[2:], strict=False):
        if b.action != "move" or c.action != "move":
            continue
        incoming = (b.position[0] - a.position[0], b.position[1] - a.position[1])
        outgoing = (c.position[0] - b.position[0], c.position[1] - b.position[1])
        lengths = (math.hypot(*incoming), math.hypot(*outgoing))
        if min(lengths) < 0.5:
            continue
        cosine = sum(x * y for x, y in zip(incoming, outgoing, strict=True)) / math.prod(lengths)
        count += cosine < math.cos(math.radians(120))
    return count


def refine_mission_trajectory(
    scenario: DynamicScenario,
    raw: TimedPath,
    envelope: DiscreteExecutionEnvelope,
    *,
    turn_scale_m: float = 60.0,
    sample_spacing_m: float = 2.0,
) -> PredictiveSmoothingResult:
    """Compare safe local curves instead of accepting the first qualified radius.

    Height knots, ordered fly-through anchors and real waits remain constraints.
    Only independently collision/kinematic-qualified results can win. Minimize
    execution duration with a five-second cost per meaningful reversal, then length.
    This is bounded local refinement, not an optimal-control certificate.
    """
    candidates = []
    for factor in (1.0, 0.75, 0.5, 0.25):
        candidate = smooth_predictive_timed_path(
            scenario,
            raw,
            requested_radius_m=turn_scale_m * factor,
            sample_spacing_m=sample_spacing_m,
            max_speed_mps=envelope.max_speed_mps,
            execution_envelope=envelope,
            shortcut=True,
            preserve_altitude=True,
            curve_method="bspline",
            schedule_dynamic_waits=True,
            round_reversals=True,
        )
        # The public protocol declares the upper bound, not the winning local radius.
        candidates.append(replace(candidate, requested_radius_m=turn_scale_m))

    def cost(candidate: PredictiveSmoothingResult) -> tuple[float, float, float]:
        path = candidate.execution_candidate
        if path is None:
            return (math.inf, math.inf, math.inf)
        return (
            path.duration_s + 5 * horizontal_reversals(path),
            horizontal_reversals(path),
            math.fsum(distance(a.position, b.position) for a, b in pairwise(path.waypoints)),
        )

    return min(candidates, key=cost)
