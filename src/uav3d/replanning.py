"""Deterministic receding-horizon simulation for dynamic 3D planning."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, replace
from itertools import pairwise

from uav3d.collision import path_is_free, point_is_free, segment_is_free
from uav3d.dynamic import (
    DynamicScenario,
    MovingSphere,
    dynamic_scenario_fingerprint,
    snapshot_scene,
)
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.flight_cost import segment_flight_time, validate_vertical_scale, with_turn_clearance
from uav3d.geometry import Point3, almost_equal, distance, lerp, polyline_length
from uav3d.horizontal_curves import cast_vector, smooth_curves
from uav3d.planners import (
    AStar3D,
    AStarConfig,
    DStarLite3D,
    DStarLiteConfig,
    LazyThetaStar,
    LazyThetaStarConfig,
)
from uav3d.planners.base import Planner, PlanningResult
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.scene import Cylinder, Scene
from uav3d.smoothing import farthest_visible_shortcut, smooth_path

REPLANNING_ALGORITHMS = (
    "repeated-astar-3d",
    "repeated-lazy-theta-star",
    "dstar-lite-3d",
)


@dataclass(frozen=True, slots=True)
class DynamicFrame:
    time_s: float
    position: Point3
    status: str
    planned_path: tuple[Point3, ...]
    replanned: bool = False
    replan_reason: str | None = None
    planner_success: bool | None = None
    planning_work: int = 0
    changed_edges: int = 0

    def to_dict(self) -> dict[str, object]:
        return {
            "time_s": self.time_s,
            "position": list(self.position),
            "status": self.status,
            "planned_path": [list(point) for point in self.planned_path],
            "replanned": self.replanned,
            "replan_reason": self.replan_reason,
            "planner_success": self.planner_success,
            "planning_work": self.planning_work,
            "changed_edges": self.changed_edges,
        }


@dataclass(frozen=True, slots=True)
class DynamicMetrics:
    success: bool
    failure_reason: str | None
    completion_time_s: float | None
    executed_path_length_m: float
    direct_distance_m: float
    path_excess_ratio: float | None
    replans: int
    failed_replans: int
    holds: int
    safety_gate_activations: int
    collision_count: int
    total_planning_work: int
    total_changed_edges: int

    def to_dict(self) -> dict[str, object]:
        return {
            "success": self.success,
            "failure_reason": self.failure_reason,
            "completion_time_s": self.completion_time_s,
            "executed_path_length_m": self.executed_path_length_m,
            "direct_distance_m": self.direct_distance_m,
            "path_excess_ratio": self.path_excess_ratio,
            "replans": self.replans,
            "failed_replans": self.failed_replans,
            "holds": self.holds,
            "safety_gate_activations": self.safety_gate_activations,
            "collision_count": self.collision_count,
            "total_planning_work": self.total_planning_work,
            "total_changed_edges": self.total_changed_edges,
        }


@dataclass(frozen=True, slots=True)
class DynamicRun:
    scenario_id: str
    scenario_fingerprint: str
    algorithm: str
    parameters: dict[str, float | int]
    frames: tuple[DynamicFrame, ...]
    metrics: DynamicMetrics
    execution_timed_path: TimedPath | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": "dynamic-run-v1",
            "scenario_id": self.scenario_id,
            "scenario_fingerprint": self.scenario_fingerprint,
            "algorithm": self.algorithm,
            "parameters": self.parameters,
            "frames": [frame.to_dict() for frame in self.frames],
            "metrics": self.metrics.to_dict(),
        }


@dataclass(frozen=True, slots=True)
class _Traversal:
    start: Point3
    end: Point3
    start_time: float
    end_time: float


@dataclass(slots=True)
class _TrafficEncounter:
    aircraft: MovingSphere
    volume: Cylinder
    forward: tuple[float, float]
    origin: Point3
    clear_since: float | None = None
    side: float = 0.0
    side_limit: float = -math.inf

    def active(
        self,
        position: Point3,
        velocity: Point3,
        clock: float,
        step: float,
        clearance: float,
        observed: Cylinder,
    ) -> bool:
        centre = self.aircraft.position_at(clock)
        previous = self.aircraft.position_at(max(0.0, clock - step))
        interval = min(step, clock)
        relative = tuple(position[i] - centre[i] for i in range(3))
        relative_velocity = tuple(
            velocity[i] - ((centre[i] - previous[i]) / interval if interval > 0 else 0.0)
            for i in range(3)
        )
        safe_distance = self.aircraft.radius + clearance
        ahead = sum(relative[i] * self.forward[i] for i in range(2)) > safe_distance
        separating = sum(relative[i] * relative_velocity[i] for i in range(3)) > 1e-6
        lateral = self.forward[0] * (centre[1] - self.origin[1]) - self.forward[1] * (
            centre[0] - self.origin[0]
        )
        observed_lateral = self.forward[0] * (observed.center[1] - self.origin[1]) - self.forward[
            1
        ] * (observed.center[0] - self.origin[0])
        lateral_motion = self.forward[0] * (centre[1] - previous[1]) - self.forward[1] * (
            centre[0] - previous[0]
        )
        corridor_clear = (
            abs(observed_lateral) > observed.radius + clearance and lateral * lateral_motion > 1e-6
        )
        vertical_clear = (position[2] > observed.z_max + clearance and velocity[2] >= 0) or (
            position[2] < observed.z_min - clearance and velocity[2] <= 0
        )
        clear = math.hypot(*relative) > safe_distance and (
            (ahead and separating) or corridor_clear or vertical_clear
        )
        if clear and ahead and separating:
            # Once both aircraft have passed and are separating, a lateral
            # debounce must not delay an already unambiguous release.
            self.clear_since = clock - 2 * step
        elif not clear:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = clock
        return self.clear_since is None or clock - self.clear_since < 2 * step - 1e-9


def _planner(
    algorithm: str,
    resolution: float,
    max_expansions: int,
    incremental: DStarLite3D | None,
    vertical_cost_scale: float = 1.0,
    altitude_levels: tuple[float, ...] = (),
) -> Planner:
    if algorithm == "repeated-astar-3d":
        return AStar3D(
            AStarConfig(
                resolution=resolution,
                max_expansions=max_expansions,
                vertical_cost_scale=vertical_cost_scale,
                altitude_levels=altitude_levels,
            )
        )
    if algorithm == "repeated-lazy-theta-star":
        return LazyThetaStar(
            LazyThetaStarConfig(
                resolution=resolution,
                max_expansions=max_expansions,
                vertical_cost_scale=vertical_cost_scale,
                altitude_levels=altitude_levels,
            )
        )
    if algorithm == "dstar-lite-3d":
        return incremental or DStarLite3D(
            DStarLiteConfig(
                resolution=resolution,
                max_queue_pops=max_expansions,
                vertical_cost_scale=vertical_cost_scale,
                altitude_levels=altitude_levels,
            )
        )
    choices = ", ".join(REPLANNING_ALGORITHMS)
    raise ValueError(f"unknown replanning algorithm {algorithm!r}; choose one of: {choices}")


def _trim_path(path: tuple[Point3, ...], position: Point3) -> tuple[Point3, ...]:
    if not path:
        return ()
    if almost_equal(path[0], position):
        return (position, *path[1:])
    return (position, *path)


def _candidate_traversal(
    path: tuple[Point3, ...],
    start_time: float,
    duration: float,
    cruise_speed: float,
    max_climb_rate: float | None = None,
) -> tuple[tuple[_Traversal, ...], tuple[Point3, ...], float]:
    """Advance along a polyline without replacing bends by an unsafe chord."""

    if len(path) < 2:
        return ((), path, 0.0)
    distance_budget = duration * cruise_speed
    remaining_budget = distance_budget
    elapsed = 0.0
    traversals: list[_Traversal] = []
    current = path[0]
    for index, (_, target) in enumerate(pairwise(path)):
        segment_length = distance(current, target)
        if segment_length <= 1e-12:
            current = target
            continue
        full_duration = segment_flight_time(current, target, cruise_speed, max_climb_rate)
        traveled = (
            min(segment_length, remaining_budget)
            if max_climb_rate is None
            else segment_length * min(1.0, (duration - elapsed) / full_duration)
        )
        fraction = traveled / segment_length
        endpoint = target if fraction >= 1.0 - 1e-12 else lerp(current, target, fraction)
        segment_duration = (
            traveled / cruise_speed if max_climb_rate is None else full_duration * fraction
        )
        traversals.append(
            _Traversal(
                current,
                endpoint,
                start_time + elapsed,
                start_time + elapsed + segment_duration,
            )
        )
        elapsed += segment_duration
        remaining_budget -= traveled
        if fraction < 1.0 - 1e-12:
            return (tuple(traversals), (endpoint, *path[index + 1 :]), elapsed)
        current = target
        if remaining_budget <= 1e-12 or elapsed >= duration - 1e-12:
            return (tuple(traversals), (current, *path[index + 2 :]), elapsed)
    return (tuple(traversals), (current,), elapsed)


def _safe_traversals(scenario: DynamicScenario, traversals: tuple[_Traversal, ...]) -> bool:
    return all(
        spacetime_segment_is_free(
            scenario,
            traversal.start,
            traversal.end,
            traversal.start_time,
            traversal.end_time,
        )
        for traversal in traversals
    )


def _path_duration(
    path: tuple[Point3, ...], cruise_speed: float, max_climb_rate: float | None
) -> float:
    return math.fsum(
        segment_flight_time(a, b, cruise_speed, max_climb_rate) for a, b in pairwise(path)
    )


def _horizontal_escape(
    scenario: DynamicScenario,
    position: Point3,
    time_s: float,
    duration: float,
    cruise_speed: float,
    start_heading: tuple[float, ...] | None = None,
    max_climb_rate: float | None = None,
    lookahead_s: float = 0.0,
    preferred_direction: tuple[float, float] | None = None,
    continuation_check: Callable[[tuple[Point3, ...]], bool] | None = None,
) -> tuple[Point3, ...]:
    """Locally observed traffic can require movement instead of an unsafe hover.

    A swept-volume snapshot can already contain the current position while the
    actual aircraft is still approaching. In that case no static search can
    escape the snapshot. Test bounded avoidance toward level targets against exact
    relative motion. Prefer a complete local manoeuvre over a one-step dodge;
    only the local observation window is used, never the full mission schedule.
    """
    goal = scenario.static_scene.goal
    heading = math.atan2(goal[1] - position[1], goal[0] - position[0])
    horizon = max(duration, lookahead_s)
    candidates: list[tuple[bool, float, tuple[Point3, ...]]] = []
    for scale in (1.0, 0.5):
        span = cruise_speed * horizon * scale
        for index in range(32):
            angle = heading + index * math.tau / 32
            endpoint = (
                position[0] + span * math.cos(angle),
                position[1] + span * math.sin(angle),
                position[2],
            )
            path: tuple[Point3, ...] = (position, endpoint)
            if start_heading is not None:
                # Geometry proposal only. The exact, arc-length flight clock
                # and every executed chord are certified immediately below.
                path = smooth_curves(
                    path,
                    (0.0, 1.0),
                    lambda *_: True,
                    start_direction=start_heading,
                    turn_scale_m=60,
                    sample_spacing_m=1,
                    round_reversals=True,
                    preserve_altitude=len(start_heading) < 3,
                    max_altitude_deviation_m=12,
                    boundary_height_excursion=True,
                ).points
            speed = min(cruise_speed, polyline_length(path) / duration)
            travel_time = _path_duration(path, speed, max_climb_rate)
            traversals, _, _ = _candidate_traversal(
                path, time_s, max(duration, travel_time), speed, max_climb_rate
            )
            if not traversals or not _safe_traversals(scenario, traversals):
                continue
            reached = traversals[-1].end
            if not spacetime_segment_is_free(
                scenario,
                reached,
                reached,
                traversals[-1].end_time,
                max(traversals[-1].end_time, time_s + horizon) + duration,
            ):
                continue
            separations = [
                distance(reached, aircraft.position_at(traversals[-1].end_time))
                - aircraft.radius
                - scenario.static_scene.required_clearance
                for aircraft in scenario.moving_spheres
            ]
            progress = distance(position, goal) - distance(reached, goal)
            clearance = min(separations, default=80.0)
            turn_cost = 0.0
            if start_heading is not None and math.hypot(*start_heading[:2]) > 1e-8:
                cosine = (
                    math.cos(angle) * start_heading[0] + math.sin(angle) * start_heading[1]
                ) / math.hypot(*start_heading[:2])
                turn_cost = cruise_speed * duration * (1 - cosine)
            if preferred_direction is not None:
                cosine = (
                    math.cos(angle) * preferred_direction[0]
                    + math.sin(angle) * preferred_direction[1]
                ) / math.hypot(*preferred_direction)
                turn_cost += 0.5 * cruise_speed * horizon * (1 - cosine)
            candidates.append(
                (
                    continuation_check is None or continuation_check(path),
                    progress + min(80, clearance) - turn_cost,
                    path,
                )
            )
    if not candidates:
        if horizon > duration:
            return _horizontal_escape(
                scenario,
                position,
                time_s,
                duration,
                cruise_speed,
                start_heading,
                max_climb_rate,
                preferred_direction=preferred_direction,
                continuation_check=continuation_check,
            )
        return ()
    return max(candidates, key=lambda item: item[:2])[2]


def simulate_replanning(
    scenario: DynamicScenario,
    algorithm: str,
    *,
    time_step: float = 1.0,
    replan_interval: float = 4.0,
    cruise_speed: float = 8.0,
    max_time: float = 180.0,
    resolution: float = 4.0,
    max_expansions: int = 120_000,
    reuse_search_state: bool = True,
    shortcut_paths: bool = False,
    preserve_altitude: bool = False,
    start_time: float = 0.0,
    planning_guard_s: float = 0.0,
    smooth_turns: bool = False,
    turn_scale_m: float = 60.0,
    curve_sample_spacing_m: float = 2.0,
    initial_heading: tuple[float, ...] | None = None,
    arrival_heading: tuple[float, ...] | None = None,
    spatial_curves: bool = False,
    allow_horizontal_escape: bool = False,
    vertical_cost_scale: float = 1.0,
    altitude_levels: tuple[float, ...] = (),
    max_climb_rate: float | None = None,
) -> DynamicRun:
    """Execute a deterministic online replanning run with an exact dynamic safety gate."""

    numeric = (time_step, replan_interval, cruise_speed, max_time, resolution)
    validate_vertical_scale(vertical_cost_scale)
    if max_climb_rate is not None and (not math.isfinite(max_climb_rate) or max_climb_rate <= 0):
        raise ValueError("max_climb_rate must be finite and positive")
    if not all(math.isfinite(value) and value > 0 for value in numeric):
        raise ValueError("simulation times, speed, and resolution must be finite and positive")
    if max_expansions <= 0:
        raise ValueError("max_expansions must be positive")
    if not math.isfinite(start_time) or not 0 <= start_time < max_time:
        raise ValueError("start_time must be finite and lie before max_time")
    if not math.isfinite(planning_guard_s) or planning_guard_s < 0:
        raise ValueError("planning_guard_s must be finite and non-negative")
    if smooth_turns and not preserve_altitude:
        raise ValueError("horizontal turn smoothing requires altitude preservation")
    if spatial_curves and not smooth_turns:
        raise ValueError("spatial curves require smooth_turns")
    if not all(
        math.isfinite(value) and value > 0 for value in (turn_scale_m, curve_sample_spacing_m)
    ):
        raise ValueError("turn scale and curve spacing must be finite and positive")
    if algorithm not in REPLANNING_ALGORITHMS:
        choices = ", ".join(REPLANNING_ALGORITHMS)
        raise ValueError(f"unknown replanning algorithm {algorithm!r}; choose one of: {choices}")

    incremental = (
        DStarLite3D(
            DStarLiteConfig(
                resolution=resolution,
                max_queue_pops=max_expansions,
                vertical_cost_scale=vertical_cost_scale,
                altitude_levels=altitude_levels,
            )
        )
        if algorithm == "dstar-lite-3d" and reuse_search_state
        else None
    )
    parameters: dict[str, float | int] = {
        "time_step": time_step,
        "replan_interval": replan_interval,
        "cruise_speed": cruise_speed,
        "max_time": max_time,
        "resolution": resolution,
        "max_expansions": max_expansions,
    }
    # Preserve the byte-for-byte v0.3 default contract while making the new reset ablation
    # self-describing when callers explicitly disable incremental-state reuse.
    if algorithm == "dstar-lite-3d" and not reuse_search_state:
        parameters["reuse_search_state"] = 0
    if shortcut_paths:
        parameters["path_shortcut"] = 1
    if preserve_altitude:
        parameters["preserve_altitude"] = 1
    if start_time:
        parameters["start_time"] = start_time
    if planning_guard_s:
        parameters["planning_guard_s"] = planning_guard_s
    if smooth_turns:
        parameters.update(
            horizontal_curve_degree=5,
            turn_scale_m=turn_scale_m,
            curve_sample_spacing_m=curve_sample_spacing_m,
        )
        if spatial_curves:
            parameters["curve_dimensions"] = 3
    if allow_horizontal_escape:
        parameters["horizontal_escape"] = 1
    if vertical_cost_scale != 1:
        parameters["vertical_cost_scale"] = vertical_cost_scale
    if max_climb_rate is not None:
        parameters["max_climb_rate"] = max_climb_rate
    exact_trace = [TimedWaypoint(start_time, scenario.static_scene.start, "start")]
    frames: list[DynamicFrame] = []
    executed: list[Point3] = [scenario.static_scene.start]
    position = scenario.static_scene.start
    current_path: tuple[Point3, ...] = ()
    time_s = start_time
    next_replan = start_time
    replans = 0
    failed_replans = 0
    holds = 0
    safety_activations = 0
    collision_count = 0
    total_work = 0
    total_changed_edges = 0
    failure_reason: str | None = None
    avoidance_active = False
    avoidance_heading: tuple[float, float] | None = None
    encounters: dict[str, _TrafficEncounter] = {}
    active_encounters: list[_TrafficEncounter] = []
    local_horizon = max(time_step * 2, planning_guard_s)
    release_traffic = replace(
        scenario, static_scene=replace(scenario.static_scene, buildings=(), no_fly_zones=())
    )

    def observed_snapshot(snapshot: Scene) -> Scene:
        """Remember a blocking traffic footprint until the encounter is passed.

        Both ordinary successful detours and emergency escapes need this state;
        an instantaneous shorter path is not evidence that the aircraft has passed.
        Only locally observed swept volumes and past relative motion are used.
        """
        velocity = (0.0, 0.0, 0.0)
        if frames and time_s > frames[-1].time_s:
            velocity = cast_vector(
                tuple(
                    (position[i] - frames[-1].position[i]) / (time_s - frames[-1].time_s)
                    for i in range(3)
                )
            )
        prefix, _, _ = _candidate_traversal(
            current_path or (position, snapshot.goal),
            time_s,
            local_horizon * 2,
            cruise_speed,
            max_climb_rate,
        )
        nominal, _, _ = _candidate_traversal(
            (position, snapshot.goal), time_s, local_horizon * 2, cruise_speed, max_climb_rate
        )
        zones = {zone.zone_id: zone for zone in snapshot.no_fly_zones}
        for aircraft in scenario.moving_spheres:
            if aircraft.sphere_id in encounters:
                continue
            zone = zones.get(aircraft.sphere_id)
            if zone is None:
                continue
            local = replace(release_traffic.static_scene, no_fly_zones=(zone,))
            if any(not segment_is_free(local, leg.start, leg.end) for leg in (*prefix, *nominal)):
                direction = tuple(snapshot.goal[i] - position[i] for i in range(2))
                length = math.hypot(*direction)
                if length > 1e-8:
                    encounters[aircraft.sphere_id] = _TrafficEncounter(
                        aircraft,
                        replace(zone, zone_id=f"avoidance:{aircraft.sphere_id}"),
                        (direction[0] / length, direction[1] / length),
                        position,
                    )
        active_encounters[:] = [
            e
            for e in encounters.values()
            if e.active(
                position,
                velocity,
                time_s,
                time_step,
                snapshot.required_clearance + min(turn_scale_m / 4, cruise_speed * time_step / 2),
                zones[e.aircraft.sphere_id],
            )
        ]
        for encounter in active_encounters:
            if encounter.side:
                volume = zones[encounter.aircraft.sphere_id]
                encounter.side_limit = max(
                    encounter.side_limit,
                    encounter.side
                    * (
                        encounter.forward[0] * volume.center[1]
                        - encounter.forward[1] * volume.center[0]
                    )
                    + volume.radius
                    + snapshot.required_clearance,
                )
        remembered = []
        for encounter in active_encounters:
            local = replace(release_traffic.static_scene, no_fly_zones=(encounter.volume,))
            # Remembered geometry must not trap a vehicle already inside a sweep
            # or make a required rooftop unreachable. Exact escape checks still apply.
            if point_is_free(local, position) and point_is_free(local, snapshot.goal):
                remembered.append(encounter.volume)
        return replace(snapshot, no_fly_zones=(*snapshot.no_fly_zones, *remembered))

    def clears_remembered(path: tuple[Point3, ...]) -> bool:
        return all(
            path_is_free(replace(release_traffic.static_scene, no_fly_zones=(e.volume,)), path)
            for e in active_encounters
        )

    def keeps_passing_side(path: tuple[Point3, ...]) -> bool:
        traversals, _, _ = _candidate_traversal(
            path, time_s, local_horizon, cruise_speed, max_climb_rate
        )
        for encounter in active_encounters:
            if not encounter.side:
                continue
            axis = encounter.forward
            clearance = (
                encounter.aircraft.radius
                + scenario.static_scene.required_clearance
                + min(turn_scale_m / 4, cruise_speed * time_step / 2)
            )
            offset = encounter.side * (axis[0] * position[1] - axis[1] * position[0])
            minimum = min(offset, encounter.side_limit) - 2.0
            for leg in traversals:
                centre = encounter.aircraft.position_at(leg.end_time)
                relative = (leg.end[0] - centre[0], leg.end[1] - centre[1])
                if (
                    sum(axis[i] * relative[i] for i in range(2)) < clearance
                    and encounter.side * (axis[0] * leg.end[1] - axis[1] * leg.end[0]) < minimum
                ):
                    return False
        return True

    def remember_side(path: tuple[Point3, ...]) -> None:
        for encounter in active_encounters:
            if encounter.side or not path:
                continue
            axis = encounter.forward
            centre = encounter.aircraft.position_at(time_s)
            crossings = list(path)
            for a, b in pairwise(path):
                start = sum((a[i] - centre[i]) * axis[i] for i in range(2))
                change = sum((b[i] - a[i]) * axis[i] for i in range(2))
                if abs(change) > 1e-8:
                    crossings.append(lerp(a, b, max(0.0, min(1.0, -start / change))))
            nearest = min(
                crossings, key=lambda p: abs(sum((p[i] - centre[i]) * axis[i] for i in range(2)))
            )
            lateral = axis[0] * (nearest[1] - centre[1]) - axis[1] * (nearest[0] - centre[0])
            if abs(lateral) > 2:
                encounter.side = math.copysign(1.0, lateral)
                encounter.side_limit = (
                    encounter.side
                    * (axis[0] * encounter.volume.center[1] - axis[1] * encounter.volume.center[0])
                    + encounter.volume.radius
                    + scenario.static_scene.required_clearance
                )

    def safe_prefix(path: tuple[Point3, ...], *, returning: bool = False) -> bool:
        traversals, _, _ = _candidate_traversal(
            path, time_s, local_horizon, cruise_speed, max_climb_rate
        )
        # A wider release envelope prevents returning toward the same traffic
        # immediately after barely clearing its collision boundary.
        clearance = scenario.static_scene.required_clearance + (
            min(turn_scale_m / 4, cruise_speed * time_step / 2) if returning else 0.0
        )
        return bool(traversals) and all(
            spacetime_segment_is_free(
                release_traffic if returning else scenario,
                leg.start,
                leg.end,
                leg.start_time,
                leg.end_time,
                clearance=clearance,
            )
            for leg in traversals
        )

    def is_escape(path: tuple[Point3, ...]) -> bool:
        return bool(path) and not almost_equal(path[-1], scenario.static_scene.goal, 1e-8)

    def prepare_path(snapshot: Scene, path: tuple[Point3, ...]) -> tuple[Point3, ...]:
        if smooth_turns:
            if len(path) < 2:
                return path
            base = smooth_path(
                snapshot,
                path,
                optimize_shortcuts=shortcut_paths,
                preserve_altitude=True,
                round_corners=False,
            )
            heading = initial_heading
            for a, b in reversed(list(pairwise(executed))):
                vector = tuple(b[i] - a[i] for i in range(3 if spatial_curves else 2))
                if math.hypot(*vector) > 1e-8:
                    heading = vector
                    break
            return smooth_curves(
                base.path,
                base.altitude_progress,
                lambda a, b, _u, _v: segment_is_free(snapshot, a, b),
                turn_scale_m=turn_scale_m,
                sample_spacing_m=curve_sample_spacing_m,
                start_direction=cast_vector(heading) if heading is not None else None,
                end_direction=cast_vector(arrival_heading) if arrival_heading is not None else None,
                round_reversals=vertical_cost_scale > 1,
                preserve_altitude=not spatial_curves,
                max_altitude_deviation_m=12,
                boundary_height_excursion=spatial_curves,
            ).points
        return (
            tuple(farthest_visible_shortcut(snapshot, path, preserve_altitude=preserve_altitude))
            if shortcut_paths
            else path
        )

    def choose_path(snapshot: Scene, path: tuple[Point3, ...]) -> tuple[Point3, ...]:
        nonlocal avoidance_active, avoidance_heading
        candidate = _trim_path(prepare_path(snapshot, path), position)
        returning_early = bool(active_encounters) and (
            not clears_remembered(candidate) or not keeps_passing_side(candidate)
        )
        if (avoidance_active or active_encounters) and (
            returning_early or not safe_prefix(candidate, returning=True)
        ):
            avoidance_active = True
            if (
                _path_duration(current_path, cruise_speed, max_climb_rate) > time_step + 1e-8
                and safe_prefix(current_path)
                and (is_escape(current_path) or keeps_passing_side(current_path))
            ):
                return current_path
            return ()
        remember_side(candidate)
        avoidance_active = bool(active_encounters)
        if not avoidance_active:
            avoidance_heading = None
        if (
            smooth_turns
            and len(current_path) > 1
            and almost_equal(current_path[-1], snapshot.goal, 1e-8)
            and path_is_free(snapshot, current_path)
            # Grid re-anchoring must save more than one control step before
            # replacing an already safe, tangent-connected continuation.
            and _path_duration(candidate, cruise_speed, max_climb_rate)
            >= _path_duration(current_path, cruise_speed, max_climb_rate) - max(0.25, time_step)
        ):
            return current_path
        return candidate

    while time_s < max_time - 1e-12:
        guard = (
            snapshot_scene(scenario, time_s, start=position, lookahead_s=planning_guard_s)
            if planning_guard_s
            else None
        )
        if guard is not None and smooth_turns and allow_horizontal_escape:
            guard = observed_snapshot(guard)
        replanned = False
        replan_reason: str | None = None
        planner_success: bool | None = None
        frame_work = 0
        frame_changed_edges = 0

        scheduled = time_s + 1e-12 >= next_replan
        local_avoidance = is_escape(current_path)
        missing_plan = len(current_path) < 2 or (
            local_avoidance
            and _path_duration(current_path, cruise_speed, max_climb_rate) <= time_step + 1e-8
        )
        if scheduled or missing_plan:
            replan_reason = "initial" if replans == 0 else ("scheduled" if scheduled else "no-plan")
            planner = _planner(
                algorithm,
                resolution,
                max_expansions,
                incremental,
                vertical_cost_scale,
                altitude_levels,
            )
            snapshot = guard or snapshot_scene(scenario, time_s, start=position)
            if vertical_cost_scale > 1:
                snapshot = with_turn_clearance(snapshot)
            result = planner.plan(snapshot)
            replans += 1
            replanned = True
            planner_success = result.success
            frame_work = result.expanded_nodes
            frame_changed_edges = _changed_edges(result)
            total_work += frame_work
            total_changed_edges += frame_changed_edges
            next_replan = time_s + replan_interval
            if result.success:
                current_path = choose_path(snapshot, result.path)
            else:
                failed_replans += 1
                if not (
                    smooth_turns
                    and current_path
                    and almost_equal(current_path[-1], snapshot.goal, 1e-8)
                    and path_is_free(snapshot, current_path)
                ):
                    current_path = ()

        if almost_equal(position, scenario.static_scene.goal, 1e-8):
            frames.append(
                DynamicFrame(
                    time_s,
                    position,
                    "arrived",
                    current_path,
                    replanned,
                    replan_reason,
                    planner_success,
                    frame_work,
                    frame_changed_edges,
                )
            )
            break

        step_duration = min(time_step, max_time - time_s)
        traversals, remaining_path, movement_duration = _candidate_traversal(
            current_path, time_s, step_duration, cruise_speed, max_climb_rate
        )
        safe = (
            bool(traversals)
            and _safe_traversals(scenario, traversals)
            and (
                guard is None
                # A swept snapshot may contain the aircraft itself. A committed
                # escape is checked on its actual clock, again at every step.
                or is_escape(current_path)
                or all(segment_is_free(guard, segment.start, segment.end) for segment in traversals)
            )
            and (not allow_horizontal_escape or not smooth_turns or safe_prefix(current_path))
            and (is_escape(current_path) or keeps_passing_side(current_path))
        )
        if traversals and not safe:
            safety_activations += 1
            # Replan once; the exact dynamic gate still owns the final decision.
            planner = _planner(
                algorithm,
                resolution,
                max_expansions,
                incremental,
                vertical_cost_scale,
                altitude_levels,
            )
            snapshot = guard or snapshot_scene(scenario, time_s, start=position)
            if vertical_cost_scale > 1:
                snapshot = with_turn_clearance(snapshot)
            result = planner.plan(snapshot)
            replans += 1
            replanned = True
            replan_reason = "safety-gate"
            planner_success = result.success
            work = result.expanded_nodes
            changed = _changed_edges(result)
            frame_work += work
            frame_changed_edges += changed
            total_work += work
            total_changed_edges += changed
            if result.success:
                current_path = choose_path(snapshot, result.path)
                traversals, remaining_path, movement_duration = _candidate_traversal(
                    current_path, time_s, step_duration, cruise_speed, max_climb_rate
                )
                safe = (
                    bool(traversals)
                    and _safe_traversals(scenario, traversals)
                    and (
                        guard is None
                        or is_escape(current_path)
                        or all(
                            segment_is_free(guard, segment.start, segment.end)
                            for segment in traversals
                        )
                    )
                    and (
                        not allow_horizontal_escape or not smooth_turns or safe_prefix(current_path)
                    )
                    and (is_escape(current_path) or keeps_passing_side(current_path))
                )
            else:
                failed_replans += 1
                current_path = ()
                traversals = ()
                safe = False

        # Do not wait inside an approaching aircraft's swept volume until the
        # hold itself becomes unsafe. Defaults of frozen studies are unchanged.
        if (
            not safe
            and allow_horizontal_escape
            and guard is not None
            and (
                avoidance_active
                or bool(active_encounters)
                or (bool(traversals) and not safe_prefix(current_path))
                or not point_is_free(guard, position)
                or not spacetime_segment_is_free(
                    scenario, position, position, time_s, time_s + step_duration
                )
            )
        ):
            heading = initial_heading
            if vertical_cost_scale > 1:
                for a, b in reversed(list(pairwise(executed))):
                    vector = tuple(b[i] - a[i] for i in range(3 if spatial_curves else 2))
                    if math.hypot(*vector) > 1e-8:
                        heading = vector
                        break
            escape = _horizontal_escape(
                scenario,
                position,
                time_s,
                step_duration,
                cruise_speed,
                start_heading=heading if vertical_cost_scale > 1 else None,
                max_climb_rate=max_climb_rate,
                lookahead_s=local_horizon if smooth_turns else 0.0,
                preferred_direction=next(
                    (
                        (
                            e.forward[0] - e.forward[1] * e.side * 0.5,
                            e.forward[1] + e.forward[0] * e.side * 0.5,
                        )
                        for e in active_encounters
                        if e.side
                    ),
                    avoidance_heading,
                ),
                continuation_check=keeps_passing_side if active_encounters else None,
            )
            if escape:
                current_path = escape
                traversals, remaining_path, movement_duration = _candidate_traversal(
                    current_path,
                    time_s,
                    step_duration,
                    min(cruise_speed, polyline_length(current_path) / step_duration),
                    max_climb_rate,
                )
                safe = bool(traversals) and _safe_traversals(scenario, traversals)
                avoidance_active = True
                if not keeps_passing_side(escape):
                    # A new hazard may make the remembered flank unavailable.
                    # Commit the certified alternative instead of fighting it
                    # again at the next control step.
                    for encounter in active_encounters:
                        encounter.side = 0.0
                remember_side(escape)
                if avoidance_heading is None:
                    direction = (escape[-1][0] - position[0], escape[-1][1] - position[1])
                    if math.hypot(*direction) > 1e-8:
                        avoidance_heading = direction
                next_replan = time_s + max(
                    step_duration,
                    min(local_horizon, _path_duration(escape, cruise_speed, max_climb_rate))
                    - step_duration,
                )
                replan_reason = "safety-gate"
                safety_activations += 1

        if safe:
            if is_escape(current_path):
                replan_reason = "safety-gate"
            frames.append(
                DynamicFrame(
                    time_s,
                    position,
                    "move",
                    current_path,
                    replanned,
                    replan_reason,
                    planner_success,
                    frame_work,
                    frame_changed_edges,
                )
            )
            for traversal in traversals:
                # This second check makes the collision metric an audit of executed motion.
                if not spacetime_segment_is_free(
                    scenario,
                    traversal.start,
                    traversal.end,
                    traversal.start_time,
                    traversal.end_time,
                ):
                    collision_count += 1
                position = traversal.end
                executed.append(position)
                if max_climb_rate is not None:
                    exact_trace.append(TimedWaypoint(traversal.end_time, position, "move"))
            current_path = remaining_path
            if almost_equal(position, scenario.static_scene.goal, 1e-8):
                time_s += movement_duration
                current_path = (scenario.static_scene.goal,)
                frames.append(DynamicFrame(time_s, position, "arrived", current_path))
                break
            time_s += step_duration
            if time_s >= max_time - 1e-12:
                frames.append(DynamicFrame(time_s, position, "timeout", current_path))
                break
            continue

        # A hold is only allowed when remaining stationary is dynamically safe for the full step.
        hold_is_safe = spacetime_segment_is_free(
            scenario, position, position, time_s, time_s + step_duration
        )
        status = "hold" if hold_is_safe else "blocked"
        frames.append(
            DynamicFrame(
                time_s,
                position,
                status,
                current_path,
                replanned,
                replan_reason,
                planner_success,
                frame_work,
                frame_changed_edges,
            )
        )
        if not hold_is_safe:
            failure_reason = "no-safe-action"
            break
        holds += 1
        time_s += step_duration
        if max_climb_rate is not None:
            exact_trace.append(TimedWaypoint(time_s, position, "wait"))
        if time_s >= max_time - 1e-12:
            frames.append(DynamicFrame(time_s, position, "timeout", current_path))
            break
        # Retry immediately after a safety hold so event boundaries are observed without delay.
        next_replan = min(next_replan, time_s)

    success = almost_equal(position, scenario.static_scene.goal, 1e-8) and collision_count == 0
    if not success and failure_reason is None:
        failure_reason = "maximum-simulation-time" if time_s >= max_time - 1e-12 else "no-path"
    direct = distance(scenario.static_scene.start, scenario.static_scene.goal)
    executed_length = polyline_length(executed)
    metrics = DynamicMetrics(
        success=success,
        failure_reason=None if success else failure_reason,
        completion_time_s=time_s if success else None,
        executed_path_length_m=executed_length,
        direct_distance_m=direct,
        path_excess_ratio=(executed_length / direct - 1.0) if success and direct > 0 else None,
        replans=replans,
        failed_replans=failed_replans,
        holds=holds,
        safety_gate_activations=safety_activations,
        collision_count=collision_count,
        total_planning_work=total_work,
        total_changed_edges=total_changed_edges,
    )
    return DynamicRun(
        scenario.scenario_id,
        dynamic_scenario_fingerprint(scenario),
        algorithm,
        parameters,
        tuple(frames),
        metrics,
        TimedPath(tuple(exact_trace)) if max_climb_rate is not None else None,
    )


def _changed_edges(result: PlanningResult) -> int:
    value = result.parameters.get("changed_edges", 0)
    return value if isinstance(value, int) and not isinstance(value, bool) else 0


__all__ = [
    "REPLANNING_ALGORITHMS",
    "DynamicFrame",
    "DynamicMetrics",
    "DynamicRun",
    "simulate_replanning",
]
