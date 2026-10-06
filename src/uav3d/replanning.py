"""Deterministic receding-horizon simulation for dynamic 3D planning."""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from uav3d.collision import point_is_free, segment_is_free
from uav3d.dynamic import DynamicScenario, dynamic_scenario_fingerprint, snapshot_scene
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import Point3, almost_equal, distance, lerp, polyline_length
from uav3d.horizontal_curves import smooth_horizontal_curves
from uav3d.planners import (
    AStar3D,
    AStarConfig,
    DStarLite3D,
    DStarLiteConfig,
    LazyThetaStar,
    LazyThetaStarConfig,
)
from uav3d.planners.base import Planner, PlanningResult
from uav3d.scene import Scene
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


def list_replanning_algorithms() -> tuple[str, ...]:
    return REPLANNING_ALGORITHMS


def _planner(
    algorithm: str, resolution: float, max_expansions: int, incremental: DStarLite3D | None
) -> Planner:
    if algorithm == "repeated-astar-3d":
        return AStar3D(AStarConfig(resolution=resolution, max_expansions=max_expansions))
    if algorithm == "repeated-lazy-theta-star":
        return LazyThetaStar(
            LazyThetaStarConfig(resolution=resolution, max_expansions=max_expansions)
        )
    if algorithm == "dstar-lite-3d":
        return incremental or DStarLite3D(
            DStarLiteConfig(resolution=resolution, max_queue_pops=max_expansions)
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
        traveled = min(segment_length, remaining_budget)
        fraction = traveled / segment_length
        endpoint = target if fraction >= 1.0 - 1e-12 else lerp(current, target, fraction)
        segment_duration = traveled / cruise_speed
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
        if remaining_budget <= 1e-12:
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


def _horizontal_escape(
    scenario: DynamicScenario,
    position: Point3,
    time_s: float,
    duration: float,
    cruise_speed: float,
) -> tuple[Point3, ...]:
    """Locally observed traffic can require movement instead of an unsafe hover.

    A swept-volume snapshot can already contain the current position while the
    actual aircraft is still approaching. In that case no static search can
    escape the snapshot. Test bounded, level avoidance primitives against exact
    relative motion, including a full safe hover at the new position. This uses
    only the next two control steps, not the entire future mission schedule.
    """
    goal = scenario.static_scene.goal
    heading = math.atan2(goal[1] - position[1], goal[0] - position[0])
    candidates: list[tuple[float, Point3]] = []
    for scale in (1.0, 0.5):
        span = cruise_speed * duration * scale
        for index in range(32):
            angle = heading + index * math.tau / 32
            endpoint = (
                position[0] + span * math.cos(angle),
                position[1] + span * math.sin(angle),
                position[2],
            )
            if not spacetime_segment_is_free(
                scenario, position, endpoint, time_s, time_s + duration
            ):
                continue
            if not spacetime_segment_is_free(
                scenario, endpoint, endpoint, time_s + duration, time_s + duration * 2
            ):
                continue
            separations = [
                distance(endpoint, aircraft.position_at(time_s + duration * 2))
                - aircraft.radius
                - scenario.static_scene.required_clearance
                for aircraft in scenario.moving_spheres
            ]
            progress = distance(position, goal) - distance(endpoint, goal)
            clearance = min(separations, default=80.0)
            candidates.append((progress + 2 * min(80, clearance), endpoint))
    if not candidates:
        return ()
    return (position, max(candidates, key=lambda item: item[0])[1])


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
    initial_heading: tuple[float, float] | None = None,
    arrival_heading: tuple[float, float] | None = None,
    allow_horizontal_escape: bool = False,
) -> DynamicRun:
    """Execute a deterministic online replanning run with an exact dynamic safety gate."""

    numeric = (time_step, replan_interval, cruise_speed, max_time, resolution)
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
    if not all(
        math.isfinite(value) and value > 0 for value in (turn_scale_m, curve_sample_spacing_m)
    ):
        raise ValueError("turn scale and curve spacing must be finite and positive")
    if algorithm not in REPLANNING_ALGORITHMS:
        choices = ", ".join(REPLANNING_ALGORITHMS)
        raise ValueError(f"unknown replanning algorithm {algorithm!r}; choose one of: {choices}")

    incremental = (
        DStarLite3D(DStarLiteConfig(resolution=resolution, max_queue_pops=max_expansions))
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
    if allow_horizontal_escape:
        parameters["horizontal_escape"] = 1
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
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-8:
                    heading = (b[0] - a[0], b[1] - a[1])
                    break
            return smooth_horizontal_curves(
                base.path,
                base.altitude_progress,
                lambda a, b, _u, _v: segment_is_free(snapshot, a, b),
                turn_scale_m=turn_scale_m,
                sample_spacing_m=curve_sample_spacing_m,
                start_direction=heading,
                end_direction=arrival_heading,
            ).points
        return (
            tuple(farthest_visible_shortcut(snapshot, path, preserve_altitude=preserve_altitude))
            if shortcut_paths
            else path
        )

    while time_s < max_time - 1e-12:
        guard = (
            snapshot_scene(scenario, time_s, start=position, lookahead_s=planning_guard_s)
            if planning_guard_s
            else None
        )
        replanned = False
        replan_reason: str | None = None
        planner_success: bool | None = None
        frame_work = 0
        frame_changed_edges = 0

        scheduled = time_s + 1e-12 >= next_replan
        missing_plan = len(current_path) < 2
        if scheduled or missing_plan:
            replan_reason = "initial" if replans == 0 else ("scheduled" if scheduled else "no-plan")
            planner = _planner(algorithm, resolution, max_expansions, incremental)
            snapshot = guard or snapshot_scene(scenario, time_s, start=position)
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
                path = prepare_path(snapshot, result.path)
                current_path = _trim_path(path, position)
            else:
                current_path = ()
                failed_replans += 1

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
            current_path, time_s, step_duration, cruise_speed
        )
        safe = (
            bool(traversals)
            and _safe_traversals(scenario, traversals)
            and (
                guard is None
                or all(segment_is_free(guard, segment.start, segment.end) for segment in traversals)
            )
        )
        if traversals and not safe:
            safety_activations += 1
            # Replan once; the exact dynamic gate still owns the final decision.
            planner = _planner(algorithm, resolution, max_expansions, incremental)
            snapshot = guard or snapshot_scene(scenario, time_s, start=position)
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
                path = prepare_path(snapshot, result.path)
                current_path = _trim_path(path, position)
                traversals, remaining_path, movement_duration = _candidate_traversal(
                    current_path, time_s, step_duration, cruise_speed
                )
                safe = (
                    bool(traversals)
                    and _safe_traversals(scenario, traversals)
                    and (
                        guard is None
                        or all(
                            segment_is_free(guard, segment.start, segment.end)
                            for segment in traversals
                        )
                    )
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
                not point_is_free(guard, position)
                or not spacetime_segment_is_free(
                    scenario, position, position, time_s, time_s + step_duration
                )
            )
        ):
            escape = _horizontal_escape(scenario, position, time_s, step_duration, cruise_speed)
            if escape:
                current_path = escape
                traversals, remaining_path, movement_duration = _candidate_traversal(
                    current_path, time_s, step_duration, cruise_speed
                )
                safe = bool(traversals) and _safe_traversals(scenario, traversals)
                next_replan = time_s + step_duration
                replan_reason = "safety-gate"
                safety_activations += 1

        if safe:
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
    )


def _changed_edges(result: PlanningResult) -> int:
    value = result.parameters.get("changed_edges", 0)
    return value if isinstance(value, int) and not isinstance(value, bool) else 0


__all__ = [
    "REPLANNING_ALGORITHMS",
    "DynamicFrame",
    "DynamicMetrics",
    "DynamicRun",
    "list_replanning_algorithms",
    "simulate_replanning",
]
