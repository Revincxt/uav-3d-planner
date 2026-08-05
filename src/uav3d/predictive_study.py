"""Fixed v0.5 protocol for complex-city predictive planning demonstrations."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path

from uav3d.dynamic import DynamicScenario, dynamic_scenario_fingerprint
from uav3d.geometry import Point3, almost_equal, distance, polyline_length
from uav3d.planners.space_time_astar import SpaceTimeAStar3D, SpaceTimeAStarConfig
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.predictive_scenarios import build_predictive_cohort, load_predictive_scenario
from uav3d.predictive_smoothing import (
    PredictiveSmoothingResult,
    smooth_predictive_timed_path,
)
from uav3d.replanning import DynamicFrame, DynamicRun, simulate_replanning

PROTOCOL_ID = "predictive-space-time-v2"
VERIFICATION_STATUS = "PREDICTIVE_DEMO_NON_CONFIRMATORY"
# Eleven decimal places remain far below the spatial and temporal resolution of the
# protocol while absorbing platform-level libm drift observed in the twelfth place.
SERIALIZATION_DECIMAL_PLACES = 11

TIME_STEP_S = 1.0
REPLAN_INTERVAL_S = 4.0
CRUISE_SPEED_MPS = 8.0
MAX_TIME_S = 90.0
RESOLUTION_M = 4.0
TIME_RESOLUTION_S = 0.5
PLANNING_HORIZON_S = 90.0
PREDICTION_HORIZON_S = 90.0
MAX_EXPANDED_STATES = 240_000
SMOOTHING_TURN_RADIUS_M = 6.0
SMOOTHING_SAMPLE_SPACING_M = 0.5
TRAJECTORY_POSTPROCESSOR = "wait-preserving-certified-sampled-circular-fillet-v1"

PREDICTIVE_ALGORITHMS = (
    "repeated-astar-3d",
    "dstar-lite-reset-3d",
    "dstar-lite-reuse-3d",
    "space-time-astar-4d",
)
PLANNER_LABELS = {
    "repeated-astar-3d": "Repeated 3D A*",
    "dstar-lite-reset-3d": "3D D* Lite (reset)",
    "dstar-lite-reuse-3d": "3D D* Lite (state reuse)",
    "space-time-astar-4d": "4D Space-Time A*",
}
PREDICTIVE_FLAGS = {
    "repeated-astar-3d": False,
    "dstar-lite-reset-3d": False,
    "dstar-lite-reuse-3d": False,
    "space-time-astar-4d": True,
}
WORK_UNITS = {
    "repeated-astar-3d": "expanded-nodes",
    "dstar-lite-reset-3d": "queue-pops",
    "dstar-lite-reuse-3d": "queue-pops",
    "space-time-astar-4d": "expanded-spacetime-states",
}
DOWNLOAD_ARTIFACTS = {
    "recordsCsv": "predictive-records.csv",
    "scenarioManifest": "predictive-scenario-manifest.json",
}
RECORD_FIELDS = (
    "source_commit",
    "protocol_id",
    "generated_at",
    "run_id",
    "scenario_id",
    "scenario_fingerprint",
    "cohort",
    "planner_id",
    "predictive",
    "status",
    "failure_reason",
    "success",
    "arrival_time_s",
    "travel_time_s",
    "wait_time_s",
    "executed_path_length_m",
    "direct_distance_m",
    "path_excess_pct",
    "replans",
    "expanded_states",
    "work_unit",
    "safety_violations",
    "raw_waypoint_count",
    "waypoint_count",
    "smoothing_method",
    "smoothing_applied",
    "smoothing_certified",
    "rounded_corner_count",
    "applied_turn_radius_m",
    "max_turn_before_deg",
    "max_turn_after_deg",
    "parameters_json",
)


@dataclass(frozen=True, slots=True)
class PredictiveEpisodeMetrics:
    success: bool
    failure_reason: str | None
    arrival_time_s: float | None
    travel_time_s: float | None
    wait_time_s: float
    executed_path_length_m: float
    direct_distance_m: float
    path_excess_ratio: float | None
    replans: int
    expanded_states: int
    work_unit: str
    minimum_separation_m: float | None
    safety_violations: int

    def to_dict(self) -> dict[str, object]:
        return {
            "success": self.success,
            "failure_reason": self.failure_reason,
            "arrival_time_s": self.arrival_time_s,
            "travel_time_s": self.travel_time_s,
            "wait_time_s": self.wait_time_s,
            "executed_path_length_m": self.executed_path_length_m,
            "direct_distance_m": self.direct_distance_m,
            "path_excess_ratio": self.path_excess_ratio,
            "replans": self.replans,
            "expanded_states": self.expanded_states,
            "work_unit": self.work_unit,
            "minimum_separation_m": self.minimum_separation_m,
            "safety_violations": self.safety_violations,
        }


@dataclass(frozen=True, slots=True)
class PredictiveEpisode:
    scenario_id: str
    scenario_fingerprint: str
    planner_id: str
    predictive: bool
    parameters: dict[str, float | int]
    raw_timed_path: TimedPath
    timed_path: TimedPath
    smoothing: PredictiveSmoothingResult
    metrics: PredictiveEpisodeMetrics

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": "predictive-run-v2",
            "scenario_id": self.scenario_id,
            "scenario_fingerprint": self.scenario_fingerprint,
            "planner_id": self.planner_id,
            "predictive": self.predictive,
            "parameters": self.parameters,
            "raw_timed_path": self.raw_timed_path.to_dict(),
            "timed_path": self.timed_path.to_dict(),
            "smoothing": self.smoothing.to_dict(),
            "metrics": self.metrics.to_dict(),
        }


def _validate_source_commit(source_commit: str) -> None:
    if re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", source_commit) is None:
        raise ValueError("source_commit must be a full lowercase Git object ID")


def _validate_generated_at(generated_at: str) -> None:
    try:
        parsed = datetime.fromisoformat(generated_at)
    except ValueError as error:
        raise ValueError("generated_at must be an ISO-8601 timestamp") from error
    if parsed.tzinfo is None:
        raise ValueError("generated_at must include a timezone")


def _protocol() -> dict[str, object]:
    return {
        "id": PROTOCOL_ID,
        "timeStepS": TIME_STEP_S,
        "cruiseSpeedMps": CRUISE_SPEED_MPS,
        "maxTimeS": MAX_TIME_S,
        "resolutionM": RESOLUTION_M,
        "timeResolutionS": TIME_RESOLUTION_S,
        "planningHorizonS": PLANNING_HORIZON_S,
        "predictionHorizonS": PREDICTION_HORIZON_S,
        "reactiveMaxWorkPerReplan": MAX_EXPANDED_STATES,
        "predictiveMaxExpandedStatesPerMission": MAX_EXPANDED_STATES,
        "reactiveReplanIntervalS": REPLAN_INTERVAL_S,
        "trajectoryPostprocessor": TRAJECTORY_POSTPROCESSOR,
        "informationModel": "reactive snapshots and deterministic complete-schedule conditions",
        "analysisBoundary": "scenario-level descriptive contrasts; no pooled effect estimator",
        "independenceUnit": "scenario",
    }


def _canonical_number(value: float) -> str:
    normalized = 0.0 if float(value) == 0 else float(value)
    return normalized.hex()


def predictive_run_id(
    scenario_fingerprint: str,
    planner_id: str,
    protocol: dict[str, object],
    parameters: dict[str, float | int],
) -> str:
    canonical_parameters = {
        key: _canonical_number(float(value)) for key, value in sorted(parameters.items())
    }
    payload = json.dumps(
        {
            "schema": "uav3d-predictive-run-v2",
            "scenario_fingerprint": scenario_fingerprint,
            "planner_id": planner_id,
            "protocol": protocol,
            "parameters": canonical_parameters,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _point_on_segment(point: Point3, start: Point3, end: Point3) -> bool:
    return math.isclose(
        distance(start, point) + distance(point, end),
        distance(start, end),
        rel_tol=1e-9,
        abs_tol=1e-8,
    )


def _movement_targets(source: DynamicFrame, destination: Point3) -> tuple[Point3, ...]:
    if len(source.planned_path) < 2 or not almost_equal(
        source.planned_path[0], source.position, 1e-8
    ):
        raise ValueError("a moving reactive frame must expose its current planned path")
    targets: list[Point3] = []
    cursor = source.position
    for target in source.planned_path[1:]:
        if almost_equal(target, destination, 1e-8):
            targets.append(destination)
            return tuple(targets)
        if _point_on_segment(destination, cursor, target):
            targets.append(destination)
            return tuple(targets)
        targets.append(target)
        cursor = target
    raise ValueError("reactive destination does not lie on the preceding planned path")


def _reactive_timed_path(run: DynamicRun) -> TimedPath:
    if not run.frames:
        raise ValueError("a reactive run requires at least one frame")
    first = run.frames[0]
    waypoints = [TimedWaypoint(first.time_s, first.position, "start")]
    for source, destination in pairwise(run.frames):
        if destination.time_s <= source.time_s:
            raise ValueError("reactive frames must be strictly increasing in time")
        if almost_equal(source.position, destination.position, 1e-8):
            waypoints.append(TimedWaypoint(destination.time_s, destination.position, "wait"))
            continue
        current = source.position
        current_time = source.time_s
        for target in _movement_targets(source, destination.position):
            current_time += distance(current, target) / CRUISE_SPEED_MPS
            if current_time > destination.time_s + 1e-8:
                raise ValueError("reactive motion exceeds the declared cruise speed")
            waypoints.append(TimedWaypoint(current_time, target, "move"))
            current = target
        if current_time < destination.time_s - 1e-8:
            waypoints.append(TimedWaypoint(destination.time_s, destination.position, "wait"))
        elif not math.isclose(current_time, destination.time_s, abs_tol=1e-8):
            raise ValueError("reactive trajectory time does not match its destination frame")
    return TimedPath(tuple(waypoints))


def _postprocess_trajectory(
    scenario: DynamicScenario,
    raw_timed_path: TimedPath,
    *,
    raw_safe: bool,
) -> PredictiveSmoothingResult:
    """Return a certified common post-processing result or an explicit unsafe fallback."""

    if raw_safe:
        return smooth_predictive_timed_path(
            scenario,
            raw_timed_path,
            requested_radius_m=SMOOTHING_TURN_RADIUS_M,
            sample_spacing_m=SMOOTHING_SAMPLE_SPACING_M,
            max_speed_mps=CRUISE_SPEED_MPS,
        )
    return PredictiveSmoothingResult(
        timed_path=raw_timed_path,
        method="not-run-uncertified-raw-path",
        applied=False,
        certified=False,
        raw_waypoint_count=len(raw_timed_path.waypoints),
        output_waypoint_count=len(raw_timed_path.waypoints),
        rounded_corners=0,
        requested_radius_m=SMOOTHING_TURN_RADIUS_M,
        applied_radius_m=None,
        sample_spacing_m=SMOOTHING_SAMPLE_SPACING_M,
        max_turn_before_deg=0.0,
        max_turn_after_deg=0.0,
    )


def _reactive_episode(
    scenario: DynamicScenario,
    planner_id: str,
    *,
    algorithm: str,
    reuse_search_state: bool,
) -> PredictiveEpisode:
    run = simulate_replanning(
        scenario,
        algorithm,
        time_step=TIME_STEP_S,
        replan_interval=REPLAN_INTERVAL_S,
        cruise_speed=CRUISE_SPEED_MPS,
        max_time=MAX_TIME_S,
        resolution=RESOLUTION_M,
        max_expansions=MAX_EXPANDED_STATES,
        reuse_search_state=reuse_search_state,
    )
    raw_timed_path = _reactive_timed_path(run)
    raw_safe = raw_timed_path.is_safe(scenario)
    smoothing = _postprocess_trajectory(scenario, raw_timed_path, raw_safe=raw_safe)
    timed_path = smoothing.timed_path
    safe = smoothing.certified and timed_path.is_safe(scenario)
    direct = distance(scenario.static_scene.start, scenario.static_scene.goal)
    length = polyline_length(timed_path.positions)
    success = run.metrics.success and safe
    arrival = timed_path.arrival_time_s if success else None
    metrics = PredictiveEpisodeMetrics(
        success=success,
        failure_reason=None
        if success
        else (run.metrics.failure_reason or "trajectory-audit-failed"),
        arrival_time_s=arrival,
        travel_time_s=(timed_path.duration_s - timed_path.wait_time_s) if success else None,
        wait_time_s=timed_path.wait_time_s,
        executed_path_length_m=length,
        direct_distance_m=direct,
        path_excess_ratio=(length / direct - 1.0) if success and direct > 0 else None,
        replans=run.metrics.replans,
        expanded_states=run.metrics.total_planning_work,
        work_unit=WORK_UNITS[planner_id],
        minimum_separation_m=None,
        safety_violations=run.metrics.collision_count + (0 if safe else 1),
    )
    parameters: dict[str, float | int] = {
        "timeStepS": TIME_STEP_S,
        "replanIntervalS": REPLAN_INTERVAL_S,
        "cruiseSpeedMps": CRUISE_SPEED_MPS,
        "maxTimeS": MAX_TIME_S,
        "resolutionM": RESOLUTION_M,
        "maxWorkPerReplan": MAX_EXPANDED_STATES,
        "reuseSearchState": int(reuse_search_state),
        "trajectorySmoothing": 1,
        "smoothingTurnRadiusM": SMOOTHING_TURN_RADIUS_M,
        "smoothingSampleSpacingM": SMOOTHING_SAMPLE_SPACING_M,
    }
    return PredictiveEpisode(
        scenario.scenario_id,
        dynamic_scenario_fingerprint(scenario),
        planner_id,
        False,
        parameters,
        raw_timed_path,
        timed_path,
        smoothing,
        metrics,
    )


def _predictive_episode(scenario: DynamicScenario) -> PredictiveEpisode:
    config = SpaceTimeAStarConfig(
        resolution=RESOLUTION_M,
        time_step=TIME_RESOLUTION_S,
        cruise_speed=CRUISE_SPEED_MPS,
        time_horizon=PLANNING_HORIZON_S,
        max_expansions=MAX_EXPANDED_STATES,
    )
    result = SpaceTimeAStar3D(config).plan(scenario)
    raw_timed_path = result.timed_path or TimedPath(
        (TimedWaypoint(0.0, scenario.static_scene.start, "start"),)
    )
    raw_safe = raw_timed_path.is_safe(scenario)
    smoothing = _postprocess_trajectory(scenario, raw_timed_path, raw_safe=raw_safe)
    timed_path = smoothing.timed_path
    trajectory_safe = smoothing.certified and timed_path.is_safe(scenario)
    direct = distance(scenario.static_scene.start, scenario.static_scene.goal)
    length = polyline_length(timed_path.positions)
    success = result.success and trajectory_safe
    metrics = PredictiveEpisodeMetrics(
        success=success,
        failure_reason=None if success else (result.failure_reason or "trajectory-audit-failed"),
        arrival_time_s=timed_path.arrival_time_s if success else None,
        travel_time_s=(timed_path.duration_s - timed_path.wait_time_s) if success else None,
        wait_time_s=timed_path.wait_time_s,
        executed_path_length_m=length,
        direct_distance_m=direct,
        path_excess_ratio=(length / direct - 1.0) if success and direct > 0 else None,
        replans=1,
        expanded_states=result.expanded_spacetime_states,
        work_unit=WORK_UNITS[result.algorithm],
        minimum_separation_m=None,
        safety_violations=0 if trajectory_safe else 1,
    )
    parameters: dict[str, float | int] = {
        "timeStepS": TIME_STEP_S,
        "cruiseSpeedMps": CRUISE_SPEED_MPS,
        "maxTimeS": MAX_TIME_S,
        "resolutionM": RESOLUTION_M,
        "timeResolutionS": TIME_RESOLUTION_S,
        "planningHorizonS": PLANNING_HORIZON_S,
        "predictionHorizonS": PREDICTION_HORIZON_S,
        "maxExpandedStatesPerMission": MAX_EXPANDED_STATES,
        "trajectorySmoothing": 1,
        "smoothingTurnRadiusM": SMOOTHING_TURN_RADIUS_M,
        "smoothingSampleSpacingM": SMOOTHING_SAMPLE_SPACING_M,
    }
    return PredictiveEpisode(
        scenario.scenario_id,
        dynamic_scenario_fingerprint(scenario),
        result.algorithm,
        True,
        parameters,
        raw_timed_path,
        timed_path,
        smoothing,
        metrics,
    )


def run_predictive_episode(scenario: DynamicScenario, planner_id: str) -> PredictiveEpisode:
    if planner_id == "repeated-astar-3d":
        return _reactive_episode(
            scenario,
            planner_id,
            algorithm="repeated-astar-3d",
            reuse_search_state=False,
        )
    if planner_id == "dstar-lite-reset-3d":
        return _reactive_episode(
            scenario,
            planner_id,
            algorithm="dstar-lite-3d",
            reuse_search_state=False,
        )
    if planner_id == "dstar-lite-reuse-3d":
        return _reactive_episode(
            scenario,
            planner_id,
            algorithm="dstar-lite-3d",
            reuse_search_state=True,
        )
    if planner_id == "space-time-astar-4d":
        return _predictive_episode(scenario)
    choices = ", ".join(PREDICTIVE_ALGORITHMS)
    raise ValueError(f"unknown predictive-study planner {planner_id!r}; choose one of: {choices}")


def run_predictive_study() -> list[tuple[DynamicScenario, list[PredictiveEpisode]]]:
    scenarios, _ = build_predictive_cohort()
    return [
        (
            scenario,
            [run_predictive_episode(scenario, planner_id) for planner_id in PREDICTIVE_ALGORITHMS],
        )
        for scenario in scenarios
    ]


def _normalize_record_numbers(value: object) -> object:
    if isinstance(value, bool) or value is None or isinstance(value, (str, int)):
        return value
    if isinstance(value, float):
        return round(value, SERIALIZATION_DECIMAL_PLACES)
    if isinstance(value, list):
        return [_normalize_record_numbers(item) for item in value]
    if isinstance(value, tuple):
        return [_normalize_record_numbers(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _normalize_record_numbers(item) for key, item in value.items()}
    raise TypeError(f"unsupported predictive export value: {type(value).__name__}")


def _wait_intervals(episode: PredictiveEpisode) -> list[dict[str, object]]:
    intervals: list[dict[str, object]] = []
    for previous, current in pairwise(episode.timed_path.waypoints):
        if current.action != "wait":
            continue
        duration = current.time_s - previous.time_s
        if not episode.predictive:
            reason = "reactive safety hold"
        elif duration < TIME_RESOLUTION_S - 1e-12:
            reason = "time-lattice alignment"
        else:
            reason = "forecast-aware waiting action"
        intervals.append(
            {
                "startTimeS": previous.time_s,
                "endTimeS": current.time_s,
                "position": list(current.position),
                "reason": reason,
            }
        )
    return intervals


def _frame_event(
    episode: PredictiveEpisode,
    index: int,
    active: tuple[str, ...],
    previous_active: tuple[str, ...],
) -> dict[str, str | None]:
    waypoint = episode.timed_path.waypoints[index]
    following = (
        episode.timed_path.waypoints[index + 1]
        if index + 1 < len(episode.timed_path.waypoints)
        else None
    )
    activated = sorted(set(active) - set(previous_active))
    deactivated = sorted(set(previous_active) - set(active))
    if index == len(episode.timed_path.waypoints) - 1 and episode.metrics.success:
        return {"kind": "goal-reached", "label": "Goal reached", "subjectId": None}
    if index == len(episode.timed_path.waypoints) - 1 and not episode.metrics.success:
        return {"kind": "no-path", "label": "Goal not reached", "subjectId": None}
    waiting_before = waypoint.action == "wait"
    waiting_after = following is not None and following.action == "wait"
    if waiting_after and not waiting_before:
        return {"kind": "wait-start", "label": "Wait started", "subjectId": None}
    if waiting_before and not waiting_after:
        return {"kind": "wait-end", "label": "Wait completed", "subjectId": None}
    if waiting_before and waiting_after:
        return {"kind": "wait", "label": "Waiting", "subjectId": None}
    if activated:
        return {
            "kind": "temporary-zone-activated",
            "label": f"{activated[0]} activated",
            "subjectId": activated[0],
        }
    if deactivated:
        return {
            "kind": "temporary-zone-deactivated",
            "label": f"{deactivated[0]} deactivated",
            "subjectId": deactivated[0],
        }
    if index == 0:
        return {
            "kind": "prediction-update" if episode.predictive else "replan",
            "label": "Full schedule planned" if episode.predictive else "Reactive plan initialized",
            "subjectId": None,
        }
    return {"kind": "none", "label": "Nominal execution", "subjectId": None}


def _export_frames(
    scenario: DynamicScenario, episode: PredictiveEpisode
) -> list[dict[str, object]]:
    """Export semantic event anchors without duplicating dense trajectory samples."""

    waypoints = episode.timed_path.waypoints
    previous_active: tuple[str, ...] = ()
    frames: list[dict[str, object]] = []
    for index, waypoint in enumerate(waypoints):
        active = tuple(
            zone.zone_id for zone in scenario.temporary_cylinders if zone.is_active(waypoint.time_s)
        )
        event = _frame_event(episode, index, active, previous_active)
        if event["kind"] != "none":
            frames.append(
                {
                    "timeS": waypoint.time_s,
                    "vehicle": list(waypoint.position),
                    "activeTemporaryZoneIds": list(active),
                    "movingSpheres": [
                        {
                            "id": sphere.sphere_id,
                            "position": list(sphere.position_at(waypoint.time_s)),
                            "radiusM": sphere.radius,
                        }
                        for sphere in scenario.moving_spheres
                    ],
                    "event": event,
                }
            )
        previous_active = active
    return frames


def _run_status(episode: PredictiveEpisode) -> str:
    if episode.metrics.success:
        return "success"
    if episode.metrics.safety_violations:
        return "invalid"
    if episode.metrics.failure_reason in {"maximum-simulation-time", "time-horizon-exhausted"}:
        return "timeout"
    return "no-path"


def _export_metrics(episode: PredictiveEpisode) -> dict[str, object]:
    metrics = episode.metrics
    return {
        "success": metrics.success,
        "failureReason": metrics.failure_reason,
        "arrivalTimeS": metrics.arrival_time_s,
        "travelTimeS": metrics.travel_time_s,
        "waitTimeS": metrics.wait_time_s,
        "executedPathLengthM": metrics.executed_path_length_m,
        "directDistanceM": metrics.direct_distance_m,
        "pathExcessPct": (
            metrics.path_excess_ratio * 100 if metrics.path_excess_ratio is not None else None
        ),
        "replans": metrics.replans,
        "expandedStates": metrics.expanded_states,
        "workUnit": metrics.work_unit,
        "minimumSeparationM": metrics.minimum_separation_m,
        "safetyViolations": metrics.safety_violations,
    }


def _export_run(scenario: DynamicScenario, episode: PredictiveEpisode) -> dict[str, object]:
    protocol = _protocol()
    smoothing = episode.smoothing
    return {
        "runId": predictive_run_id(
            episode.scenario_fingerprint,
            episode.planner_id,
            protocol,
            episode.parameters,
        ),
        "plannerId": episode.planner_id,
        "predictive": episode.predictive,
        "status": _run_status(episode),
        "failureReason": episode.metrics.failure_reason,
        "parameters": episode.parameters,
        "rawTimedPath": [
            {"timeS": waypoint.time_s, "position": list(waypoint.position)}
            for waypoint in episode.raw_timed_path.waypoints
        ],
        "timedPath": [
            {"timeS": waypoint.time_s, "position": list(waypoint.position)}
            for waypoint in episode.timed_path.waypoints
        ],
        "smoothing": {
            "method": smoothing.method,
            "applied": smoothing.applied,
            "certified": smoothing.certified,
            "rawWaypointCount": smoothing.raw_waypoint_count,
            "outputWaypointCount": smoothing.output_waypoint_count,
            "roundedCornerCount": smoothing.rounded_corners,
            "requestedTurnRadiusM": smoothing.requested_radius_m,
            "appliedTurnRadiusM": smoothing.applied_radius_m,
            "sampleSpacingM": smoothing.sample_spacing_m,
            "maxTurnAngleBeforeDeg": smoothing.max_turn_before_deg,
            "maxTurnAngleAfterDeg": smoothing.max_turn_after_deg,
        },
        "waitIntervals": _wait_intervals(episode),
        "metrics": _export_metrics(episode),
        "frames": _export_frames(scenario, episode),
    }


def _export_scenario(
    scenario: DynamicScenario, episodes: Sequence[PredictiveEpisode]
) -> dict[str, object]:
    scene = scenario.static_scene
    return {
        "id": scenario.scenario_id,
        "label": scenario.name,
        "description": str(
            scenario.metadata.get(
                "decision_contract", "A deterministic predictive space-time planning scenario."
            )
        ),
        "fingerprint": dynamic_scenario_fingerprint(scenario),
        "cohort": str(scenario.metadata.get("cohort", "unspecified")),
        "bounds": {"min": list(scene.bounds.minimum), "max": list(scene.bounds.maximum)},
        "start": list(scene.start),
        "goal": list(scene.goal),
        "constraints": {
            "vehicleRadiusM": scene.drone_radius,
            "safetyMarginM": scene.safety_margin,
        },
        "environment": {
            "district": str(scenario.metadata.get("district", "unspecified urban district")),
            "streetPattern": str(
                scenario.metadata.get("street_pattern", "unspecified street pattern")
            ),
            "buildingCount": len(scene.buildings),
            "hazardCount": (
                len(scene.no_fly_zones)
                + len(scenario.temporary_cylinders)
                + len(scenario.moving_spheres)
            ),
        },
        "buildings": [
            {
                "id": building.obstacle_id,
                "min": list(building.minimum),
                "max": list(building.maximum),
            }
            for building in scene.buildings
        ],
        "staticNoFlyZones": [
            {
                "id": zone.zone_id,
                "center": list(zone.center),
                "radiusM": zone.radius,
                "zMinM": zone.z_min,
                "zMaxM": zone.z_max,
            }
            for zone in scene.no_fly_zones
        ],
        "temporaryNoFlyZones": [
            {
                "id": zone.zone_id,
                "center": list(zone.center),
                "radiusM": zone.radius,
                "zMinM": zone.z_min,
                "zMaxM": zone.z_max,
                "activeFromS": zone.active_from,
                "activeUntilS": zone.active_until,
            }
            for zone in scenario.temporary_cylinders
        ],
        "movingSpheres": [
            {
                "id": sphere.sphere_id,
                "radiusM": sphere.radius,
                "keyframes": [
                    {"timeS": time_s, "position": list(position)}
                    for time_s, position in sphere.keyframes
                ],
            }
            for sphere in scenario.moving_spheres
        ],
        "runs": [_export_run(scenario, episode) for episode in episodes],
    }


def _scenario_manifest(
    study: Sequence[tuple[DynamicScenario, list[PredictiveEpisode]]],
    *,
    source_commit: str,
    generated_at: str,
) -> dict[str, object]:
    _, cohort_manifest = build_predictive_cohort()
    return {
        "schemaVersion": 2,
        "datasetId": "predictive-complex-city-v0.5",
        "sourceCommit": source_commit,
        "generatedAt": generated_at,
        "protocolId": PROTOCOL_ID,
        "selection": cohort_manifest["selection"],
        "requested": cohort_manifest["requested"],
        "accepted": cohort_manifest["accepted"],
        "rejected": cohort_manifest["rejected"],
        "acceptedByCohort": cohort_manifest["accepted_by_cohort"],
        "scenarioCount": len(study),
        "runCount": sum(len(episodes) for _, episodes in study),
        "scenarios": [
            {
                "id": scenario.scenario_id,
                "cohort": scenario.metadata.get("cohort"),
                "eventFamily": scenario.metadata.get("event_family"),
                "caseId": scenario.metadata.get("case_id"),
                "fingerprint": dynamic_scenario_fingerprint(scenario),
                "selected": True,
            }
            for scenario, _ in study
        ],
    }


def build_predictive_bundle(
    *,
    source_commit: str,
    generated_at: str | None = None,
) -> tuple[dict[str, object], dict[str, object]]:
    """Execute and serialize the fixed predictive protocol without writing artifacts."""

    _validate_source_commit(source_commit)
    timestamp = generated_at or datetime.now(UTC).replace(microsecond=0).isoformat()
    _validate_generated_at(timestamp)
    study = run_predictive_study()
    bundle: dict[str, object] = {
        "schemaVersion": 2,
        "generatedAt": timestamp,
        "sourceCommit": source_commit,
        "verificationStatus": VERIFICATION_STATUS,
        "protocol": _protocol(),
        "planners": [
            {
                "id": planner_id,
                "label": PLANNER_LABELS[planner_id],
                "predictive": PREDICTIVE_FLAGS[planner_id],
            }
            for planner_id in PREDICTIVE_ALGORITHMS
        ],
        "scenarios": [
            _normalize_record_numbers(_export_scenario(scenario, episodes))
            for scenario, episodes in study
        ],
    }
    return bundle, _scenario_manifest(
        study,
        source_commit=source_commit,
        generated_at=timestamp,
    )


def _csv_scalar(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("predictive CSV metrics must be numeric or null")
    return str(value)


def predictive_record_rows(bundle: dict[str, object]) -> list[dict[str, str]]:
    scenarios = bundle.get("scenarios")
    if not isinstance(scenarios, list):
        raise TypeError("predictive bundle scenarios must be an array")
    rows: list[dict[str, str]] = []
    for scenario in scenarios:
        if not isinstance(scenario, dict) or not isinstance(scenario.get("runs"), list):
            raise TypeError("predictive scenario runs must be an array")
        for run in scenario["runs"]:
            if not isinstance(run, dict):
                raise TypeError("predictive runs must be objects")
            metrics = run.get("metrics")
            raw_timed_path = run.get("rawTimedPath")
            timed_path = run.get("timedPath")
            smoothing = run.get("smoothing")
            parameters = run.get("parameters")
            if (
                not isinstance(metrics, dict)
                or not isinstance(raw_timed_path, list)
                or not isinstance(timed_path, list)
                or not isinstance(smoothing, dict)
            ):
                raise TypeError("predictive run metrics, paths, or smoothing have invalid types")
            if not isinstance(parameters, dict):
                raise TypeError("predictive run parameters must be an object")
            rows.append(
                {
                    "source_commit": str(bundle["sourceCommit"]),
                    "protocol_id": PROTOCOL_ID,
                    "generated_at": str(bundle["generatedAt"]),
                    "run_id": str(run["runId"]),
                    "scenario_id": str(scenario["id"]),
                    "scenario_fingerprint": str(scenario["fingerprint"]),
                    "cohort": str(scenario["cohort"]),
                    "planner_id": str(run["plannerId"]),
                    "predictive": str(run["predictive"]).lower(),
                    "status": str(run["status"]),
                    "failure_reason": (
                        "" if run["failureReason"] is None else str(run["failureReason"])
                    ),
                    "success": str(metrics["success"]).lower(),
                    "arrival_time_s": _csv_scalar(metrics["arrivalTimeS"]),
                    "travel_time_s": _csv_scalar(metrics["travelTimeS"]),
                    "wait_time_s": _csv_scalar(metrics["waitTimeS"]),
                    "executed_path_length_m": _csv_scalar(metrics["executedPathLengthM"]),
                    "direct_distance_m": _csv_scalar(metrics["directDistanceM"]),
                    "path_excess_pct": _csv_scalar(metrics["pathExcessPct"]),
                    "replans": str(metrics["replans"]),
                    "expanded_states": str(metrics["expandedStates"]),
                    "work_unit": str(metrics["workUnit"]),
                    "safety_violations": str(metrics["safetyViolations"]),
                    "raw_waypoint_count": str(len(raw_timed_path)),
                    "waypoint_count": str(len(timed_path)),
                    "smoothing_method": str(smoothing["method"]),
                    "smoothing_applied": str(smoothing["applied"]).lower(),
                    "smoothing_certified": str(smoothing["certified"]).lower(),
                    "rounded_corner_count": str(smoothing["roundedCornerCount"]),
                    "applied_turn_radius_m": _csv_scalar(smoothing["appliedTurnRadiusM"]),
                    "max_turn_before_deg": _csv_scalar(smoothing["maxTurnAngleBeforeDeg"]),
                    "max_turn_after_deg": _csv_scalar(smoothing["maxTurnAngleAfterDeg"]),
                    "parameters_json": json.dumps(
                        parameters, sort_keys=True, separators=(",", ":")
                    ),
                }
            )
    return rows


def _write_json(path: Path, value: object) -> None:
    # The browser bundle is machine-readable evidence. Compact encoding keeps GitHub Pages transfer
    # small; stable key ordering and the trailing newline preserve deterministic byte identities.
    path.write_text(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _write_records_csv(path: Path, rows: Sequence[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=RECORD_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def artifact_reference(path: Path) -> dict[str, str | int]:
    return {
        "path": path.name,
        "sha256": "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
    }


def export_predictive_study(output_dir: Path, *, source_commit: str) -> dict[str, object]:
    """Execute the v0.5 protocol and write its self-describing public data bundle."""

    bundle, manifest = build_predictive_bundle(source_commit=source_commit)
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / DOWNLOAD_ARTIFACTS["scenarioManifest"]
    records_path = output_dir / DOWNLOAD_ARTIFACTS["recordsCsv"]
    _write_json(manifest_path, manifest)
    _write_records_csv(records_path, predictive_record_rows(bundle))
    bundle["downloads"] = {
        key: artifact_reference(output_dir / filename)
        for key, filename in DOWNLOAD_ARTIFACTS.items()
    }
    _write_json(output_dir / "predictive-data.json", bundle)
    return bundle


def load_and_run_predictive_episode(scenario_id: str, planner_id: str) -> PredictiveEpisode:
    return run_predictive_episode(load_predictive_scenario(scenario_id), planner_id)


__all__ = [
    "CRUISE_SPEED_MPS",
    "DOWNLOAD_ARTIFACTS",
    "MAX_EXPANDED_STATES",
    "MAX_TIME_S",
    "PLANNER_LABELS",
    "PREDICTION_HORIZON_S",
    "PREDICTIVE_ALGORITHMS",
    "PREDICTIVE_FLAGS",
    "PROTOCOL_ID",
    "RECORD_FIELDS",
    "REPLAN_INTERVAL_S",
    "RESOLUTION_M",
    "SERIALIZATION_DECIMAL_PLACES",
    "SMOOTHING_SAMPLE_SPACING_M",
    "SMOOTHING_TURN_RADIUS_M",
    "TIME_RESOLUTION_S",
    "TIME_STEP_S",
    "TRAJECTORY_POSTPROCESSOR",
    "VERIFICATION_STATUS",
    "WORK_UNITS",
    "PredictiveEpisode",
    "PredictiveEpisodeMetrics",
    "build_predictive_bundle",
    "export_predictive_study",
    "load_and_run_predictive_episode",
    "predictive_record_rows",
    "predictive_run_id",
    "run_predictive_episode",
    "run_predictive_study",
]
