"""Export the fixed, non-confirmatory dynamic replanning study for the web demo."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
from collections.abc import Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path

from uav3d.dynamic import (
    DynamicScenario,
    dynamic_scenario_fingerprint,
    list_builtin_dynamic_scenarios,
    load_builtin_dynamic_scenario,
)
from uav3d.geometry import Point3, almost_equal, distance
from uav3d.replanning import REPLANNING_ALGORITHMS, DynamicFrame, DynamicRun, simulate_replanning

PROTOCOL_ID = "dynamic-replanning-demo-v1"
TIME_STEP_S = 1.0
REPLAN_INTERVAL_S = 4.0
CRUISE_SPEED_MPS = 8.0
MAX_TIME_S = 180.0
RESOLUTION_M = 4.0
MAX_EXPANSIONS = 120_000
VERIFICATION_STATUS = "DYNAMIC_DEMO_NON_CONFIRMATORY"

PLANNER_LABELS = {
    "repeated-astar-3d": "Repeated 3D A*",
    "repeated-lazy-theta-star": "Repeated Lazy Theta*",
    "dstar-lite-3d": "3D D* Lite",
}

WORK_UNITS = {
    "repeated-astar-3d": "expanded-nodes",
    "repeated-lazy-theta-star": "expanded-nodes",
    "dstar-lite-3d": "queue-pops",
}

SCENARIO_DESCRIPTIONS = {
    "pop-up-nfz": "A temporary no-fly zone activates across the nominal route.",
    "crossing-traffic": "A moving traffic obstacle crosses the nominal route.",
    "closing-gate": "A scheduled exclusion zone closes a low urban gate.",
    "vertical-escape": "A low-altitude closure forces a climb over an urban wall.",
}

DOWNLOAD_ARTIFACTS = {
    "recordsCsv": "dynamic-records.csv",
    "scenarioManifest": "dynamic-scenario-manifest.json",
}

RECORD_FIELDS = (
    "source_commit",
    "protocol_id",
    "generated_at",
    "run_id",
    "scenario_id",
    "scenario_fingerprint",
    "planner_id",
    "status",
    "failure_reason",
    "success",
    "completion_time_s",
    "executed_path_length_m",
    "direct_distance_m",
    "path_excess_pct",
    "replans",
    "failed_replans",
    "holds",
    "safety_gate_activations",
    "collision_count",
    "total_planning_work",
    "work_unit",
    "total_changed_edges",
    "deadline_misses",
    "frame_count",
    "parameters_json",
)


def _validate_source_commit(source_commit: str) -> None:
    if re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", source_commit) is None:
        raise ValueError("source_commit must be a full lowercase Git object ID")


def _validate_generated_at(generated_at: str) -> None:
    try:
        parsed = datetime.fromisoformat(generated_at.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError("generated_at must be a parseable ISO timestamp") from error
    if parsed.tzinfo is None:
        raise ValueError("generated_at must include a timezone offset")


def _protocol() -> dict[str, object]:
    return {
        "id": PROTOCOL_ID,
        "timeStepS": TIME_STEP_S,
        "replanIntervalS": REPLAN_INTERVAL_S,
        "cruiseSpeedMps": CRUISE_SPEED_MPS,
        "maxTimeS": MAX_TIME_S,
        "resolutionM": RESOLUTION_M,
        "maxExpansions": MAX_EXPANSIONS,
    }


def _parameters() -> dict[str, float | int]:
    return {
        "timeStepS": TIME_STEP_S,
        "replanIntervalS": REPLAN_INTERVAL_S,
        "cruiseSpeedMps": CRUISE_SPEED_MPS,
        "maxTimeS": MAX_TIME_S,
        "resolutionM": RESOLUTION_M,
        "maxExpansions": MAX_EXPANSIONS,
    }


def dynamic_run_id(
    scenario_fingerprint: str,
    planner_id: str,
    protocol: Mapping[str, object],
    parameters: Mapping[str, object],
) -> str:
    """Identify a deterministic episode from semantics, planner, and full protocol."""

    payload = {
        "identitySchema": "uav3d-dynamic-run-v1",
        "scenarioFingerprint": scenario_fingerprint,
        "plannerId": planner_id,
        "protocol": protocol,
        "parameters": parameters,
    }
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode()
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


def run_dynamic_study() -> list[tuple[DynamicScenario, list[DynamicRun]]]:
    """Execute the fixed four-scenario by three-planner diagnostic protocol."""

    study: list[tuple[DynamicScenario, list[DynamicRun]]] = []
    for scenario_id in list_builtin_dynamic_scenarios():
        scenario = load_builtin_dynamic_scenario(scenario_id)
        runs = [
            simulate_replanning(
                scenario,
                algorithm,
                time_step=TIME_STEP_S,
                replan_interval=REPLAN_INTERVAL_S,
                cruise_speed=CRUISE_SPEED_MPS,
                max_time=MAX_TIME_S,
                resolution=RESOLUTION_M,
                max_expansions=MAX_EXPANSIONS,
            )
            for algorithm in REPLANNING_ALGORITHMS
        ]
        study.append((scenario, runs))
    return study


def _point_on_segment(point: Point3, start: Point3, end: Point3) -> bool:
    segment_length = distance(start, end)
    return math.isclose(
        distance(start, point) + distance(point, end),
        segment_length,
        rel_tol=1e-9,
        abs_tol=1e-8,
    )


def _extend_executed_path(
    executed: list[Point3], source: DynamicFrame, destination: Point3
) -> None:
    """Append the exact source-plan prefix used to reach the next recorded position."""

    if almost_equal(source.position, destination, 1e-8):
        return
    if len(source.planned_path) < 2 or not almost_equal(
        source.planned_path[0], source.position, 1e-8
    ):
        raise ValueError("a moving dynamic frame must expose its current planned-path prefix")
    cursor = source.position
    for target in source.planned_path[1:]:
        if almost_equal(target, destination, 1e-8):
            if not almost_equal(executed[-1], target, 1e-8):
                executed.append(target)
            return
        if _point_on_segment(destination, cursor, target):
            if not almost_equal(executed[-1], destination, 1e-8):
                executed.append(destination)
            return
        if not almost_equal(executed[-1], target, 1e-8):
            executed.append(target)
        cursor = target
    raise ValueError("the next recorded vehicle position does not lie on the preceding plan")


def _frame_event(
    frame: DynamicFrame,
    active: tuple[str, ...],
    previous_active: tuple[str, ...],
) -> dict[str, str | None]:
    activated = sorted(set(active) - set(previous_active))
    deactivated = sorted(set(previous_active) - set(active))
    if frame.status == "arrived":
        return {"kind": "goal-reached", "label": "Goal reached", "subjectId": None}
    if frame.status == "blocked":
        return {"kind": "no-path", "label": "No safe action", "subjectId": None}
    if frame.status == "hold":
        return {"kind": "wait", "label": "Safety hold", "subjectId": None}
    if activated:
        zone_id = activated[0]
        return {
            "kind": "temporary-zone-activated",
            "label": f"{zone_id} activated",
            "subjectId": zone_id,
        }
    if deactivated:
        zone_id = deactivated[0]
        return {
            "kind": "temporary-zone-deactivated",
            "label": f"{zone_id} deactivated",
            "subjectId": zone_id,
        }
    if frame.replanned:
        reason = frame.replan_reason or "unspecified"
        return {
            "kind": "replan",
            "label": f"Replanned: {reason}",
            "subjectId": None,
        }
    return {"kind": "none", "label": "Nominal execution", "subjectId": None}


def _export_frames(scenario: DynamicScenario, run: DynamicRun) -> list[dict[str, object]]:
    executed: list[Point3] = [scenario.static_scene.start]
    exported: list[dict[str, object]] = []
    previous: DynamicFrame | None = None
    previous_active: tuple[str, ...] = ()
    for frame in run.frames:
        if previous is not None:
            _extend_executed_path(executed, previous, frame.position)
        if not almost_equal(executed[-1], frame.position, 1e-8):
            raise ValueError("cumulative executed path must end at the recorded vehicle position")
        active = tuple(
            zone.zone_id for zone in scenario.temporary_cylinders if zone.is_active(frame.time_s)
        )
        moving = [
            {
                "id": sphere.sphere_id,
                "position": list(sphere.position_at(frame.time_s)),
                "radiusM": sphere.radius,
            }
            for sphere in scenario.moving_spheres
        ]
        planned = (
            [list(point) for point in frame.planned_path] if len(frame.planned_path) >= 2 else []
        )
        exported.append(
            {
                "timeS": frame.time_s,
                "vehicle": list(frame.position),
                "path": planned,
                "executedPath": [list(point) for point in executed],
                "activeTemporaryZoneIds": list(active),
                "movingSpheres": moving,
                "event": _frame_event(frame, active, previous_active),
                "replanned": frame.replanned,
                "replanReason": frame.replan_reason,
                "plannerSuccess": frame.planner_success,
                # Wall-clock observations are deliberately absent from deterministic records.
                "planningTimeMs": None,
                "workUsed": frame.planning_work,
                "changedEdges": frame.changed_edges,
            }
        )
        previous = frame
        previous_active = active
    return exported


def _run_status(run: DynamicRun) -> str:
    if run.metrics.success:
        return "success"
    if run.metrics.collision_count:
        return "invalid"
    if run.metrics.failure_reason == "maximum-simulation-time":
        return "timeout"
    return "no-path"


def _export_metrics(run: DynamicRun) -> dict[str, object]:
    metrics = run.metrics
    return {
        "success": metrics.success,
        "failureReason": metrics.failure_reason,
        "completionTimeS": metrics.completion_time_s,
        "executedPathLengthM": metrics.executed_path_length_m,
        "directDistanceM": metrics.direct_distance_m,
        "pathExcessPct": (
            metrics.path_excess_ratio * 100 if metrics.path_excess_ratio is not None else None
        ),
        "replans": metrics.replans,
        "failedReplans": metrics.failed_replans,
        "holds": metrics.holds,
        "safetyGateActivations": metrics.safety_gate_activations,
        "collisionCount": metrics.collision_count,
        "totalPlanningWork": metrics.total_planning_work,
        "workUnit": WORK_UNITS[run.algorithm],
        "totalChangedEdges": metrics.total_changed_edges,
        "deadlineMisses": 0,
        "minimumClearanceM": None,
    }


def _export_run(scenario: DynamicScenario, run: DynamicRun) -> dict[str, object]:
    status = _run_status(run)
    parameters = _parameters()
    fingerprint = dynamic_scenario_fingerprint(scenario)
    return {
        "runId": dynamic_run_id(fingerprint, run.algorithm, _protocol(), parameters),
        "plannerId": run.algorithm,
        "status": status,
        "failureReason": run.metrics.failure_reason,
        "parameters": parameters,
        "metrics": _export_metrics(run),
        "frames": _export_frames(scenario, run),
    }


def _export_scenario(scenario: DynamicScenario, runs: Sequence[DynamicRun]) -> dict[str, object]:
    scene = scenario.static_scene
    return {
        "id": scenario.scenario_id,
        "label": scenario.name,
        "description": SCENARIO_DESCRIPTIONS.get(
            scenario.scenario_id, "A deterministic dynamic replanning scenario."
        ),
        "fingerprint": dynamic_scenario_fingerprint(scenario),
        "bounds": {"min": list(scene.bounds.minimum), "max": list(scene.bounds.maximum)},
        "start": list(scene.start),
        "goal": list(scene.goal),
        "constraints": {
            "vehicleRadiusM": scene.drone_radius,
            "safetyMarginM": scene.safety_margin,
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
                "kind": "cylinder",
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
                "kind": "cylinder",
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
                "label": sphere.sphere_id.replace("-", " ").title(),
                "radiusM": sphere.radius,
                "keyframes": [
                    {"timeS": time_s, "position": list(position)}
                    for time_s, position in sphere.keyframes
                ],
            }
            for sphere in scenario.moving_spheres
        ],
        "runs": [_export_run(scenario, run) for run in runs],
    }


def _scenario_manifest(
    study: Sequence[tuple[DynamicScenario, list[DynamicRun]]],
    *,
    source_commit: str,
    generated_at: str,
) -> dict[str, object]:
    return {
        "schemaVersion": 1,
        "datasetId": "curated-dynamic-diagnostic-v1",
        "sourceCommit": source_commit,
        "generatedAt": generated_at,
        "protocolId": PROTOCOL_ID,
        "selection": {
            "scenarioRule": "all built-in dynamic scenarios",
            "algorithmFiltering": "none",
        },
        "scenarioCount": len(study),
        "runCount": sum(len(runs) for _, runs in study),
        "scenarios": [
            {
                "id": scenario.scenario_id,
                "label": scenario.name,
                "fingerprint": dynamic_scenario_fingerprint(scenario),
                "selected": True,
            }
            for scenario, _ in study
        ],
    }


def _record_row(
    source_commit: str,
    generated_at: str,
    scenario: dict[str, object],
    run: dict[str, object],
) -> dict[str, str]:
    metrics = run["metrics"]
    if not isinstance(metrics, dict):
        raise TypeError("exported dynamic run metrics must be an object")
    parameters = run["parameters"]
    frames = run["frames"]
    if not isinstance(frames, list):
        raise TypeError("exported dynamic run frames must be an array")
    return {
        "source_commit": source_commit,
        "protocol_id": PROTOCOL_ID,
        "generated_at": generated_at,
        "run_id": str(run["runId"]),
        "scenario_id": str(scenario["id"]),
        "scenario_fingerprint": str(scenario["fingerprint"]),
        "planner_id": str(run["plannerId"]),
        "status": str(run["status"]),
        "failure_reason": "" if run["failureReason"] is None else str(run["failureReason"]),
        "success": str(metrics["success"]).lower(),
        "completion_time_s": _csv_scalar(metrics["completionTimeS"]),
        "executed_path_length_m": _csv_scalar(metrics["executedPathLengthM"]),
        "direct_distance_m": _csv_scalar(metrics["directDistanceM"]),
        "path_excess_pct": _csv_scalar(metrics["pathExcessPct"]),
        "replans": str(metrics["replans"]),
        "failed_replans": str(metrics["failedReplans"]),
        "holds": str(metrics["holds"]),
        "safety_gate_activations": str(metrics["safetyGateActivations"]),
        "collision_count": str(metrics["collisionCount"]),
        "total_planning_work": str(metrics["totalPlanningWork"]),
        "work_unit": str(metrics["workUnit"]),
        "total_changed_edges": str(metrics["totalChangedEdges"]),
        "deadline_misses": str(metrics["deadlineMisses"]),
        "frame_count": str(len(frames)),
        "parameters_json": json.dumps(parameters, sort_keys=True, separators=(",", ":")),
    }


def _csv_scalar(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("dynamic CSV metrics must be numeric or null")
    return str(value)


def dynamic_record_rows(bundle: dict[str, object]) -> list[dict[str, str]]:
    source_commit = str(bundle["sourceCommit"])
    generated_at = str(bundle["generatedAt"])
    scenarios = bundle["scenarios"]
    if not isinstance(scenarios, list):
        raise TypeError("dynamic bundle scenarios must be an array")
    rows: list[dict[str, str]] = []
    for raw_scenario in scenarios:
        if not isinstance(raw_scenario, dict):
            raise TypeError("dynamic scenarios must be objects")
        runs = raw_scenario.get("runs")
        if not isinstance(runs, list):
            raise TypeError("dynamic scenario runs must be an array")
        for run in runs:
            if not isinstance(run, dict):
                raise TypeError("dynamic runs must be objects")
            rows.append(_record_row(source_commit, generated_at, raw_scenario, run))
    return rows


def build_dynamic_bundle(
    *,
    source_commit: str,
    generated_at: str | None = None,
) -> tuple[dict[str, object], dict[str, object]]:
    """Execute and serialize the fixed public protocol without writing artifacts."""

    _validate_source_commit(source_commit)
    timestamp = generated_at or datetime.now(UTC).replace(microsecond=0).isoformat()
    _validate_generated_at(timestamp)
    study = run_dynamic_study()
    bundle: dict[str, object] = {
        "schemaVersion": 1,
        "generatedAt": timestamp,
        "sourceCommit": source_commit,
        "verificationStatus": VERIFICATION_STATUS,
        "protocol": _protocol(),
        "planners": [
            {"id": algorithm, "label": PLANNER_LABELS[algorithm]}
            for algorithm in REPLANNING_ALGORITHMS
        ],
        "scenarios": [_export_scenario(scenario, runs) for scenario, runs in study],
    }
    return bundle, _scenario_manifest(
        study,
        source_commit=source_commit,
        generated_at=timestamp,
    )


def _write_json(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
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


def export_dynamic_study(output_dir: Path, *, source_commit: str) -> dict[str, object]:
    """Execute the protocol and write the self-describing public data bundle."""

    bundle, manifest = build_dynamic_bundle(source_commit=source_commit)
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / DOWNLOAD_ARTIFACTS["scenarioManifest"]
    records_path = output_dir / DOWNLOAD_ARTIFACTS["recordsCsv"]
    _write_json(manifest_path, manifest)
    _write_records_csv(records_path, dynamic_record_rows(bundle))
    bundle["downloads"] = {
        key: artifact_reference(output_dir / filename)
        for key, filename in DOWNLOAD_ARTIFACTS.items()
    }
    _write_json(output_dir / "dynamic-data.json", bundle)
    return bundle


# A discoverable alias matching the existing static study export naming convention.
export_web_dynamic = export_dynamic_study


__all__ = [
    "CRUISE_SPEED_MPS",
    "DOWNLOAD_ARTIFACTS",
    "MAX_EXPANSIONS",
    "MAX_TIME_S",
    "PLANNER_LABELS",
    "PROTOCOL_ID",
    "RECORD_FIELDS",
    "REPLAN_INTERVAL_S",
    "RESOLUTION_M",
    "TIME_STEP_S",
    "VERIFICATION_STATUS",
    "WORK_UNITS",
    "artifact_reference",
    "build_dynamic_bundle",
    "dynamic_record_rows",
    "dynamic_run_id",
    "export_dynamic_study",
    "export_web_dynamic",
    "run_dynamic_study",
]
