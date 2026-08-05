"""Generate the committed descriptive benchmark bundle for the academic results page."""

from __future__ import annotations

import csv
import hashlib
import json
import re
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, cast

from uav3d.analysis import (
    BOOTSTRAP_RESAMPLES,
    BOOTSTRAP_SEED,
    RecordLike,
    clustered_metric_summary,
    planner_summaries,
)
from uav3d.benchmark import (
    ExperimentRecord,
    problem_fingerprint,
    run_resolution_sweep,
    run_rrt_budget_curve,
)
from uav3d.reporting import write_records_csv
from uav3d.scene import Scene
from uav3d.timing import run_timing_harness

NOMINAL_RESOLUTION_M = 4.0
RESOLUTION_VALUES_M = (3.0, 4.0, 6.0, 8.0)
RRT_BUDGETS = (250, 500, 1_000, 2_000, 3_000)
RRT_SEEDS = (11, 23, 37, 47, 59)
PROTOCOL_ID = "static-city-descriptive-v2"
DOWNLOAD_ARTIFACTS = {
    "recordsCsv": "benchmark-records.csv",
    "summariesCsv": "benchmark-summary.csv",
    "datasetManifest": "dataset-manifest.json",
    "timingManifest": "timing-manifest.json",
}


def _validate_source_commit(source_commit: str) -> None:
    if re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", source_commit) is None:
        raise ValueError("source_commit must be a full lowercase Git object ID")


def _artifact_reference(path: Path) -> dict[str, str | int]:
    return {
        "path": path.name,
        "sha256": "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
    }


def _raw_excess(record: RecordLike) -> float | None:
    if record.status != "success" or record.raw_audit is None:
        return None
    return (record.raw_audit.length_m / record.straight_line_distance_m - 1) * 100


def _distribution(metric: dict[str, object]) -> dict[str, object]:
    return {
        "estimator": "median-of-scene-medians",
        "median": metric["value"],
        "q1": metric["q1"],
        "q3": metric["q3"],
        "ci95Low": metric["ci95Low"],
        "ci95High": metric["ci95High"],
        "conditioning": metric["conditioning"],
        "nScenes": metric["nScenes"],
        "nDefinedScenes": metric["nDefinedScenes"],
        "nRuns": metric["nRuns"],
        "nSuccesses": metric["nSuccesses"],
    }


def _success_summary(metric: dict[str, object]) -> dict[str, object]:
    return {
        "estimator": "scene-weighted-mean",
        "value": metric["value"],
        "ci95Low": metric["ci95Low"],
        "ci95High": metric["ci95High"],
        "nScenes": metric["nScenes"],
        "nDefinedScenes": metric["nDefinedScenes"],
        "nRuns": metric["nRuns"],
        "nSuccesses": metric["nSuccesses"],
    }


@dataclass(frozen=True, slots=True)
class _TimingObservation:
    scene_id: str
    scene_fingerprint: str
    status: str
    value: float


@dataclass(frozen=True, slots=True)
class _TraceObservation:
    scene_id: str
    scene_fingerprint: str
    status: str
    value: float | None


def _timing_metric(payload: dict[str, Any], algorithm: str) -> dict[str, object]:
    observations: list[_TimingObservation] = []
    for item in payload["records"]:
        if item["algorithm"] != algorithm or item["process_status"] != "completed":
            continue
        planner_record = item["planner_record"]
        if not isinstance(planner_record, dict):
            continue
        planning_time = planner_record.get("planning_time_ms")
        if isinstance(planning_time, (int, float)):
            observations.append(
                _TimingObservation(
                    str(planner_record["scene_id"]),
                    str(planner_record["problem_fingerprint"]),
                    str(planner_record["status"]),
                    float(planning_time),
                )
            )
    metric = clustered_metric_summary(
        cast(Sequence[RecordLike], observations),
        lambda record: cast(_TimingObservation, record).value,
        conditioning="all-runs",
    )
    return _distribution(metric)


def _aggregate_resolution(
    records: list[ExperimentRecord],
) -> list[dict[str, object]]:
    points: list[dict[str, object]] = []
    typed = cast(Sequence[RecordLike], records)
    for algorithm in ("astar-3d", "lazy-theta-star"):
        for resolution in RESOLUTION_VALUES_M:
            marker = f"resolution={resolution:g}|"
            group = [
                record
                for record in typed
                if record.algorithm == algorithm and marker in record.configuration_id
            ]
            metric = clustered_metric_summary(group, _raw_excess, conditioning="successful-runs")
            points.append(
                {
                    "plannerId": algorithm,
                    "voxelResolutionM": resolution,
                    "rawPathExcessPct": _distribution(metric),
                }
            )
    return points


def _aggregate_rrt(records: list[ExperimentRecord]) -> list[dict[str, object]]:
    points: list[dict[str, object]] = []
    for budget in RRT_BUDGETS:
        observations: list[_TraceObservation] = []
        for record in records:
            trace = next((point for point in record.quality_trace if point.work == budget), None)
            length = trace.best_path_length_m if trace is not None else None
            excess = (
                (length / record.straight_line_distance_m - 1) * 100 if length is not None else None
            )
            observations.append(
                _TraceObservation(
                    record.scene_id,
                    record.scene_fingerprint,
                    "success" if excess is not None else "budget-exhausted",
                    excess,
                )
            )
        metric = clustered_metric_summary(
            cast(Sequence[RecordLike], observations),
            lambda record: cast(_TraceObservation, record).value,
            conditioning="successful-runs",
        )
        points.append(
            {
                "sampleBudget": budget,
                "rawPathExcessPct": _distribution(metric),
            }
        )
    return points


def _dataset_manifest(scenes: list[Scene]) -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "dataset_id": "curated-static-diagnostic-v1",
        "selection": "all committed curated scenes; no planner-based filtering",
        "split": "diagnostic",
        "requested": len(scenes),
        "accepted": len(scenes),
        "rejected": 0,
        "records": [
            {
                "scene_id": scene.scene_id,
                "status": "accepted",
                "problem_fingerprint": problem_fingerprint(scene),
            }
            for scene in scenes
        ],
    }


def _compact_timing(payload: dict[str, Any]) -> dict[str, Any]:
    compact = dict(payload)
    compact_records: list[dict[str, object]] = []
    for item in payload["records"]:
        record = {key: value for key, value in item.items() if key != "planner_record"}
        planner = item.get("planner_record")
        if isinstance(planner, dict):
            record["planner_record"] = {
                key: planner.get(key)
                for key in (
                    "run_id",
                    "run_purpose",
                    "timing_repetition",
                    "scene_id",
                    "problem_fingerprint",
                    "algorithm",
                    "configuration_id",
                    "status",
                    "failure_reason",
                    "planning_time_ms",
                    "timing",
                    "budget",
                    "budget_usage",
                )
            }
        compact_records.append(record)
    compact["records"] = compact_records
    return compact


def _write_web_summary_csv(
    summaries: list[dict[str, object]],
    path: Path,
    *,
    metadata: dict[str, str],
) -> None:
    fields = [
        *metadata,
        "planner_id",
        "budget_id",
        "path_runs",
        "path_scenes",
        "path_successes",
        "timing_runs",
        "timing_scenes",
        "timing_successes",
        "success_rate",
        "success_ci95_low",
        "success_ci95_high",
        "planning_time_median_ms",
        "raw_path_excess_median_pct",
        "smoothed_path_excess_median_pct",
        "minimum_clearance_median_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for summary in summaries:
            success = cast(dict[str, object], summary["successRate"])
            timing = cast(dict[str, object], summary["planningTimeMs"])
            raw = cast(dict[str, object], summary["rawPathExcessPct"])
            smoothed = cast(dict[str, object], summary["smoothedPathExcessPct"])
            clearance = cast(dict[str, object], summary["minimumClearanceM"])
            writer.writerow(
                metadata
                | {
                    "planner_id": summary["plannerId"],
                    "budget_id": summary["budgetId"],
                    "path_runs": success["nRuns"],
                    "path_scenes": success["nScenes"],
                    "path_successes": success["nSuccesses"],
                    "timing_runs": timing["nRuns"],
                    "timing_scenes": timing["nScenes"],
                    "timing_successes": timing["nSuccesses"],
                    "success_rate": success["value"],
                    "success_ci95_low": success["ci95Low"],
                    "success_ci95_high": success["ci95High"],
                    "planning_time_median_ms": timing["median"],
                    "raw_path_excess_median_pct": raw["median"],
                    "smoothed_path_excess_median_pct": smoothed["median"],
                    "minimum_clearance_median_m": clearance["median"],
                }
            )


def build_web_benchmark_bundle(
    scenes: list[Scene],
    *,
    source_commit: str,
    timing_repetitions: int = 3,
    working_directory: Path | None = None,
) -> tuple[dict[str, object], dict[str, object], dict[str, Any], list[ExperimentRecord]]:
    """Execute the fixed descriptive protocol and return all publication inputs."""

    if not scenes:
        raise ValueError("benchmark bundle requires at least one scene")
    _validate_source_commit(source_commit)
    resolution_records = run_resolution_sweep(scenes, list(RESOLUTION_VALUES_M))
    rrt_records = run_rrt_budget_curve(scenes, list(RRT_BUDGETS), list(RRT_SEEDS))
    nominal_records = [
        record
        for record in resolution_records
        if f"resolution={NOMINAL_RESOLUTION_M:g}|" in record.configuration_id
    ] + rrt_records

    references = [scene.scene_id for scene in scenes]
    timing_payload = run_timing_harness(
        references,
        ["astar-3d", "lazy-theta-star", "rrt-star"],
        timing_repetitions,
        planner_seed=17,
        order_seed=BOOTSTRAP_SEED,
        resolution=NOMINAL_RESOLUTION_M,
        working_directory=working_directory,
    )
    base_summaries = planner_summaries(cast(Sequence[RecordLike], nominal_records))
    budget_ids = {
        "astar-3d": "astar-r4-e120000",
        "lazy-theta-star": "theta-r4-e120000",
        "rrt-star": "rrt-s3000",
    }
    summaries: list[dict[str, object]] = []
    for base in base_summaries:
        algorithm = str(base["plannerId"])
        metrics = cast(dict[str, dict[str, object]], base["metrics"])
        summaries.append(
            {
                "plannerId": algorithm,
                "budgetId": budget_ids[algorithm],
                "successRate": _success_summary(metrics["successRate"]),
                "planningTimeMs": _timing_metric(timing_payload, algorithm),
                "rawPathExcessPct": _distribution(metrics["rawPathExcessPct"]),
                "smoothedPathExcessPct": _distribution(metrics["smoothedPathExcessPct"]),
                "minimumClearanceM": _distribution(metrics["minimumClearanceM"]),
            }
        )

    manifest = _dataset_manifest(scenes)
    manifest_bytes = (
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode()
    manifest_digest = "sha256:" + hashlib.sha256(manifest_bytes).hexdigest()
    generated_at = datetime.now(UTC).replace(microsecond=0).isoformat()
    budgets: list[dict[str, object]] = []
    for algorithm, prefix in (("astar-3d", "astar"), ("lazy-theta-star", "theta")):
        for resolution in RESOLUTION_VALUES_M:
            budgets.append(
                {
                    "id": f"{prefix}-r{resolution:g}-e120000",
                    "kind": "voxel",
                    "plannerId": algorithm,
                    "voxelResolutionM": resolution,
                    "maxExpansions": 120_000,
                    "wallClockLimitMs": None,
                }
            )
    for budget in RRT_BUDGETS:
        budgets.append(
            {
                "id": f"rrt-s{budget}",
                "kind": "samples",
                "plannerId": "rrt-star",
                "sampleBudget": budget,
                "wallClockLimitMs": None,
            }
        )
    bundle: dict[str, object] = {
        "schemaVersion": 2,
        "generatedAt": generated_at,
        "evidenceLabel": "DESCRIPTIVE_BENCHMARK",
        "sourceCommit": source_commit,
        "protocol": {
            "id": PROTOCOL_ID,
            "rawPathPrimary": True,
            "confidenceLevel": 0.95,
            "pathExcessDefinition": "(path_length / euclidean_start_goal - 1) * 100",
            "bootstrap": {
                "method": "scene-clustered-percentile",
                "resamples": BOOTSTRAP_RESAMPLES,
                "seed": BOOTSTRAP_SEED,
            },
            "timing": {
                "isolatedProcesses": True,
                "repetitionsPerCell": timing_repetitions,
            },
            "quality": {
                "deterministicRunsPerScene": 1,
                "rrtPlannerSeeds": list(RRT_SEEDS),
            },
        },
        "dataset": {
            "id": "curated-static-diagnostic-v1",
            "label": "Four curated static-city scenes (exploratory small-n)",
            "split": "diagnostic",
            "attemptedScenes": len(scenes),
            "acceptedScenes": len(scenes),
            "rejectedScenes": 0,
            "manifestSha256": manifest_digest,
        },
        "planners": [
            {"id": "astar-3d", "label": "3D A*"},
            {"id": "lazy-theta-star", "label": "Lazy Theta*"},
            {"id": "rrt-star", "label": "RRT*"},
        ],
        "budgets": budgets,
        "nominalBudgetSetId": "nominal-v0.2",
        "summaries": summaries,
        "sensitivity": {
            "resolution": _aggregate_resolution(resolution_records),
            "rrtBudget": _aggregate_rrt(rrt_records),
        },
    }
    return bundle, manifest, _compact_timing(timing_payload), resolution_records + rrt_records


def export_web_benchmark(
    scenes: list[Scene],
    output_dir: Path,
    *,
    source_commit: str,
    timing_repetitions: int = 3,
    working_directory: Path | None = None,
) -> dict[str, object]:
    bundle, manifest, timing, records = build_web_benchmark_bundle(
        scenes,
        source_commit=source_commit,
        timing_repetitions=timing_repetitions,
        working_directory=working_directory,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "dataset-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "timing-manifest.json").write_text(
        json.dumps(timing, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    metadata = {
        "source_commit": str(bundle["sourceCommit"]),
        "protocol_id": str(cast(dict[str, object], bundle["protocol"])["id"]),
        "generated_at": str(bundle["generatedAt"]),
    }
    write_records_csv(
        records,
        output_dir / "benchmark-records.csv",
        metadata=metadata,
    )
    _write_web_summary_csv(
        cast(list[dict[str, object]], bundle["summaries"]),
        output_dir / "benchmark-summary.csv",
        metadata=metadata,
    )
    bundle["downloads"] = {
        key: _artifact_reference(output_dir / filename)
        for key, filename in DOWNLOAD_ARTIFACTS.items()
    }
    (output_dir / "benchmark-data.json").write_text(
        json.dumps(bundle, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return bundle
