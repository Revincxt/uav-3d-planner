"""Re-audit committed web trajectories and benchmark provenance in Python."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
import subprocess
from pathlib import Path
from typing import Any

from uav3d.benchmark import _configuration_id, _run_id, problem_fingerprint
from uav3d.planners.base import PlanningBudget
from uav3d.scene import load_builtin_scene
from uav3d.validation import audit_path
from uav3d.version import __version__

ROOT = Path(__file__).parents[1]
PUBLIC = ROOT / "web" / "public"
DOWNLOADS = {
    "recordsCsv": "benchmark-records.csv",
    "summariesCsv": "benchmark-summary.csv",
    "datasetManifest": "dataset-manifest.json",
    "timingManifest": "timing-manifest.json",
}


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain an object")
    return value


def _audit_demo() -> int:
    bundle = _load_json(PUBLIC / "demo-data.json")
    audited = 0
    for scenario in bundle["scenarios"]:
        scene = load_builtin_scene(str(scenario["id"]))
        expected_fingerprint = problem_fingerprint(scene)
        if scenario["fingerprint"] != expected_fingerprint:
            raise ValueError(f"stale problem fingerprint for {scene.scene_id}")
        for result in scenario["results"]:
            if result["status"] != "success" or result["paths"] is None:
                raise ValueError(f"demo run is not successful: {result['runId']}")
            raw = tuple(tuple(float(value) for value in point) for point in result["paths"]["raw"])
            smoothed = tuple(
                tuple(float(value) for value in point) for point in result["paths"]["smoothed"]
            )
            raw_audit = audit_path(scene, raw)  # type: ignore[arg-type]
            smoothed_audit = audit_path(scene, smoothed)  # type: ignore[arg-type]
            if not raw_audit.valid or not smoothed_audit.valid:
                raise ValueError(f"committed trajectory failed audit: {result['runId']}")
            if abs(raw_audit.length_m - float(result["metrics"]["rawLengthM"])) > 0.02:
                raise ValueError(f"raw length mismatch: {result['runId']}")
            if abs(smoothed_audit.length_m - float(result["metrics"]["smoothedLengthM"])) > 0.02:
                raise ValueError(f"smoothed length mismatch: {result['runId']}")
            audited += 1
    return audited


def _audit_benchmark() -> int:
    bundle = _load_json(PUBLIC / "benchmark-data.json")
    source_commit = str(bundle.get("sourceCommit", ""))
    if re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", source_commit) is None:
        raise ValueError("benchmark sourceCommit must be a full lowercase Git object ID")
    source_exists = subprocess.run(
        ["git", "cat-file", "-e", f"{source_commit}^{{commit}}"],
        cwd=ROOT,
        check=False,
        capture_output=True,
    )
    if source_exists.returncode != 0:
        raise ValueError("benchmark sourceCommit is not present in repository history")
    source_is_ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", source_commit, "HEAD"],
        cwd=ROOT,
        check=False,
        capture_output=True,
    )
    if source_is_ancestor.returncode != 0:
        raise ValueError("benchmark sourceCommit is not an ancestor of HEAD")

    downloads = bundle.get("downloads")
    if not isinstance(downloads, dict) or set(downloads) != set(DOWNLOADS):
        raise ValueError("benchmark downloads must list exactly four artifacts")
    for key, filename in DOWNLOADS.items():
        reference = downloads.get(key)
        if not isinstance(reference, dict) or reference.get("path") != filename:
            raise ValueError(f"invalid benchmark download reference: {key}")
        artifact = PUBLIC / filename
        digest = "sha256:" + hashlib.sha256(artifact.read_bytes()).hexdigest()
        if reference.get("sha256") != digest or reference.get("bytes") != artifact.stat().st_size:
            raise ValueError(f"benchmark download provenance mismatch: {filename}")

    manifest_path = PUBLIC / DOWNLOADS["datasetManifest"]
    manifest_digest = "sha256:" + hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    if bundle["dataset"]["manifestSha256"] != manifest_digest:
        raise ValueError("dataset manifest digest does not match benchmark-data.json")
    manifest = _load_json(manifest_path)
    if manifest["requested"] != manifest["accepted"] + manifest["rejected"]:
        raise ValueError("dataset manifest counts are inconsistent")
    fingerprints = {
        str(record["scene_id"]): str(record["problem_fingerprint"])
        for record in manifest["records"]
        if record["status"] == "accepted"
    }
    with (PUBLIC / "benchmark-records.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    run_ids = [row["run_id"] for row in rows]
    if not rows or len(run_ids) != len(set(run_ids)):
        raise ValueError("benchmark records must contain unique run IDs")
    if {row["algorithm"] for row in rows} != {
        "astar-3d",
        "lazy-theta-star",
        "rrt-star",
    }:
        raise ValueError("benchmark records do not cover all planners")
    expected_metadata = {
        "source_commit": source_commit,
        "protocol_id": str(bundle["protocol"]["id"]),
        "generated_at": str(bundle["generatedAt"]),
    }
    for row in rows:
        if any(row[key] != value for key, value in expected_metadata.items()):
            raise ValueError(f"record CSV provenance mismatch: {row['run_id']}")
        algorithm = row["algorithm"]
        if row["problem_fingerprint"] != fingerprints.get(row["scene_id"]):
            raise ValueError(f"record fingerprint mismatch: {row['run_id']}")
        planner_seed = int(row["planner_seed"]) if row["planner_seed"] else None
        repetition = int(row["timing_repetition"]) if row["timing_repetition"] else None
        wall_limit = float(row["wall_time_limit_ms"]) if row["wall_time_limit_ms"] else None
        budget = PlanningBudget(row["work_unit"], int(row["work_limit"]), wall_limit)
        parameters = json.loads(row["parameters_json"])
        expected_configuration = _configuration_id(algorithm, budget, parameters)
        if row["configuration_id"] != expected_configuration:
            raise ValueError(f"stale configuration ID: {row['run_id']}")
        expected_run = _run_id(
            row["problem_fingerprint"],
            expected_configuration,
            planner_seed,
            row["run_purpose"],
            repetition,
        )
        if row["run_id"] != expected_run:
            raise ValueError(f"stale run ID: {row['run_id']}")
        if not row["configuration_id"].startswith(f"{algorithm}@{__version__}|"):
            raise ValueError(f"configuration version mismatch: {row['run_id']}")

    summaries = {str(item["plannerId"]): item for item in bundle["summaries"]}
    if len(summaries) != 3:
        raise ValueError("benchmark bundle must contain three unique planner summaries")
    accepted_scenes = int(bundle["dataset"]["acceptedScenes"])
    rrt_seeds = bundle["protocol"]["quality"]["rrtPlannerSeeds"]
    timing_repetitions = int(bundle["protocol"]["timing"]["repetitionsPerCell"])
    for algorithm, summary in summaries.items():
        nominal = [
            row
            for row in rows
            if row["algorithm"] == algorithm
            and (algorithm == "rrt-star" or "|resolution=4|" in row["configuration_id"])
        ]
        expected_runs = accepted_scenes * (len(rrt_seeds) if algorithm == "rrt-star" else 1)
        successes = sum(row["status"] == "success" for row in nominal)
        success_metric = summary["successRate"]
        if (
            len(nominal) != expected_runs
            or success_metric["nRuns"] != len(nominal)
            or success_metric["nScenes"] != len({row["problem_fingerprint"] for row in nominal})
            or success_metric["nSuccesses"] != successes
        ):
            raise ValueError(f"quality summary counts disagree for {algorithm}")
        for metric_name in (
            "rawPathExcessPct",
            "smoothedPathExcessPct",
            "minimumClearanceM",
        ):
            metric = summary[metric_name]
            if (
                metric["nRuns"] != success_metric["nRuns"]
                or metric["nScenes"] != success_metric["nScenes"]
                or metric["nSuccesses"] != success_metric["nSuccesses"]
            ):
                raise ValueError(f"path metric counts disagree for {algorithm}.{metric_name}")

    for point in bundle["sensitivity"]["resolution"]:
        algorithm = str(point["plannerId"])
        resolution = float(point["voxelResolutionM"])
        marker = f"|resolution={resolution:g}|"
        group = [
            row
            for row in rows
            if row["algorithm"] == algorithm and marker in row["configuration_id"]
        ]
        metric = point["rawPathExcessPct"]
        if (
            metric["nRuns"] != len(group)
            or metric["nScenes"] != len({row["problem_fingerprint"] for row in group})
            or metric["nSuccesses"] != sum(row["status"] == "success" for row in group)
        ):
            raise ValueError(
                f"resolution sensitivity counts disagree for {algorithm}@{resolution:g}"
            )

    rrt_rows = [row for row in rows if row["algorithm"] == "rrt-star"]
    for point in bundle["sensitivity"]["rrtBudget"]:
        budget = int(point["sampleBudget"])
        defined = 0
        for row in rrt_rows:
            trace = json.loads(row["quality_trace_json"])
            checkpoint = next((item for item in trace if item["work"] == budget), None)
            if checkpoint is not None and checkpoint["best_path_length_m"] is not None:
                defined += 1
        metric = point["rawPathExcessPct"]
        if (
            metric["nRuns"] != len(rrt_rows)
            or metric["nScenes"] != len({row["problem_fingerprint"] for row in rrt_rows})
            or metric["nSuccesses"] != defined
        ):
            raise ValueError(f"RRT* sensitivity counts disagree at {budget} samples")

    timing = _load_json(PUBLIC / DOWNLOADS["timingManifest"])
    timing_groups: dict[str, list[dict[str, Any]]] = {algorithm: [] for algorithm in summaries}
    for item in timing["records"]:
        planner = item.get("planner_record")
        if item["process_status"] != "completed" or not isinstance(planner, dict):
            continue
        algorithm = str(item["algorithm"])
        timing_groups[algorithm].append(planner)
        if not str(planner["configuration_id"]).startswith(f"{algorithm}@{__version__}|"):
            raise ValueError(f"timing configuration version mismatch: {planner['run_id']}")
        expected_run = _run_id(
            str(planner["problem_fingerprint"]),
            str(planner["configuration_id"]),
            int(item["planner_seed"]) if item["planner_seed"] is not None else None,
            str(planner["run_purpose"]),
            int(planner["timing_repetition"]),
        )
        if planner["run_id"] != expected_run:
            raise ValueError(f"stale timing run ID: {planner['run_id']}")
    for algorithm, group in timing_groups.items():
        metric = summaries[algorithm]["planningTimeMs"]
        if (
            len(group) != accepted_scenes * timing_repetitions
            or metric["nRuns"] != len(group)
            or metric["nScenes"] != len({item["problem_fingerprint"] for item in group})
            or metric["nSuccesses"] != sum(item["status"] == "success" for item in group)
        ):
            raise ValueError(f"timing summary counts disagree for {algorithm}")

    with (PUBLIC / DOWNLOADS["summariesCsv"]).open(encoding="utf-8", newline="") as stream:
        summary_rows = {row["planner_id"]: row for row in csv.DictReader(stream)}
    if set(summary_rows) != set(summaries):
        raise ValueError("summary CSV planner rows disagree with benchmark bundle")
    for algorithm, row in summary_rows.items():
        if any(row[key] != value for key, value in expected_metadata.items()):
            raise ValueError(f"summary CSV provenance mismatch for {algorithm}")
        summary = summaries[algorithm]
        success = summary["successRate"]
        planning = summary["planningTimeMs"]
        if (
            int(row["path_runs"]) != success["nRuns"]
            or int(row["path_scenes"]) != success["nScenes"]
            or int(row["path_successes"]) != success["nSuccesses"]
            or int(row["timing_runs"]) != planning["nRuns"]
            or int(row["timing_scenes"]) != planning["nScenes"]
            or int(row["timing_successes"]) != planning["nSuccesses"]
        ):
            raise ValueError(f"summary CSV sample counts disagree for {algorithm}")
        metric_columns = {
            "success_rate": success["value"],
            "success_ci95_low": success["ci95Low"],
            "success_ci95_high": success["ci95High"],
            "planning_time_median_ms": planning["median"],
            "raw_path_excess_median_pct": summary["rawPathExcessPct"]["median"],
            "smoothed_path_excess_median_pct": summary["smoothedPathExcessPct"]["median"],
            "minimum_clearance_median_m": summary["minimumClearanceM"]["median"],
        }
        for column, expected in metric_columns.items():
            actual = float(row[column]) if row[column] else None
            if (actual is None) != (expected is None) or (
                actual is not None
                and expected is not None
                and not math.isclose(actual, float(expected), rel_tol=1e-12, abs_tol=1e-12)
            ):
                raise ValueError(f"summary CSV metric disagrees for {algorithm}.{column}")
    return len(rows)


def main() -> int:
    trajectories = _audit_demo()
    records = _audit_benchmark()
    print(f"audited {trajectories} demo trajectories and {records} benchmark records")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
