"""Re-audit committed web trajectories and benchmark provenance in Python."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
import subprocess
from itertools import pairwise
from pathlib import Path
from typing import Any

from uav3d.benchmark import _configuration_id, _run_id, problem_fingerprint
from uav3d.dynamic import (
    DynamicScenario,
    dynamic_scenario_fingerprint,
    load_builtin_dynamic_scenario,
)
from uav3d.dynamic_collision import (
    minimum_dynamic_separation,
    point_is_free_at_time,
    spacetime_segment_is_free,
)
from uav3d.dynamic_study import (
    DOWNLOAD_ARTIFACTS as DYNAMIC_DOWNLOADS,
)
from uav3d.dynamic_study import (
    RECORD_FIELDS as DYNAMIC_RECORD_FIELDS,
)
from uav3d.dynamic_study import (
    WORK_UNITS as DYNAMIC_WORK_UNITS,
)
from uav3d.dynamic_study import (
    build_dynamic_bundle,
    dynamic_record_rows,
    dynamic_run_id,
)
from uav3d.geometry import Point3, as_point, distance, polyline_length
from uav3d.planners.base import PlanningBudget
from uav3d.predictive_scenarios import load_predictive_scenario
from uav3d.predictive_study import (
    DOWNLOAD_ARTIFACTS as PREDICTIVE_DOWNLOADS,
)
from uav3d.predictive_study import (
    PREDICTIVE_FLAGS,
    build_predictive_bundle,
    predictive_record_rows,
    predictive_run_id,
)
from uav3d.predictive_study import (
    RECORD_FIELDS as PREDICTIVE_RECORD_FIELDS,
)
from uav3d.predictive_study import (
    WORK_UNITS as PREDICTIVE_WORK_UNITS,
)
from uav3d.scene import load_builtin_scene
from uav3d.validation import audit_path

ROOT = Path(__file__).parents[1]
PUBLIC = ROOT / "web" / "public"
DOWNLOADS = {
    "recordsCsv": "benchmark-records.csv",
    "summariesCsv": "benchmark-summary.csv",
    "datasetManifest": "dataset-manifest.json",
    "timingManifest": "timing-manifest.json",
}
PREDICTIVE_SCENARIO_IDS = (
    "wait-then-straight",
    "closing-window",
    "periodic-traffic",
    "chained-restrictions",
    "multi-obstacle",
    "vertical-time-window",
    "urban-canyon-merge",
    "rooftop-transfer",
    "braided-skyway",
    "harbor-switchback",
)
PREDICTIVE_PLANNER_IDS = (
    "repeated-astar-3d",
    "dstar-lite-reset-3d",
    "dstar-lite-reuse-3d",
    "space-time-astar-4d",
)


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain an object")
    return value


def _validate_source_ancestor(source_commit: str, *, label: str) -> None:
    if re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", source_commit) is None:
        raise ValueError(f"{label} sourceCommit must be a full lowercase Git object ID")
    source_exists = subprocess.run(
        ["git", "cat-file", "-e", f"{source_commit}^{{commit}}"],
        cwd=ROOT,
        check=False,
        capture_output=True,
    )
    if source_exists.returncode != 0:
        raise ValueError(f"{label} sourceCommit is not present in repository history")
    source_is_ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", source_commit, "HEAD"],
        cwd=ROOT,
        check=False,
        capture_output=True,
    )
    if source_is_ancestor.returncode != 0:
        raise ValueError(f"{label} sourceCommit is not an ancestor of HEAD")


def _package_version_at(source_commit: str) -> str:
    result = subprocess.run(
        ["git", "show", f"{source_commit}:src/uav3d/version.py"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    match = re.search(r'^__version__\s*=\s*["\']([^"\']+)["\']\s*$', result.stdout, re.M)
    if match is None:
        raise ValueError("could not resolve package version at benchmark sourceCommit")
    return match.group(1)


def _require_paths_at_commit(source_commit: str, paths: tuple[str, ...], *, label: str) -> None:
    for path in paths:
        result = subprocess.run(
            ["git", "cat-file", "-e", f"{source_commit}:{path}"],
            cwd=ROOT,
            check=False,
            capture_output=True,
        )
        if result.returncode != 0:
            raise ValueError(f"{label} sourceCommit does not contain required generator {path}")


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
    _validate_source_ancestor(source_commit, label="benchmark")
    source_version = _package_version_at(source_commit)

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
        planning_budget = PlanningBudget(row["work_unit"], int(row["work_limit"]), wall_limit)
        parameters = json.loads(row["parameters_json"])
        expected_configuration = _configuration_id(
            algorithm,
            planning_budget,
            parameters,
            package_version=source_version,
        )
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
        if not row["configuration_id"].startswith(f"{algorithm}@{source_version}|"):
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
        sample_budget = int(point["sampleBudget"])
        defined = 0
        for row in rrt_rows:
            trace = json.loads(row["quality_trace_json"])
            checkpoint = next((item for item in trace if item["work"] == sample_budget), None)
            if checkpoint is not None and checkpoint["best_path_length_m"] is not None:
                defined += 1
        metric = point["rawPathExcessPct"]
        if (
            metric["nRuns"] != len(rrt_rows)
            or metric["nScenes"] != len({row["problem_fingerprint"] for row in rrt_rows})
            or metric["nSuccesses"] != defined
        ):
            raise ValueError(f"RRT* sensitivity counts disagree at {sample_budget} samples")

    timing = _load_json(PUBLIC / DOWNLOADS["timingManifest"])
    timing_groups: dict[str, list[dict[str, Any]]] = {algorithm: [] for algorithm in summaries}
    for item in timing["records"]:
        planner = item.get("planner_record")
        if item["process_status"] != "completed" or not isinstance(planner, dict):
            continue
        algorithm = str(item["algorithm"])
        timing_groups[algorithm].append(planner)
        if not str(planner["configuration_id"]).startswith(f"{algorithm}@{source_version}|"):
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


def _artifact_digest(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def _same_point(left: Point3, right: Point3) -> bool:
    return all(
        math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-8) for a, b in zip(left, right, strict=True)
    )


def _audit_dynamic_execution(
    scenario_id: str,
    run: dict[str, Any],
    *,
    cruise_speed: float,
) -> None:
    scenario = load_builtin_dynamic_scenario(scenario_id)
    raw_frames = run.get("frames")
    if not isinstance(raw_frames, list) or not raw_frames:
        raise ValueError(f"dynamic run has no frames: {scenario_id}/{run.get('plannerId')}")
    frames = [frame for frame in raw_frames if isinstance(frame, dict)]
    if len(frames) != len(raw_frames):
        raise ValueError(f"dynamic run contains a non-object frame: {scenario_id}")

    previous_time: float | None = None
    previous_vehicle: Point3 | None = None
    previous_executed: tuple[Point3, ...] | None = None
    for frame in frames:
        time_s = float(frame["timeS"])
        vehicle = as_point(frame["vehicle"])
        executed = tuple(as_point(point) for point in frame["executedPath"])
        if not executed or not _same_point(executed[0], scenario.static_scene.start):
            raise ValueError(f"dynamic executed path has an invalid origin: {scenario_id}")
        if not _same_point(executed[-1], vehicle):
            raise ValueError(f"dynamic executed path does not end at vehicle: {scenario_id}")
        if previous_time is None:
            if not math.isclose(time_s, 0.0, abs_tol=1e-12):
                raise ValueError(f"dynamic trace does not start at t=0: {scenario_id}")
            previous_time = time_s
            previous_vehicle = vehicle
            previous_executed = executed
            continue
        if time_s <= previous_time:
            raise ValueError(f"dynamic frame times are not strictly increasing: {scenario_id}")
        if previous_vehicle is None or previous_executed is None:
            raise AssertionError("previous dynamic state must be initialized")
        if len(executed) < len(previous_executed) or any(
            not _same_point(old, current)
            for old, current in zip(previous_executed, executed, strict=False)
        ):
            raise ValueError(f"dynamic executed path is not cumulative: {scenario_id}")

        traversal_points = (previous_vehicle, *executed[len(previous_executed) :])
        traversal_time = previous_time
        for start, end in pairwise(traversal_points):
            segment_duration = distance(start, end) / cruise_speed
            segment_end = traversal_time + segment_duration
            if segment_end > time_s + 1e-8:
                raise ValueError(f"dynamic motion exceeds the declared cruise speed: {scenario_id}")
            if not spacetime_segment_is_free(scenario, start, end, traversal_time, segment_end):
                raise ValueError(
                    f"dynamic executed trajectory failed collision audit: "
                    f"{scenario_id}/{run.get('plannerId')}"
                )
            traversal_time = segment_end
        if traversal_time < time_s - 1e-8 and not spacetime_segment_is_free(
            scenario, vehicle, vehicle, traversal_time, time_s
        ):
            raise ValueError(
                f"dynamic hold interval failed collision audit: "
                f"{scenario_id}/{run.get('plannerId')}"
            )
        previous_time = time_s
        previous_vehicle = vehicle
        previous_executed = executed
    if previous_executed is None or not math.isclose(
        polyline_length(previous_executed),
        float(run["metrics"]["executedPathLengthM"]),
        rel_tol=1e-10,
        abs_tol=1e-8,
    ):
        raise ValueError(f"dynamic executed length mismatch: {scenario_id}/{run.get('plannerId')}")


def _without_dynamic_provenance(bundle: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in bundle.items()
        if key not in {"generatedAt", "sourceCommit", "downloads"}
    }


def _first_difference(left: object, right: object, path: str = "root") -> str | None:
    """Return the first structural/value difference for actionable CI diagnostics."""

    if isinstance(left, dict) and isinstance(right, dict):
        if left.keys() != right.keys():
            return f"{path} keys differ: {sorted(left)} != {sorted(right)}"
        for key in left:
            difference = _first_difference(left[key], right[key], f"{path}.{key}")
            if difference is not None:
                return difference
        return None
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return f"{path} lengths differ: {len(left)} != {len(right)}"
        for index, (left_item, right_item) in enumerate(zip(left, right, strict=True)):
            difference = _first_difference(left_item, right_item, f"{path}[{index}]")
            if difference is not None:
                return difference
        return None
    if left != right:
        return f"{path} differs: {left!r} != {right!r}"
    return None


def _audit_dynamic() -> int:
    bundle = _load_json(PUBLIC / "dynamic-data.json")
    source_commit = str(bundle.get("sourceCommit", ""))
    _validate_source_ancestor(source_commit, label="dynamic")

    downloads = bundle.get("downloads")
    if not isinstance(downloads, dict) or set(downloads) != set(DYNAMIC_DOWNLOADS):
        raise ValueError("dynamic downloads must list exactly recordsCsv and scenarioManifest")
    for key, filename in DYNAMIC_DOWNLOADS.items():
        reference = downloads.get(key)
        if not isinstance(reference, dict) or reference.get("path") != filename:
            raise ValueError(f"invalid dynamic download reference: {key}")
        artifact = PUBLIC / filename
        if (
            reference.get("sha256") != _artifact_digest(artifact)
            or reference.get("bytes") != artifact.stat().st_size
        ):
            raise ValueError(f"dynamic download provenance mismatch: {filename}")

    expected, expected_manifest = build_dynamic_bundle(
        source_commit=source_commit,
        generated_at=str(bundle.get("generatedAt", "ignored")),
    )
    difference = _first_difference(
        _without_dynamic_provenance(bundle),
        _without_dynamic_provenance(expected),
    )
    if difference is not None:
        raise ValueError(
            "committed dynamic bundle differs from 12 deterministic reruns: " + difference
        )

    manifest = _load_json(PUBLIC / DYNAMIC_DOWNLOADS["scenarioManifest"])
    if manifest != expected_manifest:
        raise ValueError("dynamic scenario manifest differs from the fixed protocol selection")

    with (PUBLIC / DYNAMIC_DOWNLOADS["recordsCsv"]).open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if tuple(reader.fieldnames or ()) != DYNAMIC_RECORD_FIELDS:
            raise ValueError("dynamic records CSV columns differ from the declared schema")
    expected_rows = dynamic_record_rows(expected)
    if rows != expected_rows:
        raise ValueError("dynamic records CSV differs from dynamic-data.json")

    protocol = bundle.get("protocol")
    if not isinstance(protocol, dict):
        raise ValueError("dynamic protocol must be an object")
    cruise_speed = float(protocol["cruiseSpeedMps"])
    scenarios = bundle.get("scenarios")
    if not isinstance(scenarios, list):
        raise ValueError("dynamic scenarios must be an array")
    audited = 0
    run_ids: set[str] = set()
    for raw_scenario in scenarios:
        if not isinstance(raw_scenario, dict):
            raise ValueError("dynamic scenario must be an object")
        scenario_id = str(raw_scenario["id"])
        scenario_fingerprint = str(raw_scenario["fingerprint"])
        runs = raw_scenario.get("runs")
        if not isinstance(runs, list):
            raise ValueError(f"dynamic scenario has no runs: {scenario_id}")
        for run in runs:
            if not isinstance(run, dict):
                raise ValueError(f"dynamic run must be an object: {scenario_id}")
            run_id = str(run.get("runId", ""))
            planner_id = str(run.get("plannerId", ""))
            parameters = run.get("parameters")
            if not isinstance(parameters, dict):
                raise ValueError(f"dynamic run parameters must be an object: {scenario_id}")
            expected_run_id = dynamic_run_id(
                scenario_fingerprint,
                planner_id,
                protocol,
                parameters,
            )
            if run_id != expected_run_id or run_id in run_ids:
                raise ValueError(f"stale or duplicate dynamic run ID: {scenario_id}/{planner_id}")
            run_ids.add(run_id)
            metrics = run.get("metrics")
            if not isinstance(metrics, dict) or metrics.get("workUnit") != DYNAMIC_WORK_UNITS.get(
                planner_id
            ):
                raise ValueError(f"dynamic work unit mismatch: {scenario_id}/{planner_id}")
            _audit_dynamic_execution(scenario_id, run, cruise_speed=cruise_speed)
            audited += 1
    if audited != 12:
        raise ValueError("dynamic bundle must contain exactly four scenarios by three planners")
    return audited


def _audit_predictive_timed_path(
    scenario: DynamicScenario,
    planner_id: str,
    value: object,
    *,
    label: str,
    cruise_speed_mps: float,
) -> list[tuple[float, Point3]]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{label} must be a non-empty timed path: {scenario.scenario_id}")
    timed_path: list[tuple[float, Point3]] = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise ValueError(f"{label} waypoint is not an object: {scenario.scenario_id}")
        time_s = float(item["timeS"])
        position = as_point(item["position"])
        if not math.isfinite(time_s) or not all(math.isfinite(axis) for axis in position):
            raise ValueError(f"{label} contains a non-finite waypoint: {scenario.scenario_id}")
        if index == 0:
            if not math.isclose(time_s, 0.0, abs_tol=1e-12) or not _same_point(
                position, scenario.static_scene.start
            ):
                raise ValueError(f"{label} has an invalid origin: {scenario.scenario_id}")
            if not point_is_free_at_time(scenario, position, time_s):
                raise ValueError(f"{label} begins in collision: {scenario.scenario_id}")
        else:
            previous_time, previous_position = timed_path[-1]
            if time_s <= previous_time:
                raise ValueError(
                    f"{label} times are not strictly increasing: {scenario.scenario_id}"
                )
            speed = distance(previous_position, position) / (time_s - previous_time)
            if speed > cruise_speed_mps + max(1e-8, cruise_speed_mps * 1e-9):
                raise ValueError(
                    f"{label} exceeds the declared cruise speed: "
                    f"{scenario.scenario_id}/{planner_id}"
                )
        timed_path.append((time_s, position))

    for (start_time, start), (end_time, end) in pairwise(timed_path):
        if not spacetime_segment_is_free(scenario, start, end, start_time, end_time):
            raise ValueError(
                f"{label} failed continuous space-time audit: {scenario.scenario_id}/{planner_id}"
            )
    return timed_path


def _predictive_waits(
    timed_path: list[tuple[float, Point3]],
) -> list[tuple[float, float, Point3]]:
    return [
        (start_time, end_time, end)
        for (start_time, start), (end_time, end) in pairwise(timed_path)
        if _same_point(start, end)
    ]


def _same_timed_path(left: list[tuple[float, Point3]], right: list[tuple[float, Point3]]) -> bool:
    return len(left) == len(right) and all(
        math.isclose(left_time, right_time, rel_tol=1e-10, abs_tol=1e-8)
        and _same_point(left_point, right_point)
        for (left_time, left_point), (right_time, right_point) in zip(left, right, strict=True)
    )


def _audit_predictive_wait_records(
    scenario_id: str,
    value: object,
    expected: list[tuple[float, float, Point3]],
) -> None:
    if not isinstance(value, list) or len(value) != len(expected):
        raise ValueError(f"predictive wait-interval count mismatch: {scenario_id}")
    for index, (record, (start_time, end_time, position)) in enumerate(
        zip(value, expected, strict=True)
    ):
        if not isinstance(record, dict) or not isinstance(record.get("reason"), str):
            raise ValueError(f"predictive wait interval is incomplete: {scenario_id}/{index}")
        if not record["reason"].strip():
            raise ValueError(f"predictive wait interval has no reason: {scenario_id}/{index}")
        if (
            not math.isclose(float(record["startTimeS"]), start_time, rel_tol=1e-10, abs_tol=1e-8)
            or not math.isclose(float(record["endTimeS"]), end_time, rel_tol=1e-10, abs_tol=1e-8)
            or not _same_point(as_point(record["position"]), position)
        ):
            raise ValueError(f"predictive wait interval disagrees with timedPath: {scenario_id}")


def _finite_smoothing_number(smoothing: dict[str, Any], key: str) -> float:
    value = smoothing.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"predictive smoothing {key} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"predictive smoothing {key} must be finite")
    return parsed


def _audit_discrete_kinematic_diagnostic(
    scenario_id: str,
    value: object,
    *,
    path_kind: str,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"predictive {path_kind} kinematic diagnostics are missing: {scenario_id}")
    if (
        value.get("status") != "discrete-diagnostic-only"
        or value.get("continuousDynamicsCertified") is not False
    ):
        raise ValueError(f"predictive {path_kind} kinematic scope is overstated: {scenario_id}")
    integer_fields = ("segmentCount", "movementSegmentCount", "reversalCount")
    numeric_fields = (
        "reversalThresholdDeg",
        "maxSpeedMps",
        "maxDiscreteVelocityChangeMps",
        "maxDiscreteAccelerationProxyMps2",
        "maxAbsClimbRateMps",
    )
    for key in integer_fields:
        item = value.get(key)
        if isinstance(item, bool) or not isinstance(item, int) or item < 0:
            raise ValueError(f"predictive {path_kind} kinematic {key} is invalid: {scenario_id}")
    if value["movementSegmentCount"] > value["segmentCount"]:
        raise ValueError(f"predictive {path_kind} movement count exceeds segments: {scenario_id}")
    for key in numeric_fields:
        item = value.get(key)
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(
                f"predictive {path_kind} kinematic {key} is not numeric: {scenario_id}"
            )
        parsed = float(item)
        if not math.isfinite(parsed) or parsed < 0.0:
            raise ValueError(f"predictive {path_kind} kinematic {key} is invalid: {scenario_id}")
    threshold = float(value["reversalThresholdDeg"])
    if threshold <= 0.0 or threshold > 180.0:
        raise ValueError(f"predictive {path_kind} reversal threshold is invalid: {scenario_id}")
    return value


def _audit_kinematic_diagnostics(scenario_id: str, value: object) -> None:
    if not isinstance(value, dict):
        raise ValueError(f"predictive kinematic diagnostics are missing: {scenario_id}")
    if (
        value.get("status") != "discrete-diagnostic-only"
        or value.get("continuousDynamicsCertified") is not False
    ):
        raise ValueError(f"predictive kinematic scope is overstated: {scenario_id}")
    for path_kind in ("raw", "output"):
        _audit_discrete_kinematic_diagnostic(
            scenario_id,
            value.get(path_kind),
            path_kind=path_kind,
        )


def _execution_number(
    scenario_id: str,
    value: dict[str, Any],
    key: str,
    *,
    nullable: bool = False,
) -> float | None:
    item = value.get(key)
    if nullable and item is None:
        return None
    if isinstance(item, bool) or not isinstance(item, (int, float)):
        raise ValueError(f"predictive execution {key} must be numeric: {scenario_id}")
    parsed = float(item)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(
            f"predictive execution {key} must be finite and non-negative: {scenario_id}"
        )
    return parsed


def _audit_execution_envelope(scenario_id: str, value: object) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"predictive execution envelope is missing: {scenario_id}")
    expected = {
        "model": "discrete-segment-average-envelope-v1",
        "maxSpeedMps": 8.0,
        "maxAbsClimbRateMps": 3.0,
        "maxDiscreteAccelerationProxyMps2": 4.0,
        "reversalThresholdDeg": 150.0,
        "allowReversals": False,
        "maxExecutionTimeS": 90.0,
        "continuousDynamicsCertified": False,
    }
    for key, expected_value in expected.items():
        item = value.get(key)
        if isinstance(expected_value, float):
            if (
                isinstance(item, bool)
                or not isinstance(item, (int, float))
                or not math.isclose(float(item), expected_value, rel_tol=1e-10, abs_tol=1e-8)
            ):
                raise ValueError(
                    f"predictive execution envelope {key} is not frozen: {scenario_id}"
                )
        elif item != expected_value:
            raise ValueError(f"predictive execution envelope {key} is invalid: {scenario_id}")
    return value


def _audit_execution_qualification(
    scenario_id: str,
    value: object,
) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError(f"predictive execution qualification is invalid: {scenario_id}")
    status = value.get("status")
    qualified = value.get("qualified")
    violations = value.get("violations")
    allowed_violations = {
        "speed-limit-exceeded",
        "climb-rate-limit-exceeded",
        "acceleration-proxy-limit-exceeded",
        "reversal-not-allowed",
        "execution-time-limit-exceeded",
    }
    if (
        status not in {"qualified", "not-qualified"}
        or not isinstance(qualified, bool)
        or value.get("continuousDynamicsCertified") is not False
        or not isinstance(violations, list)
        or len(set(violations)) != len(violations)
        or any(not isinstance(item, str) or item not in allowed_violations for item in violations)
        or qualified is not (status == "qualified")
        or qualified is not (len(violations) == 0)
    ):
        raise ValueError(f"predictive execution qualification is inconsistent: {scenario_id}")
    _audit_discrete_kinematic_diagnostic(
        scenario_id,
        value.get("diagnostics"),
        path_kind="execution qualification",
    )
    _execution_number(
        scenario_id,
        value,
        "boundaryAwareMaxDiscreteAccelerationProxyMps2",
    )
    return value


def _audit_execution_metadata(scenario_id: str, value: object) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"predictive execution metadata is missing: {scenario_id}")
    status = value.get("status")
    statuses = {
        "not-evaluated",
        "qualified",
        "reversal-not-allowed",
        "execution-time-limit-exceeded",
        "time-parameterization-did-not-converge",
        "dynamic-collision-after-retiming",
    }
    qualified = value.get("qualified")
    collision_certified = value.get("collisionCertified")
    if (
        status not in statuses
        or not isinstance(qualified, bool)
        or not isinstance(collision_certified, bool)
        or value.get("collisionCertificationScope") != "dense-piecewise-linear-space-time-path"
        or value.get("continuousDynamicsCertified") is not False
    ):
        raise ValueError(f"predictive execution status is invalid: {scenario_id}")
    _audit_execution_envelope(scenario_id, value.get("envelope"))
    qualification = _audit_execution_qualification(scenario_id, value.get("qualification"))
    iterations = value.get("timingIterations")
    if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations < 0:
        raise ValueError(f"predictive execution timingIterations is invalid: {scenario_id}")
    durations = {
        key: _execution_number(scenario_id, value, key, nullable=True)
        for key in ("originalDurationS", "candidateDurationS", "addedDurationS")
    }
    if status == "not-evaluated":
        if (
            qualified
            or collision_certified
            or qualification is not None
            or iterations != 0
            or any(item is not None for item in durations.values())
        ):
            raise ValueError(f"predictive not-evaluated execution is inconsistent: {scenario_id}")
        return value
    if qualification is None or any(item is None for item in durations.values()):
        raise ValueError(f"predictive evaluated execution lacks evidence: {scenario_id}")
    original = durations["originalDurationS"]
    candidate = durations["candidateDurationS"]
    added = durations["addedDurationS"]
    assert original is not None and candidate is not None and added is not None
    if candidate < original - 1e-8 or not math.isclose(
        added,
        candidate - original,
        rel_tol=1e-10,
        abs_tol=1e-8,
    ):
        raise ValueError(f"predictive execution retiming shortened the path: {scenario_id}")
    if status == "qualified":
        if not qualified or not collision_certified or qualification.get("qualified") is not True:
            raise ValueError(f"predictive qualified execution is incomplete: {scenario_id}")
    elif qualified or collision_certified:
        raise ValueError(f"predictive failed execution exposes a candidate: {scenario_id}")
    elif (
        status == "dynamic-collision-after-retiming" and qualification.get("qualified") is not True
    ):
        raise ValueError(f"predictive retimed collision lacks timing qualification: {scenario_id}")
    return value


def _audit_predictive_smoothing(
    scenario_id: str,
    raw_path: list[tuple[float, Point3]],
    timed_path: list[tuple[float, Point3]],
    value: object,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"predictive run has no smoothing record: {scenario_id}")
    method = value.get("method")
    applied = value.get("applied")
    certified = value.get("certified")
    if (
        not isinstance(method, str)
        or not isinstance(applied, bool)
        or not isinstance(certified, bool)
    ):
        raise ValueError(f"predictive smoothing status is invalid: {scenario_id}")
    if (
        value.get("collisionCertified") is not certified
        or value.get("collisionCertificationScope") != "dense-piecewise-linear-space-time-path"
        or (applied and not certified)
    ):
        raise ValueError(f"predictive collision-certification scope is invalid: {scenario_id}")
    _audit_kinematic_diagnostics(scenario_id, value.get("kinematicDiagnostics"))
    execution = _audit_execution_metadata(scenario_id, value.get("execution"))
    raw_count = value.get("rawWaypointCount")
    output_count = value.get("outputWaypointCount")
    rounded_count = value.get("roundedCornerCount")
    if (
        isinstance(raw_count, bool)
        or not isinstance(raw_count, int)
        or isinstance(output_count, bool)
        or not isinstance(output_count, int)
        or isinstance(rounded_count, bool)
        or not isinstance(rounded_count, int)
        or raw_count != len(raw_path)
        or output_count != len(timed_path)
        or rounded_count < 0
    ):
        raise ValueError(f"predictive smoothing counts are inconsistent: {scenario_id}")

    requested_radius = _finite_smoothing_number(value, "requestedTurnRadiusM")
    sample_spacing = _finite_smoothing_number(value, "sampleSpacingM")
    before = _finite_smoothing_number(value, "maxTurnAngleBeforeDeg")
    after = _finite_smoothing_number(value, "maxTurnAngleAfterDeg")
    if requested_radius <= 0 or sample_spacing <= 0 or before < 0 or after < 0:
        raise ValueError(f"predictive smoothing parameters are invalid: {scenario_id}")

    applied_radius = value.get("appliedTurnRadiusM")
    if applied:
        if method != "sampled-circular-fillet" or rounded_count <= 0:
            raise ValueError(f"applied predictive smoothing has invalid metadata: {scenario_id}")
        if isinstance(applied_radius, bool) or not isinstance(applied_radius, (int, float)):
            raise ValueError(f"applied predictive smoothing has no radius: {scenario_id}")
        parsed_radius = float(applied_radius)
        if (
            not math.isfinite(parsed_radius)
            or parsed_radius <= 0
            or parsed_radius > requested_radius + 1e-8
            or _same_timed_path(raw_path, timed_path)
            or output_count < raw_count
            # acos near a 180-degree reversal is numerically ill-conditioned; tolerate only the
            # micro-degree serialization drift observed at that boundary.
            or after > before + 1e-5
        ):
            raise ValueError(f"applied predictive smoothing is inconsistent: {scenario_id}")
        return execution

    if method not in {
        "raw-fallback",
        "raw-no-roundable-corners",
        "not-run-uncertified-raw-path",
    }:
        raise ValueError(f"predictive smoothing fallback method is invalid: {scenario_id}")
    if (
        applied_radius is not None
        or rounded_count != 0
        or raw_count != output_count
        or not _same_timed_path(raw_path, timed_path)
        or not math.isclose(before, after, rel_tol=1e-10, abs_tol=1e-8)
    ):
        raise ValueError(f"predictive smoothing fallback is inconsistent: {scenario_id}")
    return execution


def _audit_predictive_frames(
    scenario_id: str,
    value: object,
    timed_path: list[tuple[float, Point3]],
) -> None:
    if not isinstance(value, list) or not value or len(value) > len(timed_path):
        raise ValueError(
            f"predictive event frames must be a compact timedPath subset: {scenario_id}"
        )
    previous_time = -math.inf
    for index, frame in enumerate(value):
        if not isinstance(frame, dict):
            raise ValueError(f"predictive frame is not an object: {scenario_id}/{index}")
        if "path" in frame or "executedPath" in frame:
            raise ValueError(
                f"predictive frames must not duplicate trajectory arrays: {scenario_id}"
            )
        time_s = float(frame["timeS"])
        if time_s <= previous_time:
            raise ValueError(f"predictive event frames are not strictly ordered: {scenario_id}")
        matching_waypoint = next(
            (
                position
                for waypoint_time, position in timed_path
                if math.isclose(waypoint_time, time_s, rel_tol=1e-10, abs_tol=1e-8)
            ),
            None,
        )
        if matching_waypoint is None:
            raise ValueError(f"predictive frame time is absent from timedPath: {scenario_id}")
        if not _same_point(as_point(frame["vehicle"]), matching_waypoint):
            raise ValueError(f"predictive frame vehicle disagrees with timedPath: {scenario_id}")
        event = frame.get("event")
        if not isinstance(event, dict) or event.get("kind") in {None, "none"}:
            raise ValueError(f"predictive compact frame lacks a semantic event: {scenario_id}")
        previous_time = time_s
    if not math.isclose(
        float(value[0]["timeS"]), timed_path[0][0], abs_tol=1e-8
    ) or not math.isclose(float(value[-1]["timeS"]), timed_path[-1][0], abs_tol=1e-8):
        raise ValueError(f"predictive event frames must anchor mission endpoints: {scenario_id}")


def _audit_predictive_metrics(
    scenario: DynamicScenario,
    path: list[tuple[float, Point3]],
    value: object,
    *,
    label: str,
    expected_success: bool,
    expected_failure_reason: object,
) -> dict[str, Any]:
    scenario_id = scenario.scenario_id
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object: {scenario_id}")
    metrics = value
    positions = tuple(point for _, point in path)
    waits = _predictive_waits(path)
    observed_length = polyline_length(positions)
    observed_wait = math.fsum(end_time - start_time for start_time, end_time, _ in waits)
    direct_distance = distance(scenario.static_scene.start, scenario.static_scene.goal)
    expected_numbers = {
        "executedPathLengthM": observed_length,
        "waitTimeS": observed_wait,
        "directDistanceM": direct_distance,
    }
    for key, expected in expected_numbers.items():
        item = metrics.get(key)
        if (
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(float(item))
            or not math.isclose(float(item), expected, rel_tol=1e-10, abs_tol=1e-8)
        ):
            raise ValueError(f"{label} {key} disagrees with its path: {scenario_id}")

    separation = minimum_dynamic_separation(scenario, path)
    recorded_separation = metrics.get("minimumSeparationM")
    recorded_witness = metrics.get("minimumSeparationWitness")
    if separation is None:
        if recorded_separation is not None or recorded_witness is not None:
            raise ValueError(f"{label} separation diagnostic is spurious: {scenario_id}")
    else:
        if (
            isinstance(recorded_separation, bool)
            or not isinstance(recorded_separation, (int, float))
            or not math.isfinite(float(recorded_separation))
            or not math.isclose(
                float(recorded_separation),
                separation.separation_m,
                rel_tol=1e-10,
                abs_tol=1e-8,
            )
            or not isinstance(recorded_witness, dict)
        ):
            raise ValueError(f"{label} minimum separation mismatch: {scenario_id}")
        expected_text = {
            "obstacleId": separation.obstacle_id,
            "obstacleKind": separation.obstacle_kind,
            "method": separation.method,
        }
        if any(recorded_witness.get(key) != expected for key, expected in expected_text.items()):
            raise ValueError(f"{label} separation witness identity mismatch: {scenario_id}")
        if recorded_witness.get("exact") is not separation.exact:
            raise ValueError(f"{label} separation witness exactness mismatch: {scenario_id}")
        witness_numbers = {
            "separationM": separation.separation_m,
            "timeS": separation.time_s,
            "declaredSafetyMarginM": separation.declared_safety_margin_m,
        }
        for key, expected in witness_numbers.items():
            item = recorded_witness.get(key)
            absolute_tolerance = 1e-6 if key == "timeS" and not separation.exact else 1e-8
            if (
                isinstance(item, bool)
                or not isinstance(item, (int, float))
                or not math.isfinite(float(item))
                or not math.isclose(
                    float(item), expected, rel_tol=1e-10, abs_tol=absolute_tolerance
                )
            ):
                raise ValueError(f"{label} separation witness {key} mismatch: {scenario_id}")
        position_tolerance = 1e-8 if separation.exact else 1e-6
        for key, expected in {
            "vehiclePosition": separation.vehicle_position,
            "obstaclePosition": separation.obstacle_position,
        }.items():
            item = recorded_witness.get(key)
            if (
                not isinstance(item, list)
                or len(item) != 3
                or any(
                    isinstance(coordinate, bool)
                    or not isinstance(coordinate, (int, float))
                    or not math.isfinite(float(coordinate))
                    or not math.isclose(
                        float(coordinate),
                        expected[index],
                        rel_tol=1e-10,
                        abs_tol=position_tolerance,
                    )
                    for index, coordinate in enumerate(item)
                )
            ):
                raise ValueError(f"{label} separation witness {key} mismatch: {scenario_id}")
        if float(recorded_separation) < separation.declared_safety_margin_m - 1e-8:
            raise ValueError(f"{label} minimum separation violates safety margin: {scenario_id}")

    success = metrics.get("success")
    failure_reason = metrics.get("failureReason")
    if success is not expected_success or failure_reason != expected_failure_reason:
        raise ValueError(f"{label} status disagrees with its evidence layer: {scenario_id}")
    if success:
        arrival = metrics.get("arrivalTimeS")
        travel = metrics.get("travelTimeS")
        path_excess = metrics.get("pathExcessPct")
        if (
            failure_reason is not None
            or arrival is None
            or travel is None
            or path_excess is None
            or not _same_point(positions[-1], scenario.static_scene.goal)
            or not math.isclose(float(arrival), path[-1][0], rel_tol=1e-10, abs_tol=1e-8)
            or not math.isclose(
                float(travel),
                path[-1][0] - path[0][0] - observed_wait,
                rel_tol=1e-10,
                abs_tol=1e-8,
            )
            or not math.isclose(
                float(path_excess),
                (observed_length / direct_distance - 1.0) * 100.0,
                rel_tol=1e-10,
                abs_tol=1e-8,
            )
        ):
            raise ValueError(f"{label} successful metrics are incomplete: {scenario_id}")
    elif failure_reason is None or any(
        metrics.get(key) is not None for key in ("arrivalTimeS", "travelTimeS", "pathExcessPct")
    ):
        raise ValueError(f"{label} failed metrics fabricate success values: {scenario_id}")
    safety_violations = metrics.get("safetyViolations")
    if isinstance(safety_violations, bool) or safety_violations != 0:
        raise ValueError(f"{label} contains a safety violation: {scenario_id}")
    return metrics


def _audit_predictive_execution(
    scenario: DynamicScenario,
    run: dict[str, Any],
    *,
    cruise_speed_mps: float,
) -> None:
    scenario_id = scenario.scenario_id
    planner_id = str(run.get("plannerId", ""))
    required_fields = {
        "rawTimedPath",
        "geometryTimedPath",
        "executionTimedPath",
        "smoothing",
        "geometryWaitIntervals",
        "executionWaitIntervals",
        "plannerMetrics",
        "geometryMetrics",
        "executionMetrics",
        "geometryFrames",
        "executionFrames",
    }
    if not required_fields.issubset(run):
        missing = sorted(required_fields.difference(run))
        raise ValueError(f"predictive v3 run lacks fields {missing}: {scenario_id}/{planner_id}")
    raw_path = _audit_predictive_timed_path(
        scenario,
        planner_id,
        run["rawTimedPath"],
        label="rawTimedPath",
        cruise_speed_mps=cruise_speed_mps,
    )
    geometry_path = _audit_predictive_timed_path(
        scenario,
        planner_id,
        run["geometryTimedPath"],
        label="geometryTimedPath",
        cruise_speed_mps=cruise_speed_mps,
    )
    if (
        not math.isclose(raw_path[0][0], geometry_path[0][0], rel_tol=0.0, abs_tol=1e-12)
        or not math.isclose(raw_path[-1][0], geometry_path[-1][0], rel_tol=1e-10, abs_tol=1e-8)
        or not _same_point(raw_path[0][1], geometry_path[0][1])
        or not _same_point(raw_path[-1][1], geometry_path[-1][1])
    ):
        raise ValueError(f"predictive smoothing changed path endpoints or times: {scenario_id}")
    raw_waits = _predictive_waits(raw_path)
    geometry_waits = _predictive_waits(geometry_path)
    if len(raw_waits) != len(geometry_waits) or any(
        not math.isclose(raw_start, geometry_start, rel_tol=1e-10, abs_tol=1e-8)
        or not math.isclose(raw_end, geometry_end, rel_tol=1e-10, abs_tol=1e-8)
        or not _same_point(raw_position, geometry_position)
        for (raw_start, raw_end, raw_position), (
            geometry_start,
            geometry_end,
            geometry_position,
        ) in zip(raw_waits, geometry_waits, strict=True)
    ):
        raise ValueError(f"predictive smoothing changed a planner wait interval: {scenario_id}")
    _audit_predictive_wait_records(
        scenario_id,
        run["geometryWaitIntervals"],
        geometry_waits,
    )
    execution = _audit_predictive_smoothing(
        scenario_id,
        raw_path,
        geometry_path,
        run["smoothing"],
    )
    _audit_predictive_frames(scenario_id, run["geometryFrames"], geometry_path)

    planner_success = run.get("status") == "success"
    planner_failure_reason = run.get("failureReason")
    planner_metrics = _audit_predictive_metrics(
        scenario,
        raw_path,
        run["plannerMetrics"],
        label="plannerMetrics",
        expected_success=planner_success,
        expected_failure_reason=planner_failure_reason,
    )
    geometry_metrics = _audit_predictive_metrics(
        scenario,
        geometry_path,
        run["geometryMetrics"],
        label="geometryMetrics",
        expected_success=planner_success,
        expected_failure_reason=planner_failure_reason,
    )
    for key in ("replans", "expandedStates", "workUnit"):
        if geometry_metrics.get(key) != planner_metrics.get(key):
            raise ValueError(f"predictive geometry {key} borrows inconsistent work: {scenario_id}")

    available = (
        execution.get("status") == "qualified"
        and execution.get("qualified") is True
        and execution.get("collisionCertified") is True
    )
    execution_fields = (
        run["executionTimedPath"],
        run["executionWaitIntervals"],
        run["executionMetrics"],
        run["executionFrames"],
    )
    if available is not all(item is not None for item in execution_fields):
        raise ValueError(
            f"predictive execution evidence availability is inconsistent: {scenario_id}"
        )
    if not available and any(item is not None for item in execution_fields):
        raise ValueError(f"predictive failed execution exposes candidate evidence: {scenario_id}")

    original_duration = execution.get("originalDurationS")
    if original_duration is not None and not math.isclose(
        float(original_duration),
        geometry_path[-1][0] - geometry_path[0][0],
        rel_tol=1e-10,
        abs_tol=1e-8,
    ):
        raise ValueError(f"predictive execution original duration mismatch: {scenario_id}")
    qualification = execution.get("qualification")
    if isinstance(qualification, dict):
        diagnostics = qualification.get("diagnostics")
        if (
            not isinstance(diagnostics, dict)
            or diagnostics.get("segmentCount") != len(geometry_path) - 1
        ):
            raise ValueError(f"predictive execution qualification count mismatch: {scenario_id}")

    if not available:
        return
    execution_path = _audit_predictive_timed_path(
        scenario,
        planner_id,
        run["executionTimedPath"],
        label="executionTimedPath",
        cruise_speed_mps=cruise_speed_mps,
    )
    if len(execution_path) != len(geometry_path):
        raise ValueError(f"predictive retiming changed waypoint count: {scenario_id}")
    paired_paths = zip(geometry_path, execution_path, strict=True)
    for index, (
        (geometry_start_time, geometry_start),
        (execution_start_time, execution_start),
    ) in enumerate(paired_paths):
        if not _same_point(geometry_start, execution_start):
            raise ValueError(f"predictive retiming changed geometry: {scenario_id}/{index}")
        if index == 0:
            continue
        geometry_duration = geometry_start_time - geometry_path[index - 1][0]
        execution_duration = execution_start_time - execution_path[index - 1][0]
        if execution_duration < geometry_duration - 1e-8:
            raise ValueError(f"predictive retiming shortened a segment: {scenario_id}/{index}")
        if _same_point(geometry_start, geometry_path[index - 1][1]) and not math.isclose(
            execution_duration,
            geometry_duration,
            rel_tol=1e-10,
            abs_tol=1e-8,
        ):
            raise ValueError(f"predictive retiming changed a wait duration: {scenario_id}/{index}")
    execution_waits = _predictive_waits(execution_path)
    if not math.isclose(
        math.fsum(end - start for start, end, _ in execution_waits),
        math.fsum(end - start for start, end, _ in geometry_waits),
        rel_tol=1e-10,
        abs_tol=1e-8,
    ):
        raise ValueError(f"predictive retiming changed total wait duration: {scenario_id}")
    _audit_predictive_wait_records(
        scenario_id,
        run["executionWaitIntervals"],
        execution_waits,
    )
    _audit_predictive_frames(scenario_id, run["executionFrames"], execution_path)
    candidate_duration = execution.get("candidateDurationS")
    if candidate_duration is None or not math.isclose(
        float(candidate_duration),
        execution_path[-1][0] - execution_path[0][0],
        rel_tol=1e-10,
        abs_tol=1e-8,
    ):
        raise ValueError(f"predictive execution candidate duration mismatch: {scenario_id}")
    execution_metrics = _audit_predictive_metrics(
        scenario,
        execution_path,
        run["executionMetrics"],
        label="executionMetrics",
        expected_success=planner_success,
        expected_failure_reason=planner_failure_reason,
    )
    for key in ("replans", "expandedStates", "workUnit"):
        if execution_metrics.get(key) != planner_metrics.get(key):
            raise ValueError(f"predictive execution {key} borrows inconsistent work: {scenario_id}")


def _without_predictive_provenance(bundle: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in bundle.items()
        if key not in {"generatedAt", "sourceCommit", "downloads"}
    }


def _contains_forbidden_keys(value: object, forbidden: frozenset[str]) -> bool:
    if isinstance(value, dict):
        return bool(forbidden.intersection(value)) or any(
            _contains_forbidden_keys(item, forbidden) for item in value.values()
        )
    if isinstance(value, list):
        return any(_contains_forbidden_keys(item, forbidden) for item in value)
    return False


def _audit_predictive() -> int:
    bundle = _load_json(PUBLIC / "predictive-data.json")
    if bundle.get("schemaVersion") != 3:
        raise ValueError("predictive bundle must use schemaVersion 3")
    source_commit = str(bundle.get("sourceCommit", ""))
    _validate_source_ancestor(source_commit, label="predictive")
    _require_paths_at_commit(
        source_commit,
        (
            "src/uav3d/predictive.py",
            "src/uav3d/predictive_city.py",
            "src/uav3d/predictive_scenarios.py",
            "src/uav3d/predictive_smoothing.py",
            "src/uav3d/predictive_study.py",
            "src/uav3d/trajectory_timing.py",
            "src/uav3d/dynamic_collision.py",
            "src/uav3d/kinematics.py",
            "src/uav3d/planners/dstar_lite.py",
            "src/uav3d/planners/space_time_astar.py",
        ),
        label="predictive",
    )
    if _package_version_at(source_commit) != "0.7.0":
        raise ValueError("predictive sourceCommit must identify the v0.7.0 implementation")

    protocol = bundle.get("protocol")
    scenarios = bundle.get("scenarios")
    if not isinstance(protocol, dict) or not isinstance(scenarios, list):
        raise ValueError("predictive protocol and scenarios must be structured objects")
    if protocol.get("id") != "predictive-space-time-v4":
        raise ValueError("predictive protocol must use predictive-space-time-v4")
    if protocol.get("continuousDynamicsCertified") is not False:
        raise ValueError("predictive protocol must not claim continuous-dynamics certification")
    _audit_execution_envelope("protocol", protocol.get("executionEnvelope"))
    cruise_speed_value = protocol.get("cruiseSpeedMps")
    if (
        isinstance(cruise_speed_value, bool)
        or not isinstance(cruise_speed_value, (int, float))
        or not math.isfinite(float(cruise_speed_value))
        or float(cruise_speed_value) <= 0
    ):
        raise ValueError("predictive protocol cruiseSpeedMps must be finite and positive")
    cruise_speed_mps = float(cruise_speed_value)

    expected_work_units = {
        "repeated-astar-3d": "expanded-nodes",
        "dstar-lite-reset-3d": "queue-pops",
        "dstar-lite-reuse-3d": "queue-pops",
        "space-time-astar-4d": "expanded-spacetime-states",
    }
    expected_predictive_flags = {
        "repeated-astar-3d": False,
        "dstar-lite-reset-3d": False,
        "dstar-lite-reuse-3d": False,
        "space-time-astar-4d": True,
    }
    if (
        tuple(PREDICTIVE_WORK_UNITS) != PREDICTIVE_PLANNER_IDS
        or expected_work_units != PREDICTIVE_WORK_UNITS
        or tuple(PREDICTIVE_FLAGS) != PREDICTIVE_PLANNER_IDS
        or expected_predictive_flags != PREDICTIVE_FLAGS
    ):
        raise ValueError("predictive study constants do not define the fixed four-planner matrix")
    planner_records = bundle.get("planners")
    if not isinstance(planner_records, list) or len(planner_records) != len(PREDICTIVE_PLANNER_IDS):
        raise ValueError("predictive planner metadata must contain exactly four planners")
    planner_contract: list[tuple[str, object]] = []
    for record in planner_records:
        if not isinstance(record, dict):
            raise ValueError("predictive planner metadata entries must be objects")
        planner_contract.append((str(record.get("id", "")), record.get("predictive")))
    if tuple(planner_contract) != tuple(
        (planner_id, expected_predictive_flags[planner_id]) for planner_id in PREDICTIVE_PLANNER_IDS
    ):
        raise ValueError("predictive planner metadata disagrees with the fixed planner matrix")

    downloads = bundle.get("downloads")
    if not isinstance(downloads, dict) or set(downloads) != set(PREDICTIVE_DOWNLOADS):
        raise ValueError("predictive downloads must list exactly recordsCsv and scenarioManifest")
    for key, filename in PREDICTIVE_DOWNLOADS.items():
        reference = downloads.get(key)
        artifact = PUBLIC / filename
        if not isinstance(reference, dict) or reference.get("path") != filename:
            raise ValueError(f"invalid predictive download reference: {key}")
        if (
            reference.get("sha256") != _artifact_digest(artifact)
            or reference.get("bytes") != artifact.stat().st_size
        ):
            raise ValueError(f"predictive download provenance mismatch: {filename}")

    manifest = _load_json(PUBLIC / PREDICTIVE_DOWNLOADS["scenarioManifest"])
    selection = manifest.get("selection")
    manifest_scenarios = manifest.get("scenarios")
    if manifest.get("schemaVersion") != 3:
        raise ValueError("predictive scenario manifest must use schemaVersion 3")
    if manifest.get("protocolId") != "predictive-space-time-v4":
        raise ValueError("predictive scenario manifest must use predictive-space-time-v4")
    if manifest.get("datasetId") != "predictive-execution-envelope-v0.7":
        raise ValueError("predictive scenario manifest must use predictive-execution-envelope-v0.7")
    if not isinstance(selection, dict) or selection.get("planner_outcomes_consulted") is not False:
        raise ValueError("predictive scenario selection must be independent of planner outcomes")
    if _contains_forbidden_keys(
        manifest,
        frozenset({"planner_id", "algorithm", "mission_success"}),
    ):
        raise ValueError("predictive scenario manifest contains planner-outcome selection fields")
    if (
        manifest.get("requested") != 10
        or manifest.get("accepted") != 10
        or manifest.get("rejected") != 0
        or manifest.get("acceptedByCohort") != {"calibration": 1, "demo": 3, "diagnostic": 6}
        or manifest.get("scenarioCount") != 10
        or manifest.get("runCount") != 40
        or not isinstance(manifest_scenarios, list)
    ):
        raise ValueError("predictive scenario manifest counts disagree with the fixed v0.7 cohort")
    manifest_scenario_ids: list[str] = []
    for record in manifest_scenarios:
        if not isinstance(record, dict) or record.get("selected") is not True:
            raise ValueError("predictive manifest scenario entry is invalid or unselected")
        manifest_scenario_ids.append(str(record.get("id", "")))
    if tuple(manifest_scenario_ids) != PREDICTIVE_SCENARIO_IDS:
        raise ValueError("predictive manifest scenario IDs differ from the fixed v0.7 cohort")

    expected, expected_manifest = build_predictive_bundle(
        source_commit=source_commit,
        generated_at=str(bundle.get("generatedAt", "ignored")),
    )
    difference = _first_difference(
        _without_predictive_provenance(bundle),
        _without_predictive_provenance(expected),
    )
    if difference is not None:
        raise ValueError(
            "committed predictive bundle differs from 40 deterministic reruns: " + difference
        )

    if manifest != expected_manifest:
        raise ValueError("predictive scenario manifest differs from the fixed protocol selection")

    with (PUBLIC / PREDICTIVE_DOWNLOADS["recordsCsv"]).open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if tuple(reader.fieldnames or ()) != PREDICTIVE_RECORD_FIELDS:
            raise ValueError("predictive records CSV columns differ from the declared schema")
    if rows != predictive_record_rows(expected):
        raise ValueError("predictive records CSV differs from predictive-data.json")

    scenario_ids: list[str] = []
    run_ids: set[str] = set()
    audited = 0
    for raw_scenario in scenarios:
        if not isinstance(raw_scenario, dict):
            raise ValueError("predictive scenario must be an object")
        scenario_id = str(raw_scenario["id"])
        scenario_ids.append(scenario_id)
        scenario = load_predictive_scenario(scenario_id)
        fingerprint = str(raw_scenario["fingerprint"])
        if fingerprint != dynamic_scenario_fingerprint(scenario):
            raise ValueError(f"predictive scenario fingerprint mismatch: {scenario_id}")
        runs = raw_scenario.get("runs")
        if not isinstance(runs, list) or len(runs) != len(PREDICTIVE_PLANNER_IDS):
            raise ValueError(f"predictive scenario has an incomplete planner matrix: {scenario_id}")
        planner_ids: list[str] = []
        for run in runs:
            if not isinstance(run, dict) or not isinstance(run.get("parameters"), dict):
                raise ValueError(f"predictive run must be a structured object: {scenario_id}")
            planner_id = str(run["plannerId"])
            planner_ids.append(planner_id)
            parameters = run["parameters"]
            run_id = str(run["runId"])
            expected_id = predictive_run_id(
                fingerprint,
                planner_id,
                protocol,
                parameters,
            )
            if run_id != expected_id or run_id in run_ids:
                raise ValueError(
                    f"stale or duplicate predictive run ID: {scenario_id}/{planner_id}"
                )
            run_ids.add(run_id)
            metrics = run.get("plannerMetrics")
            if (
                not isinstance(metrics, dict)
                or metrics.get("workUnit") != PREDICTIVE_WORK_UNITS.get(planner_id)
                or run.get("predictive") is not PREDICTIVE_FLAGS.get(planner_id)
            ):
                raise ValueError(
                    f"predictive planner contract mismatch: {scenario_id}/{planner_id}"
                )
            _audit_predictive_execution(
                scenario,
                run,
                cruise_speed_mps=cruise_speed_mps,
            )
            audited += 1
        if tuple(planner_ids) != PREDICTIVE_PLANNER_IDS:
            raise ValueError(f"predictive planner order or membership mismatch: {scenario_id}")
    if tuple(scenario_ids) != PREDICTIVE_SCENARIO_IDS or audited != 40 or len(run_ids) != 40:
        raise ValueError("predictive bundle must contain exactly ten scenarios by four planners")
    return audited


def main() -> int:
    trajectories = _audit_demo()
    records = _audit_benchmark()
    dynamic_runs = _audit_dynamic()
    predictive_runs = _audit_predictive()
    print(
        f"audited {trajectories} demo trajectories, {records} benchmark records, "
        f"{dynamic_runs} dynamic runs, and {predictive_runs} predictive runs"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
