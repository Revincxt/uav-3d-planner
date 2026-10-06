"""Re-audit the current Manhattan records without recomputing or replacing them."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from itertools import pairwise
from pathlib import Path
from typing import Any

from committed_inputs import assert_equivalent, input_record, load_inputs, restore_input
from export_manhattan_static_dynamic import mission_scenes, shared_dynamic_scenarios

from uav3d.benchmark import scene_fingerprint
from uav3d.dynamic import dynamic_scenario_fingerprint
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.manhattan_city import build_manhattan_city
from uav3d.manhattan_missions import audit_task_visits
from uav3d.manhattan_predictive import (
    _audit_serialized_case,
    build_manhattan_missions,
    study_runtime,
)
from uav3d.validation import audit_path

ROOT = Path(__file__).resolve().parents[1]


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def audit_downloads(bundle: dict[str, Any], public: Path) -> None:
    for artifact in bundle.get("downloads", {}).values():
        target = public / artifact["path"]
        if target.parent.resolve() != public.resolve():
            raise ValueError("Download leaves the public directory")
        digest = "sha256:" + hashlib.sha256(target.read_bytes()).hexdigest()
        if artifact["sha256"] != digest:
            raise ValueError(f"Download digest mismatch: {target.name}")


def audit_city(bundle: dict[str, Any], city: Any) -> None:
    if len(bundle["scenarios"]) != 8:
        raise ValueError("Each current study requires exactly eight missions")
    expected = city.to_web_buildings()
    for scenario in bundle["scenarios"]:
        if scenario["buildings"] != expected:
            raise ValueError(f"Changed or incomplete source geometry: {scenario['id']}")
        if scenario["city"]["sourceSha256"] != city.metadata["sourceSha256"]:
            raise ValueError("City source digest mismatch")
        if scenario["city"].get("planningRegion", {}).get("id") != "midtown-expanded-v3":
            raise ValueError("Stale planning region")
    provenance = bundle.get("computationSourceProvenance", bundle.get("sourceProvenance"))
    if provenance:
        for source in provenance["files"]:
            path = ROOT / source["path"]
            if not path.resolve().is_relative_to(ROOT):
                raise ValueError("Source provenance leaves the project")
            digest = "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != source["sha256"]:
                raise ValueError(f"Computation source changed: {source['path']}")


def audit_current(public: Path) -> dict[str, int]:
    city = build_manhattan_city()
    static = read_json(public / "demo-data.json")
    dynamic = read_json(public / "dynamic-data.json")
    predictive = read_json(public / "predictive-data.json")
    for bundle in (static, dynamic, predictive):
        audit_city(bundle, city)
        audit_downloads(bundle, public)
    inputs = load_inputs(public, city)
    missions = mission_scenes(city)
    baselines = [
        next(run for run in scenario["results"] if run["plannerId"] == "astar-3d")["paths"]["raw"]
        for scenario in static["scenarios"]
    ]
    dynamic_episodes = shared_dynamic_scenarios(missions, baselines)
    counts = {"static": 0, "dynamic": 0, "predictive": 0}
    run_ids: set[str] = set()
    for index, (_mission, regenerated_scene) in enumerate(missions):
        saved = inputs["static"][index]
        assert_equivalent(input_record(regenerated_scene), saved)
        scene = restore_input(saved, city, dynamic=False)
        exported = static["scenarios"][index]
        if exported["id"] != scene.scene_id or exported["fingerprint"] != scene_fingerprint(scene):
            raise ValueError("Static mission identity mismatch")
        if len(exported["results"]) != 3:
            raise ValueError("Static planner matrix is incomplete")
        for run in exported["results"]:
            if run["status"] != "success":
                raise ValueError("Static mission failed")
            for layer in ("raw", "smoothed"):
                path = run["paths"][layer]
                audit_task_visits(path, scene.metadata["missionTaskPoints"])
                audit = audit_path(scene, tuple(tuple(point) for point in path))
                if not audit.valid or not math.isclose(
                    audit.length_m, run["metrics"][layer + "LengthM"], abs_tol=0.001
                ):
                    raise ValueError(
                        f"Invalid static trajectory or metrics: {run['runId']}/{layer}"
                    )
            counts["static"] += 1
        saved = inputs["dynamic"][index]
        assert_equivalent(input_record(dynamic_episodes[index]), saved)
        episode = restore_input(saved, city, dynamic=True)
        recorded = dynamic["scenarios"][index]
        if recorded["id"] != episode.scenario_id or recorded[
            "fingerprint"
        ] != dynamic_scenario_fingerprint(episode):
            raise ValueError("Dynamic mission identity mismatch")
        if len(recorded["runs"]) != 3:
            raise ValueError("Dynamic planner matrix is incomplete")
        for run in recorded["runs"]:
            frames = run["frames"]
            if run["status"] != "success":
                raise ValueError("Dynamic mission failed")
            audit_task_visits(
                [f["vehicle"] for f in frames],
                scene.metadata["missionTaskPoints"],
                times=[f["timeS"] for f in frames],
            )
            for left, right in pairwise(frames):
                # Telemetry frames may bracket multiple corners. Audit the actual execution
                # prefix, not the straight chord between the two frame positions.
                points = right["executedPath"][len(left["executedPath"]) - 1 :]
                lengths = [math.dist(a, b) for a, b in pairwise(points)]
                total = math.fsum(lengths)
                clock = left["timeS"]
                if not total:
                    if not spacetime_segment_is_free(
                        episode,
                        tuple(left["vehicle"]),
                        tuple(right["vehicle"]),
                        clock,
                        right["timeS"],
                    ):
                        raise ValueError(f"Unsafe reactive hold: {run['runId']}")
                else:
                    traversed = 0.0
                    for (a, b), length in zip(pairwise(points), lengths, strict=True):
                        traversed += length
                        end = left["timeS"] + traversed / total * (right["timeS"] - left["timeS"])
                        if not spacetime_segment_is_free(episode, tuple(a), tuple(b), clock, end):
                            raise ValueError(f"Unsafe reactive execution: {run['runId']}")
                        clock = end
            counts["dynamic"] += 1
    runtime = study_runtime()
    for index, (regenerated_episode, recorded) in enumerate(
        zip(build_manhattan_missions(city), predictive["scenarios"], strict=True)
    ):
        saved = inputs["predictive"][index]
        assert_equivalent(input_record(regenerated_episode), saved)
        episode = restore_input(saved, city, dynamic=True)
        if recorded["id"] != episode.scenario_id or recorded[
            "fingerprint"
        ] != dynamic_scenario_fingerprint(episode):
            raise ValueError("Predictive mission identity mismatch")
        if len(recorded["runs"]) != 4:
            raise ValueError("Predictive planner matrix is incomplete")
        _audit_serialized_case(episode, recorded, runtime.EXECUTION_ENVELOPE)
        counts["predictive"] += len(recorded["runs"])
    for bundle in (static, dynamic, predictive):
        for scenario in bundle["scenarios"]:
            for run in scenario.get("results", scenario.get("runs", [])):
                if run["runId"] in run_ids:
                    raise ValueError("Duplicate run ID")
                run_ids.add(run["runId"])
    if counts != {"static": 24, "dynamic": 24, "predictive": 32}:
        raise ValueError("Incomplete current mission matrix")
    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--public-dir", type=Path, default=ROOT / "web" / "public")
    counts = audit_current(parser.parse_args().public_dir)
    print(f"Audited current Manhattan records: {counts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
