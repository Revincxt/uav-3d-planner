from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import pytest

import uav3d.study as study
from uav3d.benchmark import ExperimentRecord
from uav3d.scene import load_builtin_scene


def test_web_benchmark_export_is_self_consistent(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    scene = load_builtin_scene("vertical-gate")
    captured: dict[str, list[ExperimentRecord]] = {}
    real_resolution = study.run_resolution_sweep
    real_rrt = study.run_rrt_budget_curve

    def resolution_once(*args: Any, **kwargs: Any) -> list[ExperimentRecord]:
        records = real_resolution(*args, **kwargs)
        captured["resolution"] = records
        return records

    def rrt_one_seed(
        scenes: list[Any], budgets: list[int], seeds: list[int]
    ) -> list[ExperimentRecord]:
        del seeds
        records = real_rrt(scenes, budgets, [11])
        captured["rrt"] = records
        return records

    def fake_timing(*args: Any, **kwargs: Any) -> dict[str, Any]:
        del args, kwargs
        planner_records = [
            next(
                record
                for record in captured["resolution"]
                if record.algorithm == algorithm and "resolution=4|" in record.configuration_id
            )
            for algorithm in ("astar-3d", "lazy-theta-star")
        ] + [captured["rrt"][0]]
        return {
            "schema_version": "2.0",
            "harness": "isolated-process-v1",
            "order_seed": study.BOOTSTRAP_SEED,
            "planner_seed": 17,
            "repetitions_per_cell": 1,
            "process_timeout_s": 120.0,
            "environment": {"python": "test"},
            "records": [
                {
                    "case_id": f"case-{record.algorithm}",
                    "order_index": index,
                    "scene_reference": scene.scene_id,
                    "algorithm": record.algorithm,
                    "timing_repetition": 0,
                    "planner_seed": record.seed,
                    "pid": 1000 + index,
                    "process_status": "completed",
                    "process_returncode": 0,
                    "process_wall_time_ms": record.planning_time_ms + 1,
                    "stderr": None,
                    "planner_record": record.to_dict(),
                }
                for index, record in enumerate(planner_records)
            ],
        }

    monkeypatch.setattr(study, "run_resolution_sweep", resolution_once)
    monkeypatch.setattr(study, "run_rrt_budget_curve", rrt_one_seed)
    monkeypatch.setattr(study, "run_timing_harness", fake_timing)

    bundle = study.export_web_benchmark(
        [scene],
        tmp_path,
        source_commit="a" * 40,
        timing_repetitions=1,
    )

    assert bundle["schemaVersion"] == 2
    assert bundle["sourceCommit"] == "a" * 40
    assert len(bundle["summaries"]) == 3
    assert len(bundle["sensitivity"]["resolution"]) == 8
    assert len(bundle["sensitivity"]["rrtBudget"]) == 5
    manifest_path = tmp_path / "dataset-manifest.json"
    expected_digest = "sha256:" + hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    assert bundle["dataset"]["manifestSha256"] == expected_digest
    assert json.loads((tmp_path / "benchmark-data.json").read_text()) == bundle
    assert (tmp_path / "benchmark-records.csv").is_file()
    assert (tmp_path / "benchmark-summary.csv").is_file()
    with (tmp_path / "benchmark-records.csv").open(encoding="utf-8", newline="") as stream:
        first_record = next(csv.DictReader(stream))
    assert first_record["source_commit"] == "a" * 40
    assert first_record["protocol_id"] == study.PROTOCOL_ID
    timing = json.loads((tmp_path / "timing-manifest.json").read_text())
    assert all("raw_path" not in record["planner_record"] for record in timing["records"])
    for reference in bundle["downloads"].values():
        artifact = tmp_path / reference["path"]
        assert reference["bytes"] == artifact.stat().st_size
        assert reference["sha256"] == (
            "sha256:" + hashlib.sha256(artifact.read_bytes()).hexdigest()
        )


def test_web_benchmark_rejects_placeholder_source_commit() -> None:
    with pytest.raises(ValueError, match="full lowercase Git object ID"):
        study.build_web_benchmark_bundle(
            [load_builtin_scene("vertical-gate")],
            source_commit="worktree-v0.2.0",
        )
