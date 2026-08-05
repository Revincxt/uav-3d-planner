from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path

import pytest

from uav3d.benchmark import run_benchmark
from uav3d.reporting import write_report_bundle
from uav3d.scene import Bounds3D, Scene
from uav3d.timing import run_timing_harness


def _report_scene() -> Scene:
    return Scene(
        "report-open",
        "Report open",
        Bounds3D((0, 0, 0), (20, 20, 20)),
        (2, 2, 2),
        (18, 18, 18),
        drone_radius=0.5,
        safety_margin=0.5,
    )


def test_report_bundle_files_checksums_and_record_alignment(tmp_path: Path) -> None:
    records = run_benchmark(
        [_report_scene()],
        ["astar-3d", "lazy-theta-star"],
        [3, 7],
    )
    output = tmp_path / "report"
    manifest = write_report_bundle(
        records,
        output,
        bootstrap_resamples=25,
        bootstrap_seed=20260805,
    )

    expected_files = {
        "records.csv",
        "summary.csv",
        "summary.json",
        "planner-summary.svg",
        "checksums.json",
    }
    assert {path.name for path in output.iterdir()} == expected_files

    checksum_document = json.loads((output / "checksums.json").read_text(encoding="utf-8"))
    assert checksum_document == manifest
    assert {entry["path"] for entry in manifest["files"]} == expected_files - {"checksums.json"}
    for entry in manifest["files"]:
        artifact = output / entry["path"]
        assert artifact.stat().st_size == entry["bytes"]
        assert hashlib.sha256(artifact.read_bytes()).hexdigest() == entry["sha256"]

    with (output / "records.csv").open(encoding="utf-8", newline="") as stream:
        record_rows = list(csv.DictReader(stream))
    assert len(record_rows) == len(records)
    assert [row["run_id"] for row in record_rows] == [record.run_id for record in records]
    assert [row["algorithm"] for row in record_rows] == [record.algorithm for record in records]
    assert {
        "planner_solution_found",
        "raw_path_valid",
        "postprocessing_status",
        "smoothed_path_valid",
        "parameters_json",
        "wall_time_limit_ms",
        "wall_time_used_ms",
        "quality_trace_json",
    } <= record_rows[0].keys()
    assert json.loads(record_rows[0]["parameters_json"])["resolution"] == 4.0

    summary_document = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    summaries = summary_document["planner_summaries"]
    assert summary_document["bootstrap"] == {
        "method": "scene-clustered-percentile",
        "resamples": 25,
        "seed": 20260805,
        "confidence_level": 0.95,
        "quantile_method": "linear-type-7",
    }
    assert len(summaries) == 2
    assert {summary["plannerId"] for summary in summaries} == {
        "astar-3d",
        "lazy-theta-star",
    }
    assert all(summary["runs"] == 1 for summary in summaries)

    with (output / "summary.csv").open(encoding="utf-8", newline="") as stream:
        summary_rows = list(csv.DictReader(stream))
    assert len(summary_rows) == sum(len(summary["metrics"]) for summary in summaries)

    svg = (output / "planner-summary.svg").read_text(encoding="utf-8")
    assert svg.startswith('<svg xmlns="http://www.w3.org/2000/svg"')
    assert "<title>Planner benchmark summary</title>" in svg
    assert svg.rstrip().endswith("</svg>")


def test_timing_harness_runs_one_astar_case_in_an_independent_process() -> None:
    repository = Path(__file__).parents[1]
    result = run_timing_harness(
        ["open-blocks"],
        ["astar-3d"],
        1,
        planner_seed=17,
        order_seed=31,
        process_timeout_s=10,
        resolution=8,
        work_limit=5_000,
        working_directory=repository,
    )

    assert result["schema_version"] == "2.0"
    assert result["harness"] == "isolated-process-v1"
    assert result["repetitions_per_cell"] == 1
    assert len(result["records"]) == 1

    process_record = result["records"][0]
    assert process_record["process_status"] == "completed"
    assert process_record["process_returncode"] == 0
    assert process_record["parse_error"] is None
    assert process_record["pid"] != os.getpid()
    assert isinstance(process_record["pid"], int) and process_record["pid"] > 0
    assert process_record["timing_repetition"] == 0
    assert process_record["planner_seed"] is None

    planner_record = process_record["planner_record"]
    assert isinstance(planner_record, dict)
    assert planner_record["run_purpose"] == "timing"
    assert planner_record["timing_repetition"] == 0
    assert planner_record["algorithm"] == "astar-3d"
    assert planner_record["status"] == "success"
    assert planner_record["timing"]["planning_ms"] == pytest.approx(
        planner_record["timing"]["setup_ms"] + planner_record["timing"]["search_ms"],
        abs=1e-9,
    )
    assert process_record["process_wall_time_ms"] >= planner_record["timing"]["planning_ms"]

    environment = result["environment"]
    assert {
        "python",
        "implementation",
        "package_version",
        "platform",
        "machine",
        "processor",
        "cpu_count",
    } <= environment.keys()
    assert environment["python"]
    assert environment["implementation"]
    assert environment["package_version"]
    assert environment["platform"]
    assert environment["machine"]
    assert isinstance(environment["cpu_count"], int) and environment["cpu_count"] > 0
