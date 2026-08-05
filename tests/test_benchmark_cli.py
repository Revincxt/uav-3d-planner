from __future__ import annotations

import json
from pathlib import Path

import pytest

from uav3d.benchmark import run_benchmark, run_experiment, summarize_records
from uav3d.cli import main
from uav3d.scene import generate_random_city, load_builtin_scene, load_scene


def test_experiment_record_contains_raw_and_smoothed_audits() -> None:
    record = run_experiment(load_builtin_scene("vertical-gate"), "lazy-theta-star", 4)
    assert record.status == "success"
    assert record.raw_audit and record.raw_audit.valid
    assert record.smoothed_audit and record.smoothed_audit.valid
    assert record.scene_fingerprint.startswith("sha256:")
    assert record.to_dict()["smoothing"] is not None


def test_benchmark_summary_groups_successes() -> None:
    scene = load_builtin_scene("vertical-gate")
    records = run_benchmark([scene], ["astar-3d", "lazy-theta-star"], [0])
    summary = summarize_records(records)
    assert len(summary) == 2
    assert all(item["success_rate"] == 1 for item in summary)


def test_cli_generates_reloadable_scene(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    output = tmp_path / "random.json"
    arguments = [
        "scene",
        "generate",
        "--seed",
        "7",
        "--buildings",
        "5",
        "--output",
        str(output),
    ]
    assert main(arguments) == 0
    assert load_scene(output) == generate_random_city(7, 5)
    assert "saved random-city-7" in capsys.readouterr().out


def test_cli_writes_planning_result(tmp_path: Path) -> None:
    output = tmp_path / "plan.json"
    status = main(
        [
            "plan",
            "--scene",
            "vertical-gate",
            "--algorithm",
            "astar-3d",
            "--output",
            str(output),
        ]
    )
    payload = json.loads(output.read_text())
    assert status == 0
    assert payload["status"] == "success"
    assert payload["raw_audit"]["collision_free"] is True


def test_cli_returns_error_for_missing_scene(capsys: pytest.CaptureFixture[str]) -> None:
    status = main(["plan", "--scene", "missing.json", "--algorithm", "astar-3d"])
    assert status == 2
    assert "neither a built-in ID" in capsys.readouterr().err
