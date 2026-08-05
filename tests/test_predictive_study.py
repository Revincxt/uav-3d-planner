from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import pytest

from uav3d.cli import main
from uav3d.dynamic import DynamicScenario, TemporaryCylinder
from uav3d.predictive_scenarios import load_predictive_scenario
from uav3d.predictive_study import (
    DOWNLOAD_ARTIFACTS,
    PREDICTIVE_ALGORITHMS,
    RECORD_FIELDS,
    VERIFICATION_STATUS,
    build_predictive_bundle,
    export_predictive_study,
    predictive_record_rows,
    predictive_run_id,
    run_predictive_episode,
    run_predictive_study,
)
from uav3d.scene import Bounds3D, Scene

SOURCE_COMMIT = "a" * 40
GENERATED_AT = "2026-08-05T12:00:00+00:00"


def test_fixed_study_covers_six_scenarios_by_four_planners() -> None:
    study = run_predictive_study()

    assert len(study) == 6
    assert sum(len(episodes) for _, episodes in study) == 24
    for scenario, episodes in study:
        assert tuple(episode.planner_id for episode in episodes) == PREDICTIVE_ALGORITHMS
        for episode in episodes:
            assert episode.scenario_id == scenario.scenario_id
            assert episode.predictive == (episode.planner_id == "space-time-astar-4d")
            assert episode.metrics.safety_violations == 0
            assert episode.timed_path.is_safe(scenario)
            if episode.metrics.success:
                assert episode.timed_path.start == scenario.static_scene.start
                assert episode.timed_path.goal == scenario.static_scene.goal
                assert episode.metrics.arrival_time_s == episode.timed_path.arrival_time_s

    multi = next(episodes for scene, episodes in study if scene.scenario_id == "multi-obstacle")
    assert [episode.metrics.success for episode in multi] == [False, False, False, True]


def test_reset_reuse_ablation_preserves_mission_contract_and_exposes_reuse() -> None:
    scenario = load_predictive_scenario("wait-then-straight")
    reset = run_predictive_episode(scenario, "dstar-lite-reset-3d")
    reused = run_predictive_episode(scenario, "dstar-lite-reuse-3d")

    assert reset.metrics.success and reused.metrics.success
    assert reset.metrics.arrival_time_s == reused.metrics.arrival_time_s
    assert reset.metrics.executed_path_length_m == reused.metrics.executed_path_length_m
    assert reset.metrics.expanded_states > reused.metrics.expanded_states
    assert reset.metrics.work_unit == reused.metrics.work_unit == "queue-pops"


def test_wait_events_mark_the_actual_interval_boundaries() -> None:
    bundle, _ = build_predictive_bundle(
        source_commit=SOURCE_COMMIT,
        generated_at=GENERATED_AT,
    )
    scenarios = bundle["scenarios"]
    assert isinstance(scenarios, list)
    calibration = next(item for item in scenarios if item["id"] == "wait-then-straight")
    predictive = next(
        run for run in calibration["runs"] if run["plannerId"] == "space-time-astar-4d"
    )
    interval = predictive["waitIntervals"][0]
    start_event = next(
        frame for frame in predictive["frames"] if frame["timeS"] == interval["startTimeS"]
    )
    end_event = next(
        frame for frame in predictive["frames"] if frame["timeS"] == interval["endTimeS"]
    )
    assert start_event["event"]["kind"] == "wait-start"
    assert end_event["event"]["kind"] == "wait-end"


def test_predictive_episode_flags_an_initial_state_collision() -> None:
    scene = Scene(
        "invalid-start",
        "Invalid start",
        Bounds3D((0.0, 0.0, 0.0), (20.0, 20.0, 20.0)),
        (4.0, 10.0, 10.0),
        (16.0, 10.0, 10.0),
        drone_radius=0.0,
        safety_margin=0.0,
    )
    scenario = DynamicScenario(
        "invalid-start",
        "Invalid start",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("start-block", (4.0, 10.0), 2.0, 0.0, 20.0, 0.0, 5.0),
        ),
    )

    episode = run_predictive_episode(scenario, "space-time-astar-4d")

    assert not episode.metrics.success
    assert episode.metrics.failure_reason == "invalid-start"
    assert episode.metrics.safety_violations == 1
    assert not episode.timed_path.is_safe(scenario)


def test_bundle_and_run_ids_are_exactly_reproducible() -> None:
    first, first_manifest = build_predictive_bundle(
        source_commit=SOURCE_COMMIT,
        generated_at=GENERATED_AT,
    )
    second, second_manifest = build_predictive_bundle(
        source_commit=SOURCE_COMMIT,
        generated_at=GENERATED_AT,
    )

    assert first == second
    assert first_manifest == second_manifest
    assert first["verificationStatus"] == VERIFICATION_STATUS
    assert first_manifest["requested"] == first_manifest["accepted"] + first_manifest["rejected"]
    run_ids: set[str] = set()
    protocol = first["protocol"]
    assert isinstance(protocol, dict)
    scenarios = first["scenarios"]
    assert isinstance(scenarios, list)
    for scenario in scenarios:
        assert isinstance(scenario, dict)
        runs = scenario["runs"]
        assert isinstance(runs, list)
        for run in runs:
            assert isinstance(run, dict)
            parameters = run["parameters"]
            assert isinstance(parameters, dict)
            assert all(isinstance(value, (int, float)) for value in parameters.values())
            expected = predictive_run_id(
                str(scenario["fingerprint"]),
                str(run["plannerId"]),
                protocol,
                parameters,  # type: ignore[arg-type]
            )
            assert run["runId"] == expected
            assert expected not in run_ids
            run_ids.add(expected)
    assert len(run_ids) == 24


def test_records_csv_projection_is_complete() -> None:
    bundle, _ = build_predictive_bundle(
        source_commit=SOURCE_COMMIT,
        generated_at=GENERATED_AT,
    )
    rows = predictive_record_rows(bundle)

    assert len(rows) == 24
    assert all(tuple(row) == RECORD_FIELDS for row in rows)
    assert {row["planner_id"] for row in rows} == set(PREDICTIVE_ALGORITHMS)
    assert sum(row["success"] == "true" for row in rows) == 21
    assert all(row["safety_violations"] == "0" for row in rows)


def test_export_writes_self_describing_downloads(tmp_path: Path) -> None:
    bundle = export_predictive_study(tmp_path, source_commit=SOURCE_COMMIT)

    bundle_path = tmp_path / "predictive-data.json"
    assert json.loads(bundle_path.read_text(encoding="utf-8")) == bundle
    for key, filename in DOWNLOAD_ARTIFACTS.items():
        path = tmp_path / filename
        reference = bundle["downloads"][key]  # type: ignore[index]
        assert reference["bytes"] == path.stat().st_size
        assert reference["sha256"] == "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
    with (tmp_path / DOWNLOAD_ARTIFACTS["recordsCsv"]).open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        assert tuple(reader.fieldnames or ()) == RECORD_FIELDS
    assert rows == predictive_record_rows(bundle)


def test_predictive_study_rejects_unknown_inputs() -> None:
    scenario = load_predictive_scenario("periodic-traffic")
    with pytest.raises(ValueError, match="unknown predictive-study planner"):
        run_predictive_episode(scenario, "missing")
    with pytest.raises(ValueError, match="Git object ID"):
        build_predictive_bundle(source_commit="short")


def test_predictive_cli_lists_and_writes_a_timed_run(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["predictive", "list"]) == 0
    assert "wait-then-straight" in capsys.readouterr().out

    output = tmp_path / "predictive-run.json"
    status = main(
        [
            "predictive",
            "plan",
            "--scenario",
            "wait-then-straight",
            "--algorithm",
            "space-time-astar-4d",
            "--output",
            str(output),
        ]
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert status == 0
    assert payload["schema_version"] == "predictive-run-v1"
    assert payload["planner_id"] == "space-time-astar-4d"
    assert payload["metrics"]["success"] is True
    assert payload["timed_path"]["wait_time_s"] > 0
