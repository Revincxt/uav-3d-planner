from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest

from uav3d.cli import main
from uav3d.dynamic import DynamicScenario, TemporaryCylinder
from uav3d.predictive_scenarios import load_predictive_scenario
from uav3d.predictive_study import (
    DOWNLOAD_ARTIFACTS,
    PREDICTIVE_ALGORITHMS,
    RECORD_FIELDS,
    VERIFICATION_STATUS,
    PredictiveEpisode,
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


@pytest.fixture(scope="module")
def fixed_study() -> list[tuple[DynamicScenario, list[PredictiveEpisode]]]:
    """Run the expensive 32-condition matrix once for this module."""

    return run_predictive_study()


@pytest.fixture(scope="module")
def fixed_bundle(
    fixed_study: list[tuple[DynamicScenario, list[PredictiveEpisode]]],
) -> tuple[dict[str, object], dict[str, object]]:
    # Bundle serialization is exercised against the exact episodes audited by the study test,
    # without paying for the fixed matrix a second time.
    with patch("uav3d.predictive_study.run_predictive_study", return_value=fixed_study):
        return build_predictive_bundle(source_commit=SOURCE_COMMIT, generated_at=GENERATED_AT)


def test_fixed_study_covers_eight_scenarios_by_four_planners(
    fixed_study: list[tuple[DynamicScenario, list[PredictiveEpisode]]],
) -> None:
    study = fixed_study

    assert len(study) == 8
    assert sum(len(episodes) for _, episodes in study) == 32
    for scenario, episodes in study:
        assert tuple(episode.planner_id for episode in episodes) == PREDICTIVE_ALGORITHMS
        for episode in episodes:
            assert episode.scenario_id == scenario.scenario_id
            assert episode.predictive == (episode.planner_id == "space-time-astar-4d")
            assert episode.metrics.success
            assert episode.metrics.safety_violations == 0
            assert episode.raw_timed_path.is_safe(scenario)
            assert episode.timed_path.is_safe(scenario)
            assert episode.smoothing.certified
            assert episode.smoothing.raw_waypoint_count == len(episode.raw_timed_path.waypoints)
            assert episode.smoothing.output_waypoint_count == len(episode.timed_path.waypoints)
            assert episode.raw_timed_path.start == episode.timed_path.start
            assert episode.raw_timed_path.goal == episode.timed_path.goal
            assert episode.raw_timed_path.departure_time_s == episode.timed_path.departure_time_s
            assert episode.raw_timed_path.arrival_time_s == episode.timed_path.arrival_time_s
            assert episode.timed_path.start == scenario.static_scene.start
            assert episode.timed_path.goal == scenario.static_scene.goal
            assert episode.metrics.arrival_time_s == episode.timed_path.arrival_time_s


def test_reset_reuse_ablation_preserves_mission_contract_and_exposes_reuse(
    fixed_study: list[tuple[DynamicScenario, list[PredictiveEpisode]]],
) -> None:
    episodes = next(
        episodes
        for scenario, episodes in fixed_study
        if scenario.scenario_id == "wait-then-straight"
    )
    reset = next(episode for episode in episodes if episode.planner_id == "dstar-lite-reset-3d")
    reused = next(episode for episode in episodes if episode.planner_id == "dstar-lite-reuse-3d")

    assert reset.metrics.success and reused.metrics.success
    assert reset.metrics.arrival_time_s == reused.metrics.arrival_time_s
    assert reset.metrics.executed_path_length_m == reused.metrics.executed_path_length_m
    assert reset.metrics.expanded_states > reused.metrics.expanded_states
    assert reset.metrics.work_unit == reused.metrics.work_unit == "queue-pops"


def test_wait_events_mark_the_actual_interval_boundaries(
    fixed_bundle: tuple[dict[str, object], dict[str, object]],
) -> None:
    bundle, _ = fixed_bundle
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


def test_bundle_v2_and_run_ids_are_exactly_reproducible(
    fixed_bundle: tuple[dict[str, object], dict[str, object]],
) -> None:
    bundle, manifest = fixed_bundle

    assert bundle["schemaVersion"] == 2
    assert manifest["schemaVersion"] == 2
    assert manifest["datasetId"] == "predictive-complex-city-v0.5"
    assert bundle["verificationStatus"] == VERIFICATION_STATUS
    assert manifest["requested"] == manifest["accepted"] + manifest["rejected"]
    run_ids: set[str] = set()
    protocol = bundle["protocol"]
    assert isinstance(protocol, dict)
    assert protocol["id"] == "predictive-space-time-v2"
    assert "certified" in str(protocol["trajectoryPostprocessor"])
    scenarios = bundle["scenarios"]
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
            assert (
                predictive_run_id(
                    str(scenario["fingerprint"]),
                    str(run["plannerId"]),
                    protocol,
                    parameters,  # type: ignore[arg-type]
                )
                == expected
            )
            assert expected not in run_ids
            run_ids.add(expected)
    assert len(run_ids) == 32


def test_bundle_exports_raw_certified_paths_and_linear_size_frames(
    fixed_bundle: tuple[dict[str, object], dict[str, object]],
) -> None:
    bundle, _ = fixed_bundle
    scenarios = bundle["scenarios"]
    assert isinstance(scenarios, list)
    observed_compaction = False
    for scenario in scenarios:
        assert isinstance(scenario, dict)
        moving_spheres = scenario["movingSpheres"]
        runs = scenario["runs"]
        assert isinstance(moving_spheres, list)
        assert isinstance(runs, list)
        for run in runs:
            assert isinstance(run, dict)
            raw_path = run["rawTimedPath"]
            timed_path = run["timedPath"]
            smoothing = run["smoothing"]
            frames = run["frames"]
            metrics = run["metrics"]
            assert isinstance(raw_path, list)
            assert isinstance(timed_path, list)
            assert isinstance(smoothing, dict)
            assert isinstance(frames, list)
            assert isinstance(metrics, dict)
            assert smoothing["certified"] is True
            assert smoothing["rawWaypointCount"] == len(raw_path)
            assert smoothing["outputWaypointCount"] == len(timed_path)
            assert metrics["safetyViolations"] == 0
            assert raw_path[0] == timed_path[0]
            assert raw_path[-1] == timed_path[-1]
            for angle_field in ("maxTurnAngleBeforeDeg", "maxTurnAngleAfterDeg"):
                angle = smoothing[angle_field]
                assert isinstance(angle, (int, float))
                assert math.isfinite(angle)
                assert 0.0 <= angle <= 180.0 + 1e-6

            # Frames are semantic event anchors drawn from the certified path. The browser derives
            # continuous motion from timedPath, so replay records do not repeat geometry or every
            # dense smoothing sample.
            assert 1 <= len(frames) <= len(timed_path)
            assert frames[0]["timeS"] == timed_path[0]["timeS"]
            assert frames[-1]["timeS"] == timed_path[-1]["timeS"]
            observed_compaction |= len(frames) < len(timed_path)
            timed_positions = {
                waypoint["timeS"]: waypoint["position"]
                for waypoint in timed_path
                if isinstance(waypoint, dict)
            }
            previous_frame_time = -math.inf
            for frame in frames:
                assert isinstance(frame, dict)
                assert frame["timeS"] > previous_frame_time
                assert frame["timeS"] in timed_positions
                assert frame["vehicle"] == timed_positions[frame["timeS"]]
                assert "path" not in frame
                assert "executedPath" not in frame
                assert len(frame["movingSpheres"]) == len(moving_spheres)
                assert frame["event"]["kind"] != "none"
                previous_frame_time = frame["timeS"]

            if smoothing["applied"] is True:
                assert smoothing["roundedCornerCount"] > 0
                assert smoothing["appliedTurnRadiusM"] is not None
            else:
                assert timed_path == raw_path
                assert smoothing["roundedCornerCount"] == 0
                assert smoothing["appliedTurnRadiusM"] is None
    assert observed_compaction


def test_records_csv_projection_is_complete(
    fixed_bundle: tuple[dict[str, object], dict[str, object]],
) -> None:
    bundle, _ = fixed_bundle
    rows = predictive_record_rows(bundle)

    assert len(rows) == 32
    assert all(tuple(row) == RECORD_FIELDS for row in rows)
    assert {row["planner_id"] for row in rows} == set(PREDICTIVE_ALGORITHMS)
    assert sum(row["success"] == "true" for row in rows) == 32
    assert all(row["safety_violations"] == "0" for row in rows)
    assert all(row["smoothing_certified"] == "true" for row in rows)
    assert all(int(row["raw_waypoint_count"]) > 0 for row in rows)
    assert all(int(row["waypoint_count"]) > 0 for row in rows)


def test_export_writes_self_describing_downloads(
    tmp_path: Path,
    fixed_study: list[tuple[DynamicScenario, list[PredictiveEpisode]]],
) -> None:
    with patch("uav3d.predictive_study.run_predictive_study", return_value=fixed_study):
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
    assert payload["schema_version"] == "predictive-run-v2"
    assert payload["planner_id"] == "space-time-astar-4d"
    assert payload["metrics"]["success"] is True
    assert payload["raw_timed_path"]["waypoints"]
    assert payload["timed_path"]["waypoints"]
    assert payload["smoothing"]["certified"] is True
    assert payload["timed_path"]["wait_time_s"] > 0


def test_predictive_cli_help_describes_v05_city_and_certified_smoothing(
    capsys: pytest.CaptureFixture[str],
) -> None:
    with pytest.raises(SystemExit) as exit_info:
        main(["predictive", "--help"])

    assert exit_info.value.code == 0
    help_text = capsys.readouterr().out
    assert "v0.5 complex-city" in help_text
    assert "certified" in help_text
