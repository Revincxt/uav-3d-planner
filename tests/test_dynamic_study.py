from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import pytest

from uav3d.cli import main
from uav3d.dynamic import list_builtin_dynamic_scenarios
from uav3d.dynamic_study import (
    DOWNLOAD_ARTIFACTS,
    MAX_EXPANSIONS,
    PLANNER_LABELS,
    PROTOCOL_ID,
    RECORD_FIELDS,
    SERIALIZATION_DECIMAL_PLACES,
    VERIFICATION_STATUS,
    build_dynamic_bundle,
    dynamic_record_rows,
    dynamic_run_id,
    export_dynamic_study,
)
from uav3d.replanning import REPLANNING_ALGORITHMS

SOURCE_COMMIT = "a" * 40


@pytest.fixture(scope="module")
def built_bundle() -> tuple[dict[str, object], dict[str, object]]:
    return build_dynamic_bundle(
        source_commit=SOURCE_COMMIT,
        generated_at="2026-08-05T00:00:00+00:00",
    )


def test_fixed_dynamic_bundle_contract_and_determinism(
    built_bundle: tuple[dict[str, object], dict[str, object]],
) -> None:
    bundle, manifest = built_bundle
    repeated, repeated_manifest = build_dynamic_bundle(
        source_commit=SOURCE_COMMIT,
        generated_at="2026-08-06T00:00:00+00:00",
    )
    assert {key: value for key, value in bundle.items() if key != "generatedAt"} == {
        key: value for key, value in repeated.items() if key != "generatedAt"
    }
    assert {key: value for key, value in manifest.items() if key != "generatedAt"} == {
        key: value for key, value in repeated_manifest.items() if key != "generatedAt"
    }
    assert bundle["schemaVersion"] == 1
    assert bundle["verificationStatus"] == VERIFICATION_STATUS
    assert bundle["protocol"] == {
        "id": PROTOCOL_ID,
        "timeStepS": 1.0,
        "replanIntervalS": 4.0,
        "cruiseSpeedMps": 8.0,
        "maxTimeS": 180.0,
        "resolutionM": 4.0,
        "maxExpansions": MAX_EXPANSIONS,
    }
    assert bundle["planners"] == [
        {"id": planner, "label": PLANNER_LABELS[planner]} for planner in REPLANNING_ALGORITHMS
    ]
    assert manifest["selection"] == {
        "scenarioRule": "all built-in dynamic scenarios",
        "algorithmFiltering": "none",
    }
    assert manifest["sourceCommit"] == SOURCE_COMMIT
    assert manifest["generatedAt"] == "2026-08-05T00:00:00+00:00"
    assert manifest["runCount"] == 12
    assert SERIALIZATION_DECIMAL_PLACES == 12
    assert all(
        value == round(value, SERIALIZATION_DECIMAL_PLACES)
        for scenario in bundle["scenarios"]
        for run in scenario["runs"]
        for value in (
            run["metrics"]["completionTimeS"],
            run["metrics"]["executedPathLengthM"],
        )
        if value is not None
    )


def test_bundle_covers_four_scenarios_three_planners_and_complete_frames(
    built_bundle: tuple[dict[str, object], dict[str, object]],
) -> None:
    bundle, _ = built_bundle
    scenarios = bundle["scenarios"]
    assert isinstance(scenarios, list)
    assert [scenario["id"] for scenario in scenarios] == list(list_builtin_dynamic_scenarios())
    run_ids: set[str] = set()
    for scenario in scenarios:
        assert set(scenario) >= {
            "constraints",
            "staticNoFlyZones",
            "temporaryNoFlyZones",
            "movingSpheres",
            "runs",
        }
        runs = scenario["runs"]
        assert [run["plannerId"] for run in runs] == list(REPLANNING_ALGORITHMS)
        for run in runs:
            assert run["runId"].startswith("sha256:")
            assert len(run["runId"]) == 71
            assert run["runId"] == dynamic_run_id(
                scenario["fingerprint"],
                run["plannerId"],
                bundle["protocol"],
                run["parameters"],
            )
            assert run["runId"] not in run_ids
            run_ids.add(run["runId"])
            assert run["status"] in {"success", "no-path", "timeout", "invalid"}
            assert run["metrics"] == run["metrics"] | {
                "deadlineMisses": 0,
                "minimumClearanceM": None,
            }
            assert set(run["metrics"]) >= {
                "success",
                "failureReason",
                "completionTimeS",
                "executedPathLengthM",
                "directDistanceM",
                "pathExcessPct",
                "replans",
                "failedReplans",
                "holds",
                "safetyGateActivations",
                "collisionCount",
                "totalPlanningWork",
                "workUnit",
                "totalChangedEdges",
            }
            expected_unit = (
                "queue-pops" if run["plannerId"] == "dstar-lite-3d" else "expanded-nodes"
            )
            assert run["metrics"]["workUnit"] == expected_unit
            previous_executed: list[list[float]] = []
            for frame in run["frames"]:
                assert frame["planningTimeMs"] is None
                assert frame["event"]["kind"] in {
                    "none",
                    "temporary-zone-activated",
                    "temporary-zone-deactivated",
                    "replan",
                    "wait",
                    "goal-reached",
                    "no-path",
                }
                executed = frame["executedPath"]
                assert executed[: len(previous_executed)] == previous_executed
                assert executed[-1] == frame["vehicle"]
                previous_executed = executed
            assert previous_executed[0] == scenario["start"]
            assert previous_executed[-1] == scenario["goal"]


def test_export_writes_exact_downloads_and_csv_rows(tmp_path: Path) -> None:
    bundle = export_dynamic_study(tmp_path, source_commit=SOURCE_COMMIT)
    assert set(bundle["downloads"]) == set(DOWNLOAD_ARTIFACTS)
    for key, filename in DOWNLOAD_ARTIFACTS.items():
        artifact = tmp_path / filename
        reference = bundle["downloads"][key]
        assert reference == {
            "path": filename,
            "sha256": "sha256:" + hashlib.sha256(artifact.read_bytes()).hexdigest(),
            "bytes": artifact.stat().st_size,
        }

    with (tmp_path / "dynamic-records.csv").open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        assert tuple(reader.fieldnames or ()) == RECORD_FIELDS
    assert rows == dynamic_record_rows(bundle)
    manifest = json.loads((tmp_path / "dynamic-scenario-manifest.json").read_text())
    assert manifest["sourceCommit"] == bundle["sourceCommit"]
    assert manifest["generatedAt"] == bundle["generatedAt"]
    assert manifest["scenarioCount"] == 4
    assert manifest["runCount"] == 12
    assert len(manifest["scenarios"]) == 4
    assert len(rows) == 12
    assert {row["generated_at"] for row in rows} == {bundle["generatedAt"]}
    assert {row["source_commit"] for row in rows} == {bundle["sourceCommit"]}
    assert len({row["run_id"] for row in rows}) == 12


def test_dynamic_cli_lists_simulates_and_exports(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["dynamic", "list"]) == 0
    listing = capsys.readouterr().out
    assert "pop-up-nfz\tPop-up no-fly zone" in listing

    run_path = tmp_path / "run.json"
    assert (
        main(
            [
                "dynamic",
                "simulate",
                "--scenario",
                "crossing-traffic",
                "--algorithm",
                "repeated-astar-3d",
                "--time-step",
                "1",
                "--replan-interval",
                "4",
                "--cruise-speed",
                "8",
                "--max-time",
                "180",
                "--resolution",
                "4",
                "--max-expansions",
                "120000",
                "--output",
                str(run_path),
            ]
        )
        == 0
    )
    run = json.loads(run_path.read_text())
    assert run["schema_version"] == "dynamic-run-v1"
    assert run["metrics"]["success"] is True

    output_dir = tmp_path / "public"
    assert (
        main(
            [
                "export-dynamic",
                "--output-dir",
                str(output_dir),
                "--source-commit",
                SOURCE_COMMIT,
            ]
        )
        == 0
    )
    assert (output_dir / "dynamic-data.json").is_file()


def test_dynamic_export_rejects_placeholder_source_commit(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="full lowercase Git object ID"):
        export_dynamic_study(tmp_path, source_commit="worktree-v0.3")
    assert (
        main(
            [
                "export-dynamic",
                "--output-dir",
                str(tmp_path),
                "--source-commit",
                "worktree-v0.3",
            ]
        )
        == 2
    )
    with pytest.raises(ValueError, match="parseable ISO timestamp"):
        build_dynamic_bundle(source_commit=SOURCE_COMMIT, generated_at="not-a-timestamp")
