"""Audit the new city's exported evidence without modifying the frozen research cohort."""

from __future__ import annotations

import hashlib
import importlib
import json
import math
import sys
import unittest
from itertools import pairwise
from pathlib import Path
from unittest.mock import patch

from uav3d import predictive_study
from uav3d.dynamic import DynamicScenario
from uav3d.dynamic_collision import minimum_dynamic_separation, timed_path_is_free
from uav3d.geometry import almost_equal, polyline_length
from uav3d.kinematics import DiscreteExecutionEnvelope, qualify_timed_path_execution
from uav3d.manhattan_city import build_manhattan_city
from uav3d.manhattan_predictive import (
    MISSIONS,
    PROTOCOL_ID,
    _audit_serialized_case,
    build_manhattan_missions,
    source_provenance,
    study_runtime,
)
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.predictive_scenarios import list_predictive_scenarios
from uav3d.scene import AABB, Bounds3D, Scene

ROOT = Path(__file__).resolve().parents[1]


def _timed_path(records: list[dict]) -> TimedPath:
    return TimedPath(
        tuple(
            TimedWaypoint(
                point["timeS"],
                tuple(point["position"]),
                "start"
                if index == 0
                else (
                    "wait"
                    if almost_equal(records[index - 1]["position"], point["position"], 1e-8)
                    else "move"
                ),
            )
            for index, point in enumerate(records)
        )
    )


def _wait_windows(records: list[dict]) -> list[tuple[float, float, tuple]]:
    return [
        (previous["timeS"], current["timeS"], tuple(current["position"]))
        for previous, current in pairwise(records)
        if almost_equal(previous["position"], current["position"], 1e-8)
    ]


def _height_at(records: list[dict], time_s: float) -> float:
    for previous, following in pairwise(records):
        if time_s <= following["timeS"]:
            fraction = (time_s - previous["timeS"]) / (following["timeS"] - previous["timeS"])
            return previous["position"][2] + fraction * (
                following["position"][2] - previous["position"][2]
            )
    return records[-1]["position"][2]


def _max_altitude_difference(raw: list[dict], geometry: list[dict]) -> float:
    """Union knots exactly bound the difference of two serialized linear height profiles."""

    times = {point["timeS"] for point in raw + geometry}
    return max(abs(_height_at(raw, time_s) - _height_at(geometry, time_s)) for time_s in times)


class ManhattanPredictiveEvidenceTests(unittest.TestCase):
    def test_serialized_export_gate_rejects_rounded_acceleration_overflow(self) -> None:
        scene = Scene(
            "serialization-gate",
            "Serialization gate",
            Bounds3D((0.0, 0.0, 0.0), (100.0, 100.0, 100.0)),
            (10.0, 10.0, 10.0),
            (18.0, 10.0, 10.0),
            drone_radius=0.0,
            safety_margin=0.0,
        )
        scenario = DynamicScenario("serialization-gate", "Serialization gate", scene)
        records = [
            {"timeS": 0.0, "position": list(scene.start)},
            {"timeS": 1.99999998, "position": list(scene.goal)},
        ]
        exported = {
            "runs": [
                {
                    "plannerId": "test",
                    "rawTimedPath": records,
                    "geometryTimedPath": records,
                    "executionTimedPath": records,
                }
            ]
        }
        with self.assertRaisesRegex(RuntimeError, "acceleration-proxy-limit-exceeded"):
            _audit_serialized_case(scenario, exported, DiscreteExecutionEnvelope())
        records[-1]["timeS"] = 2.000001
        _audit_serialized_case(scenario, exported, DiscreteExecutionEnvelope())

    def test_serialized_export_gate_rechecks_full_collision_geometry(self) -> None:
        scene = Scene(
            "serialization-collision",
            "Serialization collision",
            Bounds3D((0.0, 0.0, 0.0), (100.0, 100.0, 100.0)),
            (10.0, 10.0, 10.0),
            (18.0, 10.0, 10.0),
            buildings=(AABB("wall", (13.0, 0.0, 0.0), (15.0, 20.0, 20.0)),),
            drone_radius=0.0,
            safety_margin=0.0,
        )
        records = [
            {"timeS": 0.0, "position": list(scene.start)},
            {"timeS": 4.0, "position": list(scene.goal)},
        ]
        with self.assertRaisesRegex(RuntimeError, "unsafe serialized trajectory"):
            _audit_serialized_case(
                DynamicScenario("serialization-collision", "Serialization collision", scene),
                {
                    "runs": [
                        {
                            "plannerId": "test",
                            "rawTimedPath": records,
                            "geometryTimedPath": records,
                            "executionTimedPath": None,
                        }
                    ]
                },
                DiscreteExecutionEnvelope(),
            )

    def test_new_runtime_preserves_the_frozen_study(self) -> None:
        runtime = study_runtime()
        self.assertEqual(runtime.PROTOCOL_ID, PROTOCOL_ID)
        self.assertEqual(runtime.CRUISE_SPEED_MPS, 15.0)
        self.assertEqual(runtime.MAX_TIME_S, 900.0)
        self.assertEqual(predictive_study.PROTOCOL_ID, "predictive-space-time-v4")
        self.assertEqual(predictive_study.CRUISE_SPEED_MPS, 8.0)
        self.assertEqual(predictive_study.MAX_TIME_S, 90.0)
        self.assertEqual(len(list_predictive_scenarios()), 10)

    def test_snapshot_provenance_hashes_scoped_source_files(self) -> None:
        provenance = source_provenance()
        self.assertEqual(provenance["kind"], "local-snapshot")
        for entry in provenance["files"]:
            expected = "sha256:" + hashlib.sha256((ROOT / entry["path"]).read_bytes()).hexdigest()
            self.assertEqual(entry["sha256"], expected)
        digest = hashlib.sha256(
            json.dumps(provenance["files"], sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        self.assertEqual(provenance["sha256"], "sha256:" + digest)

    def test_exported_paths_are_safe_against_the_full_real_city(self) -> None:
        bundle = json.loads(
            (ROOT / "web" / "public" / "predictive-data.json").read_text(encoding="utf-8")
        )
        self.assertEqual(bundle["protocol"]["id"], PROTOCOL_ID)
        self.assertEqual(bundle["protocol"]["spaceTimeConnectivity"], 26)
        self.assertIs(bundle["protocol"]["trajectoryShortcut"], True)
        self.assertIs(bundle["protocol"]["trajectoryPreserveAltitude"], True)
        self.assertEqual(
            bundle["protocol"]["trajectoryPostprocessor"],
            "horizontal-local-quintic-bspline-altitude-preserving-envelope-v5",
        )
        self.assertTrue(bundle["sourceCommit"].startswith("local-snapshot:sha256:"))
        self.assertEqual(len(bundle["scenarios"]), 8)
        city = build_manhattan_city()
        scenarios = {scenario.scenario_id: scenario for scenario in build_manhattan_missions(city)}
        self.assertGreater(len(city.buildings), 4000)
        self.assertEqual(city.metadata["planningRegion"]["id"], "midtown-expanded-v3")
        self.assertEqual(
            {entry["id"] for entry in bundle["scenarios"]},
            {mission.mission_id for mission in MISSIONS},
        )
        execution_envelope = DiscreteExecutionEnvelope(
            max_speed_mps=15.0,
            max_abs_climb_rate_mps=3.0,
            max_discrete_acceleration_proxy_mps2=4.0,
            reversal_threshold_deg=150.0,
            allow_reversals=False,
            max_execution_time_s=900.0,
        )
        for exported in bundle["scenarios"]:
            scenario = scenarios[exported["id"]]
            self.assertEqual(len(exported["buildings"]), len(city.buildings))
            self.assertEqual(
                exported["buildings"],
                study_runtime()._normalize_record_numbers(city.web_buildings()),
            )
            self.assertEqual(len(exported["runs"]), 4)
            for run in exported["runs"]:
                self.assertEqual(run["status"], "success")
                self.assertEqual(run["parameters"]["trajectoryShortcut"], 1)
                self.assertEqual(run["parameters"]["trajectoryPreserveAltitude"], 1)
                self.assertEqual(run["smoothing"]["optimizationAxes"], ["x", "y"])
                self.assertEqual(run["smoothing"]["altitudePolicy"], "preserve-raw-z-time-profile")
                if run["plannerId"] == "space-time-astar-4d":
                    self.assertEqual(run["parameters"]["spaceTimeConnectivity"], 26)
                else:
                    self.assertNotIn("spaceTimeConnectivity", run["parameters"])
                raw_points = run["rawTimedPath"]
                geometry_points = run["geometryTimedPath"]
                self.assertEqual(raw_points[0], geometry_points[0])
                self.assertEqual(raw_points[-1], geometry_points[-1])
                self.assertEqual(raw_points[0]["position"], exported["start"])
                self.assertEqual(raw_points[-1]["position"], exported["goal"])
                self.assertEqual(_wait_windows(raw_points), _wait_windows(geometry_points))
                for path_key, metric_key in [
                    ("rawTimedPath", "plannerMetrics"),
                    ("geometryTimedPath", "geometryMetrics"),
                    ("executionTimedPath", "executionMetrics"),
                ]:
                    with self.subTest(
                        scenario=exported["id"], planner=run["plannerId"], layer=path_key
                    ):
                        raw = run[path_key]
                        self.assertIsNotNone(raw)
                        points = [(point["timeS"], tuple(point["position"])) for point in raw]
                        self.assertTrue(timed_path_is_free(scenario, points))
                        metrics = run[metric_key]
                        self.assertEqual(metrics["safetyViolations"], 0)
                        if path_key == "executionTimedPath":
                            qualification = qualify_timed_path_execution(
                                _timed_path(raw), execution_envelope
                            )
                            self.assertTrue(qualification.qualified, qualification.violations)
                        self.assertAlmostEqual(
                            metrics["executedPathLengthM"],
                            polyline_length([point for _, point in points]),
                            places=6,
                        )
                        witness = minimum_dynamic_separation(scenario, points)
                        self.assertIsNotNone(witness)
                        self.assertGreaterEqual(
                            witness.separation_m, scenario.static_scene.safety_margin - 1e-6
                        )
                        self.assertAlmostEqual(
                            metrics["minimumSeparationM"], witness.separation_m, places=5
                        )

    def test_exported_geometry_preserves_every_raw_altitude_time_knot(self) -> None:
        bundle = json.loads((ROOT / "web" / "public" / "predictive-data.json").read_text())
        for scenario in bundle["scenarios"]:
            for run in scenario["runs"]:
                with self.subTest(scenario=scenario["id"], planner=run["plannerId"]):
                    raw, geometry = run["rawTimedPath"], run["geometryTimedPath"]
                    self.assertTrue(raw and geometry)
                    self.assertAlmostEqual(raw[0]["timeS"], geometry[0]["timeS"], delta=1e-6)
                    self.assertAlmostEqual(raw[-1]["timeS"], geometry[-1]["timeS"], delta=1e-6)
                    self.assertLessEqual(_max_altitude_difference(raw, geometry), 1e-6)
                    raw_travel = math.fsum(
                        abs(right["position"][2] - left["position"][2])
                        for left, right in pairwise(raw)
                    )
                    geometry_travel = math.fsum(
                        abs(right["position"][2] - left["position"][2])
                        for left, right in pairwise(geometry)
                    )
                    self.assertAlmostEqual(raw_travel, geometry_travel, delta=1e-6)
                    execution = run["executionTimedPath"]
                    self.assertIsNotNone(execution)
                    # Scheduling may add only stationary copies. Every original
                    # geometry/height knot still appears in the identical order.
                    cursor = 0
                    for point in geometry[1:]:
                        while execution[cursor + 1]["position"] != point["position"]:
                            self.assertEqual(
                                execution[cursor + 1]["position"], execution[cursor]["position"]
                            )
                            cursor += 1
                        cursor += 1
                    self.assertEqual(cursor, len(execution) - 1)

    def test_altitude_audit_detects_interior_knots_not_only_endpoints(self) -> None:
        level = [
            {"timeS": 0, "position": [0, 0, 90]},
            {"timeS": 2, "position": [2, 0, 90]},
        ]
        bump = [
            level[0],
            {"timeS": 1, "position": [1, 0, 90.0000015]},
            level[1],
        ]
        self.assertGreater(_max_altitude_difference(bump, level), 1e-6)
        self.assertGreater(_max_altitude_difference(level, bump), 1e-6)

    def test_published_source_and_download_hashes_match_their_artifacts(self) -> None:
        public = ROOT / "web" / "public"
        bundle = json.loads((public / "predictive-data.json").read_text(encoding="utf-8"))
        # Published evidence identifies the original computation bytes, not a
        # later formatting revision. Scope, hash chain and archived source bytes
        # remain checked; semantic source edits must still fail.
        provenance = bundle.get("computationSourceProvenance", bundle["sourceProvenance"])
        expected_sources = [entry["path"] for entry in source_provenance()["files"]]
        if "refinement" in bundle:
            self.assertEqual(
                bundle["refinement"]["anchorAlignment"],
                "continuously-certified-slow-connector",
            )
            self.assertIs(bundle["refinement"]["reactiveRawTracesRecomputed"], True)
            expected_sources.append("scripts/refine_predictive_demo.py")
        self.assertEqual(
            sorted(entry["path"] for entry in provenance["files"]),
            sorted(expected_sources),
        )
        digest = hashlib.sha256(
            json.dumps(provenance["files"], sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        self.assertEqual(provenance["sha256"], "sha256:" + digest)
        with patch.object(sys, "path", [str(ROOT / "scripts"), *sys.path]):
            audit_source = importlib.import_module("committed_inputs").audit_source
        for entry in provenance["files"]:
            audit_source(ROOT / entry["path"], entry["sha256"])
        for reference in bundle["downloads"].values():
            content = (public / reference["path"]).read_bytes()
            self.assertEqual(reference["bytes"], len(content))
            self.assertEqual(reference["sha256"], "sha256:" + hashlib.sha256(content).hexdigest())


if __name__ == "__main__":
    unittest.main()
