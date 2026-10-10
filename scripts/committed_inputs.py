"""Preserve exact planning inputs independently of platform-specific libm rounding.

Fingerprints encode every binary float, so regenerating ENU anchors on Linux must
not replace the inputs originally computed on macOS. This snapshot retains those
inputs, binds them to the native records, and still checks regenerated semantics.
Building envelopes come directly from the separately verified official city file.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

from export_manhattan_static_dynamic import mission_scenes, shared_dynamic_scenarios

from uav3d.benchmark import scene_fingerprint
from uav3d.dynamic import DynamicScenario, dynamic_scenario_fingerprint
from uav3d.manhattan_city import build_manhattan_city
from uav3d.manhattan_predictive import build_manhattan_missions
from uav3d.scene import Scene

ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT / "data" / "studies" / "planning-inputs.json"
NATIVE_FILES = {
    "static": "demo-data.json",
    "dynamic": "dynamic-data.json",
    "predictive": "predictive-data.json",
}
# Sub-micrometre / sub-microsecond reconstruction tolerance, never used by
# fingerprinting, collision checks, or execution qualification.
RECONSTRUCTION_TOLERANCE = 1e-7


def digest(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def audit_source(path: Path, expected: str) -> None:
    """Require the exact computation bytes bound to the current recorded studies."""
    if digest(path) != expected:
        raise ValueError(f"Computation source changed: {path.relative_to(ROOT)}")


def input_record(problem: Scene | DynamicScenario) -> dict[str, Any]:
    record: dict[str, Any] = problem.to_dict()
    scene = record["static_scene"] if isinstance(problem, DynamicScenario) else record
    del scene["buildings"]
    scene["metadata"] = {"missionTaskPoints": scene["metadata"].get("missionTaskPoints", [])}
    if isinstance(problem, DynamicScenario):
        record["metadata"] = {
            "mission": {"taskPoints": problem.metadata.get("mission", {}).get("taskPoints", [])}
        }
    return record


def assert_equivalent(expected: Any, actual: Any, path: str = "inputs") -> None:
    """Reject changed inputs while permitting only native math-library roundoff."""
    if isinstance(expected, dict) and isinstance(actual, dict) and expected.keys() == actual.keys():
        for key in expected:
            assert_equivalent(expected[key], actual[key], f"{path}.{key}")
    elif isinstance(expected, list) and isinstance(actual, list) and len(expected) == len(actual):
        for index, (a, b) in enumerate(zip(expected, actual, strict=True)):
            assert_equivalent(a, b, f"{path}[{index}]")
    elif type(expected) in (int, float) and type(actual) in (int, float):
        if not (
            math.isfinite(expected)
            and math.isfinite(actual)
            and math.isclose(expected, actual, rel_tol=0.0, abs_tol=RECONSTRUCTION_TOLERANCE)
        ):
            raise ValueError(f"Changed planning input: {path}: {expected!r} != {actual!r}")
    elif type(expected) is not type(actual) or expected != actual:
        raise ValueError(f"Changed planning input: {path}")


def restore_input(record: dict[str, Any], city: Any, dynamic: bool) -> Scene | DynamicScenario:
    scene = dict(record["static_scene"] if dynamic else record)
    if "buildings" in scene:
        raise ValueError("Snapshot must use the complete verified source city")
    scene["buildings"] = [building.to_dict() for building in city.buildings]
    if dynamic:
        return DynamicScenario.from_dict({**record, "static_scene": scene})
    return Scene.from_dict(scene)


def load_inputs(public: Path, city: Any) -> dict[str, Any]:
    snapshot = json.loads(SNAPSHOT.read_text(encoding="utf-8"))
    if snapshot.get("schemaVersion") != 1:
        raise ValueError("Unsupported planning-input snapshot")
    if snapshot.get("citySourceSha256") != city.metadata["sourceSha256"]:
        raise ValueError("Planning-input city source mismatch")
    if set(snapshot["studies"]) != set(NATIVE_FILES):
        raise ValueError("Incomplete planning-input studies")
    for study, filename in NATIVE_FILES.items():
        if snapshot["nativeSha256"].get(study) != digest(public / filename):
            raise ValueError(f"Planning inputs do not match native records: {study}")
        if len(snapshot["studies"][study]) != 8:
            raise ValueError(f"Incomplete planning-input missions: {study}")
    return snapshot["studies"]


def main() -> None:
    """Capture only when all regenerated fingerprints exactly match the native data."""
    public = ROOT / "web" / "public"
    city = build_manhattan_city()
    bundles = {key: json.loads((public / name).read_text()) for key, name in NATIVE_FILES.items()}
    missions = mission_scenes(city)
    baselines = [
        next(run for run in scene["results"] if run["plannerId"] == "astar-3d")["paths"]["raw"]
        for scene in bundles["static"]["scenarios"]
    ]
    problems = {
        "static": [scene for _, scene in missions],
        "dynamic": shared_dynamic_scenarios(missions, baselines),
        "predictive": build_manhattan_missions(city),
    }
    studies = {}
    for study, scenes in problems.items():
        fingerprint = scene_fingerprint if study == "static" else dynamic_scenario_fingerprint
        for scene, exported in zip(scenes, bundles[study]["scenarios"], strict=True):
            if fingerprint(scene) != exported["fingerprint"]:
                raise ValueError(f"Cannot snapshot different computation inputs: {study}")
        studies[study] = [input_record(scene) for scene in scenes]
    snapshot = {
        "schemaVersion": 1,
        "citySourceSha256": city.metadata["sourceSha256"],
        "nativeSha256": {key: digest(public / name) for key, name in NATIVE_FILES.items()},
        "studies": studies,
    }
    SNAPSHOT.write_text(json.dumps(snapshot, sort_keys=True, separators=(",", ":")) + "\n")
    print("Captured 24 exact planning inputs; native fingerprints and records unchanged.")


if __name__ == "__main__":
    main()
