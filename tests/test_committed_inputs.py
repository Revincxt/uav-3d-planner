"""Cross-platform input audits must keep exact fingerprints and fail closed."""

from __future__ import annotations

import hashlib
import importlib
import json
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from uav3d.benchmark import scene_fingerprint
from uav3d.dynamic import DynamicScenario, MovingSphere, dynamic_scenario_fingerprint
from uav3d.scene import AABB, Bounds3D, Cylinder, Scene


@pytest.fixture
def inputs(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
    return importlib.import_module("committed_inputs")


@pytest.fixture
def scene():
    return Scene(
        "mission",
        "Mission",
        Bounds3D((0, 0, 0), (100, 100, 100)),
        (10.123456789, 10, 20),
        (90, 90, 20),
        buildings=(AABB("roof", (30, 30, 0), (40, 40, 10)),),
        no_fly_zones=(Cylinder("fixed", (50, 50), 3, 0, 80),),
        metadata={
            "missionTaskPoints": [
                {"position": [20, 20, 20], "serviceDurationS": 0.0, "label": "Gate"}
            ]
        },
    )


def test_exact_static_inputs_round_trip_without_duplicating_city(inputs, scene):
    city = SimpleNamespace(buildings=scene.buildings)
    saved = json.loads(json.dumps(inputs.input_record(scene)))
    assert "buildings" not in saved
    restored = inputs.restore_input(saved, city, dynamic=False)
    assert scene_fingerprint(restored) == scene_fingerprint(scene)
    assert restored.start == scene.start
    assert restored.buildings == scene.buildings
    # Even accepted reconstruction roundoff is NOT accepted by the exact fingerprint.
    drifted = replace(scene, start=(scene.start[0] + 6e-10, 10, 20))
    inputs.assert_equivalent(inputs.input_record(drifted), saved)
    assert scene_fingerprint(drifted) != scene_fingerprint(restored)


def test_exact_dynamic_inputs_keep_obstacle_schedule_and_fingerprint(inputs, scene):
    episode = DynamicScenario(
        "episode",
        "Episode",
        scene,
        moving_spheres=(
            MovingSphere("traffic", 2, ((0, (60, 60, 20)), (5.123456789, (70, 70, 20)))),
        ),
        metadata={"mission": {"taskPoints": scene.metadata["missionTaskPoints"]}},
    )
    saved = json.loads(json.dumps(inputs.input_record(episode)))
    restored = inputs.restore_input(saved, SimpleNamespace(buildings=scene.buildings), dynamic=True)
    assert dynamic_scenario_fingerprint(restored) == dynamic_scenario_fingerprint(episode)
    assert restored.moving_spheres == episode.moving_spheres
    assert restored.metadata["mission"]["taskPoints"] == scene.metadata["missionTaskPoints"]


@pytest.mark.parametrize(
    ("expected", "actual"),
    [
        ({"point": [1000.0]}, {"point": [1000.00001]}),
        ({"duration": 5}, {"duration": 5.001}),
        ({"a": 1}, {"b": 1}),
        ([1, 2], [2, 1]),
        ([1, 2], [1]),
        (1, True),
        (1, "1"),
        (float("nan"), float("nan")),
        (float("inf"), float("inf")),
    ],
)
def test_reconstruction_rejects_changes_and_nonfinite_values(inputs, expected, actual):
    with pytest.raises(ValueError, match="Changed planning input"):
        inputs.assert_equivalent(expected, actual)


def test_snapshot_cannot_override_physical_city(inputs, scene):
    saved = inputs.input_record(scene)
    saved["buildings"] = []
    with pytest.raises(ValueError, match="complete verified source city"):
        inputs.restore_input(saved, SimpleNamespace(buildings=scene.buildings), dynamic=False)


@pytest.fixture
def snapshot(inputs, tmp_path, monkeypatch):
    public = tmp_path / "public"
    public.mkdir()
    for name in inputs.NATIVE_FILES.values():
        (public / name).write_text("native records\n", encoding="utf-8")
    data = {
        "schemaVersion": 1,
        "citySourceSha256": "sha256:official-city",
        "nativeSha256": {
            key: inputs.digest(public / name) for key, name in inputs.NATIVE_FILES.items()
        },
        "studies": {key: [{} for _ in range(8)] for key in inputs.NATIVE_FILES},
    }
    path = tmp_path / "planning-inputs.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    monkeypatch.setattr(inputs, "SNAPSHOT", path)
    city = SimpleNamespace(metadata={"sourceSha256": "sha256:official-city"})
    return public, city, data, path


def test_snapshot_is_bound_to_native_bytes_and_city(inputs, snapshot):
    public, city, data, _ = snapshot
    assert inputs.load_inputs(public, city) == data["studies"]
    (public / "demo-data.json").write_text("changed records\n", encoding="utf-8")
    with pytest.raises(ValueError, match="do not match native records"):
        inputs.load_inputs(public, city)


@pytest.mark.parametrize("change", ["version", "city", "study", "mission", "hash"])
def test_snapshot_rejects_stale_or_incomplete_inputs(inputs, snapshot, change):
    public, city, original, path = snapshot
    data = deepcopy(original)
    if change == "version":
        data["schemaVersion"] = 2
    elif change == "city":
        data["citySourceSha256"] = "sha256:other-city"
    elif change == "study":
        del data["studies"]["dynamic"]
    elif change == "mission":
        data["studies"]["static"].pop()
    else:
        data["nativeSha256"]["predictive"] = "sha256:other-records"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError):
        inputs.load_inputs(public, city)


@pytest.fixture
def computation_source(inputs, tmp_path, monkeypatch):
    source = b"def distance(x, y):\n    return (x*x + y*y)**0.5\n"
    sha256 = "sha256:" + hashlib.sha256(source).hexdigest()
    path = tmp_path / "planner.py"
    path.write_bytes(source)
    monkeypatch.setattr(inputs, "ROOT", tmp_path)
    return path, sha256


def test_computation_source_requires_exact_bytes(inputs, computation_source):
    path, sha256 = computation_source
    inputs.audit_source(path, sha256)


@pytest.mark.parametrize(
    "source",
    [
        "def distance(x, y):\n    return (x * x + y * y) ** 0.5\n",
        "def distance(x, y):\n    return (x * x + y * y) ** 0.6\n",
        "def distance(x, y):\n    return x + y\n",
    ],
)
def test_changed_computation_bytes_cannot_reuse_recorded_hash(inputs, computation_source, source):
    path, sha256 = computation_source
    path.write_text(source, encoding="utf-8")
    assert inputs.digest(path) != sha256
    with pytest.raises(ValueError, match="Computation source changed"):
        inputs.audit_source(path, sha256)
