from __future__ import annotations

import json
from pathlib import Path

import pytest

import uav3d.dataset as dataset
from uav3d.benchmark import problem_fingerprint
from uav3d.dataset import generate_dataset_manifest
from uav3d.scene import generate_random_city, load_scene


def test_dataset_manifest_is_deterministic(tmp_path: Path) -> None:
    first_directory = tmp_path / "first"
    second_directory = tmp_path / "second"

    first = generate_dataset_manifest([9, 3], 4, first_directory)
    second = generate_dataset_manifest([9, 3], 4, second_directory)

    assert first == second
    assert (first_directory / "manifest.json").read_bytes() == (
        second_directory / "manifest.json"
    ).read_bytes()
    assert [record["seed"] for record in first["records"]] == [3, 9]
    for record in first["records"]:
        relative_path = Path(record["relative_path"])
        assert (first_directory / relative_path).read_bytes() == (
            second_directory / relative_path
        ).read_bytes()


def test_manifest_counts_are_conserved_and_scenes_match_fingerprints(tmp_path: Path) -> None:
    output = tmp_path / "dataset"
    manifest = generate_dataset_manifest([1, 2, 3], 3, output)

    assert manifest["requested"] == manifest["accepted"] + manifest["rejected"]
    assert manifest["requested"] == 3
    assert manifest["accepted"] == 3
    assert manifest["rejected"] == 0
    assert manifest["generator"] == {
        "name": "uav3d.scene.generate_random_city",
        "version": "diagonal-corridor-v1",
        "parameters": {"building_count": 3},
    }
    assert json.loads((output / "manifest.json").read_text(encoding="utf-8")) == manifest

    for record in manifest["records"]:
        assert record["status"] == "accepted"
        scene = load_scene(output / record["relative_path"])
        assert record["fingerprint"] == problem_fingerprint(scene)


def test_generator_failures_are_logged_without_planner_filtering(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original_generator = generate_random_city

    def generate_with_one_rejection(seed: int, building_count: int) -> object:
        if seed == 5:
            raise RuntimeError("synthetic placement failure")
        return original_generator(seed, building_count)

    monkeypatch.setattr(dataset, "generate_random_city", generate_with_one_rejection)
    output = tmp_path / "dataset"
    manifest = generate_dataset_manifest([4, 5, 6], 2, output)

    assert manifest["requested"] == 3
    assert manifest["accepted"] == 2
    assert manifest["rejected"] == 1
    assert manifest["requested"] == manifest["accepted"] + manifest["rejected"]
    rejected = next(record for record in manifest["records"] if record["seed"] == 5)
    assert rejected == {
        "seed": 5,
        "status": "rejected",
        "rejection_reason": "RuntimeError: synthetic placement failure",
    }
    assert not (output / "scenes" / "random-city-5.json").exists()
