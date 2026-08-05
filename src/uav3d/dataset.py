"""Deterministic random-scene dataset generation and provenance manifests."""

from __future__ import annotations

import json
from collections.abc import Iterable
from pathlib import Path
from typing import Any

from uav3d.benchmark import problem_fingerprint
from uav3d.scene import generate_random_city, save_scene

MANIFEST_SCHEMA_VERSION = "1.0"
GENERATOR_NAME = "uav3d.scene.generate_random_city"
GENERATOR_VERSION = "diagonal-corridor-v1"


def generate_dataset_manifest(
    seeds: Iterable[int],
    building_count: int,
    output_dir: str | Path,
) -> dict[str, Any]:
    """Generate random scenes and write a deterministic dataset manifest.

    A scene is accepted whenever the scene generator succeeds. Planner outcomes
    are deliberately not consulted, so the dataset cannot be filtered in favor
    of any evaluated algorithm.
    """

    if building_count < 0:
        raise ValueError("building_count must be non-negative")
    requested_seeds = tuple(sorted(seeds))
    if len(requested_seeds) != len(set(requested_seeds)):
        raise ValueError("dataset seeds must be unique")

    destination = Path(output_dir)
    scene_directory = destination / "scenes"
    scene_directory.mkdir(parents=True, exist_ok=True)

    records: list[dict[str, object]] = []
    accepted = 0
    rejected = 0
    for seed in requested_seeds:
        try:
            scene = generate_random_city(seed, building_count)
        except (RuntimeError, ValueError) as error:
            rejected += 1
            records.append(
                {
                    "seed": seed,
                    "status": "rejected",
                    "rejection_reason": f"{type(error).__name__}: {error}",
                }
            )
            continue

        relative_path = Path("scenes") / f"{scene.scene_id}.json"
        save_scene(scene, destination / relative_path)
        accepted += 1
        records.append(
            {
                "seed": seed,
                "status": "accepted",
                "fingerprint": problem_fingerprint(scene),
                "relative_path": relative_path.as_posix(),
            }
        )

    manifest: dict[str, Any] = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "generator": {
            "name": GENERATOR_NAME,
            "version": GENERATOR_VERSION,
            "parameters": {"building_count": building_count},
        },
        "requested": len(requested_seeds),
        "accepted": accepted,
        "rejected": rejected,
        "records": records,
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return manifest
