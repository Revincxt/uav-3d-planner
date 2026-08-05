from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

import pytest

from uav3d.benchmark import scene_fingerprint
from uav3d.collision import point_is_free
from uav3d.scene import (
    AABB,
    Bounds3D,
    Cylinder,
    Scene,
    generate_random_city,
    list_builtin_scenes,
    load_builtin_scene,
    load_scene,
    save_scene,
)


def test_builtin_scenes_have_valid_endpoints_and_unique_ids() -> None:
    assert len(list_builtin_scenes()) == 4
    for scene_id in list_builtin_scenes():
        scene = load_builtin_scene(scene_id)
        assert point_is_free(scene, scene.start)
        assert point_is_free(scene, scene.goal)
        assert scene.metadata["family"] == "curated"


def test_scene_json_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "scene.json"
    original = load_builtin_scene("restricted-core")
    save_scene(original, path)
    assert load_scene(path) == original


def test_random_city_is_seeded_and_preserves_metadata() -> None:
    first = generate_random_city(42, 8)
    second = generate_random_city(42, 8)
    different = generate_random_city(43, 8)
    assert first == second
    assert first != different
    assert first.metadata["seed"] == 42
    assert len(first.buildings) == 8


def test_fingerprint_ignores_obstacle_order() -> None:
    scene = load_builtin_scene("open-blocks")
    reordered = Scene(
        scene.scene_id,
        scene.name,
        scene.bounds,
        scene.start,
        scene.goal,
        tuple(reversed(scene.buildings)),
        tuple(reversed(scene.no_fly_zones)),
        scene.drone_radius,
        scene.safety_margin,
        scene.metadata,
    )
    assert scene_fingerprint(scene) == scene_fingerprint(reordered)


@pytest.mark.parametrize(
    "constructor",
    [
        lambda: Bounds3D((0, 0, 0), (0, 1, 1)),
        lambda: AABB("bad", (0, 0, 0), (1, 1, math.inf)),
        lambda: Cylinder("bad", (0, 0), 0, 0, 1),
        lambda: Scene(
            "bad",
            "Bad",
            Bounds3D((0, 0, 0), (10, 10, 10)),
            (-1, 1, 1),
            (9, 9, 9),
        ),
        lambda: Scene(
            "duplicate",
            "Duplicate",
            Bounds3D((0, 0, 0), (10, 10, 10)),
            (1, 1, 1),
            (9, 9, 9),
            (AABB("same", (2, 2, 0), (3, 3, 3)),),
            (Cylinder("same", (5, 5), 1, 0, 4),),
        ),
    ],
)
def test_invalid_scene_contracts_are_rejected(constructor: Callable[[], object]) -> None:
    with pytest.raises(ValueError):
        constructor()


def test_unknown_builtin_scene_is_descriptive() -> None:
    with pytest.raises(ValueError, match="unknown built-in scene"):
        load_builtin_scene("missing")
