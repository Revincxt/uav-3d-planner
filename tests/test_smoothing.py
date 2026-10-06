from __future__ import annotations

import pytest

from uav3d.collision import path_is_free
from uav3d.geometry import polyline_length
from uav3d.scene import AABB, Bounds3D, Scene
from uav3d.smoothing import (
    farthest_visible_shortcut,
    remove_duplicate_points,
    sample_bspline,
    shortest_visible_shortcut,
    smooth_path,
)
from uav3d.validation import audit_path


def smoothing_scene() -> Scene:
    return Scene(
        "smooth",
        "Smooth",
        Bounds3D((0, 0, 0), (20, 20, 12)),
        (2, 10, 5),
        (18, 10, 5),
        (AABB("block", (8, 8, 0), (12, 12, 9)),),
        drone_radius=0.5,
        safety_margin=0.5,
    )


def test_duplicate_removal_and_bspline_preserve_endpoints() -> None:
    points = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (4.0, 2.0, 0.0), (8.0, 0.0, 0.0)]
    cleaned = remove_duplicate_points(points)
    sampled = sample_bspline(cleaned, 21)
    assert len(cleaned) == 3
    assert sampled[0] == cleaned[0]
    assert sampled[-1] == cleaned[-1]
    with pytest.raises(ValueError):
        sample_bspline(cleaned, 1)


def test_shared_smoothing_never_returns_an_uncertified_curve() -> None:
    scene = smoothing_scene()
    raw = [scene.start, (6, 6, 5), (14, 6, 5), scene.goal]
    assert path_is_free(scene, raw)
    shortcut = farthest_visible_shortcut(scene, raw)
    result = smooth_path(scene, raw, sample_spacing=0.75)
    assert result.collision_free
    assert result.method in {"bspline", "shortcut-fallback"}
    assert result.path[0] == scene.start
    assert result.path[-1] == scene.goal
    assert polyline_length(shortcut) <= polyline_length(raw) + 1e-9
    assert audit_path(scene, result.path).valid


def test_invalid_raw_path_is_not_smoothed() -> None:
    scene = smoothing_scene()
    result = smooth_path(scene, [scene.start, scene.goal])
    assert result.path == ()
    assert result.method == "not-run"


def test_shortest_visibility_route_avoids_farthest_index_detour() -> None:
    scene = smoothing_scene()
    raw = [scene.start, (6, 6, 5), (14, 6, 5), (18, 14, 5), (5, 18, 5), scene.goal]
    greedy = farthest_visible_shortcut(scene, raw)
    optimized = shortest_visible_shortcut(scene, raw)

    assert path_is_free(scene, raw)
    assert path_is_free(scene, optimized)
    assert optimized == [scene.start, (6, 6, 5), (14, 6, 5), scene.goal]
    assert polyline_length(optimized) < polyline_length(greedy) - 4.0
    assert optimized == shortest_visible_shortcut(scene, raw)
    result = smooth_path(scene, raw, optimize_shortcuts=True)
    assert result.collision_free
    assert audit_path(scene, result.path).valid
    assert polyline_length(result.path) <= polyline_length(greedy) + 1e-9
