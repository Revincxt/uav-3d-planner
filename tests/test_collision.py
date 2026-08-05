from __future__ import annotations

import pytest

from uav3d.collision import (
    minimum_path_clearance,
    path_is_free,
    point_is_free,
    sample_path,
    segment_is_free,
)
from uav3d.scene import AABB, Bounds3D, Cylinder, Scene


def box_scene() -> Scene:
    return Scene(
        "box",
        "Box",
        Bounds3D((0, 0, 0), (20, 20, 20)),
        (2, 10, 10),
        (18, 10, 10),
        (AABB("block", (8, 8, 8), (12, 12, 12)),),
        drone_radius=0,
        safety_margin=0,
    )


def cylinder_scene() -> Scene:
    return Scene(
        "cylinder",
        "Cylinder",
        Bounds3D((0, 0, 0), (20, 20, 20)),
        (2, 10, 5),
        (18, 10, 5),
        no_fly_zones=(Cylinder("zone", (10, 10), 2, 3, 8),),
        drone_radius=0,
        safety_margin=0,
    )


def test_aabb_segment_queries_are_continuous_and_contact_is_collision() -> None:
    scene = box_scene()
    assert point_is_free(scene, (7.99, 10, 10))
    assert not point_is_free(scene, (8, 10, 10))
    assert not segment_is_free(scene, (2, 10, 10), (18, 10, 10))
    assert not segment_is_free(scene, (2, 8, 10), (18, 8, 10))
    assert segment_is_free(scene, (2, 7, 13), (18, 7, 13))
    assert not segment_is_free(scene, (10, 10, 10), (10, 10, 10))


def test_cylinder_segment_queries_cover_side_wall_and_end_cap() -> None:
    scene = cylinder_scene()
    assert not segment_is_free(scene, (2, 10, 5), (18, 10, 5))
    assert not segment_is_free(scene, (2, 12, 5), (18, 12, 5))
    assert segment_is_free(scene, (2, 10, 10), (18, 10, 10))
    assert not segment_is_free(scene, (10, 10, 1), (10, 10, 12))
    assert segment_is_free(scene, (3, 3, 5), (3, 3, 5))


def test_boundary_is_inset_by_vehicle_clearance() -> None:
    scene = Scene(
        "clearance",
        "Clearance",
        Bounds3D((0, 0, 0), (10, 10, 10)),
        (2, 2, 2),
        (8, 8, 8),
        drone_radius=1,
        safety_margin=1,
    )
    assert point_is_free(scene, (2, 2, 2))
    assert not point_is_free(scene, (1.99, 2, 2))


def test_path_sampling_and_clearance_metrics() -> None:
    scene = box_scene()
    path = [(2.0, 2.0, 2.0), (18.0, 2.0, 2.0)]
    assert path_is_free(scene, path)
    assert len(sample_path(path, spacing=2)) == 9
    assert minimum_path_clearance(scene, path, spacing=1) == pytest.approx(2.0)
    assert not path_is_free(scene, [])
