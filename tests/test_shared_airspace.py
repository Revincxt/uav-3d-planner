from dataclasses import replace

import pytest

from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.replanning import _horizontal_escape
from uav3d.scene import Bounds3D, Scene
from uav3d.shared_airspace import share_dynamic_airspace


def query(index):
    scene = Scene(
        str(index),
        "Task",
        Bounds3D((-100, -100, 0), (100, 100, 100)),
        (-70, index * 10, 30),
        (70, index * 10, 30),
        drone_radius=0,
        safety_margin=0,
    )
    aircraft = MovingSphere(f"cargo-{index}", 8, ((0, (0, -50, 30)), (10, (0, 50, 30))))
    return DynamicScenario(str(index), "Task", scene, moving_spheres=(aircraft,))


def test_queries_receive_identical_obstacles_without_changing_endpoints():
    originals = tuple(query(i) for i in range(4))
    shared = share_dynamic_airspace(originals, "shared")
    assert len(shared[0].moving_spheres) == 4
    assert all(s.moving_spheres is shared[0].moving_spheres for s in shared)
    assert len({s.metadata["sharedWorld"]["fingerprint"] for s in shared}) == 1
    assert [s.static_scene.start for s in shared] == [s.static_scene.start for s in originals]
    assert len(originals[0].moving_spheres) == 1
    assert shared[0].metadata["sharedWorld"]["missionDeconfliction"] == "not-jointly-optimized"


def test_shared_world_rejects_different_constraints_and_duplicate_hazard_ids():
    a, b = query(0), query(1)
    with pytest.raises(ValueError, match="unique"):
        share_dynamic_airspace((a, a), "shared")
    with pytest.raises(ValueError, match="identical"):
        share_dynamic_airspace(
            (a, replace(b, static_scene=replace(b.static_scene, safety_margin=1))), "shared"
        )


def test_reservation_cannot_obstruct_another_mission_anchor():
    a, b = query(0), query(1)
    reservation = TemporaryCylinder("slot", b.static_scene.goal[:2], 10, 0, 90, 5, 10)
    with pytest.raises(ValueError, match="another mission"):
        share_dynamic_airspace((replace(a, temporary_cylinders=(reservation,)), b), "shared")


def test_level_escape_is_bounded_and_continuously_safe_when_hover_is_unsafe():
    scenario = query(0)
    scenario = replace(
        scenario,
        moving_spheres=(MovingSphere("approach", 8, ((0, (0, -20, 30)), (4, (0, 20, 30)))),),
    )
    position = (0, 0, 30)
    assert not spacetime_segment_is_free(scenario, position, position, 0, 2)
    escape = _horizontal_escape(scenario, position, 0, 2, 14)
    assert escape and escape[0] == position and escape[-1][2] == position[2]
    assert sum((a - b) ** 2 for a, b in zip(escape[-1], position, strict=True)) <= 28**2 + 1e-8
    assert spacetime_segment_is_free(scenario, *escape, 0, 2)
    assert spacetime_segment_is_free(scenario, escape[-1], escape[-1], 2, 4)


def test_escape_fails_closed_when_already_colliding():
    scenario = query(0)
    assert _horizontal_escape(scenario, (0, -50, 30), 0, 2, 14) == ()
