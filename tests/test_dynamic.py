from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from uav3d.collision import segment_is_free
from uav3d.dynamic import (
    DynamicScenario,
    MovingSphere,
    TemporaryCylinder,
    dynamic_scenario_fingerprint,
    list_builtin_dynamic_scenarios,
    load_builtin_dynamic_scenario,
    load_dynamic_scenario,
    save_dynamic_scenario,
    snapshot_scene,
)
from uav3d.dynamic_collision import (
    point_is_free_at_time,
    spacetime_segment_is_free,
    timed_path_is_free,
)
from uav3d.scene import Bounds3D, Scene


def empty_scenario(
    *,
    temporary: tuple[TemporaryCylinder, ...] = (),
    moving: tuple[MovingSphere, ...] = (),
) -> DynamicScenario:
    scene = Scene(
        "static",
        "Static",
        Bounds3D((0.0, 0.0, 0.0), (20.0, 20.0, 20.0)),
        (2.0, 10.0, 10.0),
        (18.0, 10.0, 10.0),
        drone_radius=0.0,
        safety_margin=0.0,
    )
    return DynamicScenario("dynamic", "Dynamic", scene, temporary, moving)


def test_temporary_cylinder_uses_a_half_open_activity_interval() -> None:
    zone = TemporaryCylinder("zone", (10.0, 10.0), 2.0, 5.0, 15.0, 2.0, 4.0)
    scenario = empty_scenario(temporary=(zone,))

    assert point_is_free_at_time(scenario, (10.0, 10.0, 10.0), 1.999)
    assert not point_is_free_at_time(scenario, (10.0, 10.0, 10.0), 2.0)
    assert not point_is_free_at_time(scenario, (10.0, 10.0, 10.0), 3.999)
    assert point_is_free_at_time(scenario, (10.0, 10.0, 10.0), 4.0)
    # Contact only at the exclusive right boundary is not reported as collision.
    assert spacetime_segment_is_free(scenario, (6.0, 10.0, 10.0), (8.0, 10.0, 10.0), 3.0, 4.0)


def test_moving_sphere_requires_strict_keyframes_and_interpolates_linearly() -> None:
    sphere = MovingSphere(
        "traffic",
        1.0,
        ((1.0, (0.0, 0.0, 0.0)), (3.0, (4.0, 2.0, 0.0))),
    )
    assert sphere.position_at(0.0) == (0.0, 0.0, 0.0)
    assert sphere.position_at(2.0) == (2.0, 1.0, 0.0)
    assert sphere.position_at(4.0) == (4.0, 2.0, 0.0)
    with pytest.raises(ValueError, match="at least two"):
        MovingSphere("invalid", 1.0, ((1.0, (0.0, 0.0, 0.0)),))
    with pytest.raises(ValueError, match="strictly increasing"):
        MovingSphere(
            "invalid",
            1.0,
            ((1.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0))),
        )


def test_relative_motion_detects_exact_tangency_and_splits_at_keyframes() -> None:
    tangent = MovingSphere(
        "tangent",
        1.0,
        ((0.0, (10.0, 9.0, 10.0)), (2.0, (10.0, 11.0, 10.0))),
    )
    scenario = empty_scenario(moving=(tangent,))
    assert not spacetime_segment_is_free(scenario, (8.0, 10.0, 10.0), (12.0, 10.0, 10.0), 0.0, 2.0)

    turning = MovingSphere(
        "turning",
        0.5,
        (
            (0.0, (10.0, 2.0, 10.0)),
            (1.0, (10.0, 10.0, 10.0)),
            (2.0, (10.0, 18.0, 10.0)),
        ),
    )
    turning_scenario = empty_scenario(moving=(turning,))
    assert not spacetime_segment_is_free(
        turning_scenario, (2.0, 10.0, 10.0), (18.0, 10.0, 10.0), 0.0, 2.0
    )


def test_long_patrol_uses_logarithmic_lookup_without_changing_interpolation() -> None:
    class CountedFrames(tuple):
        reads = 0

        def __getitem__(self, index):
            self.reads += 1
            return super().__getitem__(index)

    frames = CountedFrames((float(i), (float(i % 2), 10.0, 10.0)) for i in range(1025))
    sphere = MovingSphere("long-patrol", 0.5, frames)
    for clock in (0, 1, 511, 511.5, 1023.75, 1024, 2000):
        frames.reads = 0
        clamped = min(1024, max(0, clock))
        index = min(int(clamped), 1023)
        left, right = float(index % 2), float((index + 1) % 2)
        expected = left + (right - left) * (clamped - index)
        assert sphere.position_at(clock) == (expected, 10.0, 10.0)
        assert frames.reads < 20


def test_long_patrol_collision_slices_preserve_turn_and_exact_boundary_contacts() -> None:
    frames = tuple((float(i), (10.0, 2.0 if i % 2 == 0 else 18.0, 10.0)) for i in range(1025))
    full = empty_scenario(moving=(MovingSphere("patrol", 0.5, frames),))
    short = empty_scenario(moving=(MovingSphere("patrol", 0.5, frames[510:514]),))
    for start, end in ((510, 511), (510.5, 511.5), (511, 512), (511.5, 512.5)):
        for y in (2.0, 10.0, 18.0):
            a, b = (2.0, y, 10.0), (18.0, y, 10.0)
            assert spacetime_segment_is_free(full, a, b, start, end) == (
                spacetime_segment_is_free(short, a, b, start, end)
            )


def test_no_dynamic_obstacles_matches_static_collision_contract() -> None:
    scenario = empty_scenario()
    cases = (
        ((2.0, 10.0, 10.0), (18.0, 10.0, 10.0)),
        ((-1.0, 10.0, 10.0), (18.0, 10.0, 10.0)),
    )
    for start, end in cases:
        assert spacetime_segment_is_free(scenario, start, end, 0.0, 2.0) == segment_is_free(
            scenario.static_scene, start, end
        )
    assert timed_path_is_free(
        scenario,
        ((0.0, (2.0, 10.0, 10.0)), (2.0, (18.0, 10.0, 10.0))),
    )


def test_snapshot_respects_activity_boundary_and_conservatively_cylindrifies_sphere() -> None:
    zone = TemporaryCylinder("zone", (10.0, 10.0), 2.0, 5.0, 15.0, 1.0, 2.0)
    sphere = MovingSphere("sphere", 2.0, ((0.0, (4.0, 6.0, 8.0)), (2.0, (8.0, 6.0, 8.0))))
    scenario = empty_scenario(temporary=(zone,), moving=(sphere,))
    active = snapshot_scene(scenario, 1.0)
    expired = snapshot_scene(scenario, 2.0)
    assert [item.zone_id for item in active.no_fly_zones] == ["zone", "sphere"]
    assert [item.zone_id for item in expired.no_fly_zones] == ["sphere"]
    moving_zone = expired.no_fly_zones[0]
    assert moving_zone.center == (8.0, 6.0)
    assert (moving_zone.z_min, moving_zone.z_max) == (6.0, 10.0)


def test_dynamic_json_round_trip_and_semantic_fingerprint(tmp_path: Path) -> None:
    original = load_builtin_dynamic_scenario("pop-up-nfz")
    output = tmp_path / "scenario.json"
    save_dynamic_scenario(original, output)
    loaded = load_dynamic_scenario(output)
    assert loaded == original
    expected = dynamic_scenario_fingerprint(original)
    relabeled = replace(
        original,
        scenario_id="renamed",
        name="Renamed",
        metadata={"irrelevant": True},
        temporary_cylinders=tuple(reversed(original.temporary_cylinders)),
    )
    assert dynamic_scenario_fingerprint(relabeled) == expected
    changed = replace(
        original,
        temporary_cylinders=(replace(original.temporary_cylinders[0], radius=6.5),),
    )
    assert dynamic_scenario_fingerprint(changed) != expected


def test_four_builtin_dynamic_scenarios_are_reproducible() -> None:
    assert list_builtin_dynamic_scenarios() == (
        "pop-up-nfz",
        "crossing-traffic",
        "closing-gate",
        "vertical-escape",
    )
    for scenario_id in list_builtin_dynamic_scenarios():
        first = load_builtin_dynamic_scenario(scenario_id)
        second = load_builtin_dynamic_scenario(scenario_id)
        assert first == second
        assert dynamic_scenario_fingerprint(first) == dynamic_scenario_fingerprint(second)
