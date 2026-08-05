from __future__ import annotations

import json
import math

import pytest

from uav3d.dynamic import DynamicScenario, dynamic_scenario_fingerprint
from uav3d.dynamic_collision import point_is_free_at_time
from uav3d.geometry import Point3
from uav3d.predictive_scenarios import (
    build_predictive_cohort,
    list_predictive_scenarios,
    load_predictive_scenario,
)

EXPECTED_IDS = (
    "wait-then-straight",
    "closing-window",
    "periodic-traffic",
    "chained-restrictions",
    "multi-obstacle",
    "vertical-time-window",
    "urban-canyon-merge",
    "rooftop-transfer",
)


def _xy_distance_to_segment(point: Point3, start: Point3, goal: Point3) -> float:
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    denominator = dx * dx + dy * dy
    projection = (
        0.0
        if denominator == 0
        else ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator
    )
    fraction = max(0.0, min(1.0, projection))
    closest_x = start[0] + fraction * dx
    closest_y = start[1] + fraction * dy
    return math.hypot(point[0] - closest_x, point[1] - closest_y)


def test_predictive_registry_has_fixed_v05_cohorts_and_demo_order() -> None:
    assert list_predictive_scenarios() == EXPECTED_IDS
    assert list_predictive_scenarios("calibration") == ("wait-then-straight",)
    assert list_predictive_scenarios("demo") == (
        "periodic-traffic",
        "urban-canyon-merge",
        "rooftop-transfer",
    )
    assert list_predictive_scenarios("diagnostic") == (
        "closing-window",
        "chained-restrictions",
        "multi-obstacle",
        "vertical-time-window",
    )
    with pytest.raises(ValueError, match="unknown predictive cohort"):
        list_predictive_scenarios("test")
    with pytest.raises(ValueError, match="unknown predictive scenario"):
        load_predictive_scenario("missing")


def test_scenarios_are_deterministic_unique_and_json_round_trip() -> None:
    fingerprints: set[str] = set()
    for scenario_id in EXPECTED_IDS:
        first = load_predictive_scenario(scenario_id)
        second = load_predictive_scenario(scenario_id)
        assert first == second
        payload = json.loads(json.dumps(first.to_dict(), sort_keys=True))
        restored = DynamicScenario.from_dict(payload)
        assert restored == first
        fingerprint = dynamic_scenario_fingerprint(first)
        assert dynamic_scenario_fingerprint(restored) == fingerprint
        fingerprints.add(fingerprint)
    assert len(fingerprints) == len(EXPECTED_IDS)


def test_city_complexity_geometry_ids_and_time_zero_endpoints() -> None:
    for scenario_id in EXPECTED_IDS:
        scenario = load_predictive_scenario(scenario_id)
        scene = scenario.static_scene
        hazards = len(scenario.temporary_cylinders) + len(scenario.moving_spheres)
        if scenario_id == "wait-then-straight":
            assert len(scene.buildings) >= 8
        else:
            assert len(scene.buildings) >= 14
            assert len(scene.no_fly_zones) >= 1
            assert hazards >= 2
            assert len({building.maximum[2] for building in scene.buildings}) >= 5
            assert abs(scene.goal[0] - scene.start[0]) >= 72
            assert abs(scene.goal[1] - scene.start[1]) >= 24
            assert scene.goal[2] != scene.start[2]

        obstacle_ids = [building.obstacle_id for building in scene.buildings]
        obstacle_ids.extend(zone.zone_id for zone in scene.no_fly_zones)
        obstacle_ids.extend(zone.zone_id for zone in scenario.temporary_cylinders)
        obstacle_ids.extend(sphere.sphere_id for sphere in scenario.moving_spheres)
        assert len(obstacle_ids) == len(set(obstacle_ids))
        assert point_is_free_at_time(scenario, scene.start, 0.0)
        assert point_is_free_at_time(scenario, scene.goal, 0.0)

        for building in scene.buildings:
            assert all(value % 4 == 0 for value in (*building.minimum, *building.maximum))
            assert all(
                lower <= value <= upper
                for value, lower, upper in zip(
                    (*building.minimum, *building.maximum),
                    (*scene.bounds.minimum, *scene.bounds.minimum),
                    (*scene.bounds.maximum, *scene.bounds.maximum),
                    strict=True,
                )
            )
        for sphere in scenario.moving_spheres:
            for _, position in sphere.keyframes:
                assert all(
                    lower <= value <= upper
                    for value, lower, upper in zip(
                        position, scene.bounds.minimum, scene.bounds.maximum, strict=True
                    )
                )
            assert _xy_distance_to_segment(sphere.keyframes[-1][1], scene.start, scene.goal) >= 12.0


def test_cohort_manifest_is_v05_geometry_only_and_outcome_independent() -> None:
    scenarios, manifest = build_predictive_cohort()
    assert tuple(scenario.scenario_id for scenario in scenarios) == EXPECTED_IDS
    assert manifest["schema_version"] == "predictive-scenario-manifest-v2"
    assert manifest["dataset_id"] == "predictive-urban-v0.5"
    assert manifest["requested"] == manifest["accepted"] + manifest["rejected"]
    assert (manifest["requested"], manifest["accepted"], manifest["rejected"]) == (8, 8, 0)
    assert manifest["accepted_by_cohort"] == {"calibration": 1, "demo": 3, "diagnostic": 4}

    selection = manifest["selection"]
    assert isinstance(selection, dict)
    assert selection["planner_outcomes_consulted"] is False
    assert selection["preregistered"] is False
    records = manifest["records"]
    assert isinstance(records, list)
    assert all(record["status"] == "accepted" for record in records)
    assert {record["fingerprint"] for record in records} == {
        dynamic_scenario_fingerprint(scenario) for scenario in scenarios
    }
    serialized = json.dumps(manifest, sort_keys=True)
    assert "planner_id" not in serialized
    assert "algorithm" not in serialized
    assert "mission_success" not in serialized


def test_opening_and_closing_windows_keep_opposite_transitions() -> None:
    opening = load_predictive_scenario("wait-then-straight")
    gate = (52.0, 44.0, 10.0)
    assert not point_is_free_at_time(opening, gate, 0.0)
    assert not point_is_free_at_time(opening, gate, 7.999)
    assert point_is_free_at_time(opening, gate, 8.0)

    closing = load_predictive_scenario("closing-window")
    window = (54.0, 44.0, 10.0)
    assert point_is_free_at_time(closing, window, 3.999)
    assert not point_is_free_at_time(closing, window, 4.0)
    assert point_is_free_at_time(closing, window, 28.0)


def test_market_chained_and_transit_events_are_deterministic() -> None:
    periodic = load_predictive_scenario("periodic-traffic")
    sphere = periodic.moving_spheres[0]
    assert sphere.position_at(3.0) == (28.0, 44.0, 16.0)
    assert sphere.position_at(9.0) == (28.0, 44.0, 16.0)
    assert sphere.position_at(15.0) == (28.0, 44.0, 16.0)
    assert periodic.temporary_cylinders[0].is_active(9.0)

    chained = load_predictive_scenario("chained-restrictions")
    assert all(zone.is_active(7.0) for zone in chained.temporary_cylinders)
    assert not chained.temporary_cylinders[0].is_active(8.0)
    assert chained.temporary_cylinders[1].is_active(8.0)
    assert not chained.temporary_cylinders[1].is_active(16.0)

    multiple = load_predictive_scenario("multi-obstacle")
    assert len(multiple.temporary_cylinders) == 1
    assert len(multiple.moving_spheres) == 2
    assert multiple.moving_spheres[0].position_at(4.0) == (28.0, 44.0, 12.0)
    assert multiple.moving_spheres[1].position_at(6.0) == (76.0, 44.0, 16.0)


def test_vertical_canyon_and_rooftop_events_are_preserved() -> None:
    vertical = load_predictive_scenario("vertical-time-window")
    low = (52.0, 44.0, 12.0)
    high = (52.0, 44.0, 36.0)
    assert point_is_free_at_time(vertical, low, 4.999)
    assert not point_is_free_at_time(vertical, high, 4.999)
    assert not point_is_free_at_time(vertical, low, 5.0)
    assert point_is_free_at_time(vertical, high, 5.0)

    canyon = load_predictive_scenario("urban-canyon-merge")
    assert canyon.moving_spheres[0].position_at(6.0)[:2] == (56.0, 48.0)
    assert canyon.moving_spheres[1].position_at(8.0)[:2] == (56.0, 48.0)
    assert canyon.temporary_cylinders[0].is_active(8.0)

    rooftop = load_predictive_scenario("rooftop-transfer")
    assert rooftop.moving_spheres[0].position_at(6.0) == (52.0, 44.0, 40.0)
    assert rooftop.temporary_cylinders[0].is_active(4.0)
    assert not rooftop.temporary_cylinders[0].is_active(20.0)


def test_every_scenario_declares_v05_urban_metadata() -> None:
    case_ids: set[int] = set()
    for scenario_id in EXPECTED_IDS:
        scenario = load_predictive_scenario(scenario_id)
        assert scenario.metadata["study"] == "predictive-space-time-v0.5"
        assert scenario.metadata["dataset"] == "predictive-urban-v0.5"
        assert scenario.metadata["forecast_required"] is True
        assert scenario.metadata["selection_basis"] == "validated-scenario-construction-only"
        assert scenario.metadata["planner_outcome_filtering"] == "forbidden"
        assert scenario.metadata["decision_contract"]
        assert scenario.metadata["district"]
        assert scenario.metadata["street_pattern"]
        assert scenario.static_scene.metadata["dataset"] == "predictive-urban-v0.5"
        case_ids.add(int(scenario.metadata["case_id"]))
    assert case_ids == set(range(5101, 5109))
