from __future__ import annotations

import json

import pytest

from uav3d.dynamic import DynamicScenario, dynamic_scenario_fingerprint
from uav3d.dynamic_collision import point_is_free_at_time
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
)

EXPECTED_FINGERPRINTS = {
    "wait-then-straight": "sha256:d392b681dcc5753adaf377a6920e6ffae95983191a4bacb6e9a5c1b8c06cb1f5",
    "closing-window": "sha256:68daf5f28ab6b28fef7b8d37f7fba4b04280736cb9c0996cb4d53de0935bb4de",
    "periodic-traffic": "sha256:cd16d4ab5c4445f5cdacd3a04e5d36e34010729136e3cf1b75da51d123c8ae87",
    "chained-restrictions": (
        "sha256:513f2ad5b379e5209e1d73806aafd5181d9b8f7f1fd5515a1fbed443852058e8"
    ),
    "multi-obstacle": "sha256:8880996d24954dc4c461d986a7c515912d306b1c685200fa1046d9e7248289cd",
    "vertical-time-window": (
        "sha256:e0a9c994a7cd6efe5884dcb9b4bc025af82803eda2bae7023a91df9b1cde3b60"
    ),
}


def test_predictive_registry_has_calibration_demo_and_diagnostic_cohorts() -> None:
    assert list_predictive_scenarios() == EXPECTED_IDS
    assert list_predictive_scenarios("calibration") == ("wait-then-straight",)
    assert list_predictive_scenarios("demo") == ("periodic-traffic",)
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


def test_scenario_fingerprints_are_stable_unique_and_round_trip() -> None:
    observed: dict[str, str] = {}
    for scenario_id in EXPECTED_IDS:
        first = load_predictive_scenario(scenario_id)
        second = load_predictive_scenario(scenario_id)
        assert first == second
        fingerprint = dynamic_scenario_fingerprint(first)
        assert fingerprint == EXPECTED_FINGERPRINTS[scenario_id]
        restored = DynamicScenario.from_dict(first.to_dict())
        assert dynamic_scenario_fingerprint(restored) == fingerprint
        observed[scenario_id] = fingerprint
    assert len(set(observed.values())) == len(EXPECTED_IDS)


def test_cohort_manifest_acceptance_is_geometry_only_and_outcome_independent() -> None:
    scenarios, manifest = build_predictive_cohort()
    assert tuple(scenario.scenario_id for scenario in scenarios) == EXPECTED_IDS
    assert manifest["requested"] == manifest["accepted"] + manifest["rejected"]
    assert (manifest["requested"], manifest["accepted"], manifest["rejected"]) == (6, 6, 0)
    assert manifest["accepted_by_cohort"] == {"calibration": 1, "demo": 1, "diagnostic": 4}

    selection = manifest["selection"]
    assert isinstance(selection, dict)
    assert selection["planner_outcomes_consulted"] is False
    assert selection["preregistered"] is False
    assert selection["cohort_assignment_basis"] == (
        "curated diagnostic labels in this source revision"
    )
    records = manifest["records"]
    assert isinstance(records, list)
    assert all(record["status"] == "accepted" for record in records)
    assert {record["fingerprint"] for record in records} == set(EXPECTED_FINGERPRINTS.values())
    # The manifest may describe the prohibition, but it must contain no planner result payload.
    serialized = json.dumps(manifest, sort_keys=True)
    assert "planner_id" not in serialized
    assert "algorithm" not in serialized
    assert "mission_success" not in serialized


def test_wait_and_closing_window_have_opposite_forecast_transitions() -> None:
    wait = load_predictive_scenario("wait-then-straight")
    gate = (36.0, 24.0, 10.0)
    assert not point_is_free_at_time(wait, gate, 0.0)
    assert not point_is_free_at_time(wait, gate, 5.499)
    assert point_is_free_at_time(wait, gate, 5.5)

    closing = load_predictive_scenario("closing-window")
    assert point_is_free_at_time(closing, gate, 2.999)
    assert not point_is_free_at_time(closing, gate, 3.0)
    assert point_is_free_at_time(closing, gate, 24.0)


def test_periodic_chained_and_multi_obstacle_schedules_require_lookahead() -> None:
    periodic = load_predictive_scenario("periodic-traffic")
    sphere = periodic.moving_spheres[0]
    assert sphere.position_at(2.5) == (36.0, 24.0, 10.0)
    assert sphere.position_at(7.5) == (36.0, 24.0, 10.0)
    assert sphere.position_at(12.5) == (36.0, 24.0, 10.0)

    chained = load_predictive_scenario("chained-restrictions")
    assert len(chained.temporary_cylinders) == 2
    assert chained.temporary_cylinders[0].is_active(5.5)
    assert chained.temporary_cylinders[1].is_active(5.5)
    assert not chained.temporary_cylinders[0].is_active(6.0)
    assert chained.temporary_cylinders[1].is_active(6.0)

    multiple = load_predictive_scenario("multi-obstacle")
    assert len(multiple.temporary_cylinders) == 1
    assert len(multiple.moving_spheres) == 2
    assert multiple.moving_spheres[0].position_at(3.0)[1] == 24.0
    assert multiple.moving_spheres[1].position_at(5.0)[1] == 24.0


def test_vertical_time_window_switches_the_only_open_altitude_band() -> None:
    scenario = load_predictive_scenario("vertical-time-window")
    low_point = (36.0, 24.0, 10.0)
    high_point = (36.0, 24.0, 26.0)

    assert point_is_free_at_time(scenario, low_point, 2.499)
    assert not point_is_free_at_time(scenario, high_point, 2.499)
    assert not point_is_free_at_time(scenario, low_point, 2.5)
    assert point_is_free_at_time(scenario, high_point, 2.5)


def test_every_scenario_declares_forecast_and_selection_metadata() -> None:
    case_ids: set[int] = set()
    for scenario_id in EXPECTED_IDS:
        scenario = load_predictive_scenario(scenario_id)
        assert scenario.metadata["study"] == "predictive-space-time-v0.4"
        assert scenario.metadata["forecast_required"] is True
        assert scenario.metadata["selection_basis"] == "validated-scenario-construction-only"
        assert scenario.metadata["planner_outcome_filtering"] == "forbidden"
        assert scenario.metadata["decision_contract"]
        case_ids.add(int(scenario.metadata["case_id"]))
    assert case_ids == set(range(4101, 4107))
