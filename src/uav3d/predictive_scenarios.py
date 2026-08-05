"""Deterministic v0.5 urban scenarios for predictive space-time planning.

The registry is deliberately independent of every planner and mission outcome. A scenario is
accepted from construction, geometry, schedule, and endpoint contracts only.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal, TypeAlias

from uav3d.dynamic import (
    DynamicScenario,
    MovingSphere,
    TemporaryCylinder,
    dynamic_scenario_fingerprint,
)
from uav3d.dynamic_collision import point_is_free_at_time
from uav3d.predictive_city import (
    CALIBRATION_BLOCKS,
    CIVIC_COURTYARDS,
    MERGING_CANYONS,
    ORTHOGONAL_GRID,
    ROOFTOP_TOWERS,
    STAGGERED_MARKET,
    TERRACED_HEIGHTS,
    TRANSIT_BOULEVARD,
    make_city_scene,
)
from uav3d.scene import Cylinder

PredictiveCohort: TypeAlias = Literal["calibration", "demo", "diagnostic"]
PREDICTIVE_COHORTS: tuple[PredictiveCohort, ...] = ("calibration", "demo", "diagnostic")


@dataclass(frozen=True, slots=True)
class _ScenarioSpec:
    scenario_id: str
    cohort: PredictiveCohort
    event_family: str
    case_id: int
    builder: Callable[[], DynamicScenario]


def _metadata(
    cohort: PredictiveCohort,
    event_family: str,
    case_id: int,
    decision_contract: str,
    *,
    district: str,
    street_pattern: str,
) -> dict[str, Any]:
    return {
        "study": "predictive-space-time-v0.5",
        "dataset": "predictive-urban-v0.5",
        "cohort": cohort,
        "event_family": event_family,
        "case_id": case_id,
        "district": district,
        "street_pattern": street_pattern,
        "forecast_required": True,
        "decision_contract": decision_contract,
        "selection_basis": "validated-scenario-construction-only",
        "planner_outcome_filtering": "forbidden",
    }


def _wait_then_straight() -> DynamicScenario:
    district = "controlled-crossing district"
    street_pattern = "single signalized east-west passage"
    scene = make_city_scene(
        "wait-then-straight-static",
        district=district,
        street_pattern=street_pattern,
        start=(8.0, 44.0, 10.0),
        goal=(96.0, 44.0, 10.0),
        footprints=CALIBRATION_BLOCKS,
        heights=(56, 56, 24, 36, 32, 20, 40, 28),
    )
    return DynamicScenario(
        "wait-then-straight",
        "Wait, then fly straight",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("calibration-opening-gate", (52.0, 44.0), 7.0, 0.0, 58.0, 0.0, 8.0),
        ),
        metadata=_metadata(
            "calibration",
            "wait-then-straight",
            5101,
            "The central passage opens after the direct flight would reach it, so waiting at a "
            "safe location is preferable to entering the closed gate.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


def _closing_window() -> DynamicScenario:
    district = "financial grid"
    street_pattern = "orthogonal blocks with a central diagonal crossing"
    scene = make_city_scene(
        "closing-window-static",
        district=district,
        street_pattern=street_pattern,
        start=(4.0, 8.0, 8.0),
        goal=(100.0, 80.0, 16.0),
        footprints=ORTHOGONAL_GRID,
        heights=(24, 40, 20, 48, 32, 16, 44, 28, 36, 52, 24, 40, 20, 32, 48, 28),
        static_zones=(Cylinder("closing-hospital-nfz", (28.0, 68.0), 5.0, 0.0, 52.0),),
    )
    return DynamicScenario(
        "closing-window",
        "Closing downtown window",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("closing-central-window", (54.0, 44.0), 6.0, 0.0, 56.0, 4.0, 28.0),
        ),
        moving_spheres=(
            MovingSphere(
                "closing-cross-traffic",
                3.0,
                (
                    (0.0, (76.0, 4.0, 18.0)),
                    (5.0, (76.0, 44.0, 18.0)),
                    (10.0, (76.0, 84.0, 18.0)),
                    (18.0, (100.0, 4.0, 42.0)),
                ),
            ),
        ),
        metadata=_metadata(
            "diagnostic",
            "closing-window",
            5102,
            "A diagonal downtown crossing closes before arrival while cross traffic occupies a "
            "secondary avenue, requiring an early route commitment.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


def _periodic_traffic() -> DynamicScenario:
    district = "market quarter"
    street_pattern = "staggered arcades and alternating service lanes"
    scene = make_city_scene(
        "periodic-traffic-static",
        district=district,
        street_pattern=street_pattern,
        start=(4.0, 76.0, 12.0),
        goal=(100.0, 12.0, 20.0),
        footprints=STAGGERED_MARKET,
        heights=(20, 28, 36, 24, 44, 16, 32, 48, 24, 40, 20, 36, 52, 28, 44, 32),
        static_zones=(Cylinder("market-event-nfz", (52.0, 44.0), 5.0, 0.0, 48.0),),
    )
    return DynamicScenario(
        "periodic-traffic",
        "Periodic market traffic",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("market-loading-pulse", (76.0, 44.0), 5.0, 0.0, 40.0, 9.0, 16.0),
        ),
        moving_spheres=(
            MovingSphere(
                "periodic-market-crossing",
                4.0,
                (
                    (0.0, (28.0, 4.0, 16.0)),
                    (3.0, (28.0, 44.0, 16.0)),
                    (6.0, (28.0, 84.0, 16.0)),
                    (9.0, (28.0, 44.0, 16.0)),
                    (12.0, (28.0, 4.0, 16.0)),
                    (15.0, (28.0, 44.0, 16.0)),
                    (18.0, (28.0, 84.0, 16.0)),
                    (24.0, (100.0, 84.0, 48.0)),
                ),
            ),
        ),
        metadata=_metadata(
            "demo",
            "periodic-traffic",
            5103,
            "Repeated market crossings and a scheduled loading-zone pulse create several safe "
            "entry slots through a staggered city grid.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


def _chained_restrictions() -> DynamicScenario:
    district = "civic campus"
    street_pattern = "paired courtyards around a broad central boulevard"
    scene = make_city_scene(
        "chained-restrictions-static",
        district=district,
        street_pattern=street_pattern,
        start=(4.0, 20.0, 10.0),
        goal=(100.0, 68.0, 22.0),
        footprints=CIVIC_COURTYARDS,
        heights=(16, 32, 40, 24, 28, 44, 20, 36, 48, 24, 32, 52, 20, 40, 28, 44),
        static_zones=(Cylinder("civic-core-nfz", (52.0, 44.0), 6.0, 0.0, 52.0),),
    )
    return DynamicScenario(
        "chained-restrictions",
        "Chained civic restrictions",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("civic-west-release", (28.0, 44.0), 5.0, 0.0, 44.0, 0.0, 8.0),
            TemporaryCylinder("civic-east-release", (76.0, 44.0), 5.0, 0.0, 48.0, 6.0, 16.0),
        ),
        metadata=_metadata(
            "diagnostic",
            "chained-temporary-zones",
            5104,
            "The western restriction releases as the eastern restriction activates, so clearing "
            "the first courtyard greedily can lead into the second closure.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


def _multi_obstacle() -> DynamicScenario:
    district = "intermodal district"
    street_pattern = "offset transit boulevard with four cross streets"
    scene = make_city_scene(
        "multi-obstacle-static",
        district=district,
        street_pattern=street_pattern,
        start=(4.0, 44.0, 8.0),
        goal=(100.0, 68.0, 20.0),
        footprints=TRANSIT_BOULEVARD,
        heights=(20, 36, 28, 44, 32, 16, 48, 24, 40, 28, 52, 20, 24, 44, 32, 36),
        static_zones=(Cylinder("transit-terminal-nfz", (52.0, 68.0), 5.0, 0.0, 50.0),),
    )
    return DynamicScenario(
        "multi-obstacle",
        "Coordinated transit obstacles",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("transit-central-pulse", (52.0, 44.0), 5.0, 0.0, 44.0, 5.0, 13.0),
        ),
        moving_spheres=(
            MovingSphere(
                "transit-west-crossing",
                3.5,
                (
                    (0.0, (28.0, 12.0, 12.0)),
                    (4.0, (28.0, 44.0, 12.0)),
                    (8.0, (28.0, 76.0, 12.0)),
                    (18.0, (4.0, 84.0, 48.0)),
                ),
            ),
            MovingSphere(
                "transit-east-crossing",
                3.5,
                (
                    (2.0, (76.0, 76.0, 16.0)),
                    (6.0, (76.0, 44.0, 16.0)),
                    (10.0, (76.0, 12.0, 16.0)),
                    (18.0, (100.0, 4.0, 48.0)),
                ),
            ),
        ),
        metadata=_metadata(
            "diagnostic",
            "mixed-multiple-obstacles",
            5105,
            "Two opposing transit crossings and a central timed restriction require coordinated "
            "route timing through offset streets.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


def _vertical_time_window() -> DynamicScenario:
    district = "terraced high-rise district"
    street_pattern = "stepped towers around a diagonal vertical corridor"
    scene = make_city_scene(
        "vertical-time-window-static",
        district=district,
        street_pattern=street_pattern,
        start=(4.0, 20.0, 6.0),
        goal=(100.0, 68.0, 30.0),
        footprints=TERRACED_HEIGHTS,
        heights=(16, 28, 40, 52, 24, 36, 48, 20, 44, 24, 56, 32, 20, 40, 28, 48),
        static_zones=(Cylinder("terrace-west-nfz", (28.0, 44.0), 5.0, 0.0, 52.0),),
    )
    return DynamicScenario(
        "vertical-time-window",
        "Vertical downtown time window",
        scene,
        temporary_cylinders=(
            TemporaryCylinder(
                "terrace-upper-closed-first", (52.0, 44.0), 6.0, 20.0, 58.0, 0.0, 5.0
            ),
            TemporaryCylinder(
                "terrace-lower-closed-later", (52.0, 44.0), 6.0, 0.0, 20.0, 5.0, 22.0
            ),
        ),
        metadata=_metadata(
            "diagnostic",
            "vertical-time-window",
            5106,
            "The low diagonal corridor closes as its upper layer opens, requiring an early climb "
            "among unequal-height towers.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


def _urban_canyon_merge() -> DynamicScenario:
    district = "merged canyon district"
    street_pattern = "five north-south canyons joined by two cross avenues"
    scene = make_city_scene(
        "urban-canyon-merge-static",
        district=district,
        street_pattern=street_pattern,
        start=(8.0, 30.0, 10.0),
        goal=(100.0, 66.0, 18.0),
        footprints=MERGING_CANYONS,
        heights=(40, 28, 48, 32, 52, 24, 44, 36, 20, 48, 28, 56, 32, 44, 24),
        static_zones=(Cylinder("canyon-public-safety-nfz", (36.0, 66.0), 5.0, 0.0, 52.0),),
    )
    return DynamicScenario(
        "urban-canyon-merge",
        "Urban canyon merge",
        scene,
        temporary_cylinders=(
            TemporaryCylinder(
                "canyon-east-avenue-closure", (76.0, 30.0), 5.0, 0.0, 48.0, 8.0, 18.0
            ),
        ),
        moving_spheres=(
            MovingSphere(
                "canyon-southbound-traffic",
                3.5,
                (
                    (0.0, (56.0, 4.0, 14.0)),
                    (6.0, (56.0, 48.0, 14.0)),
                    (12.0, (56.0, 84.0, 14.0)),
                    (20.0, (100.0, 84.0, 48.0)),
                ),
            ),
            MovingSphere(
                "canyon-westbound-traffic",
                3.5,
                (
                    (0.0, (100.0, 48.0, 22.0)),
                    (8.0, (56.0, 48.0, 22.0)),
                    (14.0, (16.0, 48.0, 22.0)),
                    (22.0, (4.0, 84.0, 50.0)),
                ),
            ),
        ),
        metadata=_metadata(
            "demo",
            "urban-canyon-merge",
            5107,
            "Traffic streams merge between tall canyon walls while an eastern cross avenue "
            "closes, exposing both horizontal and altitude choices.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


def _rooftop_transfer() -> DynamicScenario:
    district = "rooftop logistics zone"
    street_pattern = "tower grid with a low-altitude exclusion core"
    scene = make_city_scene(
        "rooftop-transfer-static",
        district=district,
        street_pattern=street_pattern,
        start=(18.0, 10.0, 32.0),
        goal=(90.0, 82.0, 52.0),
        footprints=ROOFTOP_TOWERS,
        heights=(24, 40, 28, 48, 32, 20, 44, 36, 52, 28, 48, 24, 36, 44, 32, 44),
        static_zones=(Cylinder("rooftop-low-core-nfz", (52.0, 44.0), 7.0, 0.0, 32.0),),
    )
    return DynamicScenario(
        "rooftop-transfer",
        "Rooftop logistics transfer",
        scene,
        temporary_cylinders=(
            TemporaryCylinder(
                "rooftop-lower-layer-closure", (76.0, 68.0), 6.0, 0.0, 36.0, 4.0, 20.0
            ),
        ),
        moving_spheres=(
            MovingSphere(
                "rooftop-crane-load",
                3.0,
                (
                    (0.0, (28.0, 4.0, 40.0)),
                    (6.0, (52.0, 44.0, 40.0)),
                    (12.0, (76.0, 68.0, 40.0)),
                    (22.0, (100.0, 4.0, 56.0)),
                ),
            ),
        ),
        metadata=_metadata(
            "demo",
            "rooftop-transfer",
            5108,
            "A high-altitude transfer must clear unequal rooftops, a low exclusion core, a "
            "scheduled lower-layer closure, and a moving crane load.",
            district=district,
            street_pattern=street_pattern,
        ),
    )


_SCENARIO_SPECS: tuple[_ScenarioSpec, ...] = (
    _ScenarioSpec(
        "wait-then-straight",
        "calibration",
        "wait-then-straight",
        5101,
        _wait_then_straight,
    ),
    _ScenarioSpec("closing-window", "diagnostic", "closing-window", 5102, _closing_window),
    _ScenarioSpec("periodic-traffic", "demo", "periodic-traffic", 5103, _periodic_traffic),
    _ScenarioSpec(
        "chained-restrictions",
        "diagnostic",
        "chained-temporary-zones",
        5104,
        _chained_restrictions,
    ),
    _ScenarioSpec(
        "multi-obstacle", "diagnostic", "mixed-multiple-obstacles", 5105, _multi_obstacle
    ),
    _ScenarioSpec(
        "vertical-time-window",
        "diagnostic",
        "vertical-time-window",
        5106,
        _vertical_time_window,
    ),
    _ScenarioSpec("urban-canyon-merge", "demo", "urban-canyon-merge", 5107, _urban_canyon_merge),
    _ScenarioSpec("rooftop-transfer", "demo", "rooftop-transfer", 5108, _rooftop_transfer),
)


def list_predictive_scenarios(cohort: str | None = None) -> tuple[str, ...]:
    """List scenario IDs in stable registry order, optionally restricted by cohort."""

    if cohort is not None and cohort not in PREDICTIVE_COHORTS:
        choices = ", ".join(PREDICTIVE_COHORTS)
        raise ValueError(f"unknown predictive cohort {cohort!r}; choose one of: {choices}")
    return tuple(
        spec.scenario_id for spec in _SCENARIO_SPECS if cohort is None or spec.cohort == cohort
    )


def load_predictive_scenario(scenario_id: str) -> DynamicScenario:
    """Build one deterministic predictive scenario by ID."""

    for spec in _SCENARIO_SPECS:
        if spec.scenario_id == scenario_id:
            scenario = spec.builder()
            _validate_scenario_contract(spec, scenario)
            return scenario
    choices = ", ".join(list_predictive_scenarios())
    raise ValueError(f"unknown predictive scenario {scenario_id!r}; choose one of: {choices}")


def _validate_scenario_contract(spec: _ScenarioSpec, scenario: DynamicScenario) -> None:
    if scenario.scenario_id != spec.scenario_id:
        raise ValueError(f"predictive scenario ID mismatch for {spec.scenario_id}")
    metadata = scenario.metadata
    expected = {
        "study": "predictive-space-time-v0.5",
        "dataset": "predictive-urban-v0.5",
        "cohort": spec.cohort,
        "event_family": spec.event_family,
        "case_id": spec.case_id,
    }
    if any(metadata.get(key) != value for key, value in expected.items()):
        raise ValueError(f"predictive metadata mismatch for {spec.scenario_id}")
    if metadata.get("forecast_required") is not True:
        raise ValueError(f"predictive forecast contract missing for {spec.scenario_id}")
    if not metadata.get("district") or not metadata.get("street_pattern"):
        raise ValueError(f"predictive urban metadata missing for {spec.scenario_id}")
    hazard_count = len(scenario.temporary_cylinders) + len(scenario.moving_spheres)
    if spec.cohort == "calibration":
        if len(scenario.static_scene.buildings) < 8:
            raise ValueError("the calibration map requires at least eight buildings")
    elif (
        len(scenario.static_scene.buildings) < 14
        or not scenario.static_scene.no_fly_zones
        or hazard_count < 2
    ):
        raise ValueError(f"predictive urban complexity contract failed for {spec.scenario_id}")
    if not point_is_free_at_time(scenario, scenario.static_scene.start, 0.0):
        raise ValueError(f"predictive start is occupied at time zero for {spec.scenario_id}")
    if not point_is_free_at_time(scenario, scenario.static_scene.goal, 0.0):
        raise ValueError(f"predictive goal is occupied at time zero for {spec.scenario_id}")


def build_predictive_cohort() -> tuple[tuple[DynamicScenario, ...], dict[str, object]]:
    """Build the fixed cohort and its planner-outcome-independent acceptance manifest."""

    accepted_scenarios: list[DynamicScenario] = []
    records: list[dict[str, object]] = []
    cohort_counts = {cohort: 0 for cohort in PREDICTIVE_COHORTS}
    rejected = 0
    for spec in _SCENARIO_SPECS:
        try:
            scenario = spec.builder()
            _validate_scenario_contract(spec, scenario)
        except (RuntimeError, ValueError) as error:
            rejected += 1
            records.append(
                {
                    "scenario_id": spec.scenario_id,
                    "cohort": spec.cohort,
                    "event_family": spec.event_family,
                    "case_id": spec.case_id,
                    "status": "rejected",
                    "rejection_reason": f"{type(error).__name__}: {error}",
                }
            )
            continue
        fingerprint = dynamic_scenario_fingerprint(scenario)
        accepted_scenarios.append(scenario)
        cohort_counts[spec.cohort] += 1
        records.append(
            {
                "scenario_id": spec.scenario_id,
                "cohort": spec.cohort,
                "event_family": spec.event_family,
                "case_id": spec.case_id,
                "status": "accepted",
                "fingerprint": fingerprint,
            }
        )

    manifest: dict[str, object] = {
        "schema_version": "predictive-scenario-manifest-v2",
        "dataset_id": "predictive-urban-v0.5",
        "selection": {
            "acceptance_rule": (
                "valid constructor output satisfying the v0.5 urban scenario contract"
            ),
            "planner_outcomes_consulted": False,
            "cohort_assignment_basis": "curated non-confirmatory roles in this source revision",
            "preregistered": False,
        },
        "requested": len(_SCENARIO_SPECS),
        "accepted": len(accepted_scenarios),
        "rejected": rejected,
        "accepted_by_cohort": cohort_counts,
        "records": records,
    }
    return tuple(accepted_scenarios), manifest


__all__ = [
    "PREDICTIVE_COHORTS",
    "PredictiveCohort",
    "build_predictive_cohort",
    "list_predictive_scenarios",
    "load_predictive_scenario",
]
