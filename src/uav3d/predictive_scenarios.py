"""Deterministic v0.4 scenarios for predictive space-time planning studies.

The registry in this module is deliberately independent of every planner and
simulation outcome.  A scenario is accepted when its constructor satisfies the
declared data contract; no algorithm is run while assembling the cohort.
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
from uav3d.geometry import Point3
from uav3d.scene import AABB, Bounds3D, Scene

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
) -> dict[str, Any]:
    return {
        "study": "predictive-space-time-v0.4",
        "cohort": cohort,
        "event_family": event_family,
        "case_id": case_id,
        "forecast_required": True,
        "decision_contract": decision_contract,
        "selection_basis": "validated-scenario-construction-only",
        "planner_outcome_filtering": "forbidden",
    }


def _scene(
    scene_id: str,
    *,
    buildings: tuple[AABB, ...] = (),
    start: Point3 = (6.0, 24.0, 10.0),
    goal: Point3 = (66.0, 24.0, 10.0),
) -> Scene:
    return Scene(
        scene_id=scene_id,
        name=scene_id.replace("-", " ").title(),
        bounds=Bounds3D((0.0, 0.0, 0.0), (72.0, 48.0, 36.0)),
        start=start,
        goal=goal,
        buildings=buildings,
        drone_radius=0.5,
        safety_margin=0.5,
        metadata={"family": "predictive-controlled-v0.4"},
    )


def _gate_buildings(prefix: str) -> tuple[AABB, ...]:
    """Return a full-height wall with one narrow central passage."""

    return (
        AABB(f"{prefix}-south", (31.0, 0.0, 0.0), (41.0, 15.0, 34.0)),
        AABB(f"{prefix}-north", (31.0, 33.0, 0.0), (41.0, 48.0, 34.0)),
    )


def _wait_then_straight() -> DynamicScenario:
    return DynamicScenario(
        "wait-then-straight",
        "Wait, then fly straight",
        _scene("wait-then-straight-static", buildings=_gate_buildings("wait-gate")),
        temporary_cylinders=(
            TemporaryCylinder("opening-gate", (36.0, 24.0), 7.0, 0.0, 34.0, 0.0, 5.5),
        ),
        metadata=_metadata(
            "calibration",
            "wait-then-straight",
            4101,
            "The only passage is initially closed; forecasting its release avoids a futile detour.",
        ),
    )


def _closing_window() -> DynamicScenario:
    return DynamicScenario(
        "closing-window",
        "Closing direct-route window",
        _scene("closing-window-static"),
        temporary_cylinders=(
            TemporaryCylinder("closing-window-zone", (36.0, 24.0), 8.0, 0.0, 34.0, 3.0, 24.0),
        ),
        metadata=_metadata(
            "diagnostic",
            "closing-window",
            4102,
            "The direct route closes before arrival, so a detour must be selected "
            "before the event.",
        ),
    )


def _periodic_traffic() -> DynamicScenario:
    return DynamicScenario(
        "periodic-traffic",
        "Periodic gate traffic",
        _scene("periodic-traffic-static", buildings=_gate_buildings("traffic-gate")),
        moving_spheres=(
            MovingSphere(
                "periodic-crossing",
                7.0,
                (
                    (0.0, (36.0, 10.0, 10.0)),
                    (2.5, (36.0, 24.0, 10.0)),
                    (5.0, (36.0, 38.0, 10.0)),
                    (7.5, (36.0, 24.0, 10.0)),
                    (10.0, (36.0, 10.0, 10.0)),
                    (12.5, (36.0, 24.0, 10.0)),
                    (15.0, (36.0, 38.0, 10.0)),
                ),
            ),
        ),
        metadata=_metadata(
            "demo",
            "periodic-traffic",
            4103,
            "Repeated crossings create several safe entry slots that a time-aware "
            "planner can target.",
        ),
    )


def _chained_restrictions() -> DynamicScenario:
    return DynamicScenario(
        "chained-restrictions",
        "Chained temporary restrictions",
        _scene("chained-restrictions-static"),
        temporary_cylinders=(
            TemporaryCylinder("first-release", (25.0, 20.0), 6.0, 0.0, 30.0, 0.0, 6.0),
            TemporaryCylinder("second-release", (47.0, 28.0), 6.0, 0.0, 30.0, 5.0, 12.0),
        ),
        metadata=_metadata(
            "diagnostic",
            "chained-temporary-zones",
            4104,
            "Clearing the first restriction greedily can lead directly into the "
            "second restriction.",
        ),
    )


def _multi_obstacle() -> DynamicScenario:
    return DynamicScenario(
        "multi-obstacle",
        "Coordinated moving obstacles",
        _scene("multi-obstacle-static"),
        temporary_cylinders=(
            TemporaryCylinder("central-pulse", (36.0, 24.0), 5.0, 0.0, 28.0, 4.0, 10.0),
        ),
        moving_spheres=(
            MovingSphere(
                "westbound-crossing",
                4.0,
                (
                    (0.0, (24.0, 8.0, 10.0)),
                    (3.0, (24.0, 24.0, 10.0)),
                    (6.0, (24.0, 40.0, 10.0)),
                ),
            ),
            MovingSphere(
                "eastbound-crossing",
                4.0,
                (
                    (2.0, (48.0, 40.0, 10.0)),
                    (5.0, (48.0, 24.0, 10.0)),
                    (8.0, (48.0, 8.0, 10.0)),
                ),
            ),
        ),
        metadata=_metadata(
            "diagnostic",
            "mixed-multiple-obstacles",
            4105,
            "Two crossings and a timed central restriction require coordinated route timing.",
        ),
    )


def _vertical_time_window() -> DynamicScenario:
    return DynamicScenario(
        "vertical-time-window",
        "Vertical time window",
        _scene("vertical-time-window-static", buildings=_gate_buildings("vertical-gate")),
        temporary_cylinders=(
            TemporaryCylinder("upper-closed-first", (36.0, 24.0), 7.0, 18.0, 34.0, 0.0, 2.5),
            TemporaryCylinder("lower-closed-later", (36.0, 24.0), 7.0, 0.0, 18.0, 2.5, 18.0),
        ),
        metadata=_metadata(
            "diagnostic",
            "vertical-time-window",
            4106,
            "The low corridor closes before arrival while the upper corridor opens, "
            "requiring an early climb.",
        ),
    )


_SCENARIO_SPECS: tuple[_ScenarioSpec, ...] = (
    _ScenarioSpec(
        "wait-then-straight", "calibration", "wait-then-straight", 4101, _wait_then_straight
    ),
    _ScenarioSpec("closing-window", "diagnostic", "closing-window", 4102, _closing_window),
    _ScenarioSpec("periodic-traffic", "demo", "periodic-traffic", 4103, _periodic_traffic),
    _ScenarioSpec(
        "chained-restrictions",
        "diagnostic",
        "chained-temporary-zones",
        4104,
        _chained_restrictions,
    ),
    _ScenarioSpec(
        "multi-obstacle", "diagnostic", "mixed-multiple-obstacles", 4105, _multi_obstacle
    ),
    _ScenarioSpec(
        "vertical-time-window", "diagnostic", "vertical-time-window", 4106, _vertical_time_window
    ),
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
    if scenario.metadata.get("cohort") != spec.cohort:
        raise ValueError(f"predictive cohort mismatch for {spec.scenario_id}")
    if scenario.metadata.get("event_family") != spec.event_family:
        raise ValueError(f"predictive event-family mismatch for {spec.scenario_id}")
    if scenario.metadata.get("case_id") != spec.case_id:
        raise ValueError(f"predictive case ID mismatch for {spec.scenario_id}")
    if scenario.metadata.get("forecast_required") is not True:
        raise ValueError(f"predictive forecast contract missing for {spec.scenario_id}")
    if not scenario.temporary_cylinders and not scenario.moving_spheres:
        raise ValueError(f"predictive scenario has no scheduled obstacle: {spec.scenario_id}")


def build_predictive_cohort() -> tuple[tuple[DynamicScenario, ...], dict[str, object]]:
    """Build the fixed cohort and an outcome-independent acceptance manifest."""

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
        "schema_version": "predictive-scenario-manifest-v1",
        "dataset_id": "predictive-controlled-v0.4",
        "selection": {
            "acceptance_rule": (
                "valid constructor output satisfying the predictive scenario contract"
            ),
            "planner_outcomes_consulted": False,
            "cohort_assignment_basis": "curated diagnostic labels in this source revision",
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
