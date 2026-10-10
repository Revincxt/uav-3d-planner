"""Computed urban missions over official Manhattan building geometry.

This demo has its own protocol. The frozen v0.7 research registry and runtime remain
unchanged. Aircraft traffic and airspace schedules are explicitly simulated mission
conditions; the building footprints and available heights come from NYC Open Data.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import sys
import time
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING, Any, cast

from uav3d.collision import point_is_free
from uav3d.dynamic import (
    DynamicScenario,
    MovingSphere,
    TemporaryCylinder,
    dynamic_scenario_fingerprint,
)
from uav3d.geometry import Point3, almost_equal
from uav3d.kinematics import DiscreteExecutionEnvelope, qualify_timed_path_execution
from uav3d.manhattan_challenges import select_encounter, static_reference
from uav3d.manhattan_missions import (
    audit_task_visits,
    district_mission_evidence,
    mission_task_points,
    resolve_rooftop_anchor,
    simulate_mission_replanning,
)
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.shared_airspace import share_dynamic_airspace

if TYPE_CHECKING:
    from uav3d.manhattan_city import ManhattanCity
    from uav3d.predictive_study import PredictiveEpisode, PredictiveEpisodeMetrics

PROTOCOL_ID = "manhattan-space-time-v3"
DATASET_ID = "manhattan-urban-missions-v3"
ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True, slots=True)
class Mission:
    mission_id: str
    name: str
    origin: str
    destination: str
    purpose: str
    start_lon_lat: tuple[float, float]
    goal_lon_lat: tuple[float, float]
    preferred_altitude_m: float
    route: str = "west-delivery"


MISSIONS = (
    Mission(
        "manhattan-westside-delivery",
        "South Chelsea → North Hell's Kitchen",
        "South Chelsea dispatch roof",
        "North Hell's Kitchen receiving roof",
        "Simulated time-sensitive delivery along the western corridor",
        (-74.0062, 40.7387),
        (-73.9924, 40.7656),
        90.0,
    ),
    Mission(
        "manhattan-medical-transfer",
        "NYU medical district → Columbus Circle",
        "NYU medical district dispatch roof",
        "Columbus Circle simulated relay roof",
        "Simulated long-distance cold-chain medical specimen transfer",
        (-73.9741, 40.7424),
        (-73.9847, 40.7665),
        110.0,
        "medical-north",
    ),
    Mission(
        "manhattan-midtown-rooftops",
        "Flatiron → Midtown East",
        "South Flatiron equipment dispatch roof",
        "Midtown East receiving roof",
        "Simulated cross-district rooftop equipment delivery",
        (-73.9921, 40.7385),
        (-73.9697, 40.7595),
        90.0,
        "east-logistics",
    ),
    Mission(
        "manhattan-riverfront-logistics",
        "Chelsea Piers → East Midtown",
        "Chelsea Piers riverfront dispatch roof",
        "East Midtown logistics roof",
        "Simulated west-to-east logistics across scheduled airspace and traffic",
        (-74.0073, 40.7448),
        (-73.9728, 40.7560),
        100.0,
        "riverfront-logistics",
    ),
    Mission(
        "manhattan-westside-backhaul",
        "Central Park South → Chelsea depot",
        "Central Park South return-parcel roof",
        "Chelsea interior consolidation roof",
        "Simulated southbound return-parcel backhaul through shared scheduled traffic",
        (-73.9815, 40.7658),
        (-74.0040, 40.7389),
        90.0,
        "west-backhaul",
    ),
    Mission(
        "manhattan-medical-return",
        "Plaza district → Bellevue district",
        "Plaza district medical dispatch roof",
        "Bellevue district medical receiving roof",
        "Simulated hospital-network return of temperature-controlled equipment",
        (-73.9752, 40.7661),
        (-73.9722, 40.7417),
        110.0,
        "medical-south",
    ),
    Mission(
        "manhattan-eastside-backhaul",
        "Rockefeller district → Chelsea east depot",
        "Rockefeller district collection roof",
        "Chelsea east supply consolidation roof",
        "Simulated reusable supply collection along Fifth Avenue and Broadway",
        (-73.9771, 40.7601),
        (-73.9961, 40.7385),
        90.0,
        "east-backhaul",
    ),
    Mission(
        "manhattan-riverfront-return",
        "Sutton district → West Chelsea",
        "Sutton district priority dispatch roof",
        "West Chelsea priority receiving roof",
        "Simulated westbound spare-parts transfer through shared reserved airspace",
        (-73.9718, 40.7614),
        (-74.0063, 40.7470),
        100.0,
        "riverfront-backhaul",
    ),
)


def study_runtime() -> ModuleType:
    """Load an isolated copy of the evidence exporter with declared city parameters.

    Reusing the existing audited measurement and serialization code avoids inventing
    metrics. Its module globals are isolated so the original study remains frozen.
    """
    name = "uav3d._manhattan_predictive_runtime"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(
        name, Path(__file__).with_name("predictive_study.py")
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("predictive evidence exporter could not be loaded")
    runtime = importlib.util.module_from_spec(spec)
    sys.modules[name] = runtime
    spec.loader.exec_module(runtime)
    # Configure the isolated module's actual globals, not a detached namespace copy.
    runtime.__dict__.update(
        PROTOCOL_ID=PROTOCOL_ID,
        TIME_STEP_S=2.0,
        REPLAN_INTERVAL_S=20.0,
        CRUISE_SPEED_MPS=15.0,
        MAX_TIME_S=900.0,
        RESOLUTION_M=60.0,
        TIME_RESOLUTION_S=2.0,
        PLANNING_HORIZON_S=900.0,
        PREDICTION_HORIZON_S=900.0,
        MAX_EXPANDED_STATES=240_000,
        SMOOTHING_TURN_RADIUS_M=60.0,
        SMOOTHING_SAMPLE_SPACING_M=2.0,
        SPACE_TIME_CONNECTIVITY=26,
        TRAJECTORY_SHORTCUT=True,
        TRAJECTORY_PRESERVE_ALTITUDE=True,
        TRAJECTORY_LOCAL_CURVES=True,
        TRAJECTORY_SCHEDULE_DYNAMIC_WAITS=True,
        TRAJECTORY_POSTPROCESSOR=(
            "horizontal-local-quintic-bspline-altitude-preserving-envelope-v5"
        ),
        EXECUTION_ENVELOPE=DiscreteExecutionEnvelope(
            max_speed_mps=15.0,
            max_abs_climb_rate_mps=3.0,
            max_discrete_acceleration_proxy_mps2=4.0,
            reversal_threshold_deg=150.0,
            allow_reversals=False,
            max_execution_time_s=900.0,
        ),
    )
    original_protocol = runtime._protocol

    def city_protocol() -> dict[str, Any]:
        protocol = cast(dict[str, Any], original_protocol())
        protocol.update(
            {
                "datasetId": DATASET_ID,
                "hazardSource": "simulated-operational-conditions",
                "collisionModel": "conservative-aabb",
                "spaceTimeConnectivity": runtime.SPACE_TIME_CONNECTIVITY,
                "trajectoryShortcut": runtime.TRAJECTORY_SHORTCUT,
                "trajectoryPreserveAltitude": runtime.TRAJECTORY_PRESERVE_ALTITUDE,
                "trajectoryCurveDegree": 5,
                "reactivePlanningGuardS": runtime.REPLAN_INTERVAL_S,
                "reactiveHorizontalEscape": True,
                "trajectoryDynamicScheduling": "certified-move-block-departures",
                "trajectoryBrakingPolicy": "qualified-hover-at-retained-sharp-height-knots",
            }
        )
        return protocol

    runtime.__dict__["_protocol"] = city_protocol
    return runtime


def source_provenance() -> dict[str, Any]:
    paths = sorted((ROOT / "src" / "uav3d").rglob("*.py"))
    paths.append(ROOT / "scripts" / "export_manhattan_predictive.py")
    records = [
        {
            "path": path.relative_to(ROOT).as_posix(),
            "sha256": "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        for path in paths
    ]
    digest = hashlib.sha256(
        json.dumps(records, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return {"kind": "local-snapshot", "sha256": "sha256:" + digest, "files": records}


def build_manhattan_missions(city: ManhattanCity) -> tuple[DynamicScenario, ...]:
    from uav3d.manhattan_city import make_city_scene

    protected_anchors = tuple(
        point
        for mission in MISSIONS
        for point in (
            resolve_rooftop_anchor(
                city, *mission.start_lon_lat, preferred_altitude_m=mission.preferred_altitude_m
            ).position,
            resolve_rooftop_anchor(
                city, *mission.goal_lon_lat, preferred_altitude_m=mission.preferred_altitude_m + 20
            ).position,
            *(
                tuple(t["position"])
                for t in mission_task_points(
                    city, mission.route, preferred_altitude_m=mission.preferred_altitude_m
                )
            ),
        )
    )
    scenarios: list[DynamicScenario] = []
    patrol_centres: list[Point3] = []
    for index, mission in enumerate(MISSIONS):
        start_anchor = resolve_rooftop_anchor(
            city, *mission.start_lon_lat, preferred_altitude_m=mission.preferred_altitude_m
        )
        goal_anchor = resolve_rooftop_anchor(
            city, *mission.goal_lon_lat, preferred_altitude_m=mission.preferred_altitude_m + 20.0
        )
        start, goal = start_anchor.position, goal_anchor.position
        scale = district_mission_evidence(city, start_anchor, goal_anchor)
        tasks = mission_task_points(
            city, mission.route, preferred_altitude_m=mission.preferred_altitude_m
        )
        scene = make_city_scene(
            city,
            mission.mission_id + "-static",
            name=mission.name,
            start=start,
            goal=goal,
            drone_radius=1.0,
            safety_margin=5.0,
            metadata={
                "dataset": DATASET_ID,
                "family": "manhattan-urban",
                "mission": mission.purpose,
                "missionTaskPoints": tasks,
            },
        )
        if not point_is_free(scene, start) or not point_is_free(scene, goal):
            raise ValueError(
                f"{mission.mission_id}: landmark endpoints must lie inside free city airspace"
            )
        if any(not point_is_free(scene, tuple(task["position"])) for task in tasks):
            raise ValueError(f"{mission.mission_id}: invalid service roof")
        reference = static_reference(scene, tasks, resolution=60)
        encounter = select_encounter(
            scene,
            reference,
            tasks,
            fraction=0.26,
            speed=15,
            protected_anchors=protected_anchors,
            continuous_patrol=index < 7,
            patrol_centres=tuple(patrol_centres),
        )
        traffic: tuple[MovingSphere, ...] = (
            (
                encounter.aircraft(
                    f"{mission.mission_id}-scheduled-cargo",
                    horizon=900,
                ),
            )
            if index < 7
            else ()
        )
        if traffic:
            patrol_centres.append(encounter.position)
        titles = (
            "Reserved flight corridor",
            "Recurring crossing windows",
            "Successive airspace windows",
            "Scheduled approach slot",
            "Southbound recurring cargo patrol",
            "Medical return through known traffic",
            "Eastside scheduled backhaul",
            "Shared reserved return corridor",
        )
        scheduled: list[TemporaryCylinder] = []
        if index in (0, 2, 3):
            fractions = (0.35, 0.68) if index == 2 else ((0.78,) if index == 3 else (0.42,))
            for number, fraction in enumerate(fractions):
                slot = select_encounter(
                    scene,
                    reference,
                    tasks,
                    fraction=fraction,
                    speed=15,
                    anchor_clearance_m=130,
                    protected_anchors=protected_anchors,
                )
                scheduled.append(
                    TemporaryCylinder(
                        f"{mission.mission_id}-reservation-{number + 1}",
                        slot.position[:2],
                        105,
                        0,
                        city.bounds.maximum[2],
                        max(0, slot.arrival_s - 22),
                        slot.arrival_s + 85,
                    )
                )
            primary = (
                slot
                if index in (0, 3)
                else select_encounter(
                    scene,
                    reference,
                    tasks,
                    fraction=0.35,
                    speed=15,
                    anchor_clearance_m=130,
                    protected_anchors=protected_anchors,
                )
            )
        else:
            primary = encounter
        scenarios.append(
            DynamicScenario(
                mission.mission_id,
                mission.name,
                scene,
                temporary_cylinders=tuple(scheduled),
                moving_spheres=traffic,
                metadata={
                    "dataset": DATASET_ID,
                    "cohort": "demo",
                    "district": "Manhattan, New York City",
                    "street_pattern": "Official NYC building footprints and real street grid",
                    "decision_contract": (
                        f"{mission.purpose} through timed operational airspace "
                        "and simulated crossing traffic."
                    ),
                    "mission": {
                        "origin": mission.origin,
                        "destination": mission.destination,
                        "purpose": mission.purpose,
                        "planningScale": scale,
                        "taskPoints": tasks,
                        "challenge": primary.challenge(titles[index], "predictive", tail_s=110),
                    },
                    "hazard_source": "simulated-operational-conditions",
                    "selection_basis": "demo-corridor-layout-and-validated-fly-through-feasibility",
                },
            )
        )
    return share_dynamic_airspace(tuple(scenarios), "manhattan-predictive-shared-v2")


def _audit_serialized_case(
    scenario: DynamicScenario, exported: dict[str, Any], envelope: DiscreteExecutionEnvelope
) -> None:
    """Fail closed on the actual rounded records, not only their in-memory certificates."""

    for run in exported["runs"]:
        for layer in ("rawTimedPath", "geometryTimedPath", "executionTimedPath"):
            records = run[layer]
            if not records:
                continue
            path = TimedPath(
                tuple(
                    TimedWaypoint(
                        record["timeS"],
                        tuple(record["position"]),
                        "start"
                        if index == 0
                        else (
                            "wait"
                            if almost_equal(records[index - 1]["position"], record["position"])
                            else "move"
                        ),
                    )
                    for index, record in enumerate(records)
                )
            )
            identity = f"{scenario.scenario_id}/{run['plannerId']}/{layer}"
            if not path.is_safe(scenario):
                raise RuntimeError(f"unsafe serialized trajectory from {identity}")
            audit_task_visits(
                path.positions,
                scenario.metadata.get("mission", {}).get("taskPoints", []),
                times=[waypoint.time_s for waypoint in path.waypoints],
            )
            if layer == "executionTimedPath":
                qualification = qualify_timed_path_execution(path, envelope)
                if not qualification.qualified:
                    raise RuntimeError(
                        f"unqualified serialized execution from {identity}: "
                        f"{qualification.violations}"
                    )


def run_multistop_episode(
    scenario: DynamicScenario, planner_id: str, runtime: ModuleType
) -> PredictiveEpisode:
    """Hard service stops partition search and smoothing without resetting hazard clocks."""
    from uav3d.dynamic_collision import spacetime_segment_is_free
    from uav3d.flight_cost import mission_altitude_levels, with_turn_clearance
    from uav3d.mission_refinement import ContinuousAnchorSpaceTimeAStar, refine_mission_trajectory
    from uav3d.planners.space_time_astar import SpaceTimeAStarConfig

    tasks = scenario.metadata.get("mission", {}).get("taskPoints", [])
    if not tasks:
        return cast("PredictiveEpisode", runtime.run_predictive_episode(scenario, planner_id))
    predictive = planner_id == "space-time-astar-4d"
    parameters = {
        "timeStepS": runtime.TIME_STEP_S,
        "cruiseSpeedMps": runtime.CRUISE_SPEED_MPS,
        "maxTimeS": runtime.MAX_TIME_S,
        "resolutionM": runtime.RESOLUTION_M,
        "trajectorySmoothing": 1,
        "smoothingTurnRadiusM": runtime.SMOOTHING_TURN_RADIUS_M,
        "smoothingSampleSpacingM": runtime.SMOOTHING_SAMPLE_SPACING_M,
        "trajectoryShortcut": 1,
        "trajectoryPreserveAltitude": 1,
        "trajectoryCurveDegree": 5,
        "taskPointCount": len(tasks),
        "legCount": len(tasks) + 1,
        "serviceDurationTotalS": sum(task["serviceDurationS"] for task in tasks),
        "verticalCostScale": 5,
        "maxClimbRateMps": runtime.EXECUTION_ENVELOPE.max_abs_climb_rate_mps,
    }
    if predictive:
        parameters.update(
            timeResolutionS=runtime.TIME_RESOLUTION_S,
            planningHorizonS=runtime.PLANNING_HORIZON_S,
            predictionHorizonS=runtime.PREDICTION_HORIZON_S,
            maxExpandedStatesPerMission=runtime.MAX_EXPANDED_STATES,
            spaceTimeConnectivity=runtime.SPACE_TIME_CONNECTIVITY,
        )
        waypoints = [TimedWaypoint(0.0, scenario.static_scene.start, "start")]
        expanded, replans = 0, 0
        for index, goal in enumerate(
            [tuple(task["position"]) for task in tasks] + [scenario.static_scene.goal]
        ):
            clock = waypoints[-1].time_s
            horizon = (
                math.floor((runtime.MAX_TIME_S - clock) / runtime.TIME_RESOLUTION_S)
                * runtime.TIME_RESOLUTION_S
            )
            leg = replace(
                scenario,
                static_scene=with_turn_clearance(
                    replace(
                        scenario.static_scene,
                        start=waypoints[-1].position,
                        goal=goal,
                    )
                ),
            )
            config = SpaceTimeAStarConfig(
                resolution=runtime.RESOLUTION_M,
                time_step=runtime.TIME_RESOLUTION_S,
                cruise_speed=runtime.CRUISE_SPEED_MPS,
                time_horizon=horizon,
                max_expansions=runtime.MAX_EXPANDED_STATES - expanded,
                connectivity=runtime.SPACE_TIME_CONNECTIVITY,
                max_climb_rate=runtime.EXECUTION_ENVELOPE.max_abs_climb_rate_mps,
                altitude_levels=mission_altitude_levels(
                    scenario.static_scene, runtime.RESOLUTION_M
                ),
            )
            result = ContinuousAnchorSpaceTimeAStar(config).plan(leg, start_time=clock)
            expanded += result.expanded_spacetime_states
            replans = index + 1
            if not result.success or result.timed_path is None:
                raise RuntimeError(
                    f"{scenario.scenario_id}/{planner_id}/leg-{index + 1}: {result.failure_reason}"
                )
            waypoints.extend(result.timed_path.waypoints[1:])
            if index < len(tasks) and tasks[index]["serviceDurationS"] > 0:
                departure = waypoints[-1].time_s + tasks[index]["serviceDurationS"]
                if departure >= runtime.MAX_TIME_S or not spacetime_segment_is_free(
                    scenario, goal, goal, waypoints[-1].time_s, departure
                ):
                    raise RuntimeError(f"No safe service window at {tasks[index]['id']}")
                waypoints.append(TimedWaypoint(departure, goal, "wait"))
        raw = TimedPath(tuple(waypoints))
    else:
        parameters["horizontalEscape"] = 1
        reuse = planner_id == "dstar-lite-reuse-3d"
        parameters.update(
            replanIntervalS=runtime.REPLAN_INTERVAL_S,
            maxWorkPerReplan=runtime.MAX_EXPANDED_STATES,
            reuseSearchState=int(reuse),
            planningGuardS=runtime.REPLAN_INTERVAL_S,
        )
        run = simulate_mission_replanning(
            scenario,
            "repeated-astar-3d" if planner_id == "repeated-astar-3d" else "dstar-lite-3d",
            tasks,
            time_step=runtime.TIME_STEP_S,
            replan_interval=runtime.REPLAN_INTERVAL_S,
            cruise_speed=runtime.CRUISE_SPEED_MPS,
            max_time=runtime.MAX_TIME_S,
            resolution=runtime.RESOLUTION_M,
            max_expansions=runtime.MAX_EXPANDED_STATES,
            reuse_search_state=reuse,
            planning_guard_s=runtime.REPLAN_INTERVAL_S,
            shortcut_paths=True,
            preserve_altitude=True,
            smooth_turns=True,
            turn_scale_m=runtime.SMOOTHING_TURN_RADIUS_M,
            curve_sample_spacing_m=runtime.SMOOTHING_SAMPLE_SPACING_M,
            allow_horizontal_escape=True,
            vertical_cost_scale=5,
            altitude_levels=mission_altitude_levels(scenario.static_scene, runtime.RESOLUTION_M),
            max_climb_rate=runtime.EXECUTION_ENVELOPE.max_abs_climb_rate_mps,
        )
        if not run.metrics.success:
            raise RuntimeError(f"{scenario.scenario_id}/{planner_id}: {run.metrics.failure_reason}")
        raw = runtime._reactive_timed_path(run)
        expanded, replans = run.metrics.total_planning_work, run.metrics.replans
    if not raw.is_safe(scenario):
        raise RuntimeError(f"Unsafe multistop raw path: {scenario.scenario_id}/{planner_id}")
    smoothing = refine_mission_trajectory(scenario, raw, runtime.EXECUTION_ENVELOPE)
    for path in (raw, smoothing.timed_path, smoothing.execution_candidate):
        if path is not None:
            audit_task_visits(
                path.positions, tasks, times=[point.time_s for point in path.waypoints]
            )

    def metrics(path: TimedPath, certified: bool) -> PredictiveEpisodeMetrics:
        return cast(
            "PredictiveEpisodeMetrics",
            runtime._path_metrics(
                scenario,
                path,
                planner_success=True,
                planner_failure_reason=None,
                path_safe=certified and path.is_safe(scenario),
                replans=replans,
                expanded_states=expanded,
                work_unit=runtime.WORK_UNITS[planner_id],
            ),
        )

    return cast(
        "PredictiveEpisode",
        runtime.PredictiveEpisode(
            scenario.scenario_id,
            dynamic_scenario_fingerprint(scenario),
            planner_id,
            predictive,
            parameters,
            raw,
            smoothing.timed_path,
            smoothing,
            metrics(raw, True),
            metrics(smoothing.timed_path, smoothing.certified),
            metrics(smoothing.execution_candidate, smoothing.execution_collision_certified)
            if smoothing.execution_candidate is not None
            else None,
        ),
    )


def compute_case(
    city: ManhattanCity, scenario: DynamicScenario, runtime: ModuleType
) -> dict[str, Any]:
    episodes = []
    for planner in runtime.PREDICTIVE_ALGORITHMS:
        started = time.perf_counter()
        episode = run_multistop_episode(scenario, planner, runtime)
        if not episode.smoothing.execution_qualified or episode.execution_timed_path is None:
            raise RuntimeError(
                f"unqualified execution from {scenario.scenario_id}/{planner}: "
                f"{episode.smoothing.execution_status}"
            )
        if not episode.raw_timed_path.is_safe(scenario) or not episode.timed_path.is_safe(scenario):
            raise RuntimeError(f"unsafe trajectory from {scenario.scenario_id}/{planner}")
        if episode.execution_timed_path is not None and not episode.execution_timed_path.is_safe(
            scenario
        ):
            raise RuntimeError(f"unsafe execution candidate from {scenario.scenario_id}/{planner}")
        episodes.append(episode)
        print(
            json.dumps(
                {
                    "scenario": scenario.scenario_id,
                    "planner": planner,
                    "seconds": round(time.perf_counter() - started, 3),
                    "status": episode.metrics.success,
                    "arrivalS": episode.metrics.arrival_time_s,
                    "pathLengthM": episode.metrics.executed_path_length_m,
                    "expandedStates": episode.metrics.expanded_states,
                    "safetyViolations": episode.metrics.safety_violations,
                    "geometryCertified": episode.smoothing.certified,
                    "executionQualified": episode.smoothing.execution_qualified,
                }
            ),
            flush=True,
        )
    exported = runtime._export_scenario(scenario, episodes)
    exported["buildings"] = city.web_buildings()
    exported["city"] = city.to_web_metadata()
    exported["mission"] = scenario.metadata["mission"]
    serialized = cast(dict[str, Any], runtime._normalize_record_numbers(exported))
    _audit_serialized_case(scenario, serialized, runtime.EXECUTION_ENVELOPE)
    return serialized


def export_manhattan_predictive(
    output_dir: Path,
    *,
    probe: bool = False,
    cache_dir: Path | None = None,
    city_path: Path | None = None,
) -> dict[str, Any] | None:
    from uav3d.manhattan_city import build_manhattan_city

    city = build_manhattan_city(city_path)
    runtime = study_runtime()
    provenance = source_provenance()
    source_id = "local-snapshot:" + provenance["sha256"]
    protocol = runtime._protocol()
    scenarios = build_manhattan_missions(city)
    exported_scenarios = []
    for scenario in scenarios[:1] if probe else scenarios:
        identity = hashlib.sha256(
            json.dumps(
                {
                    "fingerprint": dynamic_scenario_fingerprint(scenario),
                    "protocol": protocol,
                    "source": source_id,
                },
                sort_keys=True,
            ).encode()
        ).hexdigest()
        cache_path = cache_dir / f"{identity}.json" if cache_dir else None
        if cache_path and cache_path.is_file():
            exported = json.loads(cache_path.read_text(encoding="utf-8"))
            _audit_serialized_case(scenario, exported, runtime.EXECUTION_ENVELOPE)
            print(f"Reused computed {scenario.scenario_id}", flush=True)
        else:
            exported = compute_case(city, scenario, runtime)
            if cache_path:
                cache_path.parent.mkdir(parents=True, exist_ok=True)
                runtime._write_json(cache_path, exported)
        exported_scenarios.append(exported)
    if probe:
        return None
    timestamp = datetime.now(UTC).replace(microsecond=0).isoformat()
    bundle = {
        "schemaVersion": 3,
        "generatedAt": timestamp,
        "sourceCommit": source_id,
        "sourceProvenance": provenance,
        "verificationStatus": runtime.VERIFICATION_STATUS,
        "protocol": protocol,
        "planners": [
            {
                "id": planner,
                "label": runtime.PLANNER_LABELS[planner],
                "predictive": runtime.PREDICTIVE_FLAGS[planner],
            }
            for planner in runtime.PREDICTIVE_ALGORITHMS
        ],
        "scenarios": exported_scenarios,
    }
    manifest = {
        "schemaVersion": 3,
        "protocolId": PROTOCOL_ID,
        "datasetId": DATASET_ID,
        "generatedAt": timestamp,
        "sourceCommit": source_id,
        "sourceProvenance": provenance,
        "city": city.to_web_metadata(),
        "requested": len(scenarios),
        "accepted": len(scenarios),
        "rejected": 0,
        "selection": {
            "acceptanceRule": (
                "Declared Manhattan landmarks, full official building geometry, "
                "and collision-audited simulated missions"
            ),
            "plannerOutcomesConsulted": True,
            "layoutRefinement": (
                "Distinct rooftop corridors and continuous task passage validated during "
                "demo design; "
                "not a preregistered or outcome-independent research cohort."
            ),
            "preregistered": False,
        },
        "acceptedByCohort": {"demo": len(scenarios)},
        "scenarios": [
            {
                "id": scenario.scenario_id,
                "fingerprint": dynamic_scenario_fingerprint(scenario),
                "mission": scenario.metadata["mission"],
            }
            for scenario in scenarios
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime._write_json(output_dir / "predictive-scenario-manifest.json", manifest)
    runtime._write_records_csv(
        output_dir / "predictive-records.csv", runtime.predictive_record_rows(bundle)
    )
    bundle["downloads"] = {
        key: runtime.artifact_reference(output_dir / filename)
        for key, filename in runtime.DOWNLOAD_ARTIFACTS.items()
    }
    runtime._write_json(output_dir / "predictive-data.json", bundle)
    return bundle
