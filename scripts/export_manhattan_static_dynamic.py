"""Generate new collision-audited Manhattan missions on the complete NYC city model.

The public scenes are simulated missions, not authorized flight routes. Planner input
contains every source building. Routes, work counters, timing, replay frames, hashes,
and download artifacts are generated here rather than transformed from old demos.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from dataclasses import replace
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path
from typing import Any

from uav3d.benchmark import scene_fingerprint
from uav3d.collision import point_is_free, segment_is_free
from uav3d.dynamic import DynamicScenario, TemporaryCylinder
from uav3d.dynamic_study import (
    PLANNER_LABELS,
    RECORD_FIELDS,
    _export_frames,
    _export_metrics,
    _export_scenario,
    _normalize_record_numbers,
    _run_status,
    artifact_reference,
    dynamic_record_rows,
    dynamic_run_id,
)
from uav3d.flight_cost import mission_altitude_levels, with_turn_clearance
from uav3d.geometry import Point3, distance, polyline_length
from uav3d.horizontal_curves import smooth_spatial_curves
from uav3d.manhattan_challenges import select_encounter, static_reference
from uav3d.manhattan_missions import (
    audit_task_visits,
    district_mission_evidence,
    mission_task_points,
    resolve_rooftop_anchor,
    simulate_mission_replanning,
)
from uav3d.planners import AStar3D, AStarConfig, LazyThetaStar, LazyThetaStarConfig
from uav3d.planners.rrt_star import RRTStar, RRTStarConfig
from uav3d.replanning import REPLANNING_ALGORITHMS
from uav3d.scene import Cylinder, Scene
from uav3d.shared_airspace import share_dynamic_airspace, world_record
from uav3d.smoothing import smooth_path
from uav3d.validation import audit_path

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = {
    "id": "manhattan-reactive-demo-v5",
    "timeStepS": 2,
    "replanIntervalS": 10,
    "cruiseSpeedMps": 14,
    "maxTimeS": 900,
    "resolutionM": 50,
    "maxExpansions": 20_000,
    "pathShortcut": 1,
    "preserveAltitude": 1,
    "planningGuardS": 10,
    "smoothTurns": 1,
    "curveDimensions": 3,
    "turnScaleM": 60,
    "curveSampleSpacingM": 2,
    "horizontalEscape": 1,
    "verticalCostScale": 5,
    "maxClimbRateMps": 3,
}
STATIC_PLANNERS = ("astar-3d", "lazy-theta-star", "rrt-star")
# Destination names designate simulated neighborhood roof targets, not helipads.
MISSIONS = (
    {
        "id": "midtown-medical-link",
        "name": "Financial District → Plaza medical relay",
        "origin": "Financial District dispatch roof",
        "destination": "Plaza district receiving roof",
        "purpose": "Simulated cross-district cold-chain medical parcel transfer",
        "start": (-74.0084, 40.7046),
        "goal": (-73.9762, 40.7644),
        "zone": True,
        "route": "medical-cross",
    },
    {
        "id": "chelsea-midtown-delivery",
        "name": "Financial west → Hell's Kitchen delivery",
        "origin": "Financial west dispatch roof",
        "destination": "North Hell's Kitchen receiving roof",
        "purpose": "Simulated northbound express parcel transfer across multiple neighborhoods",
        "start": (-74.0141, 40.7060),
        "goal": (-73.9920, 40.7658),
        "zone": False,
        "route": "west-delivery",
    },
    {
        "id": "hudson-inspection",
        "name": "Battery Park → Hudson inspection corridor",
        "origin": "Battery Park district roof",
        "destination": "North Hell's Kitchen service roof",
        "purpose": "Simulated western-corridor inspection team equipment transit",
        "start": (-74.0161, 40.7032),
        "goal": (-73.9960, 40.7650),
        "zone": True,
        "route": "hudson-inspection",
    },
    {
        "id": "flatiron-eastside-logistics",
        "name": "Seaport → Midtown East logistics",
        "origin": "Seaport supply roof",
        "destination": "Midtown East receiving roof",
        "purpose": "Simulated diagonal urban supply transfer through the central business district",
        "start": (-74.0012, 40.7070),
        "goal": (-73.9692, 40.7596),
        "zone": False,
        "route": "east-logistics",
    },
    {
        "id": "midtown-west-backhaul",
        "name": "Central Park South → Financial west backhaul",
        "origin": "Central Park South return-parcel roof",
        "destination": "Financial west consolidation roof",
        "purpose": "Simulated southbound return-parcel collection and depot backhaul",
        "start": (-73.9815, 40.7658),
        "goal": (-74.0140, 40.7042),
        "zone": False,
        "route": "west-backhaul",
    },
    {
        "id": "eastside-medical-return",
        "name": "Plaza district → Lower East Side medical return",
        "origin": "Plaza district medical dispatch roof",
        "destination": "Lower East Side medical receiving roof",
        "purpose": "Simulated hospital-network return of temperature-controlled equipment",
        "start": (-73.9752, 40.7661),
        "goal": (-73.9970, 40.7108),
        "zone": False,
        "route": "medical-south",
    },
    {
        "id": "midtown-supply-backhaul",
        "name": "Rockefeller district → Seaport supply backhaul",
        "origin": "Rockefeller district collection roof",
        "destination": "Seaport supply consolidation roof",
        "purpose": "Simulated reusable supply collection along Fifth Avenue and Broadway",
        "start": (-73.9771, 40.7601),
        "goal": (-74.0054, 40.7044),
        "zone": False,
        "route": "east-backhaul",
    },
    {
        "id": "east-to-hudson-priority",
        "name": "Sutton district → Battery Park priority relay",
        "origin": "Sutton district priority dispatch roof",
        "destination": "Battery Park priority receiving roof",
        "purpose": "Simulated westbound priority spare-parts relay across Manhattan",
        "start": (-73.9718, 40.7614),
        "goal": (-74.0168, 40.7033),
        "zone": False,
        "route": "riverfront-backhaul",
    },
)


def source_snapshot() -> tuple[str, dict[str, Any]]:
    paths = sorted([*ROOT.glob("src/uav3d/**/*.py"), *ROOT.glob("scripts/export_manhattan*.py")])
    files = [
        {
            "path": path.relative_to(ROOT).as_posix(),
            "sha256": "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        for path in paths
    ]
    digest = (
        "sha256:"
        + hashlib.sha256(
            json.dumps(files, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
    )
    return "local-snapshot:" + digest, {"kind": "local-snapshot", "sha256": digest, "files": files}


def rooftop_target(city: Any, longitude: float, latitude: float) -> Point3:
    return resolve_rooftop_anchor(city, longitude, latitude).position


def mission_scenes(city: Any) -> list[tuple[dict[str, Any], Scene]]:
    from uav3d.manhattan_city import make_city_scene

    protected_anchors = tuple(
        point
        for mission in MISSIONS
        for point in (
            resolve_rooftop_anchor(city, *mission["start"]).position,
            resolve_rooftop_anchor(city, *mission["goal"]).position,
            *(tuple(t["position"]) for t in mission_task_points(city, mission["route"])),
        )
    )
    scenes = []
    for mission in MISSIONS:
        start_anchor = resolve_rooftop_anchor(city, *mission["start"])
        goal_anchor = resolve_rooftop_anchor(city, *mission["goal"])
        start, goal = start_anchor.position, goal_anchor.position
        tasks = mission_task_points(city, mission["route"])
        mission = {
            **mission,
            "planningScale": district_mission_evidence(city, start_anchor, goal_anchor),
            "taskPoints": tasks,
        }
        probe = make_city_scene(city, mission["id"], name=mission["name"], start=start, goal=goal)
        reference = static_reference(probe, tasks)
        encounter = select_encounter(
            probe,
            reference,
            tasks,
            fraction=0.46,
            speed=15,
            anchor_clearance_m=180,
            protected_anchors=protected_anchors,
        )
        index = len(scenes)
        zones: tuple[Cylinder, ...] = ()
        titles = (
            "Fixed airspace detour",
            "Dense-block roof circuit",
            "Inspection corridor",
            "Rooftop height transitions",
            "Southbound depot backhaul",
            "Hospital-network return circuit",
            "Eastside reverse logistics",
            "Cross-city priority return",
        )
        if index not in (2, 5):
            zones = (
                Cylinder(
                    f"{mission['id']}-fixed-work-airspace",
                    encounter.position[:2],
                    70 + index % 3 * 10,
                    0,
                    city.bounds.maximum[2],
                ),
            )
        else:
            # Two fixed stand-off areas leave a 50 m horizontal passage. Actual NYC
            # buildings still decide whether that passage is usable at each height.
            dx = encounter.exit[0] - encounter.entry[0]
            dy = encounter.exit[1] - encounter.entry[1]
            norm = math.hypot(dx, dy)
            zones = tuple(
                Cylinder(
                    f"{mission['id']}-stand-off-{side}",
                    (
                        encounter.position[0] + side * dx / norm * 90,
                        encounter.position[1] + side * dy / norm * 90,
                    ),
                    65,
                    0,
                    city.bounds.maximum[2],
                )
                for side in (-1, 1)
            )
        mission["challenge"] = encounter.challenge(titles[index], "static")
        mission["challenge"]["blockedDirectLegs"] = sum(
            not segment_is_free(probe, a, b)
            for a, b in pairwise([start, *(tuple(task["position"]) for task in tasks), goal])
        )
        scene = make_city_scene(
            city,
            mission["id"],
            name=mission["name"],
            start=start,
            goal=goal,
            no_fly_zones=zones,
            metadata={
                "description": mission["purpose"]
                + "; all NYC source buildings are planner obstacles.",
                "simulationOnly": True,
                "seed": 20261005,
                "missionTaskPoints": tasks,
            },
        )
        if not point_is_free(scene, start) or not point_is_free(scene, goal):
            raise ValueError(f"Invalid endpoint for {scene.scene_id}")
        if any(not point_is_free(scene, tuple(task["position"])) for task in tasks):
            raise ValueError(f"Invalid service roof for {scene.scene_id}")
        scenes.append((mission, scene))
    shared_zones = tuple(zone for _, scene in scenes for zone in scene.no_fly_zones)
    common = replace(scenes[0][1], no_fly_zones=shared_zones)
    world = world_record("manhattan-static-shared-v2", common, len(scenes))
    shared = []
    for mission, scene in scenes:
        scene = replace(scene, no_fly_zones=shared_zones)
        for anchor in (
            scene.start,
            scene.goal,
            *(tuple(t["position"]) for t in mission["taskPoints"]),
        ):
            if not point_is_free(scene, anchor):
                raise ValueError("Shared fixed airspace obstructs a mission anchor")
        shared.append((mission | {"sharedWorld": world}, scene))
    return shared


def static_result(scene: Scene, algorithm: str, seed: int) -> dict[str, Any]:
    levels = mission_altitude_levels(scene, 50)
    if algorithm == "astar-3d":
        planner = AStar3D(
            AStarConfig(
                resolution=50,
                max_expansions=20_000,
                max_wall_time_ms=120_000,
                vertical_cost_scale=5,
                altitude_levels=levels,
            )
        )
    elif algorithm == "lazy-theta-star":
        planner = LazyThetaStar(
            LazyThetaStarConfig(
                resolution=50,
                max_expansions=20_000,
                max_wall_time_ms=120_000,
                vertical_cost_scale=5,
                altitude_levels=levels,
            )
        )
    else:
        planner = RRTStar(
            RRTStarConfig(
                max_samples=5_000,
                step_size=70,
                goal_bias=0.16,
                goal_tolerance=95,
                neighbor_radius=160,
                rewire_gamma=1000,
                direct_path_check=True,
                global_goal_connection=True,
                informed_sampling=True,
                max_wall_time_ms=120_000,
                vertical_cost_scale=5,
            )
        )
    started = time.perf_counter()
    tasks = scene.metadata.get("missionTaskPoints", [])
    targets = [scene.start, *(tuple(task["position"]) for task in tasks), scene.goal]
    plans, smoothings = [], []
    for index, (start, goal) in enumerate(pairwise(targets)):
        leg = with_turn_clearance(replace(scene, start=start, goal=goal))
        planned_leg = planner.plan(leg, seed + index)
        if not planned_leg.success:
            raise RuntimeError(
                f"{scene.scene_id}/{algorithm}/leg-{index + 1}: {planned_leg.failure_reason}"
            )
        plans.append(planned_leg)
        smoothings.append(
            smooth_path(
                leg,
                planned_leg.path,
                sample_spacing=3,
                optimize_shortcuts=True,
                preserve_altitude=True,
                round_corners=False,
            )
        )
    raw_path = tuple(
        point
        for index, plan in enumerate(plans)
        for point in (plan.path if index == 0 else plan.path[1:])
    )
    smooth_path_points = tuple(
        point
        for index, result in enumerate(smoothings)
        for point in (result.path if index == 0 else result.path[1:])
    )
    total_length = polyline_length(raw_path)
    progress, offset = [], 0.0
    for index, (plan, result) in enumerate(zip(plans, smoothings, strict=True)):
        length = polyline_length(plan.path)
        progress.extend(
            (offset + value * length) / total_length
            for value in (result.altitude_progress if index == 0 else result.altitude_progress[1:])
        )
        offset += length
    # These are exact domain endpoints. Different fsum grouping across legs can
    # otherwise produce 1 + one ulp, outside the certificate's closed [0, 1] domain.
    progress[0], progress[-1] = 0.0, 1.0
    protected = frozenset(
        value for point, value in zip(smooth_path_points, progress, strict=True) if point in targets
    )
    curves = smooth_spatial_curves(
        smooth_path_points,
        progress,
        lambda a, b, _u, _v: segment_is_free(scene, a, b),
        protected=protected,
        turn_scale_m=60,
        sample_spacing_m=2,
        round_reversals=True,
    )
    smooth_path_points = curves.points
    progress = list(curves.parameters)
    parameters = dict(plans[0].parameters)
    budget_key = "max_samples" if algorithm == "rrt-star" else "max_expansions"
    parameters[budget_key + "_per_leg"] = parameters[budget_key]
    parameters[budget_key] *= len(plans)
    parameters.update(task_point_count=len(tasks), leg_count=len(plans))
    parameters.update(trajectory_curve_degree=5, turn_scale_m=60, curve_sample_spacing_m=2)
    planned = replace(
        plans[0],
        path=raw_path,
        parameters=parameters,
        elapsed_ms=sum(plan.elapsed_ms for plan in plans),
        expanded_nodes=sum(plan.expanded_nodes for plan in plans),
        iterations=sum(plan.iterations for plan in plans),
    )
    smoothing = replace(
        smoothings[0],
        method="local-spatial-quintic-bspline",
        path=smooth_path_points,
        altitude_progress=tuple(progress),
        collision_free=all(result.collision_free for result in smoothings),
        altitude_policy="bounded-spatial-spline-v1",
        altitude_profile_max_error=curves.max_altitude_deviation_m,
    )
    audit_task_visits(raw_path, tasks)
    audit_task_visits(smooth_path_points, tasks)
    raw_audit = audit_path(scene, planned.path)
    smooth_audit = audit_path(scene, smoothing.path)
    if not raw_audit.valid or not smooth_audit.valid or not smoothing.collision_free:
        raise RuntimeError(f"{scene.scene_id}/{algorithm}: failed independent path certification")
    parameters = dict(planned.parameters)
    identity = json.dumps(
        {
            "scene": scene_fingerprint(scene),
            "algorithm": algorithm,
            "seed": seed if algorithm == "rrt-star" else None,
            "parameters": parameters,
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    print(
        json.dumps(
            {
                "phase": "static",
                "scenario": scene.scene_id,
                "planner": algorithm,
                "seconds": round(time.perf_counter() - started, 3),
                "work": planned.iterations if algorithm == "rrt-star" else planned.expanded_nodes,
                "waypoints": len(planned.path),
                "lengthM": round(raw_audit.length_m, 2),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return {
        "plannerId": algorithm,
        "runId": "sha256:" + hashlib.sha256(identity.encode()).hexdigest(),
        "plannerSeed": seed if algorithm == "rrt-star" else None,
        "status": "success",
        "failureReason": None,
        "budget": {
            "maxIterations": parameters.get("max_samples", parameters.get("max_expansions"))
        },
        "parameters": parameters,
        "searchEffort": {
            "kind": "samples" if algorithm == "rrt-star" else "expanded-nodes",
            "value": planned.iterations if algorithm == "rrt-star" else planned.expanded_nodes,
        },
        "paths": {
            "raw": [list(point) for point in planned.path],
            "smoothed": [list(point) for point in smoothing.path],
        },
        "smoothing": {
            "outcome": smoothing.method,
            "collisionFree": True,
            "shortcutPolicy": "shortest-xy-visible-preserved-altitude-dag",
            "sampleSpacingM": 2,
            "optimizationAxes": ["x", "y", "z"],
            "altitudeDeviationLimitM": 12,
            "altitudePolicy": smoothing.altitude_policy,
            "altitudeProfileMaxErrorM": smoothing.altitude_profile_max_error,
            "altitudeProfileProgress": list(smoothing.altitude_progress),
        },
        "metrics": {
            "planningTimeMs": round(planned.elapsed_ms, 3),
            "rawLengthM": round(raw_audit.length_m, 3),
            "smoothedLengthM": round(smooth_audit.length_m, 3),
            "minClearanceM": round(smooth_audit.minimum_clearance_m, 3),
        },
    }


def static_scene_record(
    city: Any, mission: dict[str, Any], scene: Scene, results: list[dict[str, Any]]
) -> dict[str, Any]:
    return {
        "id": scene.scene_id,
        "label": scene.name,
        "description": scene.metadata["description"],
        "scenarioSeed": 20261005,
        "fingerprint": scene_fingerprint(scene),
        "bounds": {"min": list(scene.bounds.minimum), "max": list(scene.bounds.maximum)},
        "start": list(scene.start),
        "goal": list(scene.goal),
        "constraints": {
            "vehicleRadiusM": scene.drone_radius,
            "safetyMarginM": scene.safety_margin,
            "maxAltitudeM": scene.bounds.maximum[2],
        },
        "buildings": city.to_web_buildings(),
        "noFlyZones": [
            {
                "id": zone.zone_id,
                "kind": "cylinder",
                "center": list(zone.center),
                "radiusM": zone.radius,
                "zMinM": zone.z_min,
                "zMaxM": zone.z_max,
            }
            for zone in scene.no_fly_zones
        ],
        "city": city.to_web_metadata(),
        "mission": {
            key: mission[key]
            for key in (
                "origin",
                "destination",
                "purpose",
                "planningScale",
                "taskPoints",
                "challenge",
                "sharedWorld",
            )
        },
        "results": results,
    }


def dynamic_scenario(
    mission: dict[str, Any],
    scene: Scene,
    baseline: list[list[float]],
    index: int,
    *,
    protected_anchors: tuple[Point3, ...] = (),
    patrol_centres: tuple[Point3, ...] = (),
) -> DynamicScenario:
    modes = (
        "crossing",
        "head-on",
        "leader",
        "crossing",
        "head-on",
        "crossing",
        "head-on",
        "crossing",
    )
    titles = (
        "Crossing cargo drone",
        "Oncoming cargo drone",
        "Slow corridor traffic",
        "Merging traffic + closure",
        "Southbound cargo patrol",
        "Hospital relay through crossing traffic",
        "Eastside cargo return lane",
        "Shared cross-city priority corridor",
    )
    path = [tuple(point) for point in baseline]
    encounter = select_encounter(
        scene,
        path,
        mission["taskPoints"],
        fraction=0.28,
        speed=PROTOCOL["cruiseSpeedMps"],
        kind=modes[index],
        anchor_clearance_m=180,
        protected_anchors=protected_anchors,
        continuous_patrol=True,
        patrol_centres=patrol_centres,
    )
    traffic = (
        encounter.aircraft(
            f"{scene.scene_id}-{modes[index]}-cargo",
            horizon=PROTOCOL["maxTimeS"],
            duration_s=48 if index == 2 else 28,
        ),
    )
    temporary: tuple[TemporaryCylinder, ...] = ()
    if index in (1, 3, 5, 7):
        second = select_encounter(
            scene,
            path,
            mission["taskPoints"],
            fraction=0.68,
            speed=14,
            anchor_clearance_m=115,
            protected_anchors=protected_anchors,
        )
        temporary = (
            TemporaryCylinder(
                f"{scene.scene_id}-temporary-approach-closure",
                second.position[:2],
                70,
                0,
                scene.bounds.maximum[2],
                second.arrival_s - 18,
                second.arrival_s + 38,
            ),
        )
    return DynamicScenario(
        f"{scene.scene_id}-reactive",
        scene.name + " · reactive",
        scene,
        temporary,
        traffic,
        metadata={
            "simulationOnly": True,
            "challenge": encounter.challenge(titles[index], "dynamic"),
            "trafficMode": modes[index],
            "airframeSpanM": 18.0,
            "patrolCentre": encounter.position if traffic else None,
        },
    )


def shared_dynamic_scenarios(
    missions: list[tuple[dict[str, Any], Scene]], baselines: list[list[list[float]]]
) -> tuple[DynamicScenario, ...]:
    anchors = tuple(
        p
        for mission, scene in missions
        for p in (scene.start, scene.goal, *(tuple(t["position"]) for t in mission["taskPoints"]))
    )
    private, centres = [], []
    for index, ((mission, scene), baseline) in enumerate(zip(missions, baselines, strict=True)):
        scenario = dynamic_scenario(
            mission,
            scene,
            baseline,
            index,
            protected_anchors=anchors,
            patrol_centres=tuple(centres),
        )
        private.append(scenario)
        if scenario.metadata["patrolCentre"] is not None:
            centres.append(scenario.metadata["patrolCentre"])
    # Four additional cargo patrols occupy the later legs across the expanded
    # island, rather than duplicating aircraft near the old Midtown center.
    for index in range(min(4, len(private))):
        mission, scene = missions[index]
        encounter = select_encounter(
            scene,
            [tuple(p) for p in baselines[index]],
            mission["taskPoints"],
            fraction=0.66,
            speed=PROTOCOL["cruiseSpeedMps"],
            anchor_clearance_m=180,
            protected_anchors=anchors,
            continuous_patrol=True,
            patrol_centres=tuple(centres),
        )
        centres.append(encounter.position)
        private[index] = replace(
            private[index],
            moving_spheres=(
                *private[index].moving_spheres,
                encounter.aircraft(
                    f"{scene.scene_id}-secondary-cargo", horizon=PROTOCOL["maxTimeS"]
                ),
            ),
        )
    return share_dynamic_airspace(tuple(private), "manhattan-dynamic-shared-v3")


def dynamic_record(city: Any, mission: dict[str, Any], scenario: DynamicScenario) -> dict[str, Any]:
    parameters = {key: value for key, value in PROTOCOL.items() if key != "id"}
    parameters.update(
        taskPointCount=len(mission["taskPoints"]),
        legCount=len(mission["taskPoints"]) + 1,
        serviceDurationTotalS=sum(task["serviceDurationS"] for task in mission["taskPoints"]),
    )
    records = []
    for algorithm in REPLANNING_ALGORITHMS:
        started = time.perf_counter()
        run = simulate_mission_replanning(
            scenario,
            algorithm,
            mission["taskPoints"],
            time_step=PROTOCOL["timeStepS"],
            replan_interval=PROTOCOL["replanIntervalS"],
            cruise_speed=PROTOCOL["cruiseSpeedMps"],
            max_time=PROTOCOL["maxTimeS"],
            resolution=PROTOCOL["resolutionM"],
            max_expansions=PROTOCOL["maxExpansions"],
            shortcut_paths=True,
            preserve_altitude=True,
            planning_guard_s=PROTOCOL["planningGuardS"],
            smooth_turns=True,
            spatial_curves=True,
            turn_scale_m=PROTOCOL["turnScaleM"],
            curve_sample_spacing_m=PROTOCOL["curveSampleSpacingM"],
            allow_horizontal_escape=True,
            vertical_cost_scale=PROTOCOL["verticalCostScale"],
            altitude_levels=mission_altitude_levels(scenario.static_scene, PROTOCOL["resolutionM"]),
            max_climb_rate=PROTOCOL["maxClimbRateMps"],
        )
        if not run.metrics.success or run.metrics.collision_count:
            raise RuntimeError(f"{scenario.scenario_id}/{algorithm}: {run.metrics.failure_reason}")
        print(
            json.dumps(
                {
                    "phase": "dynamic",
                    "scenario": scenario.scenario_id,
                    "planner": algorithm,
                    "seconds": round(time.perf_counter() - started, 3),
                    "frames": len(run.frames),
                    "replans": run.metrics.replans,
                    "work": run.metrics.total_planning_work,
                    "holds": run.metrics.holds,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        frames = _export_frames(scenario, run)
        for frame in frames:
            if frame["event"]["kind"] == "wait":
                for task in mission["taskPoints"]:
                    if distance(frame["vehicle"], task["position"]) < 1e-5:
                        frame["event"]["label"] = f"Task {task['order']} · {task['label']}"
                        break
        records.append(
            {
                "runId": dynamic_run_id(run.scenario_fingerprint, algorithm, PROTOCOL, parameters),
                "plannerId": algorithm,
                "status": _run_status(run),
                "failureReason": None,
                "parameters": parameters,
                "metrics": _export_metrics(run),
                "frames": frames,
                "executionTimedPath": [
                    {"time": p.time_s, "position": list(p.position), "action": p.action}
                    for p in run.execution_timed_path.waypoints
                ]
                if run.execution_timed_path is not None
                else None,
            }
        )
    record = _export_scenario(scenario, [])
    record.update(
        {
            "description": mission["purpose"]
            + "; simulated temporary restrictions and air traffic.",
            "buildings": city.to_web_buildings(),
            "city": city.to_web_metadata(),
            "mission": {
                key: mission[key]
                for key in ("origin", "destination", "purpose", "planningScale", "taskPoints")
            }
            | {
                "challenge": scenario.metadata["challenge"],
                "sharedWorld": scenario.metadata["sharedWorld"],
            },
            "runs": records,
        }
    )
    serialized = _normalize_record_numbers(record)
    for run in serialized["runs"]:
        audit_task_visits(
            [frame["vehicle"] for frame in run["frames"]],
            mission["taskPoints"],
            times=[frame["timeS"] for frame in run["frames"]],
        )
    return serialized


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        + "\n"
    )


def refresh_artifacts(output_dir: Path, city: Any) -> None:
    """Serialize existing certified runs compactly without changing recorded metrics."""
    source, provenance = source_snapshot()
    expected_buildings = city.to_web_buildings()
    for name in ("demo-data.json", "dynamic-data.json"):
        path = output_dir / name
        bundle = json.loads(path.read_text(encoding="utf-8"))
        if not bundle["sourceCommit"].startswith("local-snapshot:sha256:"):
            raise ValueError("Only this generator's recorded Manhattan runs may be reused")
        for scenario in bundle["scenarios"]:
            if (
                scenario["buildings"] != expected_buildings
                or scenario["city"]["id"] != city.city_id
            ):
                raise ValueError(
                    "Cached missions do not contain the unchanged complete official city"
                )
        bundle.setdefault("computationSourceProvenance", bundle["sourceProvenance"])
        bundle["sourceCommit"] = source
        bundle["sourceProvenance"] = provenance
        if name == "dynamic-data.json":
            manifest_path = output_dir / "dynamic-scenario-manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["sourceCommit"] = source
            manifest["sourceProvenance"] = provenance
            write_json(manifest_path, manifest)
            records_path = output_dir / "dynamic-records.csv"
            rows = dynamic_record_rows(bundle)
            for row in rows:
                row["protocol_id"] = bundle["protocol"]["id"]
            with records_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=RECORD_FIELDS, lineterminator="\n")
                writer.writeheader()
                writer.writerows(rows)
            bundle["downloads"] = {
                "recordsCsv": artifact_reference(records_path),
                "scenarioManifest": artifact_reference(manifest_path),
            }
        write_json(path, bundle)
        print(
            json.dumps(
                {"artifact": name, "bytes": path.stat().st_size, "recordedRunsUnchanged": True}
            ),
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "web/public")
    parser.add_argument(
        "--city-file", type=Path, help="Compute against a staged physical city before publishing."
    )
    parser.add_argument(
        "--benchmark-first",
        action="store_true",
        help="Run one complete static/dynamic mission without writing bundles.",
    )
    parser.add_argument("--static-only", action="store_true")
    parser.add_argument("--dynamic-only", action="store_true")
    parser.add_argument(
        "--refresh-artifacts",
        action="store_true",
        help="Compact existing certified city runs, retaining their computation provenance.",
    )
    args = parser.parse_args()
    from uav3d.manhattan_city import build_manhattan_city

    city = build_manhattan_city(args.city_file)
    if args.refresh_artifacts:
        refresh_artifacts(args.output_dir, city)
        return
    missions = mission_scenes(city)
    timestamp = datetime.now(UTC).replace(microsecond=0).isoformat()
    source, provenance = source_snapshot()
    static_scenes = []
    dynamic_scenes = []
    baselines = []
    for index, (mission, scene) in enumerate(missions[:1] if args.benchmark_first else missions):
        results = []
        if not args.dynamic_only:
            results = [static_result(scene, algorithm, 17 + index) for algorithm in STATIC_PLANNERS]
            static_scenes.append(static_scene_record(city, mission, scene, results))
        if not args.static_only:
            baseline = (
                results[0]["paths"]["raw"]
                if results
                else static_result(scene, "astar-3d", 17 + index)["paths"]["raw"]
            )
            baselines.append(baseline)
    if not args.static_only:
        selected = missions[:1] if args.benchmark_first else missions
        for (mission, _), scenario in zip(
            selected, shared_dynamic_scenarios(selected, baselines), strict=True
        ):
            dynamic_scenes.append(dynamic_record(city, mission, scenario))
    if args.benchmark_first:
        print(
            json.dumps(
                {
                    "benchmark": "complete",
                    "buildings": len(city.buildings),
                    "bounds": city.bounds.to_dict(),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        return
    if static_scenes:
        write_json(
            args.output_dir / "demo-data.json",
            {
                "schemaVersion": 1,
                "generatedAt": timestamp,
                "sourceCommit": source,
                "sourceProvenance": provenance,
                "verificationStatus": "DEMO_NON_CONFIRMATORY",
                "coordinateSystem": {"frame": "ENU", "distanceUnit": "m", "timeUnit": "ms"},
                "defaultScenarioId": static_scenes[0]["id"],
                "planners": [
                    {"id": planner, "label": label}
                    for planner, label in zip(
                        STATIC_PLANNERS, ("3D A*", "Lazy Theta*", "RRT*"), strict=True
                    )
                ],
                "scenarios": static_scenes,
            },
        )
    if dynamic_scenes:
        bundle = {
            "schemaVersion": 1,
            "generatedAt": timestamp,
            "sourceCommit": source,
            "sourceProvenance": provenance,
            "verificationStatus": "DYNAMIC_DEMO_NON_CONFIRMATORY",
            "protocol": PROTOCOL,
            "planners": [
                {"id": algorithm, "label": PLANNER_LABELS[algorithm]}
                for algorithm in REPLANNING_ALGORITHMS
            ],
            "scenarios": dynamic_scenes,
        }
        manifest_path = args.output_dir / "dynamic-scenario-manifest.json"
        records_path = args.output_dir / "dynamic-records.csv"
        write_json(
            manifest_path,
            {
                "schemaVersion": 1,
                "datasetId": PROTOCOL["id"],
                "sourceCommit": source,
                "sourceProvenance": provenance,
                "generatedAt": timestamp,
                "protocolId": PROTOCOL["id"],
                "selection": {
                    "scenarioRule": "eight declared Manhattan simulation missions",
                    "algorithmFiltering": "none",
                },
                "scenarioCount": len(dynamic_scenes),
                "runCount": len(dynamic_scenes) * 3,
                "scenarios": [
                    {
                        "id": scene["id"],
                        "label": scene["label"],
                        "fingerprint": scene["fingerprint"],
                        "selected": True,
                    }
                    for scene in dynamic_scenes
                ],
            },
        )
        rows = dynamic_record_rows(bundle)
        for row in rows:
            row["protocol_id"] = PROTOCOL["id"]
        with records_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=RECORD_FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        bundle["downloads"] = {
            "recordsCsv": artifact_reference(records_path),
            "scenarioManifest": artifact_reference(manifest_path),
        }
        write_json(args.output_dir / "dynamic-data.json", bundle)


if __name__ == "__main__":
    main()
