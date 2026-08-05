"""Export recorded, non-confirmatory results for the static Three.js demo."""

from __future__ import annotations

from datetime import UTC, datetime
from typing import Any

from uav3d.benchmark import ExperimentRecord, run_experiment, scene_fingerprint
from uav3d.scene import Scene


def _demo_result(record: ExperimentRecord) -> dict[str, Any]:
    success = record.status == "success" and record.smoothed_path_valid
    effort_kind = "samples" if record.algorithm == "rrt-star" else "expanded-nodes"
    raw_audit = record.raw_audit
    smoothed_audit = record.smoothed_audit
    return {
        "plannerId": record.algorithm,
        "runId": f"{record.scene_id}:{record.algorithm}:seed-{record.seed}",
        "plannerSeed": record.seed if record.algorithm == "rrt-star" else None,
        "status": record.status if success else "invalid",
        "failureReason": (
            record.failure_reason
            if record.status != "success"
            else "postprocessing-certification-failed"
            if not success
            else None
        ),
        "budget": {
            "maxIterations": record.parameters.get(
                "max_samples", record.parameters.get("max_expansions")
            )
        },
        "searchEffort": {
            "kind": effort_kind,
            "value": record.iterations if effort_kind == "samples" else record.expanded_nodes,
        },
        "paths": {
            "raw": [list(point) for point in record.raw_path],
            "smoothed": [list(point) for point in record.smoothed_path],
        }
        if success
        else None,
        "smoothing": {
            "outcome": record.smoothing.method if record.smoothing else "not-run",
            "collisionFree": bool(record.smoothing and record.smoothing.collision_free),
        },
        "metrics": {
            "planningTimeMs": round(record.planning_time_ms, 3),
            "rawLengthM": round(raw_audit.length_m, 3) if raw_audit else None,
            "smoothedLengthM": round(smoothed_audit.length_m, 3) if smoothed_audit else None,
            "minClearanceM": round(smoothed_audit.minimum_clearance_m, 3)
            if smoothed_audit
            else None,
        },
    }


def build_demo_bundle(scenes: list[Scene], planner_seed: int = 17) -> dict[str, Any]:
    scenario_items: list[dict[str, Any]] = []
    for scene_index, scene in enumerate(scenes):
        results = [
            run_experiment(scene, algorithm, planner_seed + scene_index)
            for algorithm in ("astar-3d", "lazy-theta-star", "rrt-star")
        ]
        scenario_items.append(
            {
                "id": scene.scene_id,
                "label": scene.name,
                "description": str(scene.metadata.get("description", "Static urban scene.")),
                "scenarioSeed": scene.metadata.get("seed"),
                "fingerprint": scene_fingerprint(scene),
                "bounds": {
                    "min": list(scene.bounds.minimum),
                    "max": list(scene.bounds.maximum),
                },
                "start": list(scene.start),
                "goal": list(scene.goal),
                "constraints": {
                    "vehicleRadiusM": scene.drone_radius,
                    "safetyMarginM": scene.safety_margin,
                    "maxAltitudeM": scene.bounds.maximum[2],
                },
                "buildings": [
                    {
                        "id": building.obstacle_id,
                        "min": list(building.minimum),
                        "max": list(building.maximum),
                    }
                    for building in scene.buildings
                ],
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
                "results": [_demo_result(record) for record in results],
            }
        )
    return {
        "schemaVersion": 1,
        "generatedAt": datetime.now(UTC).replace(microsecond=0).isoformat(),
        "verificationStatus": "DEMO_NON_CONFIRMATORY",
        "coordinateSystem": {"frame": "ENU", "distanceUnit": "m", "timeUnit": "ms"},
        "defaultScenarioId": scenes[0].scene_id if scenes else None,
        "planners": [
            {"id": "astar-3d", "label": "3D A*"},
            {"id": "lazy-theta-star", "label": "Lazy Theta*"},
            {"id": "rrt-star", "label": "RRT*"},
        ],
        "scenarios": scenario_items,
    }
