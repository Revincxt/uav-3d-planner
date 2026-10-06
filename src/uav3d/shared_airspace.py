"""One obstacle world and one absolute clock for a cohort of mission queries.

Mission vehicles are not automatically obstacles to one another. This contract
shares the city's physical constraints and declared exogenous traffic, not a
multi-agent joint-trajectory/deconfliction certificate.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, replace
from itertools import pairwise
from typing import Any

from uav3d.collision import point_is_free, segment_is_free
from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.scene import Scene


def world_record(
    identifier: str,
    scene: Scene,
    mission_count: int,
    temporary: tuple[TemporaryCylinder, ...] = (),
    traffic: tuple[MovingSphere, ...] = (),
) -> dict[str, Any]:
    payload = {
        "bounds": asdict(scene.bounds),
        "buildings": [asdict(building) for building in scene.buildings],
        "zones": [asdict(zone) for zone in scene.no_fly_zones],
        "temporary": [asdict(zone) for zone in temporary],
        "traffic": [asdict(aircraft) for aircraft in traffic],
        "clearance": scene.required_clearance,
    }
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
    return {
        "id": identifier,
        "fingerprint": "sha256:" + digest,
        "missionCount": mission_count,
        "clockOriginS": 0,
        "airframeSpanM": 18,
        "missionDeconfliction": "not-jointly-optimized",
    }


def share_dynamic_airspace(
    scenarios: tuple[DynamicScenario, ...], identifier: str
) -> tuple[DynamicScenario, ...]:
    """Join before any planner runs; each query receives the exact same hazards."""
    if not scenarios:
        return ()
    zones = tuple(zone for s in scenarios for zone in s.temporary_cylinders)
    traffic = tuple(aircraft for s in scenarios for aircraft in s.moving_spheres)
    if len({zone.zone_id for zone in zones}) != len(zones) or len(
        {aircraft.sphere_id for aircraft in traffic}
    ) != len(traffic):
        raise ValueError("Shared obstacle identifiers must be unique")
    reference = scenarios[0].static_scene
    for scenario in scenarios:
        scene = scenario.static_scene
        if (scene.bounds, scene.buildings, scene.no_fly_zones, scene.required_clearance) != (
            reference.bounds, reference.buildings, reference.no_fly_zones,
            reference.required_clearance,
        ):
            raise ValueError("Shared missions must have identical physical city constraints")
        tasks = scene.metadata.get("missionTaskPoints", [])
        anchors = [scene.start, scene.goal, *(tuple(t["position"]) for t in tasks)]
        if any(not point_is_free(scene, anchor) for anchor in anchors):
            raise ValueError("Shared mission has an obstructed service anchor")
        for zone in zones:
            if any(
                (anchor[0] - zone.center[0]) ** 2 + (anchor[1] - zone.center[1]) ** 2
                <= (zone.radius + scene.required_clearance) ** 2
                for anchor in anchors
            ):
                raise ValueError("Shared reservation obstructs another mission's service roof")
    for aircraft in traffic:
        if any(not segment_is_free(reference, a, b,
                    clearance=aircraft.radius + reference.required_clearance)
               for (_, a), (_, b) in pairwise(aircraft.keyframes)):
            raise ValueError("Shared aircraft motion intersects the physical city")
    world = world_record(identifier, reference, len(scenarios), zones, traffic)
    shared = []
    for scenario in scenarios:
        metadata = dict(scenario.metadata) | {"sharedWorld": world}
        if "mission" in metadata:
            metadata["mission"] = dict(metadata["mission"]) | {"sharedWorld": world}
        shared.append(replace(scenario, temporary_cylinders=zones, moving_spheres=traffic,
                              metadata=metadata))
    return tuple(shared)
