"""Flight-aware search metrics; collision geometry always stays in world metres."""

from __future__ import annotations

import math
from dataclasses import replace
from itertools import pairwise

from uav3d.collision import point_is_free
from uav3d.geometry import Point3, distance
from uav3d.scene import Scene


def validate_vertical_scale(value: float) -> None:
    if not math.isfinite(value) or value < 1:
        raise ValueError("vertical cost scale must be finite and at least one")


def flight_distance(a: Point3, b: Point3, vertical_scale: float = 1.0) -> float:
    """A norm with a consistent heuristic, in cruise-equivalent metres.

    Scaling Z by cruise/climb speed penalizes avoidable elevation excursions.
    This is a search objective, not a geometric path-length measurement.
    """
    if vertical_scale == 1:
        return distance(a, b)
    return math.hypot(b[0] - a[0], b[1] - a[1], (b[2] - a[2]) * vertical_scale)


def segment_flight_time(
    a: Point3, b: Point3, cruise_speed: float, max_climb_rate: float | None = None
) -> float:
    duration = distance(a, b) / cruise_speed
    return duration if max_climb_rate is None else max(duration, abs(b[2] - a[2]) / max_climb_rate)


def mission_altitude_levels(scene: Scene, resolution: float) -> tuple[float, ...]:
    """Retain all required anchor heights plus escape layers, without moving tasks."""
    lower, upper = scene.bounds.minimum[2], scene.bounds.maximum[2]
    levels = {lower + i * resolution for i in range(math.floor((upper - lower) / resolution) + 1)}
    levels.update((scene.start[2], scene.goal[2]))
    levels.update(task["position"][2] for task in scene.metadata.get("missionTaskPoints", []))
    return tuple(sorted(z for z in levels if lower <= z <= upper))


def vertical_travel(points: tuple[Point3, ...]) -> float:
    return math.fsum(abs(b[2] - a[2]) for a, b in pairwise(points))


def with_turn_clearance(scene: Scene, reserve_m: float = 2.0) -> Scene:
    """Reserve curve room only when both hard endpoints remain valid.

    Rooftop anchors may already be close to neighboring facades. An optional
    smoothing buffer must never turn a valid task into an unreachable endpoint.
    Original collision clearance is never reduced.
    """
    if not math.isfinite(reserve_m) or reserve_m <= 0:
        raise ValueError("turn clearance reserve must be finite and positive")
    for factor in (1.0, 0.5, 0.25, 0.125):
        candidate = replace(scene, safety_margin=scene.safety_margin + reserve_m * factor)
        if point_is_free(candidate, scene.start) and point_is_free(candidate, scene.goal):
            return candidate
    return scene
