"""Continuous collision predicates for deterministic dynamic scenarios."""

from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

from uav3d.collision import EPSILON, point_is_free, segment_is_free
from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.geometry import Point3, clamp, distance, dot, lerp, subtract


def _clearance(scenario: DynamicScenario, clearance: float | None) -> float:
    value = scenario.static_scene.required_clearance if clearance is None else clearance
    if not math.isfinite(value) or value < 0:
        raise ValueError("clearance must be finite and non-negative")
    return value


def _validate_time(time_s: float) -> None:
    if not math.isfinite(time_s) or time_s < 0:
        raise ValueError("times must be finite and non-negative")


def _cylinder_collision_parameters(
    a: Point3, b: Point3, zone: TemporaryCylinder, padding: float
) -> tuple[float, float] | None:
    """Return the closed normalized interval where a segment occupies a padded cylinder."""

    lower = 0.0
    upper = 1.0
    dz = b[2] - a[2]
    z_min = zone.z_min - padding
    z_max = zone.z_max + padding
    if abs(dz) <= EPSILON:
        if a[2] < z_min - EPSILON or a[2] > z_max + EPSILON:
            return None
    else:
        first = (z_min - a[2]) / dz
        second = (z_max - a[2]) / dz
        lower = max(lower, min(first, second))
        upper = min(upper, max(first, second))
        if lower > upper + EPSILON:
            return None

    dx = b[0] - a[0]
    dy = b[1] - a[1]
    offset_x = a[0] - zone.center[0]
    offset_y = a[1] - zone.center[1]
    radius = zone.radius + padding
    quadratic = dx * dx + dy * dy
    linear = 2.0 * (offset_x * dx + offset_y * dy)
    constant = offset_x * offset_x + offset_y * offset_y - radius * radius
    if quadratic <= EPSILON:
        if constant > EPSILON:
            return None
    else:
        discriminant = linear * linear - 4.0 * quadratic * constant
        if discriminant < -EPSILON:
            return None
        root = math.sqrt(max(0.0, discriminant))
        first = (-linear - root) / (2.0 * quadratic)
        second = (-linear + root) / (2.0 * quadratic)
        lower = max(lower, first)
        upper = min(upper, second)
        if lower > upper + EPSILON:
            return None
    lower = max(0.0, lower)
    upper = min(1.0, upper)
    if lower > upper + EPSILON:
        return None
    return (lower, upper)


def _temporary_cylinder_collision(
    zone: TemporaryCylinder,
    a: Point3,
    b: Point3,
    start_time: float,
    end_time: float,
    padding: float,
) -> bool:
    parameters = _cylinder_collision_parameters(a, b, zone, padding)
    if parameters is None:
        return False
    duration = end_time - start_time
    collision_from = start_time + parameters[0] * duration
    collision_until = start_time + parameters[1] * duration
    overlap_from = max(collision_from, start_time, zone.active_from)
    overlap_until = min(collision_until, end_time, zone.active_until)
    if overlap_from < overlap_until:
        return True
    if overlap_from > overlap_until:
        return False
    # Contact at a single instant is a collision, except at the exclusive right boundary.
    return overlap_from < zone.active_until


def _relative_segment_hits_sphere(relative_a: Point3, relative_b: Point3, radius: float) -> bool:
    direction = subtract(relative_b, relative_a)
    denominator = dot(direction, direction)
    if denominator <= EPSILON:
        closest = relative_a
    else:
        projection = -dot(relative_a, direction) / denominator
        closest = lerp(relative_a, relative_b, clamp(projection, 0.0, 1.0))
    squared = dot(closest, closest)
    return squared <= radius * radius + EPSILON


def _moving_sphere_collision(
    sphere: MovingSphere,
    a: Point3,
    b: Point3,
    start_time: float,
    end_time: float,
    padding: float,
) -> bool:
    if end_time == start_time:
        relative = subtract(a, sphere.position_at(start_time))
        return _relative_segment_hits_sphere(relative, relative, sphere.radius + padding)
    boundaries = [start_time]
    boundaries.extend(time_s for time_s, _ in sphere.keyframes if start_time < time_s < end_time)
    boundaries.append(end_time)
    duration = end_time - start_time
    for left, right in pairwise(boundaries):
        left_fraction = (left - start_time) / duration
        right_fraction = (right - start_time) / duration
        relative_left = subtract(lerp(a, b, left_fraction), sphere.position_at(left))
        relative_right = subtract(lerp(a, b, right_fraction), sphere.position_at(right))
        if _relative_segment_hits_sphere(relative_left, relative_right, sphere.radius + padding):
            return True
    return False


def point_is_free_at_time(
    scenario: DynamicScenario,
    point: Point3,
    time_s: float,
    clearance: float | None = None,
) -> bool:
    """Check static and dynamic contact at one instant."""

    _validate_time(time_s)
    padding = _clearance(scenario, clearance)
    if not point_is_free(scenario.static_scene, point, padding):
        return False
    for zone in scenario.temporary_cylinders:
        if zone.is_active(time_s) and _temporary_cylinder_collision(
            zone, point, point, time_s, time_s, padding
        ):
            return False
    return not any(
        _moving_sphere_collision(sphere, point, point, time_s, time_s, padding)
        for sphere in scenario.moving_spheres
    )


def spacetime_segment_is_free(
    scenario: DynamicScenario,
    a: Point3,
    b: Point3,
    start_time: float,
    end_time: float,
    clearance: float | None = None,
) -> bool:
    """Exactly test a linearly traversed segment against piecewise-linear obstacles."""

    _validate_time(start_time)
    _validate_time(end_time)
    if end_time < start_time:
        raise ValueError("end_time must not precede start_time")
    if end_time == start_time and distance(a, b) > EPSILON:
        raise ValueError("a non-zero segment requires positive traversal time")
    padding = _clearance(scenario, clearance)
    if not segment_is_free(scenario.static_scene, a, b, padding):
        return False
    if any(
        _temporary_cylinder_collision(zone, a, b, start_time, end_time, padding)
        for zone in scenario.temporary_cylinders
    ):
        return False
    return not any(
        _moving_sphere_collision(sphere, a, b, start_time, end_time, padding)
        for sphere in scenario.moving_spheres
    )


def timed_path_is_free(
    scenario: DynamicScenario,
    timed_path: Sequence[tuple[float, Point3]],
    clearance: float | None = None,
) -> bool:
    """Check a path carrying an explicit timestamp for every waypoint."""

    if not timed_path:
        return False
    if len(timed_path) == 1:
        return point_is_free_at_time(scenario, timed_path[0][1], timed_path[0][0], clearance)
    return all(
        spacetime_segment_is_free(scenario, a, b, first_time, second_time, clearance)
        for (first_time, a), (second_time, b) in pairwise(timed_path)
    )


# A concise alias used by simulator clients.
dynamic_segment_is_free = spacetime_segment_is_free


__all__ = [
    "dynamic_segment_is_free",
    "point_is_free_at_time",
    "spacetime_segment_is_free",
    "timed_path_is_free",
]
