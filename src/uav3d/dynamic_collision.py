"""Continuous collision predicates for deterministic dynamic scenarios."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

from uav3d.collision import EPSILON, point_is_free, segment_is_free
from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.geometry import Point3, clamp, distance, dot, lerp, subtract


@dataclass(frozen=True, slots=True)
class DynamicSeparationWitness:
    """A descriptive closest-approach witness for one dynamic obstacle.

    ``separation_m`` is physical surface-to-surface separation after subtracting the vehicle radius;
    the declared safety margin remains a separate threshold.  ``exact`` describes only the
    closest-approach calculation, never continuous vehicle dynamics.
    """

    separation_m: float
    time_s: float
    vehicle_position: Point3
    obstacle_id: str
    obstacle_kind: str
    obstacle_position: Point3
    declared_safety_margin_m: float
    method: str
    exact: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "separation_m": self.separation_m,
            "time_s": self.time_s,
            "vehicle_position": list(self.vehicle_position),
            "obstacle_id": self.obstacle_id,
            "obstacle_kind": self.obstacle_kind,
            "obstacle_position": list(self.obstacle_position),
            "declared_safety_margin_m": self.declared_safety_margin_m,
            "method": self.method,
            "exact": self.exact,
        }


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


def _relative_closest_parameter(relative_a: Point3, relative_b: Point3) -> float:
    direction = subtract(relative_b, relative_a)
    denominator = dot(direction, direction)
    if denominator <= EPSILON:
        return 0.0
    projection = -dot(relative_a, direction) / denominator
    return clamp(projection, 0.0, 1.0)


def _relative_segment_hits_sphere(relative_a: Point3, relative_b: Point3, radius: float) -> bool:
    closest = lerp(
        relative_a,
        relative_b,
        _relative_closest_parameter(relative_a, relative_b),
    )
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


def _sphere_surface_point(center: Point3, vehicle: Point3, radius: float) -> Point3:
    offset = subtract(vehicle, center)
    length = math.sqrt(dot(offset, offset))
    if length <= EPSILON:
        return (center[0] + radius, center[1], center[2])
    factor = radius / length
    return (
        center[0] + offset[0] * factor,
        center[1] + offset[1] * factor,
        center[2] + offset[2] * factor,
    )


def _moving_sphere_separation_witness(
    scenario: DynamicScenario,
    sphere: MovingSphere,
    a: Point3,
    b: Point3,
    start_time: float,
    end_time: float,
) -> DynamicSeparationWitness:
    boundaries = [start_time]
    boundaries.extend(time_s for time_s, _ in sphere.keyframes if start_time < time_s < end_time)
    boundaries.append(end_time)
    duration = end_time - start_time
    candidates: list[DynamicSeparationWitness] = []

    for left, right in pairwise(boundaries):
        left_fraction = 0.0 if duration <= EPSILON else (left - start_time) / duration
        right_fraction = 0.0 if duration <= EPSILON else (right - start_time) / duration
        vehicle_left = lerp(a, b, left_fraction)
        vehicle_right = lerp(a, b, right_fraction)
        sphere_left = sphere.position_at(left)
        sphere_right = sphere.position_at(right)
        relative_left = subtract(vehicle_left, sphere_left)
        relative_right = subtract(vehicle_right, sphere_right)
        fraction = _relative_closest_parameter(relative_left, relative_right)
        witness_time = left + (right - left) * fraction
        vehicle = lerp(vehicle_left, vehicle_right, fraction)
        center = lerp(sphere_left, sphere_right, fraction)
        center_distance = distance(vehicle, center)
        candidates.append(
            DynamicSeparationWitness(
                separation_m=center_distance - sphere.radius - scenario.static_scene.drone_radius,
                time_s=witness_time,
                vehicle_position=vehicle,
                obstacle_id=sphere.sphere_id,
                obstacle_kind="moving-sphere",
                obstacle_position=_sphere_surface_point(center, vehicle, sphere.radius),
                declared_safety_margin_m=scenario.static_scene.safety_margin,
                method="exact-relative-linear-motion",
                exact=True,
            )
        )

    if not candidates:
        center = sphere.position_at(start_time)
        center_distance = distance(a, center)
        return DynamicSeparationWitness(
            separation_m=center_distance - sphere.radius - scenario.static_scene.drone_radius,
            time_s=start_time,
            vehicle_position=a,
            obstacle_id=sphere.sphere_id,
            obstacle_kind="moving-sphere",
            obstacle_position=_sphere_surface_point(center, a, sphere.radius),
            declared_safety_margin_m=scenario.static_scene.safety_margin,
            method="exact-instantaneous-distance",
            exact=True,
        )
    return min(candidates, key=_separation_key)


def _closest_point_on_cylinder(point: Point3, zone: TemporaryCylinder) -> Point3:
    offset_x = point[0] - zone.center[0]
    offset_y = point[1] - zone.center[1]
    radial = math.hypot(offset_x, offset_y)
    z = clamp(point[2], zone.z_min, zone.z_max)
    if radial <= zone.radius:
        return (point[0], point[1], z)
    factor = zone.radius / radial
    return (
        zone.center[0] + offset_x * factor,
        zone.center[1] + offset_y * factor,
        z,
    )


def _temporary_cylinder_separation_witness(
    scenario: DynamicScenario,
    zone: TemporaryCylinder,
    a: Point3,
    b: Point3,
    start_time: float,
    end_time: float,
) -> DynamicSeparationWitness | None:
    if end_time == start_time:
        if not zone.is_active(start_time):
            return None
        left = right = start_time
    else:
        left = max(start_time, zone.active_from)
        right = min(end_time, zone.active_until)
        if left >= right:
            return None
        if right == zone.active_until:
            right = math.nextafter(right, left)

    duration = end_time - start_time

    def vehicle_at(time_s: float) -> Point3:
        fraction = 0.0 if duration <= EPSILON else (time_s - start_time) / duration
        return lerp(a, b, fraction)

    def squared_distance(time_s: float) -> float:
        vehicle = vehicle_at(time_s)
        closest = _closest_point_on_cylinder(vehicle, zone)
        delta = subtract(vehicle, closest)
        return dot(delta, delta)

    lower = left
    upper = right
    inverse_phi = (math.sqrt(5.0) - 1.0) / 2.0
    first = upper - inverse_phi * (upper - lower)
    second = lower + inverse_phi * (upper - lower)
    first_value = squared_distance(first)
    second_value = squared_distance(second)
    for _ in range(80):
        if first_value <= second_value:
            upper = second
            second = first
            second_value = first_value
            first = upper - inverse_phi * (upper - lower)
            first_value = squared_distance(first)
        else:
            lower = first
            first = second
            first_value = second_value
            second = lower + inverse_phi * (upper - lower)
            second_value = squared_distance(second)

    witness_time = min(
        (left, right, 0.5 * (lower + upper)),
        key=lambda time_s: (squared_distance(time_s), time_s),
    )
    vehicle = vehicle_at(witness_time)
    obstacle = _closest_point_on_cylinder(vehicle, zone)
    return DynamicSeparationWitness(
        separation_m=distance(vehicle, obstacle) - scenario.static_scene.drone_radius,
        time_s=witness_time,
        vehicle_position=vehicle,
        obstacle_id=zone.zone_id,
        obstacle_kind="temporary-cylinder",
        obstacle_position=obstacle,
        declared_safety_margin_m=scenario.static_scene.safety_margin,
        method="deterministic-convex-distance-search",
        exact=False,
    )


def _separation_key(
    witness: DynamicSeparationWitness,
) -> tuple[float, float, str, str]:
    return (
        witness.separation_m,
        witness.time_s,
        witness.obstacle_kind,
        witness.obstacle_id,
    )


def minimum_dynamic_separation(
    scenario: DynamicScenario,
    timed_path: Sequence[tuple[float, Point3]],
) -> DynamicSeparationWitness | None:
    """Return the closest dynamic-obstacle witness for a piecewise-linear timed path.

    Moving-sphere witnesses are exact under the declared piecewise-linear motion model. Temporary
    cylinders use a deterministic convex one-dimensional distance search and explicitly report
    ``exact=False``.  This diagnostic is independent from collision certification and makes no claim
    about continuous vehicle dynamics.
    """

    if not timed_path or (not scenario.moving_spheres and not scenario.temporary_cylinders):
        return None
    for time_s, _ in timed_path:
        _validate_time(time_s)
    if any(right[0] <= left[0] for left, right in pairwise(timed_path)):
        raise ValueError("timed-path diagnostic timestamps must increase strictly")

    segments = (
        [(timed_path[0][0], timed_path[0][1], timed_path[0][0], timed_path[0][1])]
        if len(timed_path) == 1
        else [
            (first_time, a, second_time, b)
            for (first_time, a), (second_time, b) in pairwise(timed_path)
        ]
    )
    candidates: list[DynamicSeparationWitness] = []
    for start_time, a, end_time, b in segments:
        candidates.extend(
            _moving_sphere_separation_witness(scenario, sphere, a, b, start_time, end_time)
            for sphere in scenario.moving_spheres
        )
        for zone in scenario.temporary_cylinders:
            witness = _temporary_cylinder_separation_witness(
                scenario, zone, a, b, start_time, end_time
            )
            if witness is not None:
                candidates.append(witness)
    return min(candidates, key=_separation_key, default=None)


# A concise alias used by simulator clients.
dynamic_segment_is_free = spacetime_segment_is_free


__all__ = [
    "DynamicSeparationWitness",
    "dynamic_segment_is_free",
    "minimum_dynamic_separation",
    "point_is_free_at_time",
    "spacetime_segment_is_free",
    "timed_path_is_free",
]
