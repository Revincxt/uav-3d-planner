"""Collision and clearance queries for static city geometry."""

from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

from uav3d.geometry import Point3, clamp, distance, lerp
from uav3d.scene import AABB, Cylinder, Scene

EPSILON = 1e-9


def _inside_inset_bounds(scene: Scene, point: Point3, clearance: float) -> bool:
    return all(
        lower + clearance - EPSILON <= value <= upper - clearance + EPSILON
        for value, lower, upper in zip(
            point, scene.bounds.minimum, scene.bounds.maximum, strict=True
        )
    )


def _point_in_aabb(point: Point3, box: AABB, padding: float) -> bool:
    return all(
        lower - padding - EPSILON <= value <= upper + padding + EPSILON
        for value, lower, upper in zip(point, box.minimum, box.maximum, strict=True)
    )


def _point_in_cylinder(point: Point3, zone: Cylinder, padding: float) -> bool:
    dx = point[0] - zone.center[0]
    dy = point[1] - zone.center[1]
    return (
        dx * dx + dy * dy <= (zone.radius + padding) ** 2 + EPSILON
        and zone.z_min - padding - EPSILON <= point[2] <= zone.z_max + padding + EPSILON
    )


def point_is_free(scene: Scene, point: Point3, clearance: float | None = None) -> bool:
    padding = scene.required_clearance if clearance is None else clearance
    if not _inside_inset_bounds(scene, point, padding):
        return False
    if any(_point_in_aabb(point, building, padding) for building in scene.buildings):
        return False
    return not any(_point_in_cylinder(point, zone, padding) for zone in scene.no_fly_zones)


def _segment_intersects_aabb(a: Point3, b: Point3, box: AABB, padding: float) -> bool:
    t_min = 0.0
    t_max = 1.0
    for origin, target, lower, upper in zip(a, b, box.minimum, box.maximum, strict=True):
        direction = target - origin
        lower -= padding
        upper += padding
        if abs(direction) <= EPSILON:
            if origin < lower or origin > upper:
                return False
            continue
        near = (lower - origin) / direction
        far = (upper - origin) / direction
        if near > far:
            near, far = far, near
        t_min = max(t_min, near)
        t_max = min(t_max, far)
        if t_min > t_max + EPSILON:
            return False
    return True


def _segment_intersects_cylinder(a: Point3, b: Point3, zone: Cylinder, padding: float) -> bool:
    z_lower = zone.z_min - padding
    z_upper = zone.z_max + padding
    dz = b[2] - a[2]
    if abs(dz) <= EPSILON:
        if a[2] < z_lower or a[2] > z_upper:
            return False
        t_lower, t_upper = 0.0, 1.0
    else:
        t0 = (z_lower - a[2]) / dz
        t1 = (z_upper - a[2]) / dz
        t_lower = max(0.0, min(t0, t1))
        t_upper = min(1.0, max(t0, t1))
        if t_lower > t_upper + EPSILON:
            return False

    dx = b[0] - a[0]
    dy = b[1] - a[1]
    offset_x = a[0] - zone.center[0]
    offset_y = a[1] - zone.center[1]
    denominator = dx * dx + dy * dy
    if denominator <= EPSILON:
        closest_t = t_lower
    else:
        closest_t = clamp(-(offset_x * dx + offset_y * dy) / denominator, t_lower, t_upper)
    closest_x = offset_x + closest_t * dx
    closest_y = offset_y + closest_t * dy
    return closest_x * closest_x + closest_y * closest_y <= (zone.radius + padding) ** 2 + EPSILON


def segment_is_free(scene: Scene, a: Point3, b: Point3, clearance: float | None = None) -> bool:
    padding = scene.required_clearance if clearance is None else clearance
    if not _inside_inset_bounds(scene, a, padding) or not _inside_inset_bounds(scene, b, padding):
        return False
    if any(_segment_intersects_aabb(a, b, building, padding) for building in scene.buildings):
        return False
    return not any(_segment_intersects_cylinder(a, b, zone, padding) for zone in scene.no_fly_zones)


def path_is_free(scene: Scene, path: Sequence[Point3], clearance: float | None = None) -> bool:
    if not path:
        return False
    if len(path) == 1:
        return point_is_free(scene, path[0], clearance)
    return all(segment_is_free(scene, a, b, clearance) for a, b in pairwise(path))


def _distance_to_aabb(point: Point3, box: AABB) -> float:
    squared = 0.0
    for value, lower, upper in zip(point, box.minimum, box.maximum, strict=True):
        delta = max(lower - value, 0.0, value - upper)
        squared += delta * delta
    return math.sqrt(squared)


def _distance_to_cylinder(point: Point3, zone: Cylinder) -> float:
    radial = math.hypot(point[0] - zone.center[0], point[1] - zone.center[1])
    radial_outside = max(0.0, radial - zone.radius)
    vertical_outside = max(0.0, zone.z_min - point[2], point[2] - zone.z_max)
    return math.hypot(radial_outside, vertical_outside)


def point_clearance(scene: Scene, point: Point3) -> float:
    boundary_clearance = min(
        *(value - lower for value, lower in zip(point, scene.bounds.minimum, strict=True)),
        *(upper - value for value, upper in zip(point, scene.bounds.maximum, strict=True)),
    )
    obstacle_clearances = [_distance_to_aabb(point, building) for building in scene.buildings]
    obstacle_clearances.extend(_distance_to_cylinder(point, zone) for zone in scene.no_fly_zones)
    geometric = min([boundary_clearance, *obstacle_clearances])
    return geometric - scene.drone_radius


def sample_path(path: Sequence[Point3], spacing: float = 0.5) -> list[Point3]:
    if not path:
        return []
    samples = [path[0]]
    for a, b in pairwise(path):
        count = max(1, math.ceil(distance(a, b) / spacing))
        samples.extend(lerp(a, b, index / count) for index in range(1, count + 1))
    return samples


def minimum_path_clearance(scene: Scene, path: Sequence[Point3], spacing: float = 0.5) -> float:
    samples = sample_path(path, spacing)
    if not samples:
        return 0.0
    return min(point_clearance(scene, point) for point in samples)
