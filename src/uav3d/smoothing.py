"""Shared, collision-certified post-processing for every planner."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from uav3d.collision import path_is_free, segment_is_free
from uav3d.geometry import Point3, almost_equal, distance, lerp, polyline_length
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class SmoothingResult:
    path: tuple[Point3, ...]
    method: str
    collision_free: bool
    shortcut_waypoints: int
    sample_count: int
    spline_blend: float | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "path": [list(point) for point in self.path],
            "method": self.method,
            "collision_free": self.collision_free,
            "shortcut_waypoints": self.shortcut_waypoints,
            "sample_count": self.sample_count,
            "spline_blend": self.spline_blend,
        }


def remove_duplicate_points(path: Sequence[Point3]) -> list[Point3]:
    cleaned: list[Point3] = []
    for point in path:
        if not cleaned or not almost_equal(cleaned[-1], point):
            cleaned.append(point)
    return cleaned


def farthest_visible_shortcut(scene: Scene, path: Sequence[Point3]) -> list[Point3]:
    points = remove_duplicate_points(path)
    if len(points) <= 2:
        return points
    result = [points[0]]
    current = 0
    while current < len(points) - 1:
        next_index = current + 1
        for candidate in range(len(points) - 1, current, -1):
            if segment_is_free(scene, points[current], points[candidate]):
                next_index = candidate
                break
        result.append(points[next_index])
        current = next_index
    return result


def _open_uniform_knots(control_count: int, degree: int) -> list[float]:
    interior_count = control_count - degree - 1
    knots = [0.0] * (degree + 1)
    if interior_count > 0:
        denominator = interior_count + 1
        knots.extend(index / denominator for index in range(1, interior_count + 1))
    knots.extend([1.0] * (degree + 1))
    return knots


def _de_boor(
    control_points: Sequence[Point3], degree: int, knots: Sequence[float], u: float
) -> Point3:
    count = len(control_points)
    if u >= 1.0:
        return control_points[-1]
    span = degree
    for index in range(degree, count):
        if knots[index] <= u < knots[index + 1]:
            span = index
            break
    working = [list(control_points[span - degree + offset]) for offset in range(degree + 1)]
    for level in range(1, degree + 1):
        for offset in range(degree, level - 1, -1):
            knot_index = span - degree + offset
            denominator = knots[knot_index + degree - level + 1] - knots[knot_index]
            alpha = 0.0 if abs(denominator) <= 1e-12 else (u - knots[knot_index]) / denominator
            working[offset] = [
                (1 - alpha) * working[offset - 1][axis] + alpha * working[offset][axis]
                for axis in range(3)
            ]
    value = working[degree]
    return (value[0], value[1], value[2])


def sample_bspline(control_points: Sequence[Point3], sample_count: int) -> list[Point3]:
    if sample_count < 2:
        raise ValueError("sample_count must be at least two")
    points = remove_duplicate_points(control_points)
    if len(points) <= 2:
        return points
    degree = min(3, len(points) - 1)
    knots = _open_uniform_knots(len(points), degree)
    samples = [
        _de_boor(points, degree, knots, index / (sample_count - 1)) for index in range(sample_count)
    ]
    samples[0] = points[0]
    samples[-1] = points[-1]
    return samples


def _resample_polyline(path: Sequence[Point3], sample_count: int) -> list[Point3]:
    if len(path) < 2:
        return list(path)
    cumulative = [0.0]
    for index in range(1, len(path)):
        cumulative.append(cumulative[-1] + distance(path[index - 1], path[index]))
    total = cumulative[-1]
    if total <= 1e-12:
        return [path[0]] * sample_count
    samples: list[Point3] = []
    segment = 0
    for sample_index in range(sample_count):
        target = total * sample_index / (sample_count - 1)
        while segment < len(path) - 2 and cumulative[segment + 1] < target:
            segment += 1
        span = cumulative[segment + 1] - cumulative[segment]
        local = 0.0 if span <= 1e-12 else (target - cumulative[segment]) / span
        samples.append(lerp(path[segment], path[segment + 1], local))
    samples[0] = path[0]
    samples[-1] = path[-1]
    return samples


def smooth_path(
    scene: Scene,
    raw_path: Sequence[Point3],
    sample_spacing: float = 1.5,
) -> SmoothingResult:
    if sample_spacing <= 0:
        raise ValueError("sample_spacing must be positive")
    if not raw_path or not path_is_free(scene, raw_path):
        return SmoothingResult((), "not-run", False, 0, 0)

    shortcut = farthest_visible_shortcut(scene, raw_path)
    if len(shortcut) <= 2:
        path = tuple(shortcut)
        return SmoothingResult(path, "shortcut", path_is_free(scene, path), len(path), len(path))

    sample_count = max(16, math.ceil(polyline_length(shortcut) / sample_spacing) + 1)
    spline = sample_bspline(shortcut, sample_count)
    reference = _resample_polyline(shortcut, sample_count)
    for blend in (1.0, 0.75, 0.5, 0.25, 0.1):
        candidate = [
            lerp(reference_point, spline_point, blend)
            for reference_point, spline_point in zip(reference, spline, strict=True)
        ]
        candidate[0] = shortcut[0]
        candidate[-1] = shortcut[-1]
        if path_is_free(scene, candidate):
            return SmoothingResult(
                tuple(candidate),
                "bspline",
                True,
                len(shortcut),
                len(candidate),
                blend,
            )
    fallback = tuple(shortcut)
    return SmoothingResult(
        fallback,
        "shortcut-fallback",
        path_is_free(scene, fallback),
        len(shortcut),
        len(fallback),
    )
