"""Shared, collision-certified post-processing for every planner."""

from __future__ import annotations

import math
from bisect import bisect_right
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

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
    altitude_policy: str | None = None
    altitude_profile_max_error: float | None = None
    altitude_progress: tuple[float, ...] = ()

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "path": [list(point) for point in self.path],
            "method": self.method,
            "collision_free": self.collision_free,
            "shortcut_waypoints": self.shortcut_waypoints,
            "sample_count": self.sample_count,
            "spline_blend": self.spline_blend,
        }
        if self.altitude_policy is not None:
            result.update(
                altitude_policy=self.altitude_policy,
                altitude_profile_max_error=self.altitude_profile_max_error,
                altitude_progress=list(self.altitude_progress),
            )
        return result


def remove_duplicate_points(path: Sequence[Point3]) -> list[Point3]:
    cleaned: list[Point3] = []
    for point in path:
        if not cleaned or not almost_equal(cleaned[-1], point):
            cleaned.append(point)
    return cleaned


def farthest_visible_shortcut(
    scene: Scene, path: Sequence[Point3], *, preserve_altitude: bool = False
) -> list[Point3]:
    if preserve_altitude:
        return _altitude_preserving_shortcut(scene, path, optimize=False)[0]
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


def shortest_visible_shortcut(
    scene: Scene, path: Sequence[Point3], *, preserve_altitude: bool = False
) -> list[Point3]:
    """Find the shortest collision-free, order-preserving subsequence of existing waypoints.

    The farthest visible *index* can require a larger detour than a closer intermediate waypoint.
    This directed acyclic visibility graph minimizes actual 3D length, using fewer waypoints as
    a deterministic tie-break. Every retained chord uses the unchanged exact collision predicate;
    no new obstacle model, point displacement, or unverified continuous curve is introduced.
    """

    if preserve_altitude:
        return _altitude_preserving_shortcut(scene, path, optimize=True)[0]
    points = remove_duplicate_points(path)
    if len(points) <= 2:
        return points
    costs = [math.inf] * len(points)
    counts = [len(points) + 1] * len(points)
    parents = [-1] * len(points)
    costs[0], counts[0] = 0.0, 1
    for following in range(1, len(points)):
        for current in range(following):
            candidate_cost = costs[current] + distance(points[current], points[following])
            candidate_count = counts[current] + 1
            improves = candidate_cost < costs[following] - 1e-9
            ties_with_fewer = (
                abs(candidate_cost - costs[following]) <= 1e-9
                and candidate_count < counts[following]
            )
            if not (improves or ties_with_fewer):
                continue
            if segment_is_free(scene, points[current], points[following]):
                costs[following] = candidate_cost
                counts[following] = candidate_count
                parents[following] = current
    if parents[-1] == -1:
        return list(points)  # The caller audits the original path before smoothing.
    indices = [len(points) - 1]
    while indices[-1] != 0:
        indices.append(parents[indices[-1]])
    return [points[index] for index in reversed(indices)]


def _altitude_reference(path: Sequence[Point3]) -> tuple[list[Point3], list[float]]:
    """Use the original 3D progress, not the changed curve's arc length, for Z.

    Only exactly repeated points are discarded: a tiny but nonzero altitude excursion must
    not disappear under the ordinary approximate waypoint deduplication tolerance.
    """

    points: list[Point3] = []
    for point in path:
        if not points or point != points[-1]:
            points.append(point)
    if not points:
        return [], []
    cumulative = [0.0]
    for previous, following in pairwise(points):
        cumulative.append(cumulative[-1] + distance(previous, following))
    total = cumulative[-1]
    progress = [value / total for value in cumulative] if total > 0.0 else [0.0] * len(points)
    return points, progress


def _point_at_progress(
    points: Sequence[Point3], progress: Sequence[float], parameter: float
) -> Point3:
    if parameter <= progress[0]:
        return points[0]
    if parameter >= progress[-1]:
        return points[-1]
    current = bisect_right(progress, parameter) - 1
    fraction = (parameter - progress[current]) / (progress[current + 1] - progress[current])
    return lerp(points[current], points[current + 1], fraction)


def _altitude_chord(
    points: Sequence[Point3], progress: Sequence[float], current: int, following: int
) -> list[Point3]:
    """Shortcut XY while retaining every intervening original height breakpoint."""

    span = progress[following] - progress[current]
    chord: list[Point3] = []
    for index in range(current, following + 1):
        fraction = (progress[index] - progress[current]) / span
        horizontal = lerp(points[current], points[following], fraction)
        chord.append((horizontal[0], horizontal[1], points[index][2]))
    chord[0], chord[-1] = points[current], points[following]
    return chord


def _altitude_preserving_shortcut(
    scene: Scene, path: Sequence[Point3], *, optimize: bool
) -> tuple[list[Point3], list[float], list[Point3]]:
    points, progress = _altitude_reference(path)
    if len(points) <= 2:
        return points, progress, points
    if optimize:
        costs = [math.inf] * len(points)
        counts = [len(points) + 1] * len(points)
        parents = [-1] * len(points)
        costs[0], counts[0] = 0.0, 1
        for following in range(1, len(points)):
            for current in range(following):
                horizontal_distance = math.hypot(
                    points[following][0] - points[current][0],
                    points[following][1] - points[current][1],
                )
                candidate_cost = costs[current] + horizontal_distance
                candidate_count = counts[current] + 1
                improves = candidate_cost < costs[following] - 1e-9
                ties_with_fewer = (
                    abs(candidate_cost - costs[following]) <= 1e-9
                    and candidate_count < counts[following]
                )
                if not (improves or ties_with_fewer):
                    continue
                if path_is_free(scene, _altitude_chord(points, progress, current, following)):
                    costs[following] = candidate_cost
                    counts[following] = candidate_count
                    parents[following] = current
        if parents[-1] == -1:
            return points, progress, points
        indices = [len(points) - 1]
        while indices[-1] != 0:
            indices.append(parents[indices[-1]])
        indices.reverse()
    else:
        indices = [0]
        while indices[-1] < len(points) - 1:
            current = indices[-1]
            following = current + 1
            for candidate in range(len(points) - 1, current, -1):
                if path_is_free(scene, _altitude_chord(points, progress, current, candidate)):
                    following = candidate
                    break
            indices.append(following)
    shortcut = [points[0]]
    for current, following in pairwise(indices):
        shortcut.extend(_altitude_chord(points, progress, current, following)[1:])
    return shortcut, progress, [points[index] for index in indices]


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
    *,
    optimize_shortcuts: bool = False,
    preserve_altitude: bool = False,
    round_corners: bool = True,
) -> SmoothingResult:
    if sample_spacing <= 0:
        raise ValueError("sample_spacing must be positive")
    if not raw_path or not path_is_free(scene, raw_path):
        return SmoothingResult((), "not-run", False, 0, 0)
    if preserve_altitude:
        return _smooth_altitude_path(
            scene, raw_path, sample_spacing, optimize_shortcuts, round_corners
        )

    shortcut = (
        shortest_visible_shortcut(scene, raw_path)
        if optimize_shortcuts
        else farthest_visible_shortcut(scene, raw_path)
    )
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


def _smooth_altitude_path(
    scene: Scene,
    raw_path: Sequence[Point3],
    sample_spacing: float,
    optimize_shortcuts: bool,
    round_corners: bool = True,
) -> SmoothingResult:
    """Optimize only XY; lift Z from the unchanged original progress profile.

    Both every original height node and every horizontal sampling node are present in the
    output. Consequently linear execution between samples preserves every original Z slope
    and extremum in this explicit parameter, without a spline-induced altitude overshoot.
    This is deliberately *not* a claim about re-normalizing the resulting curve's arc length.
    """

    reference, reference_progress = _altitude_reference(raw_path)
    shortcut, progress, anchors = _altitude_preserving_shortcut(
        scene, reference, optimize=optimize_shortcuts
    )

    def outcome(
        path: Sequence[Point3],
        method: str,
        parameters: Sequence[float],
        blend: float | None = None,
    ) -> SmoothingResult:
        profile_error = max(
            (
                abs(point[2] - _point_at_progress(reference, reference_progress, parameter)[2])
                for point, parameter in zip(path, parameters, strict=True)
            ),
            default=0.0,
        )
        return SmoothingResult(
            tuple(path),
            method,
            path_is_free(scene, path),
            len(anchors),
            len(path),
            blend,
            "preserve-raw-altitude-profile-v1",
            profile_error,
            tuple(parameters),
        )

    if len(anchors) <= 2 or not round_corners:
        return outcome(shortcut, "shortcut", progress)

    sample_count = max(16, math.ceil(polyline_length(shortcut) / sample_spacing) + 1)
    parameters = sorted(
        set(progress) | {index / (sample_count - 1) for index in range(sample_count)}
    )
    horizontal_controls = [(point[0], point[1], 0.0) for point in anchors]
    degree = min(3, len(horizontal_controls) - 1)
    knots = _open_uniform_knots(len(horizontal_controls), degree)
    horizontal_spline = [
        _de_boor(horizontal_controls, degree, knots, parameter) for parameter in parameters
    ]
    horizontal_reference = [_point_at_progress(shortcut, progress, value) for value in parameters]
    altitudes = [
        _point_at_progress(reference, reference_progress, value)[2] for value in parameters
    ]
    for blend in (1.0, 0.75, 0.5, 0.25, 0.1):
        candidate = [
            (
                original[0] + (spline[0] - original[0]) * blend,
                original[1] + (spline[1] - original[1]) * blend,
                altitude,
            )
            for original, spline, altitude in zip(
                horizontal_reference, horizontal_spline, altitudes, strict=True
            )
        ]
        candidate[0], candidate[-1] = reference[0], reference[-1]
        if path_is_free(scene, candidate):
            return outcome(candidate, "bspline", parameters, blend)
    return outcome(shortcut, "shortcut-fallback", progress)
