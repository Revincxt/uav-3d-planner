"""Local clamped quintic B-spline spans, with hard anchors and exact chord checks.

A single clamped span is evaluated in its equivalent Bernstein form. XY position,
first derivative and zero second derivative match at span/line joins. Z is lifted
from the original piecewise-linear parameter profile, never from spline controls.
The certified/executed result is the sampled polyline, not the ideal continuous spline.
"""

from __future__ import annotations

import math
from bisect import bisect_left, bisect_right
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from itertools import pairwise

from uav3d.geometry import Point3, distance, lerp

SegmentCheck = Callable[[Point3, Point3, float, float], bool]


@dataclass(frozen=True, slots=True)
class HorizontalCurveResult:
    points: tuple[Point3, ...]
    parameters: tuple[float, ...]
    rounded_corners: int
    turn_scale_m: float | None


@dataclass(frozen=True, slots=True)
class _Span:
    start: float
    end: float
    controls: tuple[Point3, ...]
    steps: int

    def xy(self, parameter: float) -> tuple[float, float]:
        u = max(0.0, min(1.0, (parameter - self.start) / (self.end - self.start)))
        coefficients = [math.comb(5, i) * u**i * (1 - u) ** (5 - i) for i in range(6)]
        return cast_xy(
            tuple(
                math.fsum(
                    weight * point[axis]
                    for weight, point in zip(coefficients, self.controls, strict=True)
                )
                for axis in range(2)
            )
        )


def cast_xy(values: tuple[float, ...]) -> tuple[float, float]:
    return values[0], values[1]


def _span(
    a: Point3,
    b: Point3,
    start: float,
    end: float,
    initial_velocity: tuple[float, float],
    final_velocity: tuple[float, float],
    spacing: float,
    angle: float,
) -> _Span:
    duration = end - start
    controls = (
        (a[0], a[1], 0.0),
        (a[0] + initial_velocity[0] * duration / 5, a[1] + initial_velocity[1] * duration / 5, 0.0),
        (
            a[0] + initial_velocity[0] * duration * 2 / 5,
            a[1] + initial_velocity[1] * duration * 2 / 5,
            0.0,
        ),
        (
            b[0] - final_velocity[0] * duration * 2 / 5,
            b[1] - final_velocity[1] * duration * 2 / 5,
            0.0,
        ),
        (b[0] - final_velocity[0] * duration / 5, b[1] - final_velocity[1] * duration / 5, 0.0),
        (b[0], b[1], 0.0),
    )
    control_length = math.fsum(distance(a, b) for a, b in pairwise(controls))
    # Dense headings as well as dense positions: a short turn must not become one chord.
    steps = max(12, math.ceil(control_length / spacing), math.ceil(math.degrees(angle)))
    return _Span(start, end, controls, steps)


def _parameters(original: Sequence[float], extra: Sequence[float]) -> list[float]:
    values = set(original)
    for value in extra:
        insertion = bisect_left(original, value)
        neighbors = original[max(0, insertion - 1) : insertion + 1]
        if not any(
            abs(value - old) <= max(1e-12, 32 * math.ulp(value), 32 * math.ulp(old))
            for old in neighbors
        ):
            values.add(value)
    return sorted(values)


def smooth_horizontal_curves(
    points: Sequence[Point3],
    parameters: Sequence[float],
    check: SegmentCheck,
    *,
    protected: frozenset[float] = frozenset(),
    turn_scale_m: float = 60.0,
    sample_spacing_m: float = 2.0,
    start_direction: tuple[float, float] | None = None,
    end_direction: tuple[float, float] | None = None,
) -> HorizontalCurveResult:
    """Shrink only a blocked corner; keep safe curves elsewhere and every height knot.

    Protected anchors are interpolated by two spans sharing their tangent. Other
    corners are cut with one span. Vertical-only intervals are never displaced.
    Boundary tangents optionally join online replan prefixes and service-leg seams.
    """
    if not math.isfinite(turn_scale_m) or turn_scale_m <= 0:
        raise ValueError("turn scale must be finite and positive")
    if not math.isfinite(sample_spacing_m) or sample_spacing_m <= 0:
        raise ValueError("curve sample spacing must be finite and positive")
    if len(points) != len(parameters) or not points:
        raise ValueError("curve points and parameters must have equal nonzero length")
    if any(not math.isfinite(p) for p in parameters) or any(
        b <= a for a, b in pairwise(parameters)
    ):
        raise ValueError("curve parameters must be finite and strictly increasing")
    if not all(
        check(a, b, u, v)
        for (a, b), (u, v) in zip(pairwise(points), pairwise(parameters), strict=True)
    ):
        raise ValueError("curve input must be collision-free")

    def reference(parameter: float) -> Point3:
        index = max(0, bisect_right(parameters, parameter) - 1)
        if index >= len(points) - 1:
            return points[-1]
        if parameter == parameters[index]:
            return points[index]
        return lerp(
            points[index],
            points[index + 1],
            (parameter - parameters[index]) / (parameters[index + 1] - parameters[index]),
        )

    # Drop only redundant XY knots, not a vertical column or a required stop.
    anchors: list[int] = []
    for index, point in enumerate(points):
        while len(anchors) >= 2:
            left, middle = anchors[-2:]
            a = (points[middle][0] - points[left][0], points[middle][1] - points[left][1])
            b = (point[0] - points[middle][0], point[1] - points[middle][1])
            lengths = math.hypot(*a) * math.hypot(*b)
            if parameters[middle] in protected or lengths <= 1e-12:
                break
            if a[0] * b[0] + a[1] * b[1] <= 0 or abs(a[0] * b[1] - a[1] * b[0]) > lengths * 1e-8:
                break
            anchors.pop()
        anchors.append(index)

    patches: list[_Span] = []
    scales: list[float] = []
    corners = 0

    def accept(candidate: list[_Span]) -> bool:
        samples = _parameters(
            parameters,
            [
                span.start + (span.end - span.start) * i / span.steps
                for span in candidate
                for i in range(span.steps + 1)
            ],
        )
        samples = [u for u in samples if candidate[0].start <= u <= candidate[-1].end]
        lifted = []
        for u in samples:
            span = candidate[0] if u <= candidate[0].end else candidate[-1]
            xy = span.xy(u)
            lifted.append((xy[0], xy[1], reference(u)[2]))
        return all(
            check(a, b, u, v)
            for (a, b), (u, v) in zip(pairwise(lifted), pairwise(samples), strict=True)
        )

    for left, middle, right in zip(anchors, anchors[1:], anchors[2:], strict=False):
        previous, c, following = points[left], points[middle], points[right]
        incoming = (c[0] - previous[0], c[1] - previous[1])
        outgoing = (following[0] - c[0], following[1] - c[1])
        length_a, length_b = math.hypot(*incoming), math.hypot(*outgoing)
        if min(length_a, length_b) <= 1e-9:
            continue
        cosine = (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]) / (length_a * length_b)
        angle = math.acos(max(-1.0, min(1.0, cosine)))
        if angle < math.radians(0.2) or angle > math.radians(175):
            continue
        tangent = math.tan(angle / 2)
        trim = min(turn_scale_m * tangent, length_a * 0.45, length_b * 0.45)
        v0 = cast_xy(tuple(value / (parameters[middle] - parameters[left]) for value in incoming))
        v1 = cast_xy(tuple(value / (parameters[right] - parameters[middle]) for value in outgoing))
        vm = ((v0[0] + v1[0]) / 2, (v0[1] + v1[1]) / 2)
        for factor in (1.0, 0.75, 0.5, 0.25, 0.125, 0.0625, 0.03125):
            extent = trim * factor
            if extent < 0.1:
                break
            entry = parameters[middle] - (parameters[middle] - parameters[left]) * extent / length_a
            exit_ = (
                parameters[middle] + (parameters[right] - parameters[middle]) * extent / length_b
            )
            e, x = reference(entry), reference(exit_)
            if parameters[middle] in protected:
                candidate = [
                    _span(e, c, entry, parameters[middle], v0, vm, sample_spacing_m, angle),
                    _span(c, x, parameters[middle], exit_, vm, v1, sample_spacing_m, angle),
                ]
            else:
                candidate = [_span(e, x, entry, exit_, v0, v1, sample_spacing_m, angle)]
            if accept(candidate):
                patches.extend(candidate)
                scales.append(extent / tangent)
                corners += 1
                break

    # A replan starts along the existing aircraft heading, not the new search chord.
    for beginning, direction in [(True, start_direction), (False, end_direction)]:
        if direction is None or len(anchors) < 2 or math.hypot(*direction) <= 1e-9:
            continue
        left, right = anchors[:2] if beginning else anchors[-2:]
        vector = (points[right][0] - points[left][0], points[right][1] - points[left][1])
        length = math.hypot(*vector)
        if length <= 1e-9:
            continue
        velocity = cast_xy(tuple(v / (parameters[right] - parameters[left]) for v in vector))
        speed = math.hypot(*velocity)
        desired = cast_xy(tuple(v * speed / math.hypot(*direction) for v in direction))
        angle = math.acos(
            max(
                -1.0,
                min(1.0, sum(a * b for a, b in zip(velocity, desired, strict=True)) / speed**2),
            )
        )
        if angle < math.radians(0.2) or angle > math.radians(175):
            continue
        for factor in (1.0, 0.5, 0.25, 0.125, 0.0625):
            fraction = min(turn_scale_m / length, 0.45) * factor
            u = (
                parameters[left]
                if beginning
                else parameters[right] - (parameters[right] - parameters[left]) * fraction
            )
            v = (
                parameters[left] + (parameters[right] - parameters[left]) * fraction
                if beginning
                else parameters[right]
            )
            candidate = [
                _span(
                    reference(u),
                    reference(v),
                    u,
                    v,
                    desired if beginning else velocity,
                    velocity if beginning else desired,
                    sample_spacing_m,
                    angle,
                )
            ]
            if accept(candidate):
                patches.extend(candidate)
                break

    patches.sort(key=lambda span: span.start)
    starts = [span.start for span in patches]
    output_parameters = _parameters(
        parameters,
        [
            span.start + (span.end - span.start) * i / span.steps
            for span in patches
            for i in range(span.steps + 1)
        ],
    )
    output = []
    for u in output_parameters:
        point = reference(u)
        index = bisect_right(starts, u) - 1
        if index >= 0 and u <= patches[index].end:
            xy = patches[index].xy(u)
            point = (xy[0], xy[1], point[2])
        # Hard anchors and endpoints remain bit-for-bit original coordinates.
        if u in protected or u == parameters[0] or u == parameters[-1]:
            point = reference(u)
        output.append(point)
    if not all(
        check(a, b, u, v)
        for (a, b), (u, v) in zip(pairwise(output), pairwise(output_parameters), strict=True)
    ):
        raise ValueError("combined horizontal curve failed its final collision audit")
    return HorizontalCurveResult(
        tuple(output), tuple(output_parameters), corners, min(scales, default=None)
    )
