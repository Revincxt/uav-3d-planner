"""Small, dependency-free 3D geometry primitives."""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from itertools import pairwise

Point3 = tuple[float, float, float]


def add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def subtract(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(v: Point3, factor: float) -> Point3:
    return (v[0] * factor, v[1] * factor, v[2] * factor)


def dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(v: Point3) -> float:
    return math.sqrt(dot(v, v))


def distance(a: Point3, b: Point3) -> float:
    return norm(subtract(a, b))


def lerp(a: Point3, b: Point3, t: float) -> Point3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def polyline_length(path: Sequence[Point3]) -> float:
    return sum(distance(a, b) for a, b in pairwise(path))


def as_point(value: Iterable[float]) -> Point3:
    values = tuple(float(component) for component in value)
    if len(values) != 3:
        raise ValueError("a 3D point must contain exactly three coordinates")
    return (values[0], values[1], values[2])


def almost_equal(a: Point3, b: Point3, tolerance: float = 1e-9) -> bool:
    return distance(a, b) <= tolerance
