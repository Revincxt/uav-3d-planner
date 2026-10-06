"""Exact piecewise-linear altitude lifting for horizontal-only post-processing.

These helpers do not certify collision safety. Callers must audit every lifted segment: an
XY chord that clears a building at its endpoints can intersect it at an intervening raw height.
"""

from __future__ import annotations

import math
from bisect import bisect_right
from collections.abc import Sequence
from itertools import pairwise

from uav3d.geometry import Point3, distance, lerp


def polyline_progress(path: Sequence[Point3]) -> list[float]:
    """Return absolute cumulative 3D chord lengths, including duplicate vertices."""

    if not path:
        return []
    progress = [0.0]
    for previous, current in pairwise(path):
        progress.append(progress[-1] + distance(previous, current))
    return progress


def _validated_progress(path: Sequence[Point3], progress: Sequence[float]) -> None:
    if not path or len(path) != len(progress):
        raise ValueError("a non-empty path and matching progress values are required")
    if any(not math.isfinite(value) for value in progress):
        raise ValueError("progress values must be finite")
    for index in range(1, len(progress)):
        if progress[index] < progress[index - 1]:
            raise ValueError("progress values must be non-decreasing")
        if progress[index] == progress[index - 1] and path[index] != path[index - 1]:
            raise ValueError("different vertices cannot occupy the same progress value")


def interpolate_polyline_progress(
    path: Sequence[Point3], progress: Sequence[float], parameter: float
) -> Point3:
    """Interpolate a polyline at an explicit monotone parameter (with endpoint clamping)."""

    _validated_progress(path, progress)
    if not math.isfinite(parameter):
        raise ValueError("interpolation parameter must be finite")
    return _interpolate_validated(path, progress, parameter)


def _interpolate_validated(
    path: Sequence[Point3], progress: Sequence[float], parameter: float
) -> Point3:
    if parameter <= progress[0]:
        return path[0]
    if parameter >= progress[-1]:
        return path[-1]
    following = bisect_right(progress, parameter)
    previous = following - 1
    fraction = (parameter - progress[previous]) / (progress[following] - progress[previous])
    return lerp(path[previous], path[following], fraction)


def lift_xy_polyline_preserving_reference_progress(
    candidate: Sequence[Point3],
    reference: Sequence[Point3],
    *,
    candidate_progress: Sequence[float] | None = None,
    reference_progress: Sequence[float] | None = None,
) -> list[Point3]:
    """Combine candidate XY with the *entire* original piecewise-linear altitude profile.

    By default, the parameter is reference cumulative 3D length and candidate cumulative
    length is normalized to the same endpoint range. Explicit parameters let callers retain
    their original knot correspondence instead. Output corresponds exactly to the sorted union
    of both sets of parameters; no coordinate-based deduplication discards a height peak, a
    trough, or a non-zero vertical segment. Only repeated identical parameter knots coalesce.
    """

    reference_parameters = (
        list(reference_progress) if reference_progress is not None else polyline_progress(reference)
    )
    _validated_progress(reference, reference_parameters)
    if candidate_progress is None:
        candidate_lengths = polyline_progress(candidate)
        _validated_progress(candidate, candidate_lengths)
        reference_span = reference_parameters[-1] - reference_parameters[0]
        if reference_span == 0:
            return list(reference)
        candidate_span = candidate_lengths[-1]
        if candidate_span == 0:
            candidate_parameters = [reference_parameters[0], reference_parameters[-1]]
            candidate = [candidate[0], candidate[0]]
        else:
            candidate_parameters = [
                reference_parameters[0] + reference_span * value / candidate_span
                for value in candidate_lengths
            ]
            candidate_parameters[0] = reference_parameters[0]
            candidate_parameters[-1] = reference_parameters[-1]
    else:
        candidate_parameters = list(candidate_progress)
    _validated_progress(candidate, candidate_parameters)
    if (
        candidate_parameters[0] != reference_parameters[0]
        or candidate_parameters[-1] != reference_parameters[-1]
    ):
        raise ValueError("candidate and reference progress must share their endpoint range")
    output: list[Point3] = []
    for parameter in sorted(set(candidate_parameters) | set(reference_parameters)):
        horizontal = _interpolate_validated(candidate, candidate_parameters, parameter)
        original = _interpolate_validated(reference, reference_parameters, parameter)
        output.append((horizontal[0], horizontal[1], original[2]))
    return output


__all__ = [
    "interpolate_polyline_progress",
    "lift_xy_polyline_preserving_reference_progress",
    "polyline_progress",
]
