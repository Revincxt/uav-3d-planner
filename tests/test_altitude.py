from __future__ import annotations

import math

import pytest

from uav3d.altitude import (
    interpolate_polyline_progress,
    lift_xy_polyline_preserving_reference_progress,
    polyline_progress,
)


def test_lift_keeps_all_height_extrema_and_candidate_knots() -> None:
    reference = [(0.0, 0.0, 10.0), (10.0, 10.0, 30.0), (20.0, 0.0, 5.0)]
    candidate = [(0.0, 0.0, -99.0), (5.0, 0.0, -99.0), (20.0, 0.0, -99.0)]
    output = lift_xy_polyline_preserving_reference_progress(
        candidate,
        reference,
        candidate_progress=[0.0, 0.5, 2.0],
        reference_progress=[0.0, 1.0, 2.0],
    )
    assert output == [
        (0.0, 0.0, 10.0),
        (5.0, 0.0, 20.0),
        (10.0, 0.0, 30.0),
        (20.0, 0.0, 5.0),
    ]


def test_absolute_3d_progress_retains_nonzero_pure_vertical_segment() -> None:
    reference = [(1.0, 2.0, 10.0), (1.0, 2.0, 20.0), (11.0, 2.0, 20.0)]
    assert polyline_progress(reference) == [0.0, 10.0, 20.0]
    output = lift_xy_polyline_preserving_reference_progress(
        reference, reference, candidate_progress=[0.0, 10.0, 20.0]
    )
    assert output == reference


def test_duplicate_parameter_knots_coalesce_but_equal_coordinates_do_not() -> None:
    reference = [(0.0, 0.0, 5.0), (0.0, 0.0, 5.0), (10.0, 0.0, 5.0)]
    candidate = [(0.0, 0.0, 100.0), (0.0, 0.0, 100.0), (10.0, 0.0, 100.0)]
    output = lift_xy_polyline_preserving_reference_progress(
        candidate,
        reference,
        candidate_progress=[0.0, 1.0, 2.0],
        reference_progress=[0.0, 1.0, 2.0],
    )
    assert len(output) == 3
    assert output[0] == output[1]
    default = lift_xy_polyline_preserving_reference_progress(candidate, reference)
    assert default == [reference[0], reference[-1]]


def test_fully_stationary_horizontal_candidate_still_retains_reference_profile() -> None:
    reference = [(1.0, 2.0, 10.0), (1.0, 2.0, 20.0)]
    output = lift_xy_polyline_preserving_reference_progress([(1.0, 2.0, 0.0)], reference)
    assert output == reference


def test_progress_interpolation_clamps_and_supports_repeated_equal_vertices() -> None:
    points = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (10.0, 0.0, 20.0)]
    assert interpolate_polyline_progress(points, [0.0, 0.0, 2.0], 1.0) == (5.0, 0.0, 10.0)
    assert interpolate_polyline_progress(points, [0.0, 0.0, 2.0], -1.0) == points[0]
    assert interpolate_polyline_progress(points, [0.0, 0.0, 2.0], 3.0) == points[-1]


@pytest.mark.parametrize("progress", [[0.0], [1.0, 0.0], [0.0, math.nan], [0.0, 0.0]])
def test_invalid_progress_is_rejected(progress: list[float]) -> None:
    with pytest.raises(ValueError):
        interpolate_polyline_progress([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], progress, 0.5)


def test_explicit_progress_requires_matching_endpoint_range() -> None:
    points = [(0.0, 0.0, 5.0), (10.0, 0.0, 10.0)]
    with pytest.raises(ValueError, match="endpoint range"):
        lift_xy_polyline_preserving_reference_progress(
            points, points, candidate_progress=[0.0, 2.0], reference_progress=[0.0, 1.0]
        )
