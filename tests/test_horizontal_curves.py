"""Local turn curves keep anchors, height knots and exact collision gates."""

from __future__ import annotations

import math
from bisect import bisect_right
from itertools import pairwise

import pytest

from uav3d.curve_timing import smooth_timed_horizontal_curves
from uav3d.dynamic import DynamicScenario
from uav3d.geometry import Point3, lerp
from uav3d.horizontal_curves import _heading_samples, _span, smooth_horizontal_curves
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.scene import Bounds3D, Scene


def maximum_xy_turn(points: tuple[Point3, ...]) -> float:
    turns = []
    for a, b, c in zip(points, points[1:], points[2:], strict=False):
        incoming = (b[0] - a[0], b[1] - a[1])
        outgoing = (c[0] - b[0], c[1] - b[1])
        denominator = math.hypot(*incoming) * math.hypot(*outgoing)
        if denominator > 1e-12:
            turns.append(
                math.degrees(
                    math.acos(
                        max(
                            -1,
                            min(
                                1,
                                sum(x * y for x, y in zip(incoming, outgoing, strict=True))
                                / denominator,
                            ),
                        )
                    )
                )
            )
    return max(turns, default=0)


def free(*_args: object) -> bool:
    return True


def position_at(path: TimedPath, time_s: float) -> Point3:
    times = [waypoint.time_s for waypoint in path.waypoints]
    index = min(len(times) - 2, max(0, bisect_right(times, time_s) - 1))
    return lerp(
        path.waypoints[index].position,
        path.waypoints[index + 1].position,
        (time_s - times[index]) / (times[index + 1] - times[index]),
    )


@pytest.mark.parametrize("protected", [False, True])
def test_a_right_angle_becomes_dense_tangent_connected_curves(protected: bool) -> None:
    points = ((0.0, 0.0, 10.0), (100.0, 0.0, 10.0), (100.0, 100.0, 10.0))
    result = smooth_horizontal_curves(
        points, [0.0, 10.0, 20.0], free, protected=frozenset({10.0}) if protected else frozenset()
    )
    assert result.rounded_corners == 1
    assert maximum_xy_turn(result.points) < 4
    assert result.points[0] == points[0] and result.points[-1] == points[-1]
    assert all(point[2] == 10 for point in result.points)
    if protected:
        assert result.points[result.parameters.index(10.0)] == points[1]
    else:
        assert points[1] not in result.points


def test_quintic_span_matches_tangents_and_zero_second_derivatives() -> None:
    span = _span(
        (0.0, 0.0, 0.0), (30.0, 20.0, 0.0), 2.0, 12.0, (4.0, 0.0), (0.0, 4.0), 2, math.pi / 2
    )
    controls = span.controls
    for axis in range(2):
        assert 5 * (controls[1][axis] - controls[0][axis]) / 10 == pytest.approx((4.0, 0.0)[axis])
        assert 5 * (controls[5][axis] - controls[4][axis]) / 10 == pytest.approx((0.0, 4.0)[axis])
        assert controls[0][axis] - 2 * controls[1][axis] + controls[2][axis] == pytest.approx(0)
        assert controls[3][axis] - 2 * controls[4][axis] + controls[5][axis] == pytest.approx(0)


def test_heading_refinement_does_not_bisect_stationary_tangents_into_numeric_noise() -> None:
    span = _span(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        440.0,
        442.0,
        (0.1, 0.0),
        (-0.1, 0.0),
        2,
        math.pi,
    )
    samples = _heading_samples(span)
    assert samples[0] == span.start and samples[-1] == span.end
    assert len(samples) < 256
    assert min(b - a for a, b in pairwise(samples)) > 1e-5


def test_height_profile_keeps_every_knot_peak_and_trough_exactly() -> None:
    points = (
        (0.0, 0.0, 10.0),
        (50.0, 0.0, 60.0),
        (100.0, 0.0, 30.0),
        (100.0, 50.0, 80.0),
        (100.0, 100.0, 20.0),
    )
    parameters = [0.0, 5.0, 10.0, 15.0, 20.0]
    result = smooth_horizontal_curves(points, parameters, free, protected=frozenset({10.0}))
    for point, u in zip(points, parameters, strict=True):
        assert result.points[result.parameters.index(u)][2] == point[2]
    for point, u in zip(result.points, result.parameters, strict=True):
        i = min(len(points) - 2, bisect_right(parameters, u) - 1)
        expected = lerp(
            points[i], points[i + 1], (u - parameters[i]) / (parameters[i + 1] - parameters[i])
        )
        assert point[2] == expected[2]


def test_a_blocked_corner_does_not_cancel_the_other_safe_curves() -> None:
    points = (
        (0.0, 0.0, 10.0),
        (100.0, 0.0, 10.0),
        (100.0, 100.0, 10.0),
        (200.0, 100.0, 10.0),
        (200.0, 200.0, 10.0),
    )

    def check(a: Point3, b: Point3, _u: float, _v: float) -> bool:
        # A corridor that permits the original first corner but no inner cut there.
        return not any(60 < p[0] < 100 - 1e-8 and 1e-8 < p[1] < 40 for p in (a, b))

    result = smooth_horizontal_curves(points, [0.0, 10.0, 20.0, 30.0, 40.0], check)
    assert result.rounded_corners == 2
    assert points[1] in result.points
    assert points[2] not in result.points and points[3] not in result.points
    assert all(
        check(a, b, u, v)
        for (a, b), (u, v) in zip(pairwise(result.points), pairwise(result.parameters), strict=True)
    )


def test_vertical_only_interval_stays_on_its_original_column() -> None:
    points = ((0.0, 0.0, 10.0), (100.0, 0.0, 10.0), (100.0, 0.0, 20.0), (100.0, 100.0, 20.0))
    result = smooth_horizontal_curves(points, [0.0, 10.0, 15.0, 25.0], free)
    assert result.points == points


def test_boundary_curve_joins_online_heading_without_changing_endpoints() -> None:
    points = ((0.0, 0.0, 10.0), (100.0, 0.0, 10.0))
    result = smooth_horizontal_curves(points, [0.0, 10.0], free, start_direction=(1.0, 1.0))
    first = result.points[1]
    assert math.degrees(math.atan2(first[1], first[0])) == pytest.approx(45, abs=1)
    assert result.points[0] == points[0] and result.points[-1] == points[-1]
    assert maximum_xy_turn(result.points) < 6


def test_timed_curve_passes_through_service_point_and_keeps_the_complete_hold() -> None:
    path = TimedPath(
        (
            TimedWaypoint(0.0, (0.0, 0.0, 10.0), "start"),
            TimedWaypoint(10.0, (100.0, 0.0, 20.0), "move"),
            TimedWaypoint(16.0, (100.0, 0.0, 20.0), "wait"),
            TimedWaypoint(26.0, (100.0, 100.0, 10.0), "move"),
        )
    )
    scene = Scene(
        "curve",
        "Curve",
        Bounds3D((-100.0, -100.0, 0.0), (300.0, 300.0, 100.0)),
        path.positions[0],
        path.positions[-1],
    )
    scenario = DynamicScenario("curve", "Curve", scene)
    result, count, _ = smooth_timed_horizontal_curves(scenario, path, 60, 2)
    assert count == 1 and result.is_safe(scenario)
    assert position_at(result, 10.0) == path.positions[1]
    assert position_at(result, 16.0) == path.positions[1]
    assert position_at(result, 13.0) == path.positions[1]
    assert maximum_xy_turn(result.positions) < 4
    for i in range(261):
        t = i / 10
        assert position_at(result, t)[2] == pytest.approx(position_at(path, t)[2], abs=1e-10)


@pytest.mark.parametrize("field,value", [("turn_scale_m", 0), ("sample_spacing_m", math.nan)])
def test_rejects_invalid_curve_parameters(field: str, value: float) -> None:
    with pytest.raises(ValueError):
        smooth_horizontal_curves(
            [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)], [0.0, 1.0], free, **{field: value}
        )


def test_decimal_service_clock_does_not_become_a_roundoff_movement() -> None:
    arrival = 1.2345678912345
    path = TimedPath(
        (
            TimedWaypoint(0.0, (0.0, 0.0, 10.0), "start"),
            TimedWaypoint(arrival, (100.0, 0.0, 10.0), "move"),
            TimedWaypoint(arrival + 6.0, (100.0, 0.0, 10.0), "wait"),
            TimedWaypoint(arrival + 20.0, (100.0, 100.0, 10.0), "move"),
        )
    )
    scene = Scene(
        "decimal",
        "Decimal",
        Bounds3D((-100.0, -100.0, 0.0), (300.0, 300.0, 100.0)),
        path.start,
        path.goal,
    )
    result, count, _ = smooth_timed_horizontal_curves(
        DynamicScenario("decimal", "Decimal", scene), path, 60, 2
    )
    assert count == 1
    assert result.wait_time_s == path.wait_time_s
    assert result.is_safe(DynamicScenario("decimal", "Decimal", scene))
