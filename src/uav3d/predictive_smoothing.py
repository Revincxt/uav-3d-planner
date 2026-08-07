"""Collision-certified geometric smoothing for timestamped predictive paths.

The smoother rounds eligible polyline corners with sampled circular fillets.  The returned
trajectory is still a :class:`~uav3d.predictive.TimedPath`: execution between consecutive output
waypoints is linear, and every such space-time segment is checked by the existing continuous
collision predicate.  Consequently, this module certifies the dense polyline, not the ideal
circle from which its samples were obtained.  It intentionally makes no C2, acceleration, or
jerk claim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from uav3d.dynamic import DynamicScenario
from uav3d.geometry import Point3, add, distance, dot, lerp, scale, subtract
from uav3d.kinematics import DiscreteKinematicDiagnostics, diagnose_timed_path_kinematics
from uav3d.predictive import TimedPath, TimedWaypoint

_GEOMETRY_EPSILON = 1e-9
_MIN_TURN_RADIANS = math.radians(1.0)
_MAX_TURN_RADIANS = math.radians(175.0)
_TRIM_FRACTION = 0.45


@dataclass(frozen=True, slots=True)
class PredictiveSmoothingResult:
    """A certified dense-polyline result and honest smoothing metadata."""

    timed_path: TimedPath
    method: str
    applied: bool
    certified: bool
    raw_waypoint_count: int
    output_waypoint_count: int
    rounded_corners: int
    requested_radius_m: float
    applied_radius_m: float | None
    sample_spacing_m: float
    max_turn_before_deg: float
    max_turn_after_deg: float
    raw_kinematics: DiscreteKinematicDiagnostics
    output_kinematics: DiscreteKinematicDiagnostics

    @property
    def collision_certified(self) -> bool:
        """Compatibility-safe explicit name for the dense-polyline collision certificate."""

        return self.certified

    def to_dict(self) -> dict[str, object]:
        return {
            "timed_path": self.timed_path.to_dict(),
            "method": self.method,
            "applied": self.applied,
            "certified": self.certified,
            "collision_certified": self.collision_certified,
            "collision_certification_scope": "dense-piecewise-linear-space-time-path",
            "raw_waypoint_count": self.raw_waypoint_count,
            "output_waypoint_count": self.output_waypoint_count,
            "rounded_corners": self.rounded_corners,
            "requested_radius_m": self.requested_radius_m,
            "applied_radius_m": self.applied_radius_m,
            "sample_spacing_m": self.sample_spacing_m,
            "max_turn_before_deg": self.max_turn_before_deg,
            "max_turn_after_deg": self.max_turn_after_deg,
            "kinematic_diagnostics": {
                "status": "discrete-diagnostic-only",
                "continuous_dynamics_certified": False,
                "raw": self.raw_kinematics.to_dict(),
                "output": self.output_kinematics.to_dict(),
            },
        }


@dataclass(frozen=True, slots=True)
class _Fillet:
    entry: Point3
    exit: Point3
    center: Point3
    axis: Point3
    angle_rad: float
    radius_m: float


def _cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(vector: Point3) -> float:
    return math.sqrt(dot(vector, vector))


def _unit(vector: Point3) -> Point3:
    length = _norm(vector)
    if length <= _GEOMETRY_EPSILON:
        raise ValueError("cannot normalize a zero-length vector")
    return scale(vector, 1.0 / length)


def _clamped_cosine(a: Point3, b: Point3) -> float:
    return max(-1.0, min(1.0, dot(a, b)))


def _fillet(
    previous: Point3,
    corner: Point3,
    following: Point3,
    requested_radius_m: float,
) -> _Fillet | None:
    incoming_vector = subtract(corner, previous)
    outgoing_vector = subtract(following, corner)
    incoming_length = _norm(incoming_vector)
    outgoing_length = _norm(outgoing_vector)
    if incoming_length <= _GEOMETRY_EPSILON or outgoing_length <= _GEOMETRY_EPSILON:
        return None

    incoming = scale(incoming_vector, 1.0 / incoming_length)
    outgoing = scale(outgoing_vector, 1.0 / outgoing_length)
    cosine = _clamped_cosine(incoming, outgoing)
    angle = math.acos(cosine)
    if angle <= _MIN_TURN_RADIANS or angle >= _MAX_TURN_RADIANS:
        return None

    tangent_factor = math.tan(angle / 2.0)
    trim = min(
        requested_radius_m * tangent_factor,
        _TRIM_FRACTION * incoming_length,
        _TRIM_FRACTION * outgoing_length,
    )
    if trim <= _GEOMETRY_EPSILON or tangent_factor <= _GEOMETRY_EPSILON:
        return None
    radius = trim / tangent_factor
    entry = subtract(corner, scale(incoming, trim))
    exit_point = add(corner, scale(outgoing, trim))

    inward = subtract(outgoing, scale(incoming, cosine))
    if _norm(inward) <= _GEOMETRY_EPSILON:
        return None
    center = add(entry, scale(_unit(inward), radius))
    entry_radius = subtract(entry, center)
    exit_radius = subtract(exit_point, center)
    axis_vector = _cross(entry_radius, exit_radius)
    if _norm(axis_vector) <= _GEOMETRY_EPSILON:
        return None
    axis = _unit(axis_vector)
    observed_angle = math.acos(
        max(
            -1.0,
            min(
                1.0,
                dot(entry_radius, exit_radius) / (_norm(entry_radius) * _norm(exit_radius)),
            ),
        )
    )
    return _Fillet(entry, exit_point, center, axis, observed_angle, radius)


def _append_line(points: list[Point3], end: Point3, sample_spacing_m: float) -> None:
    start = points[-1]
    length = distance(start, end)
    if length <= _GEOMETRY_EPSILON:
        points[-1] = end
        return
    steps = max(1, math.ceil(length / sample_spacing_m))
    points.extend(lerp(start, end, index / steps) for index in range(1, steps + 1))
    points[-1] = end


def _rotate_about_axis(vector: Point3, axis: Point3, angle_rad: float) -> Point3:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return add(
        add(scale(vector, cosine), scale(_cross(axis, vector), sine)),
        scale(axis, dot(axis, vector) * (1.0 - cosine)),
    )


def _append_arc(points: list[Point3], fillet: _Fillet, sample_spacing_m: float) -> None:
    radial = subtract(fillet.entry, fillet.center)
    arc_length = fillet.radius_m * fillet.angle_rad
    steps = max(2, math.ceil(arc_length / sample_spacing_m))
    for index in range(1, steps + 1):
        rotated = _rotate_about_axis(radial, fillet.axis, fillet.angle_rad * index / steps)
        points.append(add(fillet.center, rotated))
    points[-1] = fillet.exit


def _rounded_geometry(
    points: tuple[Point3, ...],
    radius_m: float,
    sample_spacing_m: float,
) -> tuple[list[Point3], int, float | None]:
    if len(points) < 3:
        return list(points), 0, None
    fillets = [
        _fillet(points[index - 1], points[index], points[index + 1], radius_m)
        for index in range(1, len(points) - 1)
    ]
    rounded = sum(item is not None for item in fillets)
    if not rounded:
        return list(points), 0, None

    output = [points[0]]
    applied_radii: list[float] = []
    for index, item in enumerate(fillets, start=1):
        if item is None:
            _append_line(output, points[index], sample_spacing_m)
            continue
        _append_line(output, item.entry, sample_spacing_m)
        _append_arc(output, item, sample_spacing_m)
        applied_radii.append(item.radius_m)
    _append_line(output, points[-1], sample_spacing_m)
    output[0] = points[0]
    output[-1] = points[-1]
    return output, rounded, min(applied_radii)


def _time_parameterize(
    points: list[Point3],
    start_time_s: float,
    end_time_s: float,
) -> list[TimedWaypoint]:
    segment_lengths = [distance(start, end) for start, end in pairwise(points)]
    total_length = math.fsum(segment_lengths)
    if total_length <= _GEOMETRY_EPSILON:
        raise ValueError("a moving block must have positive geometric length")
    duration = end_time_s - start_time_s
    cumulative = 0.0
    waypoints = [TimedWaypoint(start_time_s, points[0], "start")]
    for index, (point, segment_length) in enumerate(
        zip(points[1:], segment_lengths, strict=True), start=1
    ):
        cumulative += segment_length
        time_s = (
            end_time_s
            if index == len(points) - 1
            else start_time_s + duration * cumulative / total_length
        )
        if time_s <= waypoints[-1].time_s:
            raise ValueError("sampled smoothing produced non-increasing timestamps")
        waypoints.append(TimedWaypoint(time_s, point, "move"))
    return waypoints


def _build_candidate(
    raw_path: TimedPath,
    radius_m: float,
    sample_spacing_m: float,
) -> tuple[TimedPath, int, float | None]:
    raw = raw_path.waypoints
    output = [raw[0]]
    rounded_corners = 0
    applied_radii: list[float] = []
    index = 1
    while index < len(raw):
        if raw[index].action == "wait":
            output.append(raw[index])
            index += 1
            continue

        block_start = index - 1
        block_end = index
        while block_end + 1 < len(raw) and raw[block_end + 1].action == "move":
            block_end += 1
        block = raw[block_start : block_end + 1]
        geometry, count, applied_radius = _rounded_geometry(
            tuple(item.position for item in block), radius_m, sample_spacing_m
        )
        if count:
            timed = _time_parameterize(geometry, block[0].time_s, block[-1].time_s)
            output.extend(timed[1:])
            rounded_corners += count
            assert applied_radius is not None
            applied_radii.append(applied_radius)
        else:
            output.extend(block[1:])
        index = block_end + 1

    return TimedPath(tuple(output)), rounded_corners, min(applied_radii, default=None)


def _max_turn_degrees(path: TimedPath) -> float:
    observed = 0.0
    waypoints = path.waypoints
    for previous, current, following in zip(waypoints, waypoints[1:], waypoints[2:], strict=False):
        if current.action != "move" or following.action != "move":
            continue
        incoming_vector = subtract(current.position, previous.position)
        outgoing_vector = subtract(following.position, current.position)
        if (
            _norm(incoming_vector) <= _GEOMETRY_EPSILON
            or _norm(outgoing_vector) <= _GEOMETRY_EPSILON
        ):
            continue
        incoming = _unit(incoming_vector)
        outgoing = _unit(outgoing_vector)
        observed = max(observed, math.degrees(math.acos(_clamped_cosine(incoming, outgoing))))
    return observed


def _within_speed_limit(path: TimedPath, max_speed_mps: float) -> bool:
    tolerance = max(1e-9, max_speed_mps * 1e-9)
    return all(speed <= max_speed_mps + tolerance for speed in path.segment_speeds())


def _positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


def smooth_predictive_timed_path(
    scenario: DynamicScenario,
    raw_path: TimedPath,
    *,
    requested_radius_m: float = 6.0,
    sample_spacing_m: float = 0.5,
    max_speed_mps: float = 8.0,
) -> PredictiveSmoothingResult:
    """Round movement blocks and return only an exactly audited dense linear trajectory.

    Four deterministic radius candidates are attempted from largest to smallest. Wait segments,
    their absolute timestamps, the overall endpoints, and the departure/arrival timestamps are
    retained exactly. Movement samples are mapped by cumulative chord length over each original
    movement block's time range. If no candidate is both safe and within ``max_speed_mps``, the
    already-certified raw path is returned with ``method='raw-fallback'``.

    The input path must itself be safe and within the requested speed limit so that fallback keeps
    the same guarantees.
    """

    _positive_finite("requested_radius_m", requested_radius_m)
    _positive_finite("sample_spacing_m", sample_spacing_m)
    _positive_finite("max_speed_mps", max_speed_mps)
    if not raw_path.is_safe(scenario):
        raise ValueError("raw_path must be collision-free before smoothing")
    if not _within_speed_limit(raw_path, max_speed_mps):
        raise ValueError("raw_path exceeds max_speed_mps and cannot be a certified fallback")

    max_turn_before = _max_turn_degrees(raw_path)
    raw_kinematics = diagnose_timed_path_kinematics(raw_path)
    saw_roundable_corner = False
    for scale_factor in (1.0, 0.75, 0.5, 0.25):
        candidate_radius = requested_radius_m * scale_factor
        try:
            candidate, rounded_corners, applied_radius = _build_candidate(
                raw_path, candidate_radius, sample_spacing_m
            )
        except ValueError:
            continue
        if not rounded_corners:
            continue
        saw_roundable_corner = True
        if not _within_speed_limit(candidate, max_speed_mps) or not candidate.is_safe(scenario):
            continue
        return PredictiveSmoothingResult(
            timed_path=candidate,
            method="sampled-circular-fillet",
            applied=True,
            certified=True,
            raw_waypoint_count=len(raw_path.waypoints),
            output_waypoint_count=len(candidate.waypoints),
            rounded_corners=rounded_corners,
            requested_radius_m=requested_radius_m,
            applied_radius_m=applied_radius,
            sample_spacing_m=sample_spacing_m,
            max_turn_before_deg=max_turn_before,
            max_turn_after_deg=_max_turn_degrees(candidate),
            raw_kinematics=raw_kinematics,
            output_kinematics=diagnose_timed_path_kinematics(candidate),
        )

    method = "raw-fallback" if saw_roundable_corner else "raw-no-roundable-corners"
    return PredictiveSmoothingResult(
        timed_path=raw_path,
        method=method,
        applied=False,
        certified=True,
        raw_waypoint_count=len(raw_path.waypoints),
        output_waypoint_count=len(raw_path.waypoints),
        rounded_corners=0,
        requested_radius_m=requested_radius_m,
        applied_radius_m=None,
        sample_spacing_m=sample_spacing_m,
        max_turn_before_deg=max_turn_before,
        max_turn_after_deg=max_turn_before,
        raw_kinematics=raw_kinematics,
        output_kinematics=raw_kinematics,
    )


__all__ = ["PredictiveSmoothingResult", "smooth_predictive_timed_path"]
