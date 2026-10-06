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
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field
from itertools import pairwise
from typing import Literal

from uav3d.curve_timing import smooth_timed_horizontal_curves
from uav3d.dynamic import DynamicScenario
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import Point3, add, almost_equal, distance, dot, lerp, scale, subtract
from uav3d.kinematics import (
    DiscreteExecutionEnvelope,
    DiscreteExecutionQualification,
    DiscreteKinematicDiagnostics,
    diagnose_timed_path_kinematics,
    qualify_timed_path_execution,
)
from uav3d.predictive import TimedAction, TimedPath, TimedWaypoint
from uav3d.spacetime_timing import insert_braking_holds, schedule_safe_departures
from uav3d.trajectory_timing import TrajectoryTimingResult, retime_timed_path

_GEOMETRY_EPSILON = 1e-9
_MIN_TURN_RADIANS = math.radians(1.0)
_MAX_TURN_RADIANS = math.radians(175.0)
_TRIM_FRACTION = 0.45

ExecutionCandidateStatus = Literal[
    "not-evaluated",
    "qualified",
    "reversal-not-allowed",
    "execution-time-limit-exceeded",
    "time-parameterization-did-not-converge",
    "dynamic-collision-after-retiming",
]


@dataclass(frozen=True, slots=True)
class PredictiveSmoothingResult:
    """A collision-certified geometry candidate plus an optional execution candidate."""

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
    execution_candidate: TimedPath | None = None
    execution_status: ExecutionCandidateStatus = "not-evaluated"
    execution_collision_certified: bool = False
    execution_envelope: DiscreteExecutionEnvelope = field(default_factory=DiscreteExecutionEnvelope)
    execution_qualification: DiscreteExecutionQualification | None = None
    execution_timing_iterations: int = 0
    execution_original_duration_s: float | None = None
    execution_candidate_duration_s: float | None = None
    preserve_altitude: bool = False

    def __post_init__(self) -> None:
        if self.execution_timing_iterations < 0:
            raise ValueError("execution timing iterations must be non-negative")
        if self.execution_status == "qualified":
            if (
                self.execution_candidate is None
                or not self.execution_collision_certified
                or self.execution_qualification is None
                or not self.execution_qualification.qualified
            ):
                raise ValueError("qualified execution status requires a certified candidate")
        elif self.execution_candidate is not None:
            raise ValueError("failed execution status must not expose an execution candidate")

    @property
    def collision_certified(self) -> bool:
        """Compatibility-safe explicit name for the dense-polyline collision certificate."""

        return self.certified

    @property
    def geometry_candidate(self) -> TimedPath:
        """Compatibility-safe explicit name for ``timed_path``."""

        return self.timed_path

    @property
    def execution_qualified(self) -> bool:
        return self.execution_status == "qualified"

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "timed_path": self.timed_path.to_dict(),
            "geometry_candidate": self.geometry_candidate.to_dict(),
            "execution_candidate": (
                self.execution_candidate.to_dict() if self.execution_candidate is not None else None
            ),
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
            "execution": {
                "status": self.execution_status,
                "qualified": self.execution_qualified,
                "collision_certified": self.execution_collision_certified,
                "continuous_dynamics_certified": False,
                "envelope": self.execution_envelope.to_dict(),
                "qualification": (
                    self.execution_qualification.to_dict()
                    if self.execution_qualification is not None
                    else None
                ),
                "timing_iterations": self.execution_timing_iterations,
                "original_duration_s": self.execution_original_duration_s,
                "candidate_duration_s": self.execution_candidate_duration_s,
                "added_duration_s": (
                    None
                    if self.execution_original_duration_s is None
                    or self.execution_candidate_duration_s is None
                    else self.execution_candidate_duration_s - self.execution_original_duration_s
                ),
            },
        }
        if self.preserve_altitude:
            result["optimization_axes"] = ["x", "y"]
            result["altitude_policy"] = "preserve-raw-z-time-profile"
        return result


@dataclass(frozen=True, slots=True)
class _ExecutionEvaluation:
    candidate: TimedPath | None
    status: ExecutionCandidateStatus
    collision_certified: bool
    timing: TrajectoryTimingResult


def _evaluate_execution_candidate(
    scenario: DynamicScenario,
    geometry_candidate: TimedPath,
    envelope: DiscreteExecutionEnvelope,
    *,
    preserve_altitude: bool = False,
    schedule_dynamic_waits: bool = False,
) -> _ExecutionEvaluation:
    timing_input = (
        insert_braking_holds(geometry_candidate, envelope)
        if schedule_dynamic_waits
        else geometry_candidate
    )
    timing = retime_timed_path(
        timing_input,
        envelope,
        serialization_decimal_places=11 if preserve_altitude else None,
    )
    if timing_input is not geometry_candidate:
        timing = TrajectoryTimingResult(
            timing.status,
            timing.timed_path,
            timing.qualification,
            timing.iterations,
            geometry_candidate.duration_s,
            timing.candidate_duration_s,
        )
    candidate = timing.timed_path
    if candidate is None:
        return _ExecutionEvaluation(None, timing.status, False, timing)
    if not candidate.is_safe(scenario):
        if schedule_dynamic_waits:
            scheduled = schedule_safe_departures(scenario, candidate, envelope)
            if scheduled is not None:
                timing = TrajectoryTimingResult(
                    "qualified",
                    scheduled,
                    qualify_timed_path_execution(scheduled, envelope),
                    timing.iterations,
                    timing.original_duration_s,
                    scheduled.duration_s,
                )
                return _ExecutionEvaluation(scheduled, "qualified", True, timing)
        return _ExecutionEvaluation(
            None,
            "dynamic-collision-after-retiming",
            False,
            timing,
        )
    return _ExecutionEvaluation(candidate, "qualified", True, timing)


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


def _horizontal_progress(points: tuple[Point3, ...] | list[Point3]) -> list[float]:
    progress = [0.0]
    for previous, current in pairwise(points):
        progress.append(
            progress[-1] + math.hypot(current[0] - previous[0], current[1] - previous[1])
        )
    return progress


def _interpolate_progress_xy(
    points: list[Point3], progress: list[float], parameter: float
) -> tuple[float, float]:
    if parameter <= 0:
        return points[0][0], points[0][1]
    if parameter >= progress[-1]:
        return points[-1][0], points[-1][1]
    following = bisect_right(progress, parameter)
    previous = following - 1
    fraction = (parameter - progress[previous]) / (progress[following] - progress[previous])
    point = lerp(points[previous], points[following], fraction)
    return point[0], point[1]


def _horizontal_geometry_to_timed_block(
    points: list[Point3], reference: tuple[TimedWaypoint, ...]
) -> list[TimedWaypoint]:
    """Lift XY by the original horizontal progress and retain the entire raw z(t).

    Mapping candidate arc progress to original horizontal progress, instead of uniform time,
    keeps pure vertical intervals stationary in XY. Provided the candidate is no longer in XY,
    its horizontal speed cannot exceed the reference horizontal speed; unchanged z(t) then
    cannot increase 3D speed either. Inserting both sets of knots makes these statements exact
    for the returned piecewise-linear trajectory, not just true at sampled reference points.
    """

    original_positions = tuple(item.position for item in reference)
    original_progress = _horizontal_progress(original_positions)
    total_original = original_progress[-1]
    candidate_progress = _horizontal_progress(points)
    total_candidate = candidate_progress[-1]
    if total_original <= _GEOMETRY_EPSILON or total_candidate <= _GEOMETRY_EPSILON:
        return list(reference)
    if total_candidate > total_original + max(1e-9, total_original * 1e-9):
        raise ValueError("horizontal candidate cannot lengthen the reference XY route")
    original_times = [item.time_s for item in reference]
    times = set(original_times)
    derived_times: list[float] = []

    def numerically_same_time(left: float, right: float) -> bool:
        # Snap only *new* sampling knots, never two raw knots. Accumulated length ratios and
        # inverse interpolation can miss the exact raw knot by several ulps. Even a tiny fake
        # stationary segment would introduce a zero-speed boundary during execution retiming.
        return abs(left - right) <= max(1e-12, 32.0 * math.ulp(left), 32.0 * math.ulp(right))

    for progress in candidate_progress[1:-1]:
        original_target = progress * (total_original / total_candidate)
        following = bisect_right(original_progress, original_target)
        if following == len(original_progress):
            times.add(original_times[-1])
            continue
        previous = following - 1
        fraction = (original_target - original_progress[previous]) / (
            original_progress[following] - original_progress[previous]
        )
        derived_time = original_times[previous] + fraction * (
            original_times[following] - original_times[previous]
        )
        insertion = bisect_left(original_times, derived_time)
        neighbors = original_times[max(0, insertion - 1) : insertion + 1]
        if any(numerically_same_time(derived_time, original) for original in neighbors):
            continue
        if derived_times and numerically_same_time(derived_time, derived_times[-1]):
            continue
        derived_times.append(derived_time)
        times.add(derived_time)
    output: list[TimedWaypoint] = []
    for time_s in sorted(times):
        knot = bisect_right(original_times, time_s) - 1
        if knot == len(reference) - 1:
            original_position = reference[-1].position
            progress = total_original
        else:
            duration = original_times[knot + 1] - original_times[knot]
            fraction = (time_s - original_times[knot]) / duration
            original_position = lerp(
                reference[knot].position, reference[knot + 1].position, fraction
            )
            progress = original_progress[knot] + fraction * (
                original_progress[knot + 1] - original_progress[knot]
            )
        xy = _interpolate_progress_xy(
            points, candidate_progress, progress * (total_candidate / total_original)
        )
        position = (xy[0], xy[1], original_position[2])
        action: TimedAction = (
            "start"
            if not output
            else "wait"
            if almost_equal(output[-1].position, position)
            else "move"
        )
        output.append(TimedWaypoint(time_s, position, action))
    # Preserve endpoint coordinates bit-for-bit, including supplied non-zero altitudes.
    output[0] = TimedWaypoint(reference[0].time_s, reference[0].position, "start")
    last_action: TimedAction = (
        "wait" if almost_equal(output[-2].position, reference[-1].position) else "move"
    )
    output[-1] = TimedWaypoint(reference[-1].time_s, reference[-1].position, last_action)
    return output


def _compressed_horizontal_points(waypoints: tuple[TimedWaypoint, ...]) -> list[Point3]:
    """Discard redundant XY knots only; altitude knots will be restored during lifting."""

    output: list[Point3] = []
    for item in waypoints:
        point = (item.position[0], item.position[1], 0.0)
        if output and almost_equal(output[-1], point):
            continue
        while len(output) >= 2:
            incoming = subtract(output[-1], output[-2])
            outgoing = subtract(point, output[-1])
            if dot(incoming, outgoing) <= 0:
                break
            scale_factor = _norm(incoming) * _norm(outgoing)
            if _norm(_cross(incoming, outgoing)) > _GEOMETRY_EPSILON * scale_factor:
                break
            output.pop()
        output.append(point)
    return output


def _build_horizontal_candidate(
    geometry_input: TimedPath,
    raw_reference: TimedPath,
    radius_m: float,
    sample_spacing_m: float,
) -> tuple[TimedPath, int, float | None]:
    """Apply planar fillets, then lift every block against untouched raw height/time knots."""

    raw = geometry_input.waypoints
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
        reference = tuple(
            item
            for item in raw_reference.waypoints
            if block[0].time_s <= item.time_s <= block[-1].time_s
        )
        geometry, count, applied_radius = _rounded_geometry(
            tuple(_compressed_horizontal_points(block)), radius_m, sample_spacing_m
        )
        if count:
            timed = _horizontal_geometry_to_timed_block(geometry, reference)
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
    # Subtracting large absolute timestamps loses precision on sub-millisecond
    # curve segments. Compare displacement, accounting only for representable
    # clock roundoff; do not amplify that error by dividing by a tiny duration.
    for previous, current in pairwise(path.waypoints):
        duration = current.time_s - previous.time_s
        clock_roundoff = math.ulp(previous.time_s) + math.ulp(current.time_s)
        allowance = (max_speed_mps + tolerance) * duration + max_speed_mps * clock_roundoff
        if distance(previous.position, current.position) > allowance:
            return False
    return True


def _positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


def _shortcut_horizontal_timed_path(scenario: DynamicScenario, path: TimedPath) -> TimedPath:
    raw = path.waypoints
    task_positions = [tuple(task["position"])
                      for task in scenario.static_scene.metadata.get("missionTaskPoints", [])]
    output = [raw[0]]
    index = 0
    while index < len(raw) - 1:
        if raw[index + 1].action == "wait":
            output.append(raw[index + 1])
            index += 1
            continue
        block_end = index + 1
        while (block_end + 1 < len(raw) and raw[block_end + 1].action == "move"
               and not any(almost_equal(raw[block_end].position, anchor)
                           for anchor in task_positions)):
            block_end += 1
        following = index + 1
        selected = list(raw[index : following + 1])
        for candidate in range(block_end, index, -1):
            if (
                math.hypot(
                    raw[index].position[0] - raw[candidate].position[0],
                    raw[index].position[1] - raw[candidate].position[1],
                )
                <= _GEOMETRY_EPSILON
            ):
                continue  # Keep moving XY loops and pure vertical blocks explicit.
            lifted = _horizontal_geometry_to_timed_block(
                [raw[index].position, raw[candidate].position], raw[index : candidate + 1]
            )
            trial = TimedPath(tuple(lifted))
            if trial.is_safe(scenario):
                following = candidate
                selected = lifted
                break
        output.extend(selected[1:])
        index = following
    candidate_path = TimedPath(tuple(output))
    if not candidate_path.is_safe(scenario):
        raise ValueError("shortcut input must expose a collision-free timed path")
    return candidate_path


def shortcut_timed_path(
    scenario: DynamicScenario, path: TimedPath, *, preserve_altitude: bool = False
) -> TimedPath:
    """Remove visible movement waypoints while preserving absolute times and every wait.

    A chord is accepted only after auditing its *whole* time interval against static buildings,
    scheduled exclusions, and moving traffic. Keeping the original endpoints' timestamps makes
    a chord no faster than the polyline it replaces. Wait starts/ends are hard block boundaries;
    no shortcut can move a wait, erase one, or borrow time from it. Collinear tick/grid samples
    disappear naturally when their longer chord passes the same exact predicate.
    """

    if not path.is_safe(scenario):
        raise ValueError("shortcut input must expose a collision-free timed path")
    if preserve_altitude:
        return _shortcut_horizontal_timed_path(scenario, path)
    raw = path.waypoints
    output = [raw[0]]
    index = 0
    while index < len(raw) - 1:
        if raw[index + 1].action == "wait":
            output.append(raw[index + 1])
            index += 1
            continue
        block_end = index + 1
        while block_end + 1 < len(raw) and raw[block_end + 1].action == "move":
            block_end += 1
        following = index + 1
        for candidate in range(block_end, index, -1):
            if almost_equal(raw[index].position, raw[candidate].position):
                continue  # Do not silently turn a moving loop into an undocumented wait.
            if spacetime_segment_is_free(
                scenario,
                raw[index].position,
                raw[candidate].position,
                raw[index].time_s,
                raw[candidate].time_s,
            ):
                following = candidate
                break
        output.append(raw[following])
        index = following
    candidate_path = TimedPath(tuple(output))
    # Keep the standalone utility fail-closed too, including the original adjacent-segment case.
    if not candidate_path.is_safe(scenario):
        raise ValueError("shortcut input must expose a collision-free timed path")
    return candidate_path


def smooth_predictive_timed_path(
    scenario: DynamicScenario,
    raw_path: TimedPath,
    *,
    requested_radius_m: float = 6.0,
    sample_spacing_m: float = 0.5,
    max_speed_mps: float = 8.0,
    execution_envelope: DiscreteExecutionEnvelope | None = None,
    shortcut: bool = False,
    preserve_altitude: bool = False,
    curve_method: Literal["fillet", "bspline"] = "fillet",
    schedule_dynamic_waits: bool = False,
) -> PredictiveSmoothingResult:
    """Round movement blocks and return only an exactly audited dense linear trajectory.

    Four deterministic radius candidates are attempted from largest to smallest. Wait segments,
    their absolute timestamps, the overall endpoints, and the departure/arrival timestamps are
    retained exactly. Movement samples are mapped by cumulative chord length over each original
    movement block's time range. If no candidate is both safe and within ``max_speed_mps``, the
    already-certified raw path is returned with ``method='raw-fallback'``.

    The input path must itself be safe and within the requested speed limit so that fallback keeps
    the same guarantees. ``shortcut=True`` additionally compresses each movement block using
    exact space-time line of sight before rounding; the raw evidence is never modified. It is
    opt-in so frozen studies keep their declared geometry postprocessor. A geometrically safe
    but execution-invalid radius does not stop the search for a qualified alternate radius.

    ``preserve_altitude=True`` confines shortcuts and fillets to XY. The complete original
    piecewise-linear z(t), including peaks, troughs and vertical-only intervals, is a hard
    constraint of every geometry candidate. Execution qualification may stretch timestamps,
    but never changes candidate positions or removes any original height knot.

    ``curve_method='bspline'`` uses local quintic spans, including tangent-connected
    interpolation through service stops. Colliding turns shrink independently. Geometry
    retains z(t); its optional final execution must still pass retiming and collision gates.
    """

    _positive_finite("requested_radius_m", requested_radius_m)
    _positive_finite("sample_spacing_m", sample_spacing_m)
    _positive_finite("max_speed_mps", max_speed_mps)
    if curve_method not in ("fillet", "bspline"):
        raise ValueError("unsupported horizontal curve method")
    if curve_method == "bspline" and not preserve_altitude:
        raise ValueError("local B-spline curves require XY-only altitude preservation")
    if not raw_path.is_safe(scenario):
        raise ValueError("raw_path must be collision-free before smoothing")
    if not _within_speed_limit(raw_path, max_speed_mps):
        raise ValueError("raw_path exceeds max_speed_mps and cannot be a certified fallback")

    envelope = execution_envelope or DiscreteExecutionEnvelope(max_speed_mps=max_speed_mps)
    max_turn_before = _max_turn_degrees(raw_path)
    raw_kinematics = diagnose_timed_path_kinematics(raw_path)
    geometry_input = (
        shortcut_timed_path(scenario, raw_path, preserve_altitude=preserve_altitude)
        if shortcut
        else raw_path
    )
    shortened = geometry_input != raw_path
    saw_roundable_corner = False
    best_geometry: tuple[TimedPath, int, float | None, _ExecutionEvaluation] | None = None

    def result_for(
        candidate: TimedPath,
        rounded_corners: int,
        applied_radius: float | None,
        execution: _ExecutionEvaluation,
        method: str,
    ) -> PredictiveSmoothingResult:
        return PredictiveSmoothingResult(
            timed_path=candidate,
            method=method,
            applied=shortened or bool(rounded_corners),
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
            execution_candidate=execution.candidate,
            execution_status=execution.status,
            execution_collision_certified=execution.collision_certified,
            execution_envelope=envelope,
            execution_qualification=execution.timing.qualification,
            execution_timing_iterations=execution.timing.iterations,
            execution_original_duration_s=execution.timing.original_duration_s,
            execution_candidate_duration_s=execution.timing.candidate_duration_s,
            preserve_altitude=preserve_altitude,
        )

    for scale_factor in (1.0, 0.75, 0.5, 0.25):
        candidate_radius = requested_radius_m * scale_factor
        try:
            if curve_method == "bspline":
                candidate, rounded_corners, applied_radius = smooth_timed_horizontal_curves(
                    scenario, geometry_input, candidate_radius, sample_spacing_m
                )
            elif preserve_altitude:
                candidate, rounded_corners, applied_radius = _build_horizontal_candidate(
                    geometry_input, raw_path, candidate_radius, sample_spacing_m
                )
            else:
                candidate, rounded_corners, applied_radius = _build_candidate(
                    geometry_input, candidate_radius, sample_spacing_m
                )
        except ValueError:
            continue
        if not rounded_corners:
            continue
        saw_roundable_corner = True
        # Interpolating a hard stop can lengthen its approach. Only the final retimed
        # flight is speed-qualified; do not reject geometry before its timing solver runs.
        if (
            curve_method != "bspline" and not _within_speed_limit(candidate, max_speed_mps)
        ) or not candidate.is_safe(scenario):
            continue
        execution = _evaluate_execution_candidate(
            scenario,
            candidate,
            envelope,
            preserve_altitude=preserve_altitude,
            schedule_dynamic_waits=schedule_dynamic_waits,
        )
        method = (
            "spacetime-shortcut-plus-local-quintic-bspline"
            if curve_method == "bspline"
            else (
                "spacetime-shortcut-plus-sampled-circular-fillet"
                if shortcut
                else "sampled-circular-fillet"
            )
        )
        if execution.candidate is not None:
            return result_for(candidate, rounded_corners, applied_radius, execution, method)
        if best_geometry is None or _max_turn_degrees(candidate) < _max_turn_degrees(
            best_geometry[0]
        ):
            best_geometry = (candidate, rounded_corners, applied_radius, execution)

    execution = _evaluate_execution_candidate(
        scenario,
        geometry_input,
        envelope,
        preserve_altitude=preserve_altitude,
        schedule_dynamic_waits=schedule_dynamic_waits,
    )
    if execution.candidate is None and best_geometry is not None:
        candidate, rounded_corners, applied_radius, execution = best_geometry
        method = (
            "spacetime-shortcut-plus-local-quintic-bspline"
            if curve_method == "bspline"
            else (
                "spacetime-shortcut-plus-sampled-circular-fillet"
                if shortcut
                else "sampled-circular-fillet"
            )
        )
        return result_for(candidate, rounded_corners, applied_radius, execution, method)
    if shortcut:
        method = (
            "spacetime-shortcut-fillet-fallback" if saw_roundable_corner else "spacetime-shortcut"
        )
    else:
        method = "raw-fallback" if saw_roundable_corner else "raw-no-roundable-corners"
    return result_for(geometry_input, 0, None, execution, method)


__all__ = [
    "ExecutionCandidateStatus",
    "PredictiveSmoothingResult",
    "shortcut_timed_path",
    "smooth_predictive_timed_path",
]
