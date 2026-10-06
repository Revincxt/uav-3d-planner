"""Safety and quality regressions for dense timing and optional space-time shortcuts."""

from __future__ import annotations

import math
import random
from itertools import pairwise

import pytest

from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.geometry import distance, lerp
from uav3d.kinematics import DiscreteExecutionEnvelope, qualify_timed_path_execution
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.predictive_smoothing import shortcut_timed_path, smooth_predictive_timed_path
from uav3d.scene import AABB, Bounds3D, Scene
from uav3d.trajectory_timing import retime_timed_path


def _scenario(
    path: TimedPath,
    *,
    buildings: tuple[AABB, ...] = (),
    traffic: tuple[MovingSphere, ...] = (),
    zones: tuple[TemporaryCylinder, ...] = (),
) -> DynamicScenario:
    return DynamicScenario(
        "trajectory-optimization",
        "Trajectory optimization",
        Scene(
            "trajectory-optimization-static",
            "Trajectory optimization static",
            Bounds3D((0.0, 0.0, 0.0), (24.0, 24.0, 12.0)),
            path.start,
            path.goal,
            buildings=buildings,
            drone_radius=0.0,
            safety_margin=0.0,
        ),
        temporary_cylinders=zones,
        moving_spheres=traffic,
    )


def _corner() -> TimedPath:
    return TimedPath(
        (
            TimedWaypoint(0.0, (4.0, 4.0, 5.0), "start"),
            TimedWaypoint(2.0, (12.0, 4.0, 5.0), "move"),
            TimedWaypoint(4.0, (12.0, 12.0, 5.0), "move"),
        )
    )


def _assert_no_segment_shortened(source: TimedPath, candidate: TimedPath) -> None:
    assert candidate.positions == source.positions
    for (a, b), (c, d) in zip(
        pairwise(source.waypoints), pairwise(candidate.waypoints), strict=True
    ):
        assert d.action == b.action
        assert d.time_s - c.time_s >= b.time_s - a.time_s - 1e-9
        if b.action == "wait":
            assert d.time_s - c.time_s == pytest.approx(b.time_s - a.time_s)


@pytest.mark.parametrize("spacing", (10.0, 5.0, 2.0, 1.0))
def test_dense_straight_sampling_does_not_artificially_slow_flight(spacing: float) -> None:
    count = int(1000.0 / spacing)
    raw = TimedPath(
        tuple(
            TimedWaypoint(
                index * spacing / 15.0,
                (index * spacing, 0.0, 100.0),
                "start" if index == 0 else "move",
            )
            for index in range(count + 1)
        )
    )
    envelope = DiscreteExecutionEnvelope(max_speed_mps=15.0, max_execution_time_s=900.0)

    result = retime_timed_path(raw, envelope)

    assert result.timed_path is not None
    # The old pairwise duration-scaling sweep took 247 s at 5 m, and failed at 1113 s at 1 m.
    # 1000/15 + 15/4 = 70.4167 s is a separately feasible stop/cruise/stop reference under this
    # very same discrete contract, not a claim that its proxy certifies continuous dynamics.
    assert 70.0 < result.timed_path.duration_s <= 70.42
    assert qualify_timed_path_execution(result.timed_path, envelope).qualified
    _assert_no_segment_shortened(raw, result.timed_path)


def test_sampling_resolution_has_a_bounded_effect_on_straight_timing() -> None:
    times = []
    for spacing in (10.0, 5.0, 2.0, 1.0):
        count = int(300.0 / spacing)
        raw = TimedPath(
            tuple(
                TimedWaypoint(
                    i * spacing / 15.0,
                    (i * spacing, 0.0, 10.0),
                    "start" if i == 0 else "move",
                )
                for i in range(count + 1)
            )
        )
        result = retime_timed_path(raw, DiscreteExecutionEnvelope(max_speed_mps=15.0))
        assert result.timed_path is not None
        times.append(result.timed_path.duration_s)
    assert max(times) - min(times) < 0.01


def test_uneven_spatial_samples_climbs_and_original_slow_segments_remain_qualified() -> None:
    rng = random.Random(231)
    envelope = DiscreteExecutionEnvelope(max_speed_mps=15.0, max_execution_time_s=100_000.0)
    for _ in range(30):
        position = (0.0, 0.0, 50.0)
        time_s = 0.0
        heading = 0.0
        points = [TimedWaypoint(time_s, position, "start")]
        for index in range(40):
            if index % 11 == 10:
                time_s += 2.3
                points.append(TimedWaypoint(time_s, position, "wait"))
                continue
            heading += rng.uniform(-0.8, 0.8)
            length = 10.0 ** rng.uniform(-1.0, 1.8)
            position = (
                position[0] + length * math.cos(heading),
                position[1] + length * math.sin(heading),
                position[2] + rng.uniform(-0.3, 0.3) * length,
            )
            time_s += rng.uniform(0.02, 6.0)
            points.append(TimedWaypoint(time_s, position, "move"))
        raw = TimedPath(tuple(points))
        result = retime_timed_path(raw, envelope)
        assert result.timed_path is not None
        assert qualify_timed_path_execution(result.timed_path, envelope).qualified
        _assert_no_segment_shortened(raw, result.timed_path)


def test_opt_in_shortcut_reduces_free_zigzag_without_mutating_raw_evidence() -> None:
    raw = _corner()
    original = raw.to_dict()
    scenario = _scenario(raw)
    result = smooth_predictive_timed_path(scenario, raw, max_speed_mps=4.0, shortcut=True)
    assert result.method == "spacetime-shortcut"
    assert result.applied
    assert result.raw_waypoint_count == 3
    assert result.output_waypoint_count == 2
    assert result.geometry_candidate.waypoints == (raw.waypoints[0], raw.waypoints[-1])
    assert result.geometry_candidate.is_safe(scenario)
    assert raw.to_dict() == original


@pytest.mark.parametrize("hazard", ("building", "sphere", "temporary-cylinder"))
def test_shortcut_checks_full_static_and_dynamic_chords(hazard: str) -> None:
    raw = _corner()
    center = (8.0, 8.0, 5.0)
    scenario = _scenario(
        raw,
        buildings=(AABB("block", (7.0, 7.0, 0.0), (9.0, 9.0, 10.0)),)
        if hazard == "building"
        else (),
        traffic=(MovingSphere("crossing", 1.0, ((0.0, center), (4.0, center))),)
        if hazard == "sphere"
        else (),
        zones=(TemporaryCylinder("scheduled", center[:2], 1.0, 0.0, 10.0, 1.5, 2.5),)
        if hazard == "temporary-cylinder"
        else (),
    )
    assert raw.is_safe(scenario)
    assert shortcut_timed_path(scenario, raw) == raw


def test_inactive_temporary_zone_does_not_prevent_a_safe_time_specific_shortcut() -> None:
    raw = _corner()
    zone = TemporaryCylinder("inactive", (8.0, 8.0), 1.0, 0.0, 10.0, 10.0, 20.0)
    scenario = _scenario(raw, zones=(zone,))
    candidate = shortcut_timed_path(scenario, raw)
    assert len(candidate.waypoints) == 2
    assert candidate.is_safe(scenario)


def test_collinear_compression_does_not_change_an_essential_hazard_crossing_time() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (2.0, 2.0, 5.0), "start"),
            TimedWaypoint(1.0, (10.0, 2.0, 5.0), "move"),
            TimedWaypoint(4.0, (18.0, 2.0, 5.0), "move"),
        )
    )
    zone = TemporaryCylinder("late-crossing", (10.0, 2.0), 0.5, 0.0, 10.0, 1.8, 2.2)
    scenario = _scenario(raw, zones=(zone,))
    assert raw.is_safe(scenario)
    assert shortcut_timed_path(scenario, raw) == raw


def test_shortcuts_preserve_every_wait_and_its_absolute_start_and_end() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (2.0, 2.0, 5.0), "start"),
            TimedWaypoint(2.0, (10.0, 2.0, 5.0), "move"),
            TimedWaypoint(4.0, (10.0, 10.0, 5.0), "move"),
            TimedWaypoint(6.0, (10.0, 10.0, 5.0), "wait"),
            TimedWaypoint(8.0, (18.0, 10.0, 5.0), "move"),
            TimedWaypoint(10.0, (18.0, 18.0, 5.0), "move"),
        )
    )
    scenario = _scenario(raw)
    result = smooth_predictive_timed_path(scenario, raw, max_speed_mps=4.0, shortcut=True)
    assert result.geometry_candidate.waypoints == tuple(raw.waypoints[i] for i in (0, 2, 3, 5))
    assert result.geometry_candidate.wait_time_s == 2.0
    assert result.geometry_candidate.is_safe(scenario)


def test_moving_loop_is_not_silently_replaced_with_a_wait() -> None:
    raw = TimedPath(
        (
            TimedWaypoint(0.0, (4.0, 4.0, 5.0), "start"),
            TimedWaypoint(1.0, (12.0, 4.0, 5.0), "move"),
            TimedWaypoint(2.0, (4.0, 4.0, 5.0), "move"),
            TimedWaypoint(3.0, (4.0, 12.0, 5.0), "move"),
        )
    )
    candidate = shortcut_timed_path(_scenario(raw), raw)
    assert all(item.action == "move" for item in candidate.waypoints[1:])
    assert candidate.wait_time_s == 0.0


def test_execution_collision_at_first_radius_tries_a_safe_alternate_radius() -> None:
    raw = _corner()
    open_scenario = _scenario(raw)
    envelope = DiscreteExecutionEnvelope(max_speed_mps=4.0)
    first = smooth_predictive_timed_path(
        open_scenario,
        raw,
        requested_radius_m=3.0,
        sample_spacing_m=0.25,
        max_speed_mps=4.0,
        execution_envelope=envelope,
    )
    assert first.execution_candidate is not None
    # Geometry/raw have arrived; only the slower execution intersects this gate.
    conflict_time = 4.2
    collision_position = None
    for a, b in pairwise(first.execution_candidate.waypoints):
        if a.time_s <= conflict_time <= b.time_s:
            collision_position = lerp(
                a.position, b.position, (conflict_time - a.time_s) / (b.time_s - a.time_s)
            )
            break
    assert collision_position is not None
    zone = TemporaryCylinder(
        "execution-only-gate",
        collision_position[:2],
        0.08,
        0.0,
        10.0,
        conflict_time - 0.01,
        conflict_time + 0.01,
    )
    scenario = _scenario(raw, zones=(zone,))
    assert raw.is_safe(scenario)
    assert first.geometry_candidate.is_safe(scenario)
    assert not first.execution_candidate.is_safe(scenario)

    result = smooth_predictive_timed_path(
        scenario,
        raw,
        requested_radius_m=3.0,
        sample_spacing_m=0.25,
        max_speed_mps=4.0,
        execution_envelope=envelope,
    )
    assert result.applied_radius_m == pytest.approx(2.25)
    assert result.execution_status == "qualified"
    assert result.execution_candidate is not None
    assert result.execution_candidate.is_safe(scenario)
    assert qualify_timed_path_execution(result.execution_candidate, envelope).qualified
    assert distance(result.geometry_candidate.start, raw.start) == 0.0


def test_no_execution_candidate_is_exposed_when_all_candidates_fail_the_envelope() -> None:
    raw = _corner()
    result = smooth_predictive_timed_path(
        _scenario(raw),
        raw,
        requested_radius_m=3.0,
        sample_spacing_m=0.25,
        max_speed_mps=4.0,
        execution_envelope=DiscreteExecutionEnvelope(max_speed_mps=4.0, max_execution_time_s=3.9),
    )
    assert result.method == "sampled-circular-fillet"
    assert result.geometry_candidate.is_safe(_scenario(raw))
    assert result.execution_candidate is None
    assert result.execution_status == "execution-time-limit-exceeded"
    assert not result.execution_collision_certified
