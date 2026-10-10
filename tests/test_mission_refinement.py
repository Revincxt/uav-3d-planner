from __future__ import annotations

from dataclasses import replace

import pytest

from uav3d.dynamic import DynamicScenario, MovingSphere
from uav3d.kinematics import DiscreteExecutionEnvelope
from uav3d.mission_refinement import (
    ContinuousAnchorSpaceTimeAStar,
    horizontal_reversals,
    refine_mission_trajectory,
)
from uav3d.planners.grid import VoxelGrid
from uav3d.planners.space_time_astar import SpaceTimeAStar3D, SpaceTimeAStarConfig
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.scene import Bounds3D, Scene


def fixture():
    scene = Scene(
        "anchor",
        "Anchor",
        Bounds3D((0, 0, 0), (6, 6, 4)),
        (1, 1, 2),
        (5, 5, 2),
        drone_radius=0,
        safety_margin=0,
    )
    scenario = DynamicScenario("anchor", "Anchor", scene)
    config = SpaceTimeAStarConfig(
        resolution=1, time_step=1, cruise_speed=2, time_horizon=5, connectivity=26
    )
    return scenario, config, VoxelGrid(scene, 1)


def test_safe_connector_absorbs_only_alignment_without_changing_state_clock():
    scenario, config, grid = fixture()
    old = SpaceTimeAStar3D(config)._initial_states(scenario, grid, [(2, 2, 2)], 0, 5)
    new = ContinuousAnchorSpaceTimeAStar(config)._initial_states(scenario, grid, [(2, 2, 2)], 0, 5)
    assert [s for s, _ in new] == [s for s, _ in old]
    assert old[0][1][-1].action == "wait"
    assert new[0][1][-1].action == "move"
    for (_, a), (_, b) in zip(old, new, strict=True):
        assert a[-1].time_s == b[-1].time_s
        assert a[-1].position == b[-1].position
        assert TimedPath(b).is_safe(scenario)
        assert TimedPath(b).wait_time_s <= TimedPath(a).wait_time_s


def test_alignment_is_retained_when_slow_motion_would_cross_traffic():
    scenario, config, grid = fixture()
    sphere = MovingSphere(
        "crossing", 0.08, ((0, (1.5, 8, 2)), (0.5, (1.5, 1.5, 2)), (1, (1.5, -5, 2)))
    )
    scenario = replace(scenario, moving_spheres=(sphere,))
    old = SpaceTimeAStar3D(config)._initial_states(scenario, grid, [(2, 2, 2)], 0, 5)
    new = ContinuousAnchorSpaceTimeAStar(config)._initial_states(scenario, grid, [(2, 2, 2)], 0, 5)
    assert old[0][0][1] == 1
    assert old[0][1] == new[0][1]
    assert new[0][1][-1].action == "wait"
    assert TimedPath(new[0][1]).is_safe(scenario)


def test_refinement_keeps_safe_waits_height_knots_and_qualified_result():
    scenario, _, _ = fixture()
    raw = TimedPath(
        (
            TimedWaypoint(0, (1, 1, 2), "start"),
            TimedWaypoint(2, (3, 1, 2), "move"),
            TimedWaypoint(4, (3, 1, 2), "wait"),
            TimedWaypoint(8, (5, 5, 2), "move"),
        )
    )
    result = refine_mission_trajectory(
        scenario,
        raw,
        DiscreteExecutionEnvelope(max_speed_mps=2, max_execution_time_s=40),
        turn_scale_m=1,
        sample_spacing_m=0.2,
    )
    assert result.execution_qualified
    assert result.execution_candidate.is_safe(scenario)
    assert result.execution_candidate.wait_time_s == pytest.approx(2)
    assert all(p.position[2] == 2 for p in result.timed_path.waypoints)
    assert result.requested_radius_m == 1


def test_reversal_diagnostic_ignores_noise_and_wait_boundaries():
    path = TimedPath(
        (
            TimedWaypoint(0, (0, 0, 1), "start"),
            TimedWaypoint(1, (2, 0, 1), "move"),
            TimedWaypoint(2, (1, 0, 1), "move"),
        )
    )
    assert horizontal_reversals(path) == 1
    noisy = TimedPath(
        (
            TimedWaypoint(0, (0, 0, 1), "start"),
            TimedWaypoint(1, (0.001, 0, 2), "move"),
            TimedWaypoint(2, (0, 0, 3), "move"),
        )
    )
    assert horizontal_reversals(noisy) == 0
