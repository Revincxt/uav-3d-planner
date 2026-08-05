from __future__ import annotations

from typing import cast

import pytest

import uav3d.planners.space_time_astar as space_time_module
from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.dynamic_collision import point_is_free_at_time
from uav3d.planners.space_time_astar import SpaceTimeAStar3D, SpaceTimeAStarConfig
from uav3d.predictive import TimedAction, TimedPath, TimedWaypoint
from uav3d.scene import AABB, Bounds3D, Scene


def corridor_scene(*, obstacle: bool = False) -> Scene:
    buildings = (AABB("barrier", (7.0, 3.0, 0.0), (9.0, 5.0, 4.0)),) if obstacle else ()
    return Scene(
        "predictive-corridor",
        "Predictive corridor",
        Bounds3D((0.0, 0.0, 0.0), (16.0, 8.0, 4.0)),
        (2.0, 4.0, 2.0),
        (14.0, 4.0, 2.0),
        buildings,
        drone_radius=0.0,
        safety_margin=0.0,
    )


def short_scene() -> Scene:
    return Scene(
        "short",
        "Short",
        Bounds3D((0.0, 0.0, 0.0), (10.0, 10.0, 10.0)),
        (2.0, 5.0, 5.0),
        (8.0, 5.0, 5.0),
        drone_radius=0.0,
        safety_margin=0.0,
    )


def planner(*, horizon: float = 10.0, max_expansions: int = 10_000) -> SpaceTimeAStar3D:
    return SpaceTimeAStar3D(
        SpaceTimeAStarConfig(
            resolution=2.0,
            time_step=1.0,
            cruise_speed=2.0,
            time_horizon=horizon,
            max_expansions=max_expansions,
        )
    )


def test_timed_path_contract_requires_explicit_waits_and_strict_time() -> None:
    path = TimedPath(
        (
            TimedWaypoint(2.0, (1.0, 1.0, 1.0), "start"),
            TimedWaypoint(3.0, (1.0, 1.0, 1.0), "wait"),
            TimedWaypoint(5.0, (5.0, 1.0, 1.0), "move"),
        )
    )
    assert path.departure_time_s == 2.0
    assert path.arrival_time_s == 5.0
    assert path.duration_s == 3.0
    assert path.wait_time_s == 1.0
    assert path.segment_speeds() == (0.0, 2.0)
    assert path.to_dict()["waypoints"] == [waypoint.to_dict() for waypoint in path.waypoints]

    with pytest.raises(ValueError, match="first timed waypoint"):
        TimedPath((TimedWaypoint(0.0, (1.0, 1.0, 1.0), "move"),))
    with pytest.raises(ValueError, match="strictly increasing"):
        TimedPath(
            (
                TimedWaypoint(1.0, (1.0, 1.0, 1.0), "start"),
                TimedWaypoint(1.0, (2.0, 1.0, 1.0), "move"),
            )
        )
    with pytest.raises(ValueError, match="stationary"):
        TimedPath(
            (
                TimedWaypoint(0.0, (1.0, 1.0, 1.0), "start"),
                TimedWaypoint(1.0, (1.0, 1.0, 1.0), "move"),
            )
        )
    with pytest.raises(ValueError, match="action must"):
        TimedWaypoint(0.0, (1.0, 1.0, 1.0), cast(TimedAction, "hover"))


def test_configuration_enforces_fixed_speed_time_lattice() -> None:
    config = SpaceTimeAStarConfig(
        resolution=4.0,
        time_step=0.5,
        cruise_speed=4.0,
        time_horizon=10.0,
        max_expansions=100,
    )
    assert config.movement_steps == 2
    assert config.horizon_steps == 20
    with pytest.raises(ValueError, match="integer number of time steps"):
        SpaceTimeAStarConfig(resolution=3.0, time_step=1.0, cruise_speed=2.0)
    with pytest.raises(ValueError, match="positive"):
        SpaceTimeAStarConfig(time_step=0.0)


def test_immediate_direct_path_is_globally_earliest_and_retains_exact_endpoints() -> None:
    scenario = DynamicScenario("empty", "Empty", short_scene())
    result = planner().plan(scenario, start_time=5.0)

    assert result.algorithm == "space-time-astar-4d"
    assert result.success
    assert result.failure_reason is None
    assert result.expanded_spacetime_states == 0
    assert result.generated_spacetime_states == 1
    assert result.path == (scenario.static_scene.start, scenario.static_scene.goal)
    assert result.arrival_time_s == pytest.approx(8.0)
    assert result.timed_path is not None
    assert result.timed_path.is_safe(scenario)
    assert result.timed_path.segment_speeds() == pytest.approx((2.0,))


def test_predictive_direct_path_waits_for_half_open_temporary_zone() -> None:
    scenario = DynamicScenario(
        "wait-for-zone",
        "Wait for zone",
        short_scene(),
        temporary_cylinders=(TemporaryCylinder("closure", (5.0, 5.0), 1.0, 0.0, 10.0, 0.0, 2.0),),
    )
    result = planner().plan(scenario)

    assert result.success, result.failure_reason
    assert result.timed_path is not None
    assert result.timed_path.wait_time_s == pytest.approx(1.0)
    assert [waypoint.action for waypoint in result.timed_path.waypoints] == [
        "start",
        "wait",
        "move",
    ]
    assert result.arrival_time_s == pytest.approx(4.0)
    assert result.timed_path.is_safe(scenario)


def test_moving_sphere_edges_use_exact_continuous_collision_predicate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    scenario = DynamicScenario(
        "moving-crossing",
        "Moving crossing",
        short_scene(),
        moving_spheres=(
            MovingSphere(
                "traffic",
                1.0,
                (
                    (0.0, (5.0, 5.0, 5.0)),
                    (2.0, (5.0, 5.0, 5.0)),
                    (3.0, (5.0, 9.0, 5.0)),
                ),
            ),
        ),
    )
    checked_edges: list[tuple[float, float]] = []
    exact_predicate = space_time_module._spacetime_segment_is_free

    def traced_predicate(
        checked_scenario: DynamicScenario,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        start_time: float,
        end_time: float,
    ) -> bool:
        checked_edges.append((start_time, end_time))
        return exact_predicate(checked_scenario, start, end, start_time, end_time)

    monkeypatch.setattr(space_time_module, "_spacetime_segment_is_free", traced_predicate)
    result = planner().plan(scenario)

    assert result.success, result.failure_reason
    assert checked_edges
    assert any(end > start for start, end in checked_edges)
    assert result.timed_path is not None
    assert result.timed_path.wait_time_s > 0
    assert result.timed_path.is_safe(scenario)


def test_voxel_time_search_is_deterministic_and_reports_expanded_states() -> None:
    scenario = DynamicScenario("detour", "Detour", corridor_scene(obstacle=True))
    first = planner(horizon=15.0).plan(scenario)
    second = planner(horizon=15.0).plan(scenario)

    assert first.success, first.failure_reason
    assert second.success
    assert first.timed_path == second.timed_path
    assert first.expanded_spacetime_states == second.expanded_spacetime_states
    assert first.generated_spacetime_states == second.generated_spacetime_states
    assert first.expanded_spacetime_states > 0
    assert first.parameters["work_unit"] == "expanded-spacetime-states"
    assert first.parameters["connectivity"] == 6
    assert first.timed_path is not None
    assert first.path[0] == scenario.static_scene.start
    assert first.path[-1] == scenario.static_scene.goal
    assert first.timed_path.is_safe(scenario)
    for waypoint, speed in zip(
        first.timed_path.waypoints[1:], first.timed_path.segment_speeds(), strict=True
    ):
        assert speed == pytest.approx(0.0 if waypoint.action == "wait" else 2.0)


def test_goal_is_only_accepted_when_safe_at_exact_arrival() -> None:
    scenario = DynamicScenario(
        "occupied-goal",
        "Occupied goal",
        short_scene(),
        temporary_cylinders=(TemporaryCylinder("goal-zone", (8.0, 5.0), 0.5, 0.0, 10.0, 0.0, 4.0),),
    )
    result = planner().plan(scenario)

    assert result.success, result.failure_reason
    assert result.timed_path is not None
    assert result.arrival_time_s is not None
    assert result.arrival_time_s >= 4.0
    assert point_is_free_at_time(scenario, scenario.static_scene.goal, result.arrival_time_s)
    assert result.timed_path.is_safe(scenario)


def test_finite_horizon_and_expansion_budget_have_explicit_failures() -> None:
    occupied_goal = DynamicScenario(
        "unavailable-goal",
        "Unavailable goal",
        short_scene(),
        temporary_cylinders=(
            TemporaryCylinder("goal-zone", (8.0, 5.0), 2.0, 0.0, 10.0, 0.0, 10.0),
        ),
    )
    horizon_result = planner(horizon=4.0).plan(occupied_goal)
    assert not horizon_result.success
    assert horizon_result.failure_reason == "time-horizon-exhausted"
    assert horizon_result.timed_path is None

    detour = DynamicScenario("budget", "Budget", corridor_scene(obstacle=True))
    budget_result = planner(horizon=15.0, max_expansions=1).plan(detour)
    assert not budget_result.success
    assert budget_result.failure_reason == "expansion-budget-exhausted"
    assert budget_result.expanded_spacetime_states == 1
    assert budget_result.generated_spacetime_states > 1


def test_invalid_start_and_clock_are_rejected_without_search() -> None:
    scene = short_scene()
    blocked_start = DynamicScenario(
        "blocked-start",
        "Blocked start",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("start-zone", (2.0, 5.0), 1.0, 0.0, 10.0, 0.0, 2.0),
        ),
    )
    result = planner().plan(blocked_start)
    assert not result.success
    assert result.failure_reason == "invalid-start"
    assert result.expanded_spacetime_states == 0
    with pytest.raises(ValueError, match="start_time"):
        planner().plan(DynamicScenario("empty", "Empty", scene), start_time=-1.0)
