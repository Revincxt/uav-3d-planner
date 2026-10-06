"""Audit bounded-speed diagonal actions without changing the legacy predictive graph."""

from __future__ import annotations

import heapq
import itertools
import math

import pytest

from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import distance, polyline_length
from uav3d.planners.grid import GridIndex, VoxelGrid
from uav3d.planners.space_time_astar import SpaceTimeAStar3D, SpaceTimeAStarConfig
from uav3d.scene import AABB, Bounds3D, Scene


def empty_scene() -> Scene:
    return Scene(
        "diagonal-clock",
        "Diagonal clock",
        Bounds3D((0.0, 0.0, 0.0), (20.0, 20.0, 20.0)),
        (2.0, 2.0, 10.0),
        (18.0, 18.0, 10.0),
        drone_radius=0.0,
        safety_margin=0.0,
    )


def diagonal_config(**overrides: float | int) -> SpaceTimeAStarConfig:
    parameters = {
        "resolution": 2.0,
        "time_step": 0.5,
        "cruise_speed": 2.0,
        "time_horizon": 20.0,
        "max_expansions": 20_000,
        "connectivity": 26,
    }
    parameters.update(overrides)
    return SpaceTimeAStarConfig(**parameters)


def test_default_fixed_speed_contract_and_diagonal_config_are_distinct() -> None:
    assert SpaceTimeAStarConfig().connectivity == 6
    with pytest.raises(ValueError, match="integer number of time steps"):
        SpaceTimeAStarConfig(resolution=3.0, time_step=1.0, cruise_speed=2.0)
    config = diagonal_config(resolution=3.0, time_step=1.0)
    assert config.movement_steps == 2
    for connectivity in (0, 18, 6.0, "26"):
        with pytest.raises(ValueError, match="connectivity"):
            SpaceTimeAStarConfig(connectivity=connectivity)


def test_diagonal_edge_durations_round_up_and_never_exceed_speed() -> None:
    scenario = DynamicScenario("clock", "Clock", empty_scene())
    grid = VoxelGrid(scenario.static_scene, 2.0)
    planner = SpaceTimeAStar3D(diagonal_config())
    index = (5, 5, 5)
    successors = planner._successors(scenario, grid, (index, 0), 3.25)

    assert len(successors) == 27
    assert (index, 1) in successors
    assert ((6, 5, 5), 2) in successors
    assert ((6, 6, 5), 3) in successors
    assert ((6, 6, 6), 4) in successors
    for neighbor, step in successors:
        assert distance(grid.point(index), grid.point(neighbor)) / (step * 0.5) <= 2.0

    legacy = SpaceTimeAStar3D(diagonal_config(connectivity=6))
    legacy_successors = legacy._successors(scenario, grid, (index, 0), 3.25)
    assert len(legacy_successors) == 7
    assert all(step == 2 for neighbor, step in legacy_successors if neighbor != index)


def test_diagonal_edge_checks_the_interior_and_temporary_activation() -> None:
    scene = empty_scene()
    grid = VoxelGrid(scene, 2.0)
    planner = SpaceTimeAStar3D(diagonal_config())
    scenario = DynamicScenario(
        "interior-closure",
        "Interior closure",
        scene,
        temporary_cylinders=(
            TemporaryCylinder("mid-edge", (11.0, 11.0), 0.2, 9.0, 11.0, 0.7, 0.8),
        ),
    )
    successors = planner._successors(scenario, grid, ((5, 5, 5), 0), 0.0)

    # Both vertices are clear, but the aircraft crosses the active volume at t=0.75 s.
    assert ((6, 6, 5), 3) not in successors
    assert ((6, 5, 5), 2) in successors
    assert ((5, 6, 5), 2) in successors
    assert ((5, 5, 5), 1) in successors


def test_diagonal_duration_cannot_cross_the_finite_horizon() -> None:
    scenario = DynamicScenario("horizon", "Horizon", empty_scene())
    grid = VoxelGrid(scenario.static_scene, 2.0)
    config = diagonal_config()
    successors = SpaceTimeAStar3D(config)._successors(
        scenario, grid, ((5, 5, 5), config.horizon_steps - 2), 0.0
    )

    assert ((6, 5, 5), config.horizon_steps) in successors
    assert not any(neighbor == (6, 6, 5) for neighbor, _ in successors)
    assert all(step <= config.horizon_steps for _, step in successors)


def _reference_earliest_grid_path(
    scenario: DynamicScenario, config: SpaceTimeAStarConfig
) -> tuple[float, float]:
    """Independent time-ordered graph enumeration with exact vertex endpoints.

    The oracle has no A* heuristic or incumbent pruning. It independently constructs all 26 motion
    edges and waits, retaining the least geometric length for each vertex/time state.
    """

    grid = VoxelGrid(scenario.static_scene, config.resolution)
    start = grid.nearest_index(scenario.static_scene.start)
    goal = grid.nearest_index(scenario.static_scene.goal)
    queue = [(0, 0.0, start)]
    lengths = {(start, 0): 0.0}
    while queue:
        step, length, index = heapq.heappop(queue)
        if length > lengths[(index, step)] + 1e-9:
            continue
        if index == goal:
            return step * config.time_step, length
        for delta in itertools.product((-1, 0, 1), repeat=3):
            neighbor = tuple(value + offset for value, offset in zip(index, delta, strict=True))
            if not grid.contains(neighbor):
                continue
            edge_length = distance(grid.point(index), grid.point(neighbor))
            duration_steps = max(
                1, math.ceil(edge_length / (config.cruise_speed * config.time_step) - 1e-12)
            )
            arrival_step = step + duration_steps
            if arrival_step > config.horizon_steps:
                continue
            if not spacetime_segment_is_free(
                scenario,
                grid.point(index),
                grid.point(neighbor),
                step * config.time_step,
                arrival_step * config.time_step,
            ):
                continue
            candidate = length + edge_length
            state = (neighbor, arrival_step)
            if candidate >= lengths.get(state, math.inf) - 1e-9:
                continue
            lengths[state] = candidate
            heapq.heappush(queue, (arrival_step, candidate, neighbor))
    raise AssertionError("reference fixture must have a feasible path")


@pytest.mark.parametrize("hazard", ("static", "temporary", "moving"))
def test_search_matches_exhaustive_earliest_arrival_and_length_ties(
    monkeypatch: pytest.MonkeyPatch, hazard: str
) -> None:
    scene = Scene(
        "diagonal-detour",
        "Diagonal detour",
        Bounds3D((0.0, 0.0, 0.0), (12.0, 12.0, 4.0)),
        (2.0, 2.0, 2.0),
        (10.0, 10.0, 2.0),
        buildings=(AABB("wall", (4.0, 2.0, 0.0), (6.0, 8.0, 4.0)),),
        drone_radius=0.0,
        safety_margin=0.0,
    )
    scenario = DynamicScenario(
        hazard,
        hazard,
        scene,
        temporary_cylinders=(
            (TemporaryCylinder("closing-lane", (8.0, 8.0), 0.7, 0.0, 4.0, 0.0, 7.0),)
            if hazard == "temporary"
            else ()
        ),
        moving_spheres=(
            (
                MovingSphere(
                    "crossing",
                    0.7,
                    ((0.0, (8.0, 2.0, 2.0)), (5.0, (8.0, 10.0, 2.0))),
                ),
            )
            if hazard == "moving"
            else ()
        ),
    )
    config = diagonal_config()

    # Restrict these exact vertex endpoints to one connector each, so the exhaustive oracle and
    # production search operate on the same graph. Normal endpoint-stencil behavior is unchanged.
    def exact_anchor(grid: VoxelGrid, point: tuple[float, float, float]) -> list[GridIndex]:
        index = grid.nearest_index(point)
        assert grid.point(index) == point
        return [index]

    monkeypatch.setattr(VoxelGrid, "anchor_indices", exact_anchor)
    expected_arrival, expected_length = _reference_earliest_grid_path(scenario, config)
    planner = SpaceTimeAStar3D(config)
    result = planner.plan(scenario)
    repeated = planner.plan(scenario)

    assert result.success, result.failure_reason
    assert result.arrival_time_s == pytest.approx(expected_arrival)
    assert polyline_length(result.path) == pytest.approx(expected_length)
    assert result.timed_path == repeated.timed_path
    assert result.timed_path is not None
    assert result.timed_path.is_safe(scenario)
    assert max(result.timed_path.segment_speeds()) <= config.cruise_speed + 1e-9
    assert result.parameters["connectivity"] == 26
    assert result.parameters["fixed_cruise_speed"] is False
    assert result.parameters["secondary_objective"] == "path-length-at-equal-arrival"


def test_diagonal_search_preserves_expansion_failure_contract() -> None:
    scene = Scene(
        "budget",
        "Budget",
        Bounds3D((0.0, 0.0, 0.0), (16.0, 8.0, 4.0)),
        (2.0, 4.0, 2.0),
        (14.0, 4.0, 2.0),
        buildings=(AABB("barrier", (7.0, 3.0, 0.0), (9.0, 5.0, 4.0)),),
        drone_radius=0.0,
        safety_margin=0.0,
    )
    result = SpaceTimeAStar3D(diagonal_config(max_expansions=1)).plan(
        DynamicScenario("budget", "Budget", scene)
    )
    assert not result.success
    assert result.failure_reason == "expansion-budget-exhausted"
    assert result.expanded_spacetime_states == 1
    assert result.timed_path is None
