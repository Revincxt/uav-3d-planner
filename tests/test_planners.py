from __future__ import annotations

import pytest

from uav3d.collision import path_is_free
from uav3d.planners import (
    AStar3D,
    AStarConfig,
    LazyThetaStar,
    LazyThetaStarConfig,
    RRTStar,
    RRTStarConfig,
)
from uav3d.planners.base import Planner
from uav3d.scene import AABB, Bounds3D, Scene
from uav3d.validation import audit_path


def detour_scene() -> Scene:
    return Scene(
        "detour",
        "Detour",
        Bounds3D((0, 0, 0), (30, 30, 20)),
        (3, 15, 5),
        (27, 15, 5),
        (AABB("barrier", (12, 8, 0), (18, 22, 12)),),
        drone_radius=0.5,
        safety_margin=0.5,
    )


@pytest.mark.parametrize(
    "planner",
    [
        AStar3D(AStarConfig(resolution=3, max_expansions=20_000)),
        LazyThetaStar(LazyThetaStarConfig(resolution=3, max_expansions=20_000)),
    ],
)
def test_grid_planners_return_exact_certified_endpoints(planner: Planner) -> None:
    scene = detour_scene()
    result = planner.plan(scene, seed=99)
    assert result.success, result.failure_reason
    assert result.path[0] == scene.start
    assert result.path[-1] == scene.goal
    assert audit_path(scene, result.path).valid


def test_graph_planners_use_direct_virtual_edge_when_visible() -> None:
    scene = Scene(
        "empty",
        "Empty",
        Bounds3D((0, 0, 0), (20, 20, 20)),
        (2, 2, 2),
        (18, 18, 18),
        drone_radius=0.5,
        safety_margin=0.5,
    )
    for planner in (AStar3D(), LazyThetaStar()):
        result = planner.plan(scene)
        assert result.success
        assert result.path == (scene.start, scene.goal)


def test_astar_is_deterministic() -> None:
    planner = AStar3D(AStarConfig(resolution=3))
    first = planner.plan(detour_scene())
    second = planner.plan(detour_scene())
    assert first.path == second.path
    assert first.expanded_nodes == second.expanded_nodes


def test_rrt_star_is_seeded_and_collision_free() -> None:
    scene = detour_scene()
    planner = RRTStar(
        RRTStarConfig(
            max_samples=900,
            step_size=4,
            goal_bias=0.18,
            goal_tolerance=5,
            neighbor_radius=10,
            rewire_gamma=30,
        )
    )
    first = planner.plan(scene, seed=12)
    second = planner.plan(scene, seed=12)
    assert first.success, first.failure_reason
    assert second.success
    assert first.path == second.path
    assert path_is_free(scene, first.path)


def test_rrt_star_reports_budget_exhaustion_without_claiming_no_path() -> None:
    scene = detour_scene()
    result = RRTStar(RRTStarConfig(max_samples=1, goal_bias=0)).plan(scene, seed=1)
    assert not result.success
    assert result.failure_reason == "sample-budget-exhausted"


def test_rrt_star_ancestor_check_prevents_rewire_cycles() -> None:
    planner = RRTStar()
    parents = [-1, 0, 1, 1]
    assert planner._is_ancestor(0, 3, parents)
    assert planner._is_ancestor(1, 3, parents)
    assert not planner._is_ancestor(2, 3, parents)
