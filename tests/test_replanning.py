from __future__ import annotations

from dataclasses import replace

import pytest

from uav3d.dynamic import list_builtin_dynamic_scenarios, load_builtin_dynamic_scenario
from uav3d.geometry import distance, polyline_length
from uav3d.planners import AStar3D, AStarConfig, DStarLite3D, DStarLiteConfig
from uav3d.replanning import REPLANNING_ALGORITHMS, simulate_replanning
from uav3d.scene import AABB, Bounds3D, Cylinder, Scene


def graph_scene() -> Scene:
    return Scene(
        "graph",
        "Graph",
        Bounds3D((0.0, 0.0, 0.0), (24.0, 24.0, 18.0)),
        (3.0, 12.0, 6.0),
        (21.0, 12.0, 6.0),
        (AABB("barrier", (9.0, 8.0, 0.0), (15.0, 16.0, 12.0)),),
        drone_radius=0.5,
        safety_margin=0.5,
    )


def test_dstar_lite_matches_astar_graph_cost_and_reachability() -> None:
    scene = graph_scene()
    astar = AStar3D(AStarConfig(resolution=3.0, max_expansions=20_000)).plan(scene)
    dstar = DStarLite3D(DStarLiteConfig(resolution=3.0, max_queue_pops=20_000)).plan(scene)
    assert dstar.success == astar.success
    assert dstar.success
    assert polyline_length(dstar.path) == pytest.approx(polyline_length(astar.path))

    impossible = replace(
        scene,
        buildings=(AABB("wall", (9.0, 0.0, 0.0), (15.0, 24.0, 18.0)),),
    )
    astar_failure = AStar3D(AStarConfig(resolution=3.0, max_expansions=20_000)).plan(impossible)
    dstar_failure = DStarLite3D(DStarLiteConfig(resolution=3.0, max_queue_pops=20_000)).plan(
        impossible
    )
    assert not astar_failure.success
    assert not dstar_failure.success


def test_dstar_lite_uses_the_shared_direct_endpoint_contract() -> None:
    scene = replace(graph_scene(), buildings=())
    astar = AStar3D(AStarConfig(resolution=3.0, max_expansions=20_000)).plan(scene)
    dstar = DStarLite3D(DStarLiteConfig(resolution=3.0, max_queue_pops=20_000)).plan(scene)

    assert astar.path == (scene.start, scene.goal)
    assert dstar.path == astar.path
    assert dstar.expanded_nodes == 0
    assert dstar.parameters["direct_line_of_sight"] is True


def test_dstar_lite_repeated_direct_calls_without_grid_anchors() -> None:
    scene = Scene(
        "sub-resolution-direct",
        "Sub-resolution direct",
        Bounds3D((0.0, 0.0, 0.0), (3.0, 3.0, 3.0)),
        (1.0, 1.0, 1.0),
        (2.0, 1.0, 1.0),
        drone_radius=0.1,
        safety_margin=0.1,
    )
    planner = DStarLite3D(DStarLiteConfig(resolution=4.0, max_queue_pops=100))

    first = planner.plan(scene)
    second = planner.plan(scene)

    assert first.success and second.success
    assert first.path == second.path == (scene.start, scene.goal)
    assert planner.g_values == {}
    assert planner.rhs_values == {}


def test_dstar_lite_reuses_state_and_records_changed_edges() -> None:
    initial_scene = graph_scene()
    planner = DStarLite3D(DStarLiteConfig(resolution=3.0, max_queue_pops=20_000))
    first = planner.plan(initial_scene)
    g_after_first = planner.g_values
    changed_scene = replace(
        initial_scene,
        no_fly_zones=(Cylinder("new-zone", (12.0, 4.0), 3.0, 0.0, 12.0),),
    )
    second = planner.plan(changed_scene)
    fresh = AStar3D(AStarConfig(resolution=3.0, max_expansions=20_000)).plan(changed_scene)
    assert first.success and second.success and fresh.success
    assert second.parameters["replan_index"] == 2
    assert isinstance(second.parameters["changed_edges"], int)
    assert second.parameters["changed_edges"] > 0
    assert set(g_after_first).intersection(planner.g_values)
    assert polyline_length(second.path) == pytest.approx(polyline_length(fresh.path))


def test_simulation_is_byte_for_byte_deterministic() -> None:
    scenario = load_builtin_dynamic_scenario("pop-up-nfz")
    first = simulate_replanning(scenario, "dstar-lite-3d", resolution=4.0, max_time=60.0)
    second = simulate_replanning(scenario, "dstar-lite-3d", resolution=4.0, max_time=60.0)
    assert first.to_dict() == second.to_dict()
    assert first.to_dict()["schema_version"] == "dynamic-run-v1"


@pytest.mark.parametrize("scenario_id", list_builtin_dynamic_scenarios())
@pytest.mark.parametrize("algorithm", REPLANNING_ALGORITHMS)
def test_all_builtin_runs_finish_without_executing_a_collision(
    scenario_id: str, algorithm: str
) -> None:
    run = simulate_replanning(
        load_builtin_dynamic_scenario(scenario_id),
        algorithm,
        resolution=4.0,
        max_time=100.0,
    )
    assert run.metrics.success, run.metrics.failure_reason
    assert run.metrics.collision_count == 0
    assert run.frames[-1].status == "arrived"
    if scenario_id == "crossing-traffic":
        assert run.metrics.safety_gate_activations > 0
        if algorithm == "dstar-lite-3d":
            assert run.metrics.total_changed_edges > 0


def test_simulation_rejects_unknown_algorithm_and_invalid_clock() -> None:
    scenario = load_builtin_dynamic_scenario("pop-up-nfz")
    with pytest.raises(ValueError, match="unknown replanning algorithm"):
        simulate_replanning(scenario, "unknown")
    with pytest.raises(ValueError, match="finite and positive"):
        simulate_replanning(scenario, "repeated-astar-3d", time_step=0.0)


def test_timeout_frame_records_the_post_movement_terminal_state() -> None:
    scenario = load_builtin_dynamic_scenario("pop-up-nfz")
    run = simulate_replanning(scenario, "repeated-astar-3d", max_time=0.5)

    assert not run.metrics.success
    assert run.metrics.failure_reason == "maximum-simulation-time"
    assert run.frames[-1].status == "timeout"
    assert run.frames[-1].time_s == pytest.approx(0.5)
    assert run.frames[-1].position != scenario.static_scene.start
    assert run.metrics.executed_path_length_m == pytest.approx(
        distance(scenario.static_scene.start, run.frames[-1].position)
    )
