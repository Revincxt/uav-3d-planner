"""Snapshot shortcuts never bypass the simulator's exact dynamic safety gate."""

from __future__ import annotations

import pytest

from uav3d.dynamic import DynamicScenario, load_builtin_dynamic_scenario
from uav3d.predictive_study import _reactive_timed_path
from uav3d.replanning import REPLANNING_ALGORITHMS, simulate_replanning
from uav3d.scene import AABB, Bounds3D, Scene


def barrier_scenario() -> DynamicScenario:
    return DynamicScenario(
        "shortcut-barrier",
        "Snapshot shortcut barrier",
        Scene(
            "shortcut-static",
            "Static barrier",
            Bounds3D((0.0, 0.0, 0.0), (40.0, 40.0, 20.0)),
            (4.0, 20.0, 6.0),
            (36.0, 20.0, 6.0),
            buildings=(AABB("barrier", (16.0, 14.0, 0.0), (24.0, 26.0, 16.0)),),
            drone_radius=0.5,
            safety_margin=0.5,
        ),
    )


@pytest.mark.parametrize("algorithm", REPLANNING_ALGORITHMS)
def test_snapshot_shortcut_preserves_safety_and_improves_grid_route(algorithm: str) -> None:
    scenario = barrier_scenario()
    options = {"resolution": 4.0, "max_time": 100.0, "replan_interval": 100.0}
    original = simulate_replanning(scenario, algorithm, **options)
    explicit_default = simulate_replanning(scenario, algorithm, shortcut_paths=False, **options)
    optimized = simulate_replanning(scenario, algorithm, shortcut_paths=True, **options)
    assert original.to_dict() == explicit_default.to_dict()
    assert optimized.metrics.success
    assert optimized.metrics.collision_count == 0
    assert optimized.metrics.executed_path_length_m <= original.metrics.executed_path_length_m
    assert _reactive_timed_path(optimized).is_safe(scenario)
    assert optimized.parameters["path_shortcut"] == 1
    assert "path_shortcut" not in original.parameters
    assert (
        optimized.to_dict()
        == simulate_replanning(scenario, algorithm, shortcut_paths=True, **options).to_dict()
    )


@pytest.mark.parametrize("algorithm", REPLANNING_ALGORITHMS)
@pytest.mark.parametrize("scenario_id", ["crossing-traffic", "pop-up-nfz"])
def test_snapshot_shortcut_remains_safe_under_unknown_future_events(
    algorithm: str, scenario_id: str
) -> None:
    scenario = load_builtin_dynamic_scenario(scenario_id)
    optimized = simulate_replanning(scenario, algorithm, shortcut_paths=True, max_time=100.0)
    assert optimized.metrics.success
    assert optimized.metrics.collision_count == 0
    # This independently audits every movement subsegment and wait, not just frame chords.
    assert _reactive_timed_path(optimized).is_safe(scenario)
