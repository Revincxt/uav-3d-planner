from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise
from pathlib import Path

import pytest

from uav3d.analysis import clustered_metric_summary, quantile
from uav3d.benchmark import (
    _configuration_id,
    problem_fingerprint,
    run_benchmark,
    run_experiment,
    run_resolution_sweep,
    run_rrt_budget_curve,
)
from uav3d.geometry import polyline_length
from uav3d.planners import RRTStar, RRTStarConfig
from uav3d.planners.base import PlanningBudget
from uav3d.scene import AABB, Bounds3D, Scene, load_builtin_scene, load_scene, save_scene


def _open_scene() -> Scene:
    return Scene(
        "v2-open",
        "V2 open",
        Bounds3D((0, 0, 0), (30, 30, 20)),
        (2, 15, 5),
        (26, 15, 5),
        drone_radius=0.5,
        safety_margin=0.5,
    )


def _detour_scene() -> Scene:
    return Scene(
        "v2-detour",
        "V2 detour",
        Bounds3D((0, 0, 0), (30, 30, 20)),
        (3, 15, 5),
        (27, 15, 5),
        (AABB("barrier", (12, 8, 0), (18, 22, 12)),),
        drone_radius=0.5,
        safety_margin=0.5,
    )


def test_semantic_fingerprint_survives_round_trip_and_ignores_metadata(
    tmp_path: Path,
) -> None:
    scene = load_builtin_scene("vertical-gate")
    expected = problem_fingerprint(scene)

    output = tmp_path / "scene.json"
    save_scene(scene, output)
    assert problem_fingerprint(load_scene(output)) == expected

    committed = Path(__file__).parents[1] / "scenarios" / "vertical-gate.json"
    assert problem_fingerprint(load_scene(committed)) == expected

    relabeled = replace(
        scene,
        scene_id="renamed-scene",
        name="A presentation-only name",
        metadata={"description": "changed", "display_color": "violet"},
    )
    assert problem_fingerprint(relabeled) == expected

    changed_geometry = replace(scene, goal=(scene.goal[0] - 0.25, *scene.goal[1:]))
    assert problem_fingerprint(changed_geometry) != expected


def test_benchmark_deduplicates_seeds_for_deterministic_planners() -> None:
    scene = _open_scene()
    records = run_benchmark(
        [scene],
        ["astar-3d", "lazy-theta-star"],
        [3, 7, 11],
    )

    assert len(records) == 2
    assert {record.algorithm for record in records} == {"astar-3d", "lazy-theta-star"}
    assert all(record.seed is None for record in records)
    assert all(record.status == "success" for record in records)

    first = run_experiment(scene, "astar-3d", seed=3)
    second = run_experiment(scene, "astar-3d", seed=999)
    assert first.run_id == second.run_id


def test_equivalent_numeric_spellings_share_configuration_and_run_identity() -> None:
    integer_spelling = run_experiment(
        _open_scene(),
        "astar-3d",
        resolution=4,
        wall_time_limit_ms=100,
    )
    float_spelling = run_experiment(
        _open_scene(),
        "astar-3d",
        resolution=4.0,
        wall_time_limit_ms=100.0,
    )

    assert integer_spelling.configuration_id == float_spelling.configuration_id
    assert integer_spelling.run_id == float_spelling.run_id


def test_configuration_identity_can_reproduce_an_older_release() -> None:
    identifier = _configuration_id(
        "astar-3d",
        PlanningBudget("expanded-nodes", 120_000),
        {"resolution": 4.0},
        package_version="0.2.0",
    )
    assert identifier.startswith("astar-3d@0.2.0|resolution=4|")


@pytest.mark.parametrize(
    ("algorithm", "work_unit", "failure_reason"),
    [
        ("astar-3d", "expanded-nodes", "expansion-budget-exhausted"),
        ("lazy-theta-star", "expanded-nodes", "expansion-budget-exhausted"),
        ("rrt-star", "sample-attempts", "sample-budget-exhausted"),
    ],
)
def test_work_budget_usage_is_explicit(
    algorithm: str,
    work_unit: str,
    failure_reason: str,
) -> None:
    record = run_experiment(
        _detour_scene(),
        algorithm,
        seed=7,
        resolution=3 if algorithm != "rrt-star" else None,
        work_limit=1,
    )

    assert record.status == "budget-exhausted"
    assert record.failure_reason == failure_reason
    assert record.budget is not None
    assert record.budget.work_unit == work_unit
    assert record.budget.work_limit == 1
    assert record.budget_usage is not None
    assert record.budget_usage.work_used == 1
    assert record.budget_usage.termination == failure_reason
    assert record.planning_time_ms == pytest.approx(
        record.setup_time_ms + record.search_time_ms,
        abs=1e-9,
    )


@pytest.mark.parametrize("algorithm", ["astar-3d", "lazy-theta-star", "rrt-star"])
def test_wall_time_budget_has_a_distinct_failure_and_usage(algorithm: str) -> None:
    record = run_experiment(
        _detour_scene(),
        algorithm,
        seed=7,
        resolution=3 if algorithm != "rrt-star" else None,
        work_limit=100,
        wall_time_limit_ms=1e-6,
    )

    assert record.status == "timeout"
    assert record.failure_reason == "wall-time-budget-exhausted"
    assert record.budget is not None
    assert record.budget.wall_time_limit_ms == 1e-6
    assert record.budget_usage is not None
    assert record.budget_usage.termination == "wall-time-budget-exhausted"
    assert 0 <= record.budget_usage.work_used <= record.budget.work_limit
    assert record.budget_usage.wall_time_used_ms == pytest.approx(record.planning_time_ms)
    assert record.planning_time_ms == pytest.approx(
        record.setup_time_ms + record.search_time_ms,
        abs=1e-9,
    )


@pytest.mark.parametrize("algorithm", ["astar-3d", "lazy-theta-star", "rrt-star"])
def test_wall_deadline_applies_from_entry_even_when_the_direct_path_is_free(
    algorithm: str,
) -> None:
    record = run_experiment(
        _open_scene(),
        algorithm,
        seed=7,
        resolution=3 if algorithm != "rrt-star" else None,
        work_limit=100,
        wall_time_limit_ms=1e-9,
    )

    assert record.status == "timeout"
    assert record.failure_reason == "wall-time-budget-exhausted"
    assert record.budget_usage is not None
    assert record.budget_usage.termination == "wall-time-budget-exhausted"


def test_rrt_trace_checkpoints_are_monotone_and_seed_reproducible() -> None:
    scene = _open_scene()
    config = RRTStarConfig(
        max_samples=40,
        step_size=4,
        goal_bias=0.6,
        goal_tolerance=4,
        neighbor_radius=8,
        rewire_gamma=20,
        quality_checkpoints=(5, 15, 40),
    )

    first = RRTStar(config).plan(scene, seed=23)
    second = RRTStar(config).plan(scene, seed=23)

    assert first.success and second.success
    assert first.path == second.path
    assert first.iterations == second.iterations == 40
    assert first.budget_usage is not None
    assert first.budget_usage.work_used == 40
    assert [point.work for point in first.quality_trace] == [5, 15, 40]
    assert [point.work for point in second.quality_trace] == [5, 15, 40]
    assert [point.best_path_length_m for point in first.quality_trace] == [
        point.best_path_length_m for point in second.quality_trace
    ]
    assert all(
        earlier.elapsed_ms <= later.elapsed_ms for earlier, later in pairwise(first.quality_trace)
    )

    incumbents = [
        point.best_path_length_m
        for point in first.quality_trace
        if point.best_path_length_m is not None
    ]
    assert incumbents
    assert all(later <= earlier + 1e-12 for earlier, later in pairwise(incumbents))

    prefix_config = replace(config, max_samples=15, quality_checkpoints=())
    prefix = RRTStar(prefix_config).plan(scene, seed=23)
    checkpoint = next(point for point in first.quality_trace if point.work == 15)
    assert prefix.success
    assert checkpoint.best_path_length_m == pytest.approx(polyline_length(prefix.path))


def test_rrt_timeout_does_not_invent_an_unrequested_zero_checkpoint() -> None:
    result = RRTStar(
        RRTStarConfig(
            max_samples=20,
            max_wall_time_ms=1e-9,
            quality_checkpoints=(5, 20),
        )
    ).plan(_open_scene(), seed=23)

    assert not result.success
    assert result.failure_reason == "wall-time-budget-exhausted"
    assert result.quality_trace == ()


def test_sweeps_reject_duplicate_or_non_finite_budgets() -> None:
    with pytest.raises(ValueError, match="unique"):
        run_rrt_budget_curve([_open_scene()], [100, 100], [7])
    with pytest.raises(ValueError, match="finite and positive"):
        run_resolution_sweep([_open_scene()], [float("nan")])


def test_benchmark_rejects_duplicate_semantic_problems() -> None:
    relabeled = replace(_open_scene(), scene_id="same-geometry-new-label")
    with pytest.raises(ValueError, match="semantic planning problems must be unique"):
        run_benchmark([_open_scene(), relabeled], ["astar-3d"], [0])


@pytest.mark.parametrize(
    ("probability", "expected"),
    [(0.0, 0.0), (0.25, 7.5), (0.5, 15.0), (0.75, 22.5), (1.0, 30.0)],
)
def test_quantile_uses_type_7_linear_interpolation(
    probability: float,
    expected: float,
) -> None:
    assert quantile([0.0, 10.0, 20.0, 30.0], probability) == pytest.approx(expected)


@dataclass(frozen=True, slots=True)
class _MetricRecord:
    scene_id: str
    scene_fingerprint: str
    status: str
    value: float


def test_scene_cluster_summary_is_seeded_and_weights_scenes_equally() -> None:
    many_runs_one_scene = [
        _MetricRecord("scene-a", "fingerprint-a", "success", 1.0) for _ in range(100)
    ] + [_MetricRecord("scene-b", "fingerprint-b", "success", 9.0)]
    one_run_per_scene = [
        _MetricRecord("scene-a", "fingerprint-a", "success", 1.0),
        _MetricRecord("scene-b", "fingerprint-b", "success", 9.0),
    ]

    def summarize(records: list[_MetricRecord]) -> dict[str, object]:
        return clustered_metric_summary(
            records,  # type: ignore[arg-type]
            lambda record: record.value,  # type: ignore[attr-defined]
            conditioning="all-runs",
            estimator="scene-weighted-mean",
            resamples=250,
            seed=20260805,
        )

    repeated = summarize(many_runs_one_scene)
    balanced = summarize(one_run_per_scene)

    assert repeated["value"] == pytest.approx(5.0)
    assert repeated["value"] == balanced["value"]
    assert repeated["nScenes"] == 2
    assert repeated["nRuns"] == 101
    assert repeated == summarize(many_runs_one_scene)
    assert repeated == summarize(list(reversed(many_runs_one_scene)))


def test_scene_clusters_follow_problem_identity_instead_of_display_id() -> None:
    records = [
        _MetricRecord("shared-name", "fingerprint-a", "success", 1.0),
        _MetricRecord("shared-name", "fingerprint-b", "success", 9.0),
        _MetricRecord("renamed-a", "fingerprint-a", "success", 3.0),
    ]
    summary = clustered_metric_summary(
        records,  # type: ignore[arg-type]
        lambda record: record.value,  # type: ignore[attr-defined]
        conditioning="all-runs",
        estimator="scene-weighted-mean",
        resamples=50,
        seed=20260805,
    )

    assert summary["nScenes"] == 2
    assert summary["value"] == pytest.approx(5.5)
