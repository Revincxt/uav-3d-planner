"""Scene-aware descriptive statistics for benchmark records."""

from __future__ import annotations

import random
import statistics
from collections import Counter
from collections.abc import Callable, Sequence
from typing import Protocol

from uav3d.planners.base import PlanningBudget, QualityTracePoint
from uav3d.validation import PathAudit

BOOTSTRAP_SEED = 20_260_805
BOOTSTRAP_RESAMPLES = 10_000


class RecordLike(Protocol):
    scene_id: str
    scene_fingerprint: str
    algorithm: str
    configuration_id: str
    status: str
    failure_reason: str | None
    planning_time_ms: float
    straight_line_distance_m: float
    smoothed_path_valid: bool
    raw_audit: PathAudit | None
    smoothed_audit: PathAudit | None
    budget: PlanningBudget | None
    quality_trace: tuple[QualityTracePoint, ...]


def quantile(values: Sequence[float], probability: float) -> float:
    """Return a linear-interpolated quantile with defined singleton behavior."""

    if not values:
        raise ValueError("quantile requires at least one value")
    if not 0 <= probability <= 1:
        raise ValueError("probability must be in [0, 1]")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1 - weight) + ordered[upper] * weight


def describe(values: Sequence[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "median": None, "q1": None, "q3": None, "iqr": None}
    q1 = quantile(values, 0.25)
    q3 = quantile(values, 0.75)
    return {
        "count": len(values),
        "median": statistics.median(values),
        "q1": q1,
        "q3": q3,
        "iqr": q3 - q1,
    }


def _bootstrap_interval(
    cluster_values: Sequence[float],
    statistic: Callable[[Sequence[float]], float],
    *,
    resamples: int,
    seed: int,
) -> tuple[float, float]:
    if not cluster_values:
        raise ValueError("cluster bootstrap requires at least one cluster")
    if resamples <= 0:
        raise ValueError("resamples must be positive")
    if len(cluster_values) == 1:
        return cluster_values[0], cluster_values[0]
    rng = random.Random(seed)
    count = len(cluster_values)
    draws = [
        statistic([cluster_values[rng.randrange(count)] for _ in range(count)])
        for _ in range(resamples)
    ]
    return quantile(draws, 0.025), quantile(draws, 0.975)


def clustered_metric_summary(
    records: Sequence[RecordLike],
    accessor: Callable[[RecordLike], float | None],
    *,
    conditioning: str,
    estimator: str = "median-of-scene-medians",
    resamples: int = BOOTSTRAP_RESAMPLES,
    seed: int = BOOTSTRAP_SEED,
) -> dict[str, object]:
    """Summarize one metric with scenes, rather than runs, as sampling units."""

    grouped: dict[str, list[float]] = {}
    for record in records:
        value = accessor(record)
        if value is not None:
            grouped.setdefault(record.scene_fingerprint, []).append(value)
    successes = sum(record.status == "success" for record in records)
    attempted_scenes = len({record.scene_fingerprint for record in records})
    if not grouped:
        return {
            "estimator": estimator,
            "value": None,
            "q1": None,
            "q3": None,
            "ci95Low": None,
            "ci95High": None,
            "conditioning": conditioning,
            "nScenes": attempted_scenes,
            "nDefinedScenes": 0,
            "nRuns": len(records),
            "nSuccesses": successes,
        }

    if estimator == "scene-weighted-mean":
        cluster_estimates = [statistics.fmean(grouped[key]) for key in sorted(grouped)]
        across: Callable[[Sequence[float]], float] = statistics.fmean
    elif estimator == "median-of-scene-medians":
        cluster_estimates = [statistics.median(grouped[key]) for key in sorted(grouped)]
        across = statistics.median
    else:
        raise ValueError(f"unsupported estimator: {estimator}")
    low, high = _bootstrap_interval(cluster_estimates, across, resamples=resamples, seed=seed)
    return {
        "estimator": estimator,
        "value": across(cluster_estimates),
        "q1": quantile(cluster_estimates, 0.25),
        "q3": quantile(cluster_estimates, 0.75),
        "ci95Low": low,
        "ci95High": high,
        "conditioning": conditioning,
        "nScenes": attempted_scenes,
        "nDefinedScenes": len(cluster_estimates),
        "nRuns": len(records),
        "nSuccesses": successes,
    }


def _success(record: RecordLike) -> float:
    return float(record.status == "success")


def _planning_time(record: RecordLike) -> float:
    return record.planning_time_ms


def _raw_excess(record: RecordLike) -> float | None:
    if record.status != "success" or record.raw_audit is None:
        return None
    return (record.raw_audit.length_m / record.straight_line_distance_m - 1) * 100


def _smoothed_excess(record: RecordLike) -> float | None:
    if (
        record.status != "success"
        or not record.smoothed_path_valid
        or record.smoothed_audit is None
    ):
        return None
    return (record.smoothed_audit.length_m / record.straight_line_distance_m - 1) * 100


def _clearance(record: RecordLike) -> float | None:
    if (
        record.status != "success"
        or not record.smoothed_path_valid
        or record.smoothed_audit is None
    ):
        return None
    return record.smoothed_audit.minimum_clearance_m


def planner_summaries(
    records: Sequence[RecordLike],
    *,
    resamples: int = BOOTSTRAP_RESAMPLES,
    seed: int = BOOTSTRAP_SEED,
) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[RecordLike]] = {}
    for record in records:
        grouped.setdefault((record.algorithm, record.configuration_id), []).append(record)
    summaries: list[dict[str, object]] = []
    for (algorithm, configuration_id), group in sorted(grouped.items()):
        failures = Counter(
            record.failure_reason or record.status for record in group if record.status != "success"
        )
        summaries.append(
            {
                "plannerId": algorithm,
                "budgetId": configuration_id,
                "budget": group[0].budget.to_dict() if group[0].budget else None,
                "runs": len(group),
                "successes": sum(record.status == "success" for record in group),
                "failureCounts": dict(sorted(failures.items())),
                "metrics": {
                    "successRate": clustered_metric_summary(
                        group,
                        _success,
                        conditioning="all-runs",
                        estimator="scene-weighted-mean",
                        resamples=resamples,
                        seed=seed,
                    ),
                    "planningTimeMs": clustered_metric_summary(
                        group,
                        _planning_time,
                        conditioning="all-runs",
                        resamples=resamples,
                        seed=seed,
                    ),
                    "rawPathExcessPct": clustered_metric_summary(
                        group,
                        _raw_excess,
                        conditioning="successful-runs",
                        resamples=resamples,
                        seed=seed,
                    ),
                    "smoothedPathExcessPct": clustered_metric_summary(
                        group,
                        _smoothed_excess,
                        conditioning="successful-runs",
                        resamples=resamples,
                        seed=seed,
                    ),
                    "minimumClearanceM": clustered_metric_summary(
                        group,
                        _clearance,
                        conditioning="successful-runs",
                        resamples=resamples,
                        seed=seed,
                    ),
                },
            }
        )
    return summaries


def resolution_sensitivity(records: Sequence[RecordLike]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str, float], list[RecordLike]] = {}
    for record in records:
        if record.algorithm == "rrt-star" or record.budget is None:
            continue
        resolution = _resolution_from_id(record.configuration_id)
        grouped.setdefault((record.scene_fingerprint, record.algorithm, resolution), []).append(
            record
        )
    points: list[dict[str, object]] = []
    for (fingerprint, algorithm, resolution), group in sorted(grouped.items()):
        values = [value for record in group if (value := _raw_excess(record)) is not None]
        distribution = describe(values)
        points.append(
            {
                "sceneId": sorted({record.scene_id for record in group})[0],
                "problemFingerprint": fingerprint,
                "plannerId": algorithm,
                "resolutionM": resolution,
                "runs": len(group),
                "successes": len(values),
                "medianRawExcessPct": distribution["median"],
                "q1RawExcessPct": distribution["q1"],
                "q3RawExcessPct": distribution["q3"],
            }
        )
    return points


def _resolution_from_id(configuration_id: str) -> float:
    marker = "resolution="
    try:
        tail = configuration_id.split(marker, 1)[1]
        return float(tail.split("|", 1)[0])
    except (IndexError, ValueError) as error:
        raise ValueError(f"configuration has no resolution: {configuration_id}") from error


def rrt_budget_sensitivity(records: Sequence[RecordLike]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[float]] = {}
    attempts: Counter[tuple[str, int]] = Counter()
    scene_ids: dict[str, set[str]] = {}
    for record in records:
        if record.algorithm != "rrt-star":
            continue
        scene_ids.setdefault(record.scene_fingerprint, set()).add(record.scene_id)
        for point in record.quality_trace:
            key = (record.scene_fingerprint, point.work)
            attempts[key] += 1
            if point.best_path_length_m is not None:
                excess = (point.best_path_length_m / record.straight_line_distance_m - 1) * 100
                grouped.setdefault(key, []).append(excess)
    points: list[dict[str, object]] = []
    for key in sorted(attempts):
        fingerprint, budget = key
        values = grouped.get(key, [])
        distribution = describe(values)
        points.append(
            {
                "sceneId": sorted(scene_ids[fingerprint])[0],
                "problemFingerprint": fingerprint,
                "sampleBudget": budget,
                "runs": attempts[key],
                "successes": len(values),
                "medianRawExcessPct": distribution["median"],
                "q1RawExcessPct": distribution["q1"],
                "q3RawExcessPct": distribution["q3"],
            }
        )
    return points
