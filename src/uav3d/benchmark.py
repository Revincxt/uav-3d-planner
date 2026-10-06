"""Reproducible experiment runner and v2 record contracts."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any

from uav3d.analysis import describe
from uav3d.geometry import Point3, distance
from uav3d.planners import (
    AStar3D,
    AStarConfig,
    LazyThetaStar,
    LazyThetaStarConfig,
    Planner,
    RRTStar,
    RRTStarConfig,
)
from uav3d.planners.base import BudgetUsage, PlanningBudget, QualityTracePoint
from uav3d.scene import Scene
from uav3d.smoothing import SmoothingResult, smooth_path
from uav3d.validation import PathAudit, audit_path
from uav3d.version import __version__

PLANNER_IDS = ("astar-3d", "lazy-theta-star", "rrt-star")


def list_planners() -> tuple[str, ...]:
    return PLANNER_IDS


def make_planner(
    algorithm: str,
    *,
    resolution: float | None = None,
    work_limit: int | None = None,
    wall_time_limit_ms: float | None = None,
    quality_checkpoints: tuple[int, ...] = (),
) -> Planner:
    """Construct a planner while keeping algorithm and budget overrides explicit."""

    if algorithm == "astar-3d":
        if quality_checkpoints:
            raise ValueError("quality checkpoints are only supported by RRT*")
        astar_defaults = AStarConfig()
        return AStar3D(
            AStarConfig(
                resolution=resolution if resolution is not None else astar_defaults.resolution,
                max_expansions=(
                    work_limit if work_limit is not None else astar_defaults.max_expansions
                ),
                max_wall_time_ms=wall_time_limit_ms,
            )
        )
    if algorithm == "lazy-theta-star":
        if quality_checkpoints:
            raise ValueError("quality checkpoints are only supported by RRT*")
        theta_defaults = LazyThetaStarConfig()
        return LazyThetaStar(
            LazyThetaStarConfig(
                resolution=resolution if resolution is not None else theta_defaults.resolution,
                max_expansions=(
                    work_limit if work_limit is not None else theta_defaults.max_expansions
                ),
                max_wall_time_ms=wall_time_limit_ms,
            )
        )
    if algorithm == "rrt-star":
        if resolution is not None:
            raise ValueError("voxel resolution does not apply to RRT*")
        rrt_defaults = RRTStarConfig()
        return RRTStar(
            RRTStarConfig(
                max_samples=work_limit if work_limit is not None else rrt_defaults.max_samples,
                step_size=rrt_defaults.step_size,
                goal_bias=rrt_defaults.goal_bias,
                goal_tolerance=rrt_defaults.goal_tolerance,
                neighbor_radius=rrt_defaults.neighbor_radius,
                rewire_gamma=rrt_defaults.rewire_gamma,
                max_wall_time_ms=wall_time_limit_ms,
                quality_checkpoints=quality_checkpoints,
            )
        )
    choices = ", ".join(PLANNER_IDS)
    raise ValueError(f"unknown algorithm {algorithm!r}; choose one of: {choices}")


def _canonical_number(value: float) -> str:
    normalized = 0.0 if float(value) == 0 else float(value)
    return normalized.hex()


def problem_fingerprint(scene: Scene) -> str:
    """Hash planning semantics, including reserved hard-stop constraints when present.

    Ordinary descriptive metadata remains excluded, preserving legacy fingerprints.
    """

    canonical = {
        "fingerprint_schema": "uav3d-problem-v2",
        "bounds": {
            "minimum": [_canonical_number(value) for value in scene.bounds.minimum],
            "maximum": [_canonical_number(value) for value in scene.bounds.maximum],
        },
        "start": [_canonical_number(value) for value in scene.start],
        "goal": [_canonical_number(value) for value in scene.goal],
        "drone_radius": _canonical_number(scene.drone_radius),
        "safety_margin": _canonical_number(scene.safety_margin),
        "buildings": sorted(
            (
                {
                    "minimum": [_canonical_number(value) for value in building.minimum],
                    "maximum": [_canonical_number(value) for value in building.maximum],
                }
                for building in scene.buildings
            ),
            key=lambda item: (item["minimum"], item["maximum"]),
        ),
        "no_fly_zones": sorted(
            (
                {
                    "center": [_canonical_number(value) for value in zone.center],
                    "radius": _canonical_number(zone.radius),
                    "z_min": _canonical_number(zone.z_min),
                    "z_max": _canonical_number(zone.z_max),
                }
                for zone in scene.no_fly_zones
            ),
            key=lambda item: (item["center"], item["radius"], item["z_min"], item["z_max"]),
        ),
    }
    if scene.metadata.get("missionTaskPoints"):
        canonical["ordered_service_stops"] = [
            {
                "position": [_canonical_number(value) for value in task["position"]],
                "service_duration_s": _canonical_number(task["serviceDurationS"]),
            }
            for task in scene.metadata["missionTaskPoints"]
        ]
    payload = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def scene_fingerprint(scene: Scene) -> str:
    """Backward-compatible name for the v2 semantic problem fingerprint."""

    return problem_fingerprint(scene)


def _canonical_parameter(value: object) -> object:
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, (int, float)):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("planner parameters must be finite")
        return {"number": _canonical_number(number)}
    if isinstance(value, (list, tuple)):
        return [_canonical_parameter(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _canonical_parameter(item) for key, item in sorted(value.items())}
    raise TypeError(f"unsupported planner parameter type: {type(value).__name__}")


def _configuration_id(
    algorithm: str,
    budget: PlanningBudget,
    parameters: dict[str, object],
    *,
    package_version: str = __version__,
) -> str:
    """Return a stable planner identity, with an override for auditing old releases."""

    if not package_version:
        raise ValueError("package_version must not be empty")
    parts = [f"{algorithm}@{package_version}"]
    resolution = parameters.get("resolution")
    if resolution is not None:
        if not isinstance(resolution, (int, float)):
            raise TypeError("planner resolution must be numeric")
        parts.append(f"resolution={float(resolution):g}")
    parts.append(f"{budget.work_unit}={budget.work_limit}")
    wall = "none" if budget.wall_time_limit_ms is None else f"{budget.wall_time_limit_ms:g}"
    parts.append(f"wall-ms={wall}")
    identity_parameters = _canonical_parameter(
        {key: value for key, value in parameters.items() if key != "seed"}
    )
    parameter_payload = json.dumps(
        identity_parameters,
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    parts.append(f"params={hashlib.sha256(parameter_payload).hexdigest()[:12]}")
    return "|".join(parts)


def _run_id(
    fingerprint: str,
    configuration_id: str,
    seed: int | None,
    run_purpose: str,
    timing_repetition: int | None,
) -> str:
    payload = json.dumps(
        {
            "problem_fingerprint": fingerprint,
            "configuration_id": configuration_id,
            "planner_seed": seed,
            "run_purpose": run_purpose,
            "timing_repetition": timing_repetition,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return "run-" + hashlib.sha256(payload).hexdigest()[:20]


@dataclass(frozen=True, slots=True)
class ExperimentRecord:
    run_id: str
    run_purpose: str
    timing_repetition: int | None
    scene_id: str
    scene_fingerprint: str
    algorithm: str
    configuration_id: str
    seed: int | None
    status: str
    failure_reason: str | None
    planner_solution_found: bool
    raw_path_valid: bool
    postprocessing_status: str
    smoothed_path_valid: bool
    straight_line_distance_m: float
    planning_time_ms: float
    setup_time_ms: float
    search_time_ms: float
    smoothing_time_ms: float
    validation_time_ms: float
    expanded_nodes: int
    generated_nodes: int
    iterations: int
    parameters: dict[str, object]
    budget: PlanningBudget | None
    budget_usage: BudgetUsage | None
    quality_trace: tuple[QualityTracePoint, ...]
    raw_path: tuple[Point3, ...]
    smoothed_path: tuple[Point3, ...]
    raw_audit: PathAudit | None
    smoothed_audit: PathAudit | None
    smoothing: SmoothingResult | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": "2.0",
            "run_id": self.run_id,
            "run_purpose": self.run_purpose,
            "timing_repetition": self.timing_repetition,
            "scene_id": self.scene_id,
            "problem_fingerprint": self.scene_fingerprint,
            "scene_fingerprint": self.scene_fingerprint,
            "algorithm": self.algorithm,
            "configuration_id": self.configuration_id,
            "seed": self.seed,
            "status": self.status,
            "failure_reason": self.failure_reason,
            "planner_solution_found": self.planner_solution_found,
            "raw_path_valid": self.raw_path_valid,
            "postprocessing_status": self.postprocessing_status,
            "smoothed_path_valid": self.smoothed_path_valid,
            "straight_line_distance_m": self.straight_line_distance_m,
            "planning_time_ms": self.planning_time_ms,
            "timing": {
                "planning_ms": self.planning_time_ms,
                "setup_ms": self.setup_time_ms,
                "search_ms": self.search_time_ms,
                "smoothing_ms": self.smoothing_time_ms,
                "validation_ms": self.validation_time_ms,
            },
            "expanded_nodes": self.expanded_nodes,
            "generated_nodes": self.generated_nodes,
            "iterations": self.iterations,
            "parameters": self.parameters,
            "budget": self.budget.to_dict() if self.budget else None,
            "budget_usage": self.budget_usage.to_dict() if self.budget_usage else None,
            "quality_trace": [point.to_dict() for point in self.quality_trace],
            "raw_path": [list(point) for point in self.raw_path],
            "smoothed_path": [list(point) for point in self.smoothed_path],
            "raw_audit": self.raw_audit.to_dict() if self.raw_audit else None,
            "smoothed_audit": self.smoothed_audit.to_dict() if self.smoothed_audit else None,
            "smoothing": self.smoothing.to_dict() if self.smoothing else None,
        }


def _failure_status(reason: str | None) -> str:
    if reason == "wall-time-budget-exhausted":
        return "timeout"
    if reason in {"sample-budget-exhausted", "expansion-budget-exhausted"}:
        return "budget-exhausted"
    if reason == "invalid-start-or-goal":
        return "invalid"
    return "no-path"


def _validate_scenes(scenes: list[Scene]) -> None:
    if not scenes:
        raise ValueError("provide at least one scene")
    scene_ids = [scene.scene_id for scene in scenes]
    if len(scene_ids) != len(set(scene_ids)):
        raise ValueError("scene IDs must be unique")
    fingerprints = [problem_fingerprint(scene) for scene in scenes]
    if len(fingerprints) != len(set(fingerprints)):
        raise ValueError("semantic planning problems must be unique")


def run_experiment(
    scene: Scene,
    algorithm: str,
    seed: int = 0,
    *,
    resolution: float | None = None,
    work_limit: int | None = None,
    wall_time_limit_ms: float | None = None,
    quality_checkpoints: tuple[int, ...] = (),
    run_purpose: str = "path-quality",
    timing_repetition: int | None = None,
) -> ExperimentRecord:
    planner = make_planner(
        algorithm,
        resolution=resolution,
        work_limit=work_limit,
        wall_time_limit_ms=wall_time_limit_ms,
        quality_checkpoints=quality_checkpoints,
    )
    planned = planner.plan(scene, seed)
    fingerprint = problem_fingerprint(scene)
    if planned.budget is None:
        raise RuntimeError("planner returned no explicit budget contract")
    configuration_id = _configuration_id(algorithm, planned.budget, dict(planned.parameters))
    planner_seed = seed if algorithm == "rrt-star" else None
    common: dict[str, Any] = {
        "run_id": _run_id(
            fingerprint, configuration_id, planner_seed, run_purpose, timing_repetition
        ),
        "run_purpose": run_purpose,
        "timing_repetition": timing_repetition,
        "scene_id": scene.scene_id,
        "scene_fingerprint": fingerprint,
        "algorithm": algorithm,
        "configuration_id": configuration_id,
        "seed": planner_seed,
        "straight_line_distance_m": distance(scene.start, scene.goal),
        "planning_time_ms": planned.elapsed_ms,
        "setup_time_ms": planned.setup_ms,
        "search_time_ms": planned.search_ms,
        "expanded_nodes": planned.expanded_nodes,
        "generated_nodes": planned.generated_nodes,
        "iterations": planned.iterations,
        "parameters": dict(planned.parameters),
        "budget": planned.budget,
        "budget_usage": planned.budget_usage,
        "quality_trace": planned.quality_trace,
    }
    if not planned.success:
        return ExperimentRecord(
            **common,
            status=_failure_status(planned.failure_reason),
            failure_reason=planned.failure_reason,
            planner_solution_found=False,
            raw_path_valid=False,
            postprocessing_status="not-run",
            smoothed_path_valid=False,
            smoothing_time_ms=0.0,
            validation_time_ms=0.0,
            raw_path=(),
            smoothed_path=(),
            raw_audit=None,
            smoothed_audit=None,
            smoothing=None,
        )

    validation_started = time.perf_counter()
    raw_audit = audit_path(scene, planned.path)
    validation_time_ms = (time.perf_counter() - validation_started) * 1000
    if not raw_audit.valid:
        return ExperimentRecord(
            **common,
            status="invalid",
            failure_reason="planner-returned-invalid-path",
            planner_solution_found=True,
            raw_path_valid=False,
            postprocessing_status="not-run",
            smoothed_path_valid=False,
            smoothing_time_ms=0.0,
            validation_time_ms=validation_time_ms,
            raw_path=planned.path,
            smoothed_path=(),
            raw_audit=raw_audit,
            smoothed_audit=None,
            smoothing=None,
        )

    smoothing_started = time.perf_counter()
    smoothing = smooth_path(scene, planned.path)
    smoothing_time_ms = (time.perf_counter() - smoothing_started) * 1000
    validation_started = time.perf_counter()
    smoothed_audit = audit_path(scene, smoothing.path)
    validation_time_ms += (time.perf_counter() - validation_started) * 1000
    postprocessing_status = "success" if smoothed_audit.valid else "failed"
    return ExperimentRecord(
        **common,
        status="success",
        failure_reason=None,
        planner_solution_found=True,
        raw_path_valid=True,
        postprocessing_status=postprocessing_status,
        smoothed_path_valid=smoothed_audit.valid,
        smoothing_time_ms=smoothing_time_ms,
        validation_time_ms=validation_time_ms,
        raw_path=planned.path,
        smoothed_path=smoothing.path,
        raw_audit=raw_audit,
        smoothed_audit=smoothed_audit,
        smoothing=smoothing,
    )


def run_benchmark(
    scenes: list[Scene],
    algorithms: list[str],
    seeds: list[int],
    *,
    wall_time_limit_ms: float | None = None,
) -> list[ExperimentRecord]:
    """Run quality cases without pseudo-replicating deterministic planners."""

    if not seeds:
        raise ValueError("provide at least one planner seed")
    _validate_scenes(scenes)
    if len(seeds) != len(set(seeds)):
        raise ValueError("planner seeds must be unique")
    if len(algorithms) != len(set(algorithms)):
        raise ValueError("algorithms must be unique")
    records: list[ExperimentRecord] = []
    for scene in scenes:
        for algorithm in algorithms:
            planner_seeds = seeds if algorithm == "rrt-star" else [0]
            records.extend(
                run_experiment(
                    scene,
                    algorithm,
                    seed,
                    wall_time_limit_ms=wall_time_limit_ms,
                )
                for seed in planner_seeds
            )
    return records


def run_resolution_sweep(
    scenes: list[Scene],
    resolutions: list[float],
    *,
    work_limit: int | None = None,
    wall_time_limit_ms: float | None = None,
) -> list[ExperimentRecord]:
    _validate_scenes(scenes)
    if not resolutions:
        raise ValueError("provide at least one voxel resolution")
    if any(not math.isfinite(resolution) or resolution <= 0 for resolution in resolutions):
        raise ValueError("voxel resolutions must be finite and positive")
    if len(resolutions) != len(set(resolutions)):
        raise ValueError("voxel resolutions must be unique")
    return [
        run_experiment(
            scene,
            algorithm,
            resolution=resolution,
            work_limit=work_limit,
            wall_time_limit_ms=wall_time_limit_ms,
            run_purpose="resolution-sweep",
        )
        for scene in scenes
        for algorithm in ("astar-3d", "lazy-theta-star")
        for resolution in resolutions
    ]


def run_rrt_budget_curve(
    scenes: list[Scene],
    budgets: list[int],
    seeds: list[int],
    *,
    wall_time_limit_ms: float | None = None,
) -> list[ExperimentRecord]:
    _validate_scenes(scenes)
    if not budgets or any(
        isinstance(budget, bool) or not isinstance(budget, int) or budget <= 0 for budget in budgets
    ):
        raise ValueError("RRT* budgets must contain positive integers")
    if len(budgets) != len(set(budgets)):
        raise ValueError("RRT* budgets must be unique")
    if not seeds:
        raise ValueError("provide at least one planner seed")
    if len(seeds) != len(set(seeds)):
        raise ValueError("planner seeds must be unique")
    checkpoints = tuple(sorted(set(budgets)))
    return [
        run_experiment(
            scene,
            "rrt-star",
            seed,
            work_limit=checkpoints[-1],
            wall_time_limit_ms=wall_time_limit_ms,
            quality_checkpoints=checkpoints,
            run_purpose="rrt-budget-curve",
        )
        for scene in scenes
        for seed in seeds
    ]


def summarize_records(records: list[ExperimentRecord]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str, str], list[ExperimentRecord]] = {}
    for record in records:
        grouped.setdefault((record.scene_id, record.algorithm, record.configuration_id), []).append(
            record
        )
    summaries: list[dict[str, object]] = []
    for (scene_id, algorithm, configuration_id), group in sorted(grouped.items()):
        successes = [record for record in group if record.status == "success"]
        planning = describe([record.planning_time_ms for record in group])
        raw_lengths = describe(
            [record.raw_audit.length_m for record in successes if record.raw_audit is not None]
        )
        smoothed_lengths = describe(
            [
                record.smoothed_audit.length_m
                for record in successes
                if record.smoothed_audit is not None and record.smoothed_path_valid
            ]
        )
        clearances = describe(
            [
                record.smoothed_audit.minimum_clearance_m
                for record in successes
                if record.smoothed_audit is not None and record.smoothed_path_valid
            ]
        )
        summaries.append(
            {
                "scene_id": scene_id,
                "algorithm": algorithm,
                "configuration_id": configuration_id,
                "runs": len(group),
                "successes": len(successes),
                "success_rate": len(successes) / len(group),
                "planning_time_ms": planning,
                "raw_length_m": raw_lengths,
                "smoothed_length_m": smoothed_lengths,
                "minimum_clearance_m": clearances,
                "median_planning_time_ms": planning["median"],
                "median_raw_length_m": raw_lengths["median"],
                "median_smoothed_length_m": smoothed_lengths["median"],
                "median_minimum_clearance_m": clearances["median"],
            }
        )
    return summaries
