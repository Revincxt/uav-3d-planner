"""Reproducible experiment runner and compact statistical summaries."""

from __future__ import annotations

import hashlib
import json
import statistics
from dataclasses import dataclass
from typing import Any

from uav3d.planners import AStar3D, LazyThetaStar, Planner, RRTStar
from uav3d.scene import Scene
from uav3d.smoothing import SmoothingResult, smooth_path
from uav3d.validation import PathAudit, audit_path

PLANNER_FACTORIES: dict[str, type[AStar3D] | type[LazyThetaStar] | type[RRTStar]] = {
    "astar-3d": AStar3D,
    "lazy-theta-star": LazyThetaStar,
    "rrt-star": RRTStar,
}


def list_planners() -> tuple[str, ...]:
    return tuple(PLANNER_FACTORIES)


def make_planner(algorithm: str) -> Planner:
    try:
        return PLANNER_FACTORIES[algorithm]()
    except KeyError as error:
        choices = ", ".join(PLANNER_FACTORIES)
        raise ValueError(f"unknown algorithm {algorithm!r}; choose one of: {choices}") from error


def scene_fingerprint(scene: Scene) -> str:
    canonical = scene.to_dict()
    canonical["buildings"] = sorted(canonical["buildings"], key=lambda item: item["id"])
    canonical["no_fly_zones"] = sorted(canonical["no_fly_zones"], key=lambda item: item["id"])
    payload = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()
    return "sha256:" + hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True, slots=True)
class ExperimentRecord:
    scene_id: str
    scene_fingerprint: str
    algorithm: str
    seed: int
    status: str
    failure_reason: str | None
    planning_time_ms: float
    expanded_nodes: int
    generated_nodes: int
    iterations: int
    parameters: dict[str, object]
    raw_path: tuple[tuple[float, float, float], ...]
    smoothed_path: tuple[tuple[float, float, float], ...]
    raw_audit: PathAudit | None
    smoothed_audit: PathAudit | None
    smoothing: SmoothingResult | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_id": self.scene_id,
            "scene_fingerprint": self.scene_fingerprint,
            "algorithm": self.algorithm,
            "seed": self.seed,
            "status": self.status,
            "failure_reason": self.failure_reason,
            "planning_time_ms": self.planning_time_ms,
            "expanded_nodes": self.expanded_nodes,
            "generated_nodes": self.generated_nodes,
            "iterations": self.iterations,
            "parameters": self.parameters,
            "raw_path": [list(point) for point in self.raw_path],
            "smoothed_path": [list(point) for point in self.smoothed_path],
            "raw_audit": self.raw_audit.to_dict() if self.raw_audit else None,
            "smoothed_audit": self.smoothed_audit.to_dict() if self.smoothed_audit else None,
            "smoothing": self.smoothing.to_dict() if self.smoothing else None,
        }


def run_experiment(scene: Scene, algorithm: str, seed: int = 0) -> ExperimentRecord:
    planner = make_planner(algorithm)
    planned = planner.plan(scene, seed)
    if not planned.success:
        return ExperimentRecord(
            scene.scene_id,
            scene_fingerprint(scene),
            algorithm,
            seed,
            "no-path",
            planned.failure_reason,
            planned.elapsed_ms,
            planned.expanded_nodes,
            planned.generated_nodes,
            planned.iterations,
            dict(planned.parameters),
            (),
            (),
            None,
            None,
            None,
        )
    raw_audit = audit_path(scene, planned.path)
    if not raw_audit.valid:
        return ExperimentRecord(
            scene.scene_id,
            scene_fingerprint(scene),
            algorithm,
            seed,
            "invalid",
            "planner-returned-invalid-path",
            planned.elapsed_ms,
            planned.expanded_nodes,
            planned.generated_nodes,
            planned.iterations,
            dict(planned.parameters),
            planned.path,
            (),
            raw_audit,
            None,
            None,
        )
    smoothing = smooth_path(scene, planned.path)
    smoothed_audit = audit_path(scene, smoothing.path)
    status = "success" if smoothed_audit.valid else "invalid"
    reason = None if smoothed_audit.valid else "smoothing-certification-failed"
    return ExperimentRecord(
        scene.scene_id,
        scene_fingerprint(scene),
        algorithm,
        seed,
        status,
        reason,
        planned.elapsed_ms,
        planned.expanded_nodes,
        planned.generated_nodes,
        planned.iterations,
        dict(planned.parameters),
        planned.path,
        smoothing.path,
        raw_audit,
        smoothed_audit,
        smoothing,
    )


def run_benchmark(
    scenes: list[Scene], algorithms: list[str], seeds: list[int]
) -> list[ExperimentRecord]:
    return [
        run_experiment(scene, algorithm, seed)
        for scene in scenes
        for algorithm in algorithms
        for seed in seeds
    ]


def summarize_records(records: list[ExperimentRecord]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[ExperimentRecord]] = {}
    for record in records:
        grouped.setdefault((record.scene_id, record.algorithm), []).append(record)
    summaries: list[dict[str, object]] = []
    for (scene_id, algorithm), group in sorted(grouped.items()):
        successes = [record for record in group if record.status == "success"]
        lengths = [
            record.smoothed_audit.length_m
            for record in successes
            if record.smoothed_audit is not None
        ]
        clearances = [
            record.smoothed_audit.minimum_clearance_m
            for record in successes
            if record.smoothed_audit is not None
        ]
        summaries.append(
            {
                "scene_id": scene_id,
                "algorithm": algorithm,
                "runs": len(group),
                "successes": len(successes),
                "success_rate": len(successes) / len(group),
                "median_planning_time_ms": statistics.median(
                    record.planning_time_ms for record in group
                ),
                "median_smoothed_length_m": statistics.median(lengths) if lengths else None,
                "median_minimum_clearance_m": statistics.median(clearances) if clearances else None,
            }
        )
    return summaries
