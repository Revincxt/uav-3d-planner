"""Common planner protocol and serializable result object."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from uav3d.geometry import Point3
from uav3d.scene import Scene

Scalar = str | int | float | bool | None | tuple[int, ...]


@dataclass(frozen=True, slots=True)
class PlanningBudget:
    """Planner-independent contract for finite algorithmic and wall-clock work."""

    work_unit: str
    work_limit: int
    wall_time_limit_ms: float | None = None

    def __post_init__(self) -> None:
        if not self.work_unit:
            raise ValueError("work_unit must not be empty")
        if self.work_limit <= 0:
            raise ValueError("work_limit must be positive")
        if self.wall_time_limit_ms is not None and self.wall_time_limit_ms <= 0:
            raise ValueError("wall_time_limit_ms must be positive when supplied")

    def to_dict(self) -> dict[str, object]:
        return {
            "work_unit": self.work_unit,
            "work_limit": self.work_limit,
            "wall_time_limit_ms": self.wall_time_limit_ms,
        }


@dataclass(frozen=True, slots=True)
class BudgetUsage:
    """Observed budget usage and the condition that ended planning."""

    work_used: int
    wall_time_used_ms: float
    termination: str

    def to_dict(self) -> dict[str, object]:
        return {
            "work_used": self.work_used,
            "wall_time_used_ms": self.wall_time_used_ms,
            "termination": self.termination,
        }


@dataclass(frozen=True, slots=True)
class QualityTracePoint:
    """Best feasible path cost observed at a fixed algorithmic checkpoint."""

    work: int
    elapsed_ms: float
    best_path_length_m: float | None

    def to_dict(self) -> dict[str, object]:
        return {
            "work": self.work,
            "elapsed_ms": self.elapsed_ms,
            "best_path_length_m": self.best_path_length_m,
        }


@dataclass(frozen=True, slots=True)
class PlanningResult:
    algorithm: str
    success: bool
    path: tuple[Point3, ...]
    elapsed_ms: float
    expanded_nodes: int = 0
    generated_nodes: int = 0
    iterations: int = 0
    failure_reason: str | None = None
    parameters: dict[str, Scalar] = field(default_factory=dict)
    setup_ms: float = 0.0
    search_ms: float = 0.0
    budget: PlanningBudget | None = None
    budget_usage: BudgetUsage | None = None
    quality_trace: tuple[QualityTracePoint, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "algorithm": self.algorithm,
            "success": self.success,
            "path": [list(point) for point in self.path],
            "elapsed_ms": self.elapsed_ms,
            "expanded_nodes": self.expanded_nodes,
            "generated_nodes": self.generated_nodes,
            "iterations": self.iterations,
            "failure_reason": self.failure_reason,
            "parameters": self.parameters,
            "timing": {
                "planning_ms": self.elapsed_ms,
                "setup_ms": self.setup_ms,
                "search_ms": self.search_ms,
            },
            "budget": self.budget.to_dict() if self.budget else None,
            "budget_usage": self.budget_usage.to_dict() if self.budget_usage else None,
            "quality_trace": [point.to_dict() for point in self.quality_trace],
        }


class Planner(Protocol):
    algorithm_id: str

    def plan(self, scene: Scene, seed: int = 0) -> PlanningResult: ...
