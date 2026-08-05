"""Common planner protocol and serializable result object."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from uav3d.geometry import Point3
from uav3d.scene import Scene

Scalar = str | int | float | bool | None


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
        }


class Planner(Protocol):
    algorithm_id: str

    def plan(self, scene: Scene, seed: int = 0) -> PlanningResult: ...
