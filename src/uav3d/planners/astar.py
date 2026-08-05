"""Collision-aware 26-connected A* in a 3D voxel grid."""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass

from uav3d.collision import point_is_free, segment_is_free
from uav3d.geometry import distance
from uav3d.planners.base import BudgetUsage, PlanningBudget, PlanningResult, Scalar
from uav3d.planners.grid import GridIndex, VoxelGrid, attach_exact_endpoints
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class AStarConfig:
    resolution: float = 4.0
    max_expansions: int = 120_000
    max_wall_time_ms: float | None = None

    def __post_init__(self) -> None:
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        if self.max_expansions <= 0:
            raise ValueError("max_expansions must be positive")
        if self.max_wall_time_ms is not None and self.max_wall_time_ms <= 0:
            raise ValueError("max_wall_time_ms must be positive when supplied")


class AStar3D:
    algorithm_id = "astar-3d"

    def __init__(self, config: AStarConfig | None = None) -> None:
        self.config = config or AStarConfig()

    def plan(self, scene: Scene, seed: int = 0) -> PlanningResult:
        del seed
        started = time.perf_counter()
        parameters: dict[str, Scalar] = {
            "resolution": self.config.resolution,
            "max_expansions": self.config.max_expansions,
            "max_wall_time_ms": self.config.max_wall_time_ms,
            "connectivity": 26,
        }
        budget = PlanningBudget(
            "expanded-nodes", self.config.max_expansions, self.config.max_wall_time_ms
        )
        if self._wall_time_exhausted(started):
            return self._failure(started, "wall-time-budget-exhausted", parameters, budget=budget)
        endpoints_are_free = point_is_free(scene, scene.start) and point_is_free(scene, scene.goal)
        if self._wall_time_exhausted(started):
            return self._failure(started, "wall-time-budget-exhausted", parameters, budget=budget)
        if not endpoints_are_free:
            return self._failure(started, "invalid-start-or-goal", parameters, budget=budget)
        direct_path_is_free = segment_is_free(scene, scene.start, scene.goal)
        if self._wall_time_exhausted(started):
            return self._failure(started, "wall-time-budget-exhausted", parameters, budget=budget)
        if direct_path_is_free:
            elapsed = (time.perf_counter() - started) * 1000
            return PlanningResult(
                self.algorithm_id,
                True,
                (scene.start, scene.goal),
                elapsed,
                generated_nodes=2,
                parameters=parameters,
                setup_ms=elapsed,
                budget=budget,
                budget_usage=BudgetUsage(0, elapsed, "goal-reached"),
            )

        grid = VoxelGrid(scene, self.config.resolution)
        starts = grid.anchor_indices(scene.start)
        goals = set(grid.anchor_indices(scene.goal))
        if self._wall_time_exhausted(started):
            return self._failure(started, "wall-time-budget-exhausted", parameters, budget=budget)
        if not starts or not goals:
            return self._failure(started, "no-free-grid-anchor", parameters, budget=budget)

        queue: list[tuple[float, float, int, GridIndex]] = []
        counter = itertools.count()
        g_score: dict[GridIndex, float] = {}
        parents: dict[GridIndex, GridIndex] = {}
        for start in starts:
            start_cost = distance(scene.start, grid.point(start))
            g_score[start] = start_cost
            start_h = distance(grid.point(start), scene.goal)
            heapq.heappush(queue, (start_cost + start_h, start_h, next(counter), start))
        closed: set[GridIndex] = set()
        generated = len(starts)
        search_started = time.perf_counter()

        while queue:
            if len(closed) >= self.config.max_expansions:
                return self._failure(
                    started,
                    "expansion-budget-exhausted",
                    parameters,
                    len(closed),
                    generated,
                    search_started,
                    budget,
                )
            if self._wall_time_exhausted(started):
                return self._failure(
                    started,
                    "wall-time-budget-exhausted",
                    parameters,
                    len(closed),
                    generated,
                    search_started,
                    budget,
                )
            _, _, _, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current in goals:
                grid_path = self._reconstruct(parents, current, grid)
                path = attach_exact_endpoints(scene, grid_path, scene.start, scene.goal)
                if not path:
                    return self._failure(
                        started,
                        "endpoint-connection-failed",
                        parameters,
                        len(closed),
                        generated,
                        search_started,
                        budget,
                    )
                if self._wall_time_exhausted(started):
                    return self._failure(
                        started,
                        "wall-time-budget-exhausted",
                        parameters,
                        len(closed),
                        generated,
                        search_started,
                        budget,
                    )
                elapsed = (time.perf_counter() - started) * 1000
                setup_ms = (search_started - started) * 1000
                return PlanningResult(
                    self.algorithm_id,
                    True,
                    path,
                    elapsed,
                    expanded_nodes=len(closed),
                    generated_nodes=generated,
                    iterations=len(closed),
                    parameters=parameters,
                    setup_ms=setup_ms,
                    search_ms=max(0.0, elapsed - setup_ms),
                    budget=budget,
                    budget_usage=BudgetUsage(len(closed), elapsed, "goal-reached"),
                )
            closed.add(current)
            current_point = grid.point(current)
            for neighbor in grid.neighbors(current):
                if neighbor in closed:
                    continue
                candidate = g_score[current] + distance(current_point, grid.point(neighbor))
                if candidate + 1e-12 >= g_score.get(neighbor, math.inf):
                    continue
                parents[neighbor] = current
                g_score[neighbor] = candidate
                heuristic = distance(grid.point(neighbor), scene.goal)
                heapq.heappush(
                    queue,
                    (candidate + heuristic, heuristic, next(counter), neighbor),
                )
                generated += 1

        reason = (
            "wall-time-budget-exhausted"
            if self._wall_time_exhausted(started)
            else "graph-exhausted"
        )
        return self._failure(
            started,
            reason,
            parameters,
            len(closed),
            generated,
            search_started,
            budget,
        )

    def _wall_time_exhausted(self, started: float) -> bool:
        limit = self.config.max_wall_time_ms
        return limit is not None and (time.perf_counter() - started) * 1000 >= limit

    def _reconstruct(
        self, parents: dict[GridIndex, GridIndex], current: GridIndex, grid: VoxelGrid
    ) -> list[tuple[float, float, float]]:
        indices = [current]
        while current in parents:
            current = parents[current]
            indices.append(current)
        indices.reverse()
        return [grid.point(index) for index in indices]

    def _failure(
        self,
        started: float,
        reason: str,
        parameters: dict[str, Scalar],
        expanded: int = 0,
        generated: int = 0,
        search_started: float | None = None,
        budget: PlanningBudget | None = None,
    ) -> PlanningResult:
        elapsed = (time.perf_counter() - started) * 1000
        setup_ms = elapsed if search_started is None else (search_started - started) * 1000
        return PlanningResult(
            self.algorithm_id,
            False,
            (),
            elapsed,
            expanded_nodes=expanded,
            generated_nodes=generated,
            iterations=expanded,
            failure_reason=reason,
            parameters=parameters,
            setup_ms=setup_ms,
            search_ms=max(0.0, elapsed - setup_ms),
            budget=budget,
            budget_usage=BudgetUsage(expanded, elapsed, reason) if budget else None,
        )
