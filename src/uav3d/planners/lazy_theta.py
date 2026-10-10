"""Lazy Theta* over a collision-aware 26-connected voxel graph."""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass

from uav3d.collision import point_is_free, segment_is_free
from uav3d.flight_cost import flight_distance, validate_vertical_scale
from uav3d.geometry import Point3
from uav3d.planners.base import BudgetUsage, PlanningBudget, PlanningResult, Scalar
from uav3d.planners.grid import GridIndex, VoxelGrid, attach_exact_endpoints
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class LazyThetaStarConfig:
    resolution: float = 4.0
    max_expansions: int = 120_000
    max_wall_time_ms: float | None = None
    vertical_cost_scale: float = 1.0
    altitude_levels: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        validate_vertical_scale(self.vertical_cost_scale)
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        if self.max_expansions <= 0:
            raise ValueError("max_expansions must be positive")
        if self.max_wall_time_ms is not None and self.max_wall_time_ms <= 0:
            raise ValueError("max_wall_time_ms must be positive when supplied")


class LazyThetaStar:
    algorithm_id = "lazy-theta-star"

    def __init__(self, config: LazyThetaStarConfig | None = None) -> None:
        self.config = config or LazyThetaStarConfig()

    def plan(self, scene: Scene, seed: int = 0) -> PlanningResult:
        del seed
        started = time.perf_counter()
        parameters: dict[str, Scalar] = {
            "resolution": self.config.resolution,
            "max_expansions": self.config.max_expansions,
            "max_wall_time_ms": self.config.max_wall_time_ms,
            "connectivity": 26,
        }
        if self.config.vertical_cost_scale != 1:
            parameters["vertical_cost_scale"] = self.config.vertical_cost_scale
            parameters["objective"] = "cruise-equivalent-distance"
        if self.config.altitude_levels:
            parameters["altitude_layer_policy"] = "required-anchor-heights-plus-escape-layers"
            parameters["altitude_layers"] = repr(self.config.altitude_levels)
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

        grid = VoxelGrid(scene, self.config.resolution, self.config.altitude_levels)
        starts = set(grid.anchor_indices(scene.start))
        goals = set(grid.anchor_indices(scene.goal))
        if self._wall_time_exhausted(started):
            return self._failure(started, "wall-time-budget-exhausted", parameters, budget=budget)
        if not starts or not goals:
            return self._failure(started, "no-free-grid-anchor", parameters, budget=budget)

        parents: dict[GridIndex, GridIndex] = {start: start for start in starts}
        g_score: dict[GridIndex, float] = {
            start: self._cost(scene.start, grid.point(start)) for start in starts
        }
        queue: list[tuple[float, float, int, GridIndex]] = []
        counter = itertools.count()
        for start in sorted(starts):
            heuristic = self._cost(grid.point(start), scene.goal)
            heapq.heappush(
                queue,
                (g_score[start] + heuristic, heuristic, next(counter), start),
            )
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
            self._set_vertex(current, starts, parents, g_score, closed, grid)
            if math.isinf(g_score.get(current, math.inf)):
                continue
            if current in goals:
                grid_path = self._reconstruct(parents, current, starts, grid)
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
            assumed_parent = parents[current]
            parent_point = grid.point(assumed_parent)
            for neighbor in grid.neighbors(current):
                if neighbor in closed:
                    continue
                candidate = g_score[assumed_parent] + self._cost(parent_point, grid.point(neighbor))
                if candidate + 1e-12 >= g_score.get(neighbor, math.inf):
                    continue
                g_score[neighbor] = candidate
                parents[neighbor] = assumed_parent
                neighbor_h = self._cost(grid.point(neighbor), scene.goal)
                heapq.heappush(
                    queue,
                    (candidate + neighbor_h, neighbor_h, next(counter), neighbor),
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

    def _cost(self, a: Point3, b: Point3) -> float:
        return flight_distance(a, b, self.config.vertical_cost_scale)

    def _wall_time_exhausted(self, started: float) -> bool:
        limit = self.config.max_wall_time_ms
        return limit is not None and (time.perf_counter() - started) * 1000 >= limit

    def _set_vertex(
        self,
        current: GridIndex,
        starts: set[GridIndex],
        parents: dict[GridIndex, GridIndex],
        g_score: dict[GridIndex, float],
        closed: set[GridIndex],
        grid: VoxelGrid,
    ) -> None:
        if current in starts:
            return
        parent = parents[current]
        if segment_is_free(grid.scene, grid.point(parent), grid.point(current)):
            return
        best_parent: GridIndex | None = None
        best_cost = math.inf
        for neighbor in grid.neighbors(current):
            if neighbor not in closed:
                continue
            candidate = g_score[neighbor] + self._cost(grid.point(neighbor), grid.point(current))
            if candidate < best_cost:
                best_parent = neighbor
                best_cost = candidate
        if best_parent is None:
            g_score[current] = math.inf
            return
        parents[current] = best_parent
        g_score[current] = best_cost

    def _reconstruct(
        self,
        parents: dict[GridIndex, GridIndex],
        current: GridIndex,
        starts: set[GridIndex],
        grid: VoxelGrid,
    ) -> list[tuple[float, float, float]]:
        indices = [current]
        seen = {current}
        while current not in starts:
            current = parents[current]
            if current in seen:
                return []
            seen.add(current)
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
