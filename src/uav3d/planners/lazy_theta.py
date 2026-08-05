"""Lazy Theta* over a collision-aware 26-connected voxel graph."""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass

from uav3d.collision import point_is_free, segment_is_free
from uav3d.geometry import distance
from uav3d.planners.base import PlanningResult, Scalar
from uav3d.planners.grid import GridIndex, VoxelGrid, attach_exact_endpoints
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class LazyThetaStarConfig:
    resolution: float = 4.0
    max_expansions: int = 120_000


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
            "connectivity": 26,
        }
        if not point_is_free(scene, scene.start) or not point_is_free(scene, scene.goal):
            return self._failure(started, "invalid-start-or-goal", parameters)
        if segment_is_free(scene, scene.start, scene.goal):
            return PlanningResult(
                self.algorithm_id,
                True,
                (scene.start, scene.goal),
                (time.perf_counter() - started) * 1000,
                generated_nodes=2,
                parameters=parameters,
            )

        grid = VoxelGrid(scene, self.config.resolution)
        starts = set(grid.anchor_indices(scene.start))
        goals = set(grid.anchor_indices(scene.goal))
        if not starts or not goals:
            return self._failure(started, "no-free-grid-anchor", parameters)

        parents: dict[GridIndex, GridIndex] = {start: start for start in starts}
        g_score: dict[GridIndex, float] = {
            start: distance(scene.start, grid.point(start)) for start in starts
        }
        queue: list[tuple[float, float, int, GridIndex]] = []
        counter = itertools.count()
        for start in sorted(starts):
            heuristic = distance(grid.point(start), scene.goal)
            heapq.heappush(
                queue,
                (g_score[start] + heuristic, heuristic, next(counter), start),
            )
        closed: set[GridIndex] = set()
        generated = len(starts)

        while queue and len(closed) < self.config.max_expansions:
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
                        started, "endpoint-connection-failed", parameters, len(closed), generated
                    )
                return PlanningResult(
                    self.algorithm_id,
                    True,
                    path,
                    (time.perf_counter() - started) * 1000,
                    expanded_nodes=len(closed),
                    generated_nodes=generated,
                    iterations=len(closed),
                    parameters=parameters,
                )

            closed.add(current)
            assumed_parent = parents[current]
            parent_point = grid.point(assumed_parent)
            for neighbor in grid.neighbors(current):
                if neighbor in closed:
                    continue
                candidate = g_score[assumed_parent] + distance(parent_point, grid.point(neighbor))
                if candidate + 1e-12 >= g_score.get(neighbor, math.inf):
                    continue
                g_score[neighbor] = candidate
                parents[neighbor] = assumed_parent
                neighbor_h = distance(grid.point(neighbor), scene.goal)
                heapq.heappush(
                    queue,
                    (candidate + neighbor_h, neighbor_h, next(counter), neighbor),
                )
                generated += 1

        reason = "expansion-budget-exhausted" if queue else "no-path"
        return self._failure(started, reason, parameters, len(closed), generated)

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
            candidate = g_score[neighbor] + distance(grid.point(neighbor), grid.point(current))
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
    ) -> PlanningResult:
        return PlanningResult(
            self.algorithm_id,
            False,
            (),
            (time.perf_counter() - started) * 1000,
            expanded_nodes=expanded,
            generated_nodes=generated,
            iterations=expanded,
            failure_reason=reason,
            parameters=parameters,
        )
