"""Collision-aware 26-connected A* in a 3D voxel grid."""

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
class AStarConfig:
    resolution: float = 4.0
    max_expansions: int = 120_000


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
        starts = grid.anchor_indices(scene.start)
        goals = set(grid.anchor_indices(scene.goal))
        if not starts or not goals:
            return self._failure(started, "no-free-grid-anchor", parameters)

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

        while queue and len(closed) < self.config.max_expansions:
            _, _, _, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current in goals:
                grid_path = self._reconstruct(parents, current, grid)
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

        reason = "expansion-budget-exhausted" if queue else "no-path"
        return self._failure(started, reason, parameters, len(closed), generated)

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
