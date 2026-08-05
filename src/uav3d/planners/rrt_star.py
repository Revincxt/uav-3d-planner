"""Seeded continuous-space RRT* for static 3D scenes."""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass

from uav3d.collision import point_is_free, segment_is_free
from uav3d.geometry import Point3, add, distance, scale, subtract
from uav3d.planners.base import (
    BudgetUsage,
    PlanningBudget,
    PlanningResult,
    QualityTracePoint,
    Scalar,
)
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class RRTStarConfig:
    max_samples: int = 3_000
    step_size: float = 7.0
    goal_bias: float = 0.12
    goal_tolerance: float = 9.0
    neighbor_radius: float = 15.0
    rewire_gamma: float = 45.0
    max_wall_time_ms: float | None = None
    quality_checkpoints: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        if self.max_samples <= 0:
            raise ValueError("max_samples must be positive")
        if (
            self.step_size <= 0
            or self.goal_tolerance <= 0
            or self.neighbor_radius <= 0
            or self.rewire_gamma <= 0
        ):
            raise ValueError("RRT* distances must be positive")
        if not 0 <= self.goal_bias <= 1:
            raise ValueError("goal_bias must be in [0, 1]")
        if self.max_wall_time_ms is not None and self.max_wall_time_ms <= 0:
            raise ValueError("max_wall_time_ms must be positive when supplied")
        if any(checkpoint <= 0 for checkpoint in self.quality_checkpoints):
            raise ValueError("quality checkpoints must be positive")
        if len(set(self.quality_checkpoints)) != len(self.quality_checkpoints):
            raise ValueError("quality checkpoints must be unique")


class RRTStar:
    algorithm_id = "rrt-star"

    def __init__(self, config: RRTStarConfig | None = None) -> None:
        self.config = config or RRTStarConfig()

    def plan(self, scene: Scene, seed: int = 0) -> PlanningResult:
        started = time.perf_counter()
        parameters: dict[str, Scalar] = {
            "seed": seed,
            "max_samples": self.config.max_samples,
            "step_size": self.config.step_size,
            "goal_bias": self.config.goal_bias,
            "goal_tolerance": self.config.goal_tolerance,
            "neighbor_radius": self.config.neighbor_radius,
            "rewire_gamma": self.config.rewire_gamma,
            "max_wall_time_ms": self.config.max_wall_time_ms,
            "quality_checkpoints": tuple(sorted(self.config.quality_checkpoints)),
        }
        budget = PlanningBudget(
            "sample-attempts", self.config.max_samples, self.config.max_wall_time_ms
        )
        if self._wall_time_exhausted(started):
            return self._failure(started, "wall-time-budget-exhausted", parameters, budget=budget)
        endpoints_are_free = point_is_free(scene, scene.start) and point_is_free(scene, scene.goal)
        if self._wall_time_exhausted(started):
            return self._failure(started, "wall-time-budget-exhausted", parameters, budget=budget)
        if not endpoints_are_free:
            return self._failure(started, "invalid-start-or-goal", parameters, budget=budget)

        rng = random.Random(seed)
        nodes: list[Point3] = [scene.start]
        parents: list[int] = [-1]
        costs: list[float] = [0.0]
        accepted = 0
        completed_iterations = 0
        search_started = time.perf_counter()
        checkpoint_set = {
            checkpoint
            for checkpoint in self.config.quality_checkpoints
            if checkpoint <= self.config.max_samples
        }
        trace: list[QualityTracePoint] = []
        terminated_by = "sample-budget-completed"

        for iteration in range(1, self.config.max_samples + 1):
            if self._wall_time_exhausted(started):
                terminated_by = "wall-time-budget-exhausted"
                break
            sample = (
                scene.goal if rng.random() < self.config.goal_bias else self._sample(scene, rng)
            )
            nearest = min(range(len(nodes)), key=lambda index: distance(nodes[index], sample))
            candidate = self._steer(nodes[nearest], sample)
            candidate_moves = distance(nodes[nearest], candidate) > 1e-9
            candidate_is_safe = candidate_moves and point_is_free(scene, candidate)
            if candidate_is_safe:
                candidate_is_safe = segment_is_free(scene, nodes[nearest], candidate)
            if candidate_is_safe:
                node_count = len(nodes) + 1
                asymptotic_radius = self.config.rewire_gamma * (
                    math.log(node_count) / node_count
                ) ** (1 / 3)
                near_radius = min(self.config.neighbor_radius, asymptotic_radius)
                near_set = {
                    index
                    for index, point in enumerate(nodes)
                    if distance(point, candidate) <= near_radius
                    and segment_is_free(scene, point, candidate)
                }
                near_set.add(nearest)
                near = sorted(near_set)
                parent = nearest
                candidate_cost = costs[nearest] + distance(nodes[nearest], candidate)
                for index in near:
                    alternative = costs[index] + distance(nodes[index], candidate)
                    if alternative + 1e-12 < candidate_cost:
                        parent = index
                        candidate_cost = alternative

                new_index = len(nodes)
                nodes.append(candidate)
                parents.append(parent)
                costs.append(candidate_cost)
                accepted += 1

                for index in near:
                    if index == parent or index == 0:
                        continue
                    if self._is_ancestor(index, new_index, parents):
                        continue
                    rewired_cost = candidate_cost + distance(candidate, nodes[index])
                    if rewired_cost + 1e-12 < costs[index]:
                        old_cost = costs[index]
                        parents[index] = new_index
                        costs[index] = rewired_cost
                        self._propagate_cost_delta(index, old_cost - rewired_cost, parents, costs)

            completed_iterations = iteration
            if iteration in checkpoint_set:
                trace.append(self._quality_point(scene, nodes, costs, iteration, started))
            if self._wall_time_exhausted(started):
                terminated_by = "wall-time-budget-exhausted"
                break

        best_goal_parent = self._best_goal_parent(scene, nodes, costs)
        if self._wall_time_exhausted(started):
            terminated_by = "wall-time-budget-exhausted"

        if best_goal_parent is None:
            reason = (
                "wall-time-budget-exhausted"
                if terminated_by == "wall-time-budget-exhausted"
                else "sample-budget-exhausted"
            )
            return self._failure(
                started,
                reason,
                parameters,
                expanded=len(nodes),
                generated=accepted,
                iterations=completed_iterations,
                search_started=search_started,
                budget=budget,
                quality_trace=tuple(trace),
            )

        path = self._reconstruct(nodes, parents, best_goal_parent)
        if path[-1] != scene.goal:
            path.append(scene.goal)
        elapsed = (time.perf_counter() - started) * 1000
        setup_ms = (search_started - started) * 1000
        return PlanningResult(
            self.algorithm_id,
            True,
            tuple(path),
            elapsed,
            expanded_nodes=len(nodes),
            generated_nodes=accepted,
            iterations=completed_iterations,
            parameters=parameters,
            setup_ms=setup_ms,
            search_ms=max(0.0, elapsed - setup_ms),
            budget=budget,
            budget_usage=BudgetUsage(completed_iterations, elapsed, terminated_by),
            quality_trace=tuple(trace),
        )

    def _wall_time_exhausted(self, started: float) -> bool:
        limit = self.config.max_wall_time_ms
        return limit is not None and (time.perf_counter() - started) * 1000 >= limit

    def _best_goal_parent(
        self, scene: Scene, nodes: list[Point3], costs: list[float]
    ) -> int | None:
        goal_candidates = [
            index
            for index, point in enumerate(nodes)
            if distance(point, scene.goal) <= self.config.goal_tolerance
            and segment_is_free(scene, point, scene.goal)
        ]
        return min(
            goal_candidates,
            key=lambda index: (costs[index] + distance(nodes[index], scene.goal), index),
            default=None,
        )

    def _quality_point(
        self,
        scene: Scene,
        nodes: list[Point3],
        costs: list[float],
        iteration: int,
        started: float,
    ) -> QualityTracePoint:
        parent = self._best_goal_parent(scene, nodes, costs)
        best_length = (
            costs[parent] + distance(nodes[parent], scene.goal) if parent is not None else None
        )
        return QualityTracePoint(iteration, (time.perf_counter() - started) * 1000, best_length)

    def _sample(self, scene: Scene, rng: random.Random) -> Point3:
        clearance = scene.required_clearance
        return (
            rng.uniform(scene.bounds.minimum[0] + clearance, scene.bounds.maximum[0] - clearance),
            rng.uniform(scene.bounds.minimum[1] + clearance, scene.bounds.maximum[1] - clearance),
            rng.uniform(scene.bounds.minimum[2] + clearance, scene.bounds.maximum[2] - clearance),
        )

    def _steer(self, origin: Point3, target: Point3) -> Point3:
        delta = subtract(target, origin)
        length = distance(origin, target)
        if length <= self.config.step_size:
            return target
        return add(origin, scale(delta, self.config.step_size / length))

    def _propagate_cost_delta(
        self, root: int, improvement: float, parents: list[int], costs: list[float]
    ) -> None:
        if improvement <= 0:
            return
        stack = [root]
        while stack:
            parent = stack.pop()
            for index, candidate_parent in enumerate(parents):
                if candidate_parent == parent:
                    costs[index] -= improvement
                    stack.append(index)

    def _is_ancestor(self, candidate: int, node: int, parents: list[int]) -> bool:
        current = node
        while current >= 0:
            if current == candidate:
                return True
            current = parents[current]
        return False

    def _reconstruct(self, nodes: list[Point3], parents: list[int], current: int) -> list[Point3]:
        path = [nodes[current]]
        seen = {current}
        while parents[current] >= 0:
            current = parents[current]
            if current in seen:
                raise RuntimeError("RRT* parent cycle detected")
            seen.add(current)
            path.append(nodes[current])
        path.reverse()
        return path

    def _failure(
        self,
        started: float,
        reason: str,
        parameters: dict[str, Scalar],
        expanded: int = 0,
        generated: int = 0,
        iterations: int = 0,
        search_started: float | None = None,
        budget: PlanningBudget | None = None,
        quality_trace: tuple[QualityTracePoint, ...] = (),
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
            iterations=iterations,
            failure_reason=reason,
            parameters=parameters,
            setup_ms=setup_ms,
            search_ms=max(0.0, elapsed - setup_ms),
            budget=budget,
            budget_usage=BudgetUsage(iterations, elapsed, reason) if budget else None,
            quality_trace=quality_trace,
        )
