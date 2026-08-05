"""Seeded continuous-space RRT* for static 3D scenes."""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass

from uav3d.collision import point_is_free, segment_is_free
from uav3d.geometry import Point3, add, distance, scale, subtract
from uav3d.planners.base import PlanningResult, Scalar
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class RRTStarConfig:
    max_samples: int = 3_000
    step_size: float = 7.0
    goal_bias: float = 0.12
    goal_tolerance: float = 9.0
    neighbor_radius: float = 15.0
    rewire_gamma: float = 45.0

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
        }
        if not point_is_free(scene, scene.start) or not point_is_free(scene, scene.goal):
            return self._failure(started, "invalid-start-or-goal", parameters)

        rng = random.Random(seed)
        nodes: list[Point3] = [scene.start]
        parents: list[int] = [-1]
        costs: list[float] = [0.0]
        accepted = 0

        for _iteration in range(1, self.config.max_samples + 1):
            sample = (
                scene.goal if rng.random() < self.config.goal_bias else self._sample(scene, rng)
            )
            nearest = min(range(len(nodes)), key=lambda index: distance(nodes[index], sample))
            candidate = self._steer(nodes[nearest], sample)
            if distance(nodes[nearest], candidate) <= 1e-9:
                continue
            if not point_is_free(scene, candidate) or not segment_is_free(
                scene, nodes[nearest], candidate
            ):
                continue

            node_count = len(nodes) + 1
            asymptotic_radius = self.config.rewire_gamma * (math.log(node_count) / node_count) ** (
                1 / 3
            )
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

        goal_candidates = [
            index
            for index, point in enumerate(nodes)
            if distance(point, scene.goal) <= self.config.goal_tolerance
            and segment_is_free(scene, point, scene.goal)
        ]
        best_goal_parent = min(
            goal_candidates,
            key=lambda index: (costs[index] + distance(nodes[index], scene.goal), index),
            default=None,
        )

        if best_goal_parent is None:
            return self._failure(
                started,
                "sample-budget-exhausted",
                parameters,
                expanded=len(nodes),
                generated=accepted,
                iterations=self.config.max_samples,
            )

        path = self._reconstruct(nodes, parents, best_goal_parent)
        if path[-1] != scene.goal:
            path.append(scene.goal)
        return PlanningResult(
            self.algorithm_id,
            True,
            tuple(path),
            (time.perf_counter() - started) * 1000,
            expanded_nodes=len(nodes),
            generated_nodes=accepted,
            iterations=self.config.max_samples,
            parameters=parameters,
        )

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
    ) -> PlanningResult:
        return PlanningResult(
            self.algorithm_id,
            False,
            (),
            (time.perf_counter() - started) * 1000,
            expanded_nodes=expanded,
            generated_nodes=generated,
            iterations=iterations,
            failure_reason=reason,
            parameters=parameters,
        )
