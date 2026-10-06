"""Seeded continuous-space RRT* for static 3D scenes."""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import cast

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
    # Opt-in to preserve the sampling protocol of historical experiments.
    direct_path_check: bool = False
    global_goal_connection: bool = False
    informed_sampling: bool = False
    informed_uniform_ratio: float = 0.05
    informed_refresh_interval: int = 64

    def __post_init__(self) -> None:
        for name, integer_value in (
            ("max_samples", self.max_samples),
            ("informed_refresh_interval", self.informed_refresh_interval),
        ):
            if (
                isinstance(integer_value, bool)
                or not isinstance(integer_value, int)
                or integer_value <= 0
            ):
                raise ValueError(f"{name} must be a positive integer")
        for name, value in (
            ("step_size", self.step_size),
            ("goal_tolerance", self.goal_tolerance),
            ("neighbor_radius", self.neighbor_radius),
            ("rewire_gamma", self.rewire_gamma),
            ("max_wall_time_ms", self.max_wall_time_ms),
        ):
            if name == "max_wall_time_ms" and value is None:
                continue
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        for name, value in (
            ("goal_bias", self.goal_bias),
            ("informed_uniform_ratio", self.informed_uniform_ratio),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not 0 <= value <= 1
            ):
                raise ValueError(f"{name} must be finite and in [0, 1]")
        if any(
            isinstance(checkpoint, bool) or not isinstance(checkpoint, int) or checkpoint <= 0
            for checkpoint in self.quality_checkpoints
        ):
            raise ValueError("quality checkpoints must be positive integers")
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
        if (
            self.config.direct_path_check
            or self.config.global_goal_connection
            or self.config.informed_sampling
        ):
            parameters.update(
                direct_path_check=self.config.direct_path_check,
                global_goal_connection=self.config.global_goal_connection,
                informed_sampling=self.config.informed_sampling,
                informed_uniform_ratio=self.config.informed_uniform_ratio,
                informed_refresh_interval=self.config.informed_refresh_interval,
            )
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
        if self.config.direct_path_check:
            direct_is_safe = segment_is_free(scene, scene.start, scene.goal)
            if self._wall_time_exhausted(started):
                return self._failure(
                    started, "wall-time-budget-exhausted", parameters, budget=budget
                )
            if direct_is_safe:
                # The Euclidean lower bound is attained: further samples cannot improve it.
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
        # Node positions and this Scene's static collision geometry are immutable.
        # A rewire changes the cost, not the visibility of its final goal edge.
        goal_visibility: dict[int, bool] = {}
        incumbent = (
            self._best_goal_parent(scene, nodes, costs, goal_visibility, started)
            if self.config.informed_sampling
            else None
        )

        for iteration in range(1, self.config.max_samples + 1):
            if self._wall_time_exhausted(started):
                terminated_by = "wall-time-budget-exhausted"
                break
            if rng.random() < self.config.goal_bias:
                sample = scene.goal
            elif self.config.informed_sampling and incumbent is not None:
                best_length = costs[incumbent] + distance(nodes[incumbent], scene.goal)
                sample = self._sample_informed(scene, rng, best_length)
            else:
                sample = self._sample(scene, rng)
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
                near, edge_visibility = self._near_vertices(
                    scene, nodes, candidate, nearest, near_radius
                )
                parent = nearest
                candidate_cost = costs[nearest] + distance(nodes[nearest], candidate)
                for index in near:
                    alternative = costs[index] + distance(nodes[index], candidate)
                    if alternative + 1e-12 < candidate_cost and self._candidate_edge_is_free(
                        scene, nodes, candidate, index, edge_visibility
                    ):
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
                    rewired_cost = candidate_cost + distance(candidate, nodes[index])
                    if rewired_cost + 1e-12 < costs[index]:
                        if self._is_ancestor(index, new_index, parents):
                            continue
                        if not self._candidate_edge_is_free(
                            scene, nodes, candidate, index, edge_visibility
                        ):
                            continue
                        old_cost = costs[index]
                        parents[index] = new_index
                        costs[index] = rewired_cost
                        self._propagate_cost_delta(index, old_cost - rewired_cost, parents, costs)

                if self.config.informed_sampling:
                    candidate_goal_length = costs[new_index] + distance(candidate, scene.goal)
                    incumbent_length = (
                        costs[incumbent] + distance(nodes[incumbent], scene.goal)
                        if incumbent is not None
                        else math.inf
                    )
                    eligible = self.config.global_goal_connection or (
                        distance(candidate, scene.goal) <= self.config.goal_tolerance
                    )
                    if eligible and candidate_goal_length < incumbent_length:
                        visible = segment_is_free(scene, candidate, scene.goal)
                        goal_visibility[new_index] = visible
                        if visible:
                            incumbent = new_index

            completed_iterations = iteration
            if (
                self.config.informed_sampling
                and iteration % self.config.informed_refresh_interval == 0
            ):
                # Refresh old vertices too: subtree rewiring may improve their goal cost.
                incumbent = self._best_goal_parent(scene, nodes, costs, goal_visibility, started)
            if iteration in checkpoint_set:
                trace.append(
                    self._quality_point(scene, nodes, costs, iteration, started, goal_visibility)
                )
            if self._wall_time_exhausted(started):
                terminated_by = "wall-time-budget-exhausted"
                break

        best_goal_parent = self._best_goal_parent(
            scene,
            nodes,
            costs,
            goal_visibility,
            started
            if self.config.global_goal_connection or self.config.informed_sampling
            else None,
        )
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

    def _near_vertices(
        self,
        scene: Scene,
        nodes: list[Point3],
        candidate: Point3,
        nearest: int,
        radius: float,
    ) -> tuple[list[int], dict[int, bool]]:
        # Do not query LOS for every nearby vertex in a dense informed region.
        # Parent/rewire costs are checked first, in the historical index order.
        # An edge unable to improve either cost cannot change the resulting tree.
        near = {index for index, point in enumerate(nodes) if distance(point, candidate) <= radius}
        near.add(nearest)
        return sorted(near), {nearest: True}  # the steering edge was already certified

    @staticmethod
    def _candidate_edge_is_free(
        scene: Scene,
        nodes: list[Point3],
        candidate: Point3,
        index: int,
        visibility: dict[int, bool],
    ) -> bool:
        if index not in visibility:
            visibility[index] = segment_is_free(scene, nodes[index], candidate)
        return visibility[index]

    def _best_goal_parent(
        self,
        scene: Scene,
        nodes: list[Point3],
        costs: list[float],
        visibility: dict[int, bool] | None = None,
        started: float | None = None,
    ) -> int | None:
        visible = visibility if visibility is not None else {}
        candidates = []
        for index, point in enumerate(nodes):
            goal_distance = distance(point, scene.goal)
            if self.config.global_goal_connection or goal_distance <= self.config.goal_tolerance:
                candidates.append((costs[index] + goal_distance, index))
        incumbent = min(
            (candidate for candidate in candidates if visible.get(candidate[1]) is True),
            default=None,
        )
        # Cost + the exact final edge length is a lower bound even before its LOS
        # is known. Once the cheapest visible candidate is found, no later LOS
        # query can improve it. Rewires are reflected in freshly read costs.
        for candidate in sorted(candidates):
            if incumbent is not None and candidate >= incumbent:
                break
            index = candidate[1]
            if visible.get(index) is False:
                continue
            if started is not None and self._wall_time_exhausted(started):
                break
            visible[index] = segment_is_free(scene, nodes[index], scene.goal)
            if visible[index]:
                incumbent = candidate
                break
        return incumbent[1] if incumbent is not None else None

    def _quality_point(
        self,
        scene: Scene,
        nodes: list[Point3],
        costs: list[float],
        iteration: int,
        started: float,
        visibility: dict[int, bool] | None = None,
    ) -> QualityTracePoint:
        parent = self._best_goal_parent(
            scene,
            nodes,
            costs,
            visibility,
            started
            if self.config.global_goal_connection or self.config.informed_sampling
            else None,
        )
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

    def _sample_informed(self, scene: Scene, rng: random.Random, best_length: float) -> Point3:
        """Sample the prolate ellipsoid containing every shorter Euclidean route.

        This only changes proposal distribution; all accepted points and tree
        edges still undergo the same exact static collision checks. Rejection
        at the scene boundary is bounded, and a small uniform mixture retains
        global exploration. These draws are not extra tree/sample attempts.
        """

        if rng.random() < self.config.informed_uniform_ratio:
            return self._sample(scene, rng)
        minimum_length = distance(scene.start, scene.goal)
        if not math.isfinite(best_length) or best_length < minimum_length:
            return self._sample(scene, rng)
        axis = scale(subtract(scene.goal, scene.start), 1 / minimum_length)
        reference = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (0.0, 1.0, 0.0)
        cross = (
            axis[1] * reference[2] - axis[2] * reference[1],
            axis[2] * reference[0] - axis[0] * reference[2],
            axis[0] * reference[1] - axis[1] * reference[0],
        )
        cross_length = math.sqrt(sum(value * value for value in cross))
        second = scale(cross, 1 / cross_length)
        third = (
            axis[1] * second[2] - axis[2] * second[1],
            axis[2] * second[0] - axis[0] * second[2],
            axis[0] * second[1] - axis[1] * second[0],
        )
        center = scale(add(scene.start, scene.goal), 0.5)
        major = best_length / 2
        minor = (
            math.sqrt(max(0.0, (best_length - minimum_length) * (best_length + minimum_length))) / 2
        )
        clearance = scene.required_clearance
        for _ in range(32):
            if minor <= 1e-12:
                sample = add(scene.start, scale(subtract(scene.goal, scene.start), rng.random()))
            else:
                direction = (rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1))
                norm = math.sqrt(sum(value * value for value in direction))
                if norm <= 1e-12:
                    continue
                unit = scale(direction, rng.random() ** (1 / 3) / norm)
                sample = cast(
                    Point3,
                    tuple(
                        center[i]
                        + axis[i] * major * unit[0]
                        + second[i] * minor * unit[1]
                        + third[i] * minor * unit[2]
                        for i in range(3)
                    ),
                )
            if all(
                lower + clearance <= value <= upper - clearance
                for value, lower, upper in zip(
                    sample, scene.bounds.minimum, scene.bounds.maximum, strict=True
                )
            ):
                return sample
        return self._sample(scene, rng)

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
