"""Incremental D* Lite search on the shared collision-aware 3D voxel graph."""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass

from uav3d.collision import point_is_free, segment_is_free
from uav3d.flight_cost import flight_distance, validate_vertical_scale
from uav3d.geometry import Point3, dot, subtract
from uav3d.planners.base import BudgetUsage, PlanningBudget, PlanningResult, Scalar
from uav3d.planners.grid import GridIndex, VoxelGrid, attach_exact_endpoints
from uav3d.scene import Scene

Key = tuple[float, float]
Edge = tuple[GridIndex, GridIndex]


@dataclass(frozen=True, slots=True)
class DStarLiteConfig:
    resolution: float = 4.0
    max_queue_pops: int = 120_000
    vertical_cost_scale: float = 1.0
    altitude_levels: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        validate_vertical_scale(self.vertical_cost_scale)
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("resolution must be finite and positive")
        if self.max_queue_pops <= 0:
            raise ValueError("max_queue_pops must be positive")


class DStarLite3D:
    """Stateful D* Lite planner that preserves ``g``/``rhs`` across scene updates."""

    algorithm_id = "dstar-lite-3d"

    def __init__(self, config: DStarLiteConfig | None = None) -> None:
        self.config = config or DStarLiteConfig()
        self._grid: VoxelGrid | None = None
        self._scene: Scene | None = None
        self._goal_point: Point3 | None = None
        self._goal_terminal_costs: dict[GridIndex, float] = {}
        self._start: GridIndex | None = None
        self._last_start: GridIndex | None = None
        self._start_anchor_candidates_last = 0
        self._selected_goal_last: GridIndex | None = None
        self._g: dict[GridIndex, float] = {}
        self._rhs: dict[GridIndex, float] = {}
        self._queue: list[tuple[float, float, int, GridIndex]] = []
        self._open_keys: dict[GridIndex, Key] = {}
        self._counter = itertools.count()
        self._edge_costs: dict[Edge, float] = {}
        self._km = 0.0
        self._queue_pops_last = 0
        self._queue_pops_total = 0
        self._changed_edges_last = 0
        self._replans = 0

    @property
    def queue_pops_last(self) -> int:
        return self._queue_pops_last

    @property
    def queue_pops_total(self) -> int:
        return self._queue_pops_total

    @property
    def changed_edges_last(self) -> int:
        return self._changed_edges_last

    @property
    def replans(self) -> int:
        return self._replans

    @property
    def g_values(self) -> dict[GridIndex, float]:
        """A defensive copy exposed for reproducibility audits."""

        return dict(self._g)

    @property
    def rhs_values(self) -> dict[GridIndex, float]:
        """A defensive copy exposed for reproducibility audits."""

        return dict(self._rhs)

    def reset(self) -> None:
        self._grid = None
        self._scene = None
        self._goal_point = None
        self._goal_terminal_costs.clear()
        self._start = None
        self._last_start = None
        self._start_anchor_candidates_last = 0
        self._selected_goal_last = None
        self._g.clear()
        self._rhs.clear()
        self._queue.clear()
        self._open_keys.clear()
        self._counter = itertools.count()
        self._edge_costs.clear()
        self._km = 0.0
        self._queue_pops_last = 0
        self._queue_pops_total = 0
        self._changed_edges_last = 0
        self._replans = 0

    def plan(self, scene: Scene, seed: int = 0) -> PlanningResult:
        del seed
        started = time.perf_counter()
        self._start_anchor_candidates_last = 0
        self._selected_goal_last = None
        budget = PlanningBudget("queue-pops", self.config.max_queue_pops)
        parameters: dict[str, Scalar] = {
            "resolution": self.config.resolution,
            "max_queue_pops": self.config.max_queue_pops,
            "connectivity": 26,
            "incremental": True,
            "anchor_policy": "goal-aware-start-multi-goal-v1",
        }
        if self.config.vertical_cost_scale != 1:
            parameters["vertical_cost_scale"] = self.config.vertical_cost_scale
            parameters["objective"] = "cruise-equivalent-distance"
        if self.config.altitude_levels:
            parameters["altitude_layer_policy"] = "required-anchor-heights-plus-escape-layers"
            parameters["altitude_layers"] = repr(self.config.altitude_levels)
        if not point_is_free(scene, scene.start) or not point_is_free(scene, scene.goal):
            self._queue_pops_last = 0
            self._changed_edges_last = 0
            return self._result(
                started,
                False,
                (),
                "invalid-start-or-goal",
                parameters,
                budget,
            )
        if segment_is_free(scene, scene.start, scene.goal):
            # Match the shared graph-planner endpoint contract. Existing incremental state can
            # remain cached. Updating or initializing the lazy search state here preserves the
            # replan index and lets a later obstructed snapshot reuse the same problem contract,
            # without spending queue-pop work on an already certified direct path.
            cache_ready = (
                self._update_problem(scene) if self._compatible(scene) else self._initialize(scene)
            )
            if not cache_ready:
                # Direct visibility does not require voxel anchors. Do not retain the partial
                # state left by a failed grid initialization, because a later call must not
                # mistake it for a reusable incremental problem.
                self.reset()
            self._replans += 1
            self._queue_pops_last = 0
            parameters.update(
                {
                    "queue_pops": 0,
                    "queue_pops_total": self._queue_pops_total,
                    "changed_edges": self._changed_edges_last,
                    "replan_index": self._replans,
                    "direct_line_of_sight": True,
                }
            )
            return self._result(
                started,
                True,
                (scene.start, scene.goal),
                None,
                parameters,
                budget,
            )
        if not self._compatible(scene):
            initialized = self._initialize(scene)
            if not initialized:
                return self._result(
                    started,
                    False,
                    (),
                    "no-free-grid-anchor",
                    parameters,
                    budget,
                )
        else:
            updated = self._update_problem(scene)
            if not updated:
                self.reset()
                if not self._initialize(scene):
                    return self._result(
                        started,
                        False,
                        (),
                        "no-free-grid-anchor",
                        parameters,
                        budget,
                    )
        self._replans += 1
        self._queue_pops_last = 0
        exhausted = not self._compute_shortest_path()
        parameters.update(
            {
                "queue_pops": self._queue_pops_last,
                "queue_pops_total": self._queue_pops_total,
                "changed_edges": self._changed_edges_last,
                "replan_index": self._replans,
            }
        )
        if exhausted:
            return self._result(
                started,
                False,
                (),
                "queue-pop-budget-exhausted",
                parameters,
                budget,
            )
        path = self._extract_path(scene)
        if not path:
            return self._result(
                started,
                False,
                (),
                "graph-exhausted",
                parameters,
                budget,
            )
        return self._result(started, True, path, None, parameters, budget)

    def _compatible(self, scene: Scene) -> bool:
        if (
            self._grid is None
            or self._scene is None
            or self._goal_point is None
            or self._start is None
            or not self._goal_terminal_costs
        ):
            return False
        old = self._scene
        return (
            old.bounds == scene.bounds
            and self._goal_point == scene.goal
            and old.drone_radius == scene.drone_radius
            and old.safety_margin == scene.safety_margin
        )

    def _visible_goal_terminal_costs(self, scene: Scene, grid: VoxelGrid) -> dict[GridIndex, float]:
        """Return every visible goal anchor and its exact-endpoint connector cost."""

        return {
            anchor: self._cost(grid.point(anchor), scene.goal)
            for anchor in grid.anchor_indices(scene.goal)
        }

    def _start_anchor_key(
        self,
        scene: Scene,
        grid: VoxelGrid,
        anchor: GridIndex,
        goal_terminal_costs: dict[GridIndex, float],
    ) -> tuple[bool, float, float, GridIndex]:
        """Rank a visible start anchor by progress, then a goal-aware lower bound.

        A negative projection along the start-to-goal axis indicates an avoidable backwards exact
        connector.  Such candidates remain valid fallbacks, but are ranked after candidates that
        make non-negative progress.  The second key considers every visible goal connector instead
        of coupling D* Lite to one nearest goal voxel.
        """

        anchor_point = grid.point(anchor)
        mission_axis = subtract(scene.goal, scene.start)
        connector = subtract(anchor_point, scene.start)
        backwards = dot(connector, mission_axis) < -1e-12
        connector_cost = self._cost(scene.start, anchor_point)
        goal_lower_bound = min(
            self._cost(anchor_point, grid.point(goal_anchor)) + terminal_cost
            for goal_anchor, terminal_cost in goal_terminal_costs.items()
        )
        return (backwards, connector_cost + goal_lower_bound, connector_cost, anchor)

    def _select_start_anchor(
        self,
        scene: Scene,
        grid: VoxelGrid,
        candidates: list[GridIndex],
        goal_terminal_costs: dict[GridIndex, float],
    ) -> GridIndex | None:
        self._start_anchor_candidates_last = len(candidates)
        if not candidates or not goal_terminal_costs:
            return None
        return min(
            candidates,
            key=lambda anchor: self._start_anchor_key(scene, grid, anchor, goal_terminal_costs),
        )

    def _anchor_parameters(self) -> dict[str, Scalar]:
        parameters: dict[str, Scalar] = {
            "start_anchor_candidates": self._start_anchor_candidates_last,
            "goal_anchor_candidates": len(self._goal_terminal_costs),
        }
        if self._start_anchor_candidates_last and self._start is not None:
            parameters["selected_start_anchor"] = self._start
        if self._selected_goal_last is not None:
            parameters["selected_goal_anchor"] = self._selected_goal_last
        return parameters

    def _initialize(self, scene: Scene) -> bool:
        self.reset()
        self._grid = VoxelGrid(scene, self.config.resolution, self.config.altitude_levels)
        self._scene = scene
        self._goal_point = scene.goal
        self._goal_terminal_costs = self._visible_goal_terminal_costs(scene, self._grid)
        self._start = self._select_start_anchor(
            scene,
            self._grid,
            self._grid.anchor_indices(scene.start),
            self._goal_terminal_costs,
        )
        if self._start is None or not self._goal_terminal_costs:
            return False
        self._last_start = self._start
        for goal, connector_cost in sorted(self._goal_terminal_costs.items()):
            self._rhs[goal] = connector_cost
            self._push(goal)
        return True

    def _update_problem(self, scene: Scene) -> bool:
        assert self._grid is not None
        assert self._scene is not None
        assert self._start is not None
        new_grid = VoxelGrid(scene, self.config.resolution, self.config.altitude_levels)
        new_goal_terminal_costs = self._visible_goal_terminal_costs(scene, new_grid)
        new_start = self._select_start_anchor(
            scene,
            new_grid,
            new_grid.anchor_indices(scene.start),
            new_goal_terminal_costs,
        )
        if new_start is None or new_goal_terminal_costs != self._goal_terminal_costs:
            return False
        previous_start = self._start
        self._start = new_start
        self._km += self._heuristic(previous_start, new_start)
        self._last_start = previous_start
        self._grid = new_grid
        self._scene = scene
        self._selected_goal_last = None

        changed: list[Edge] = []
        for edge, old_cost in tuple(self._edge_costs.items()):
            new_cost = self._raw_edge_cost(*edge)
            self._edge_costs[edge] = new_cost
            if (math.isinf(old_cost) != math.isinf(new_cost)) or abs(old_cost - new_cost) > 1e-12:
                changed.append(edge)
        self._changed_edges_last = len(changed)
        affected = {vertex for edge in changed for vertex in edge}
        for vertex in sorted(affected):
            self._update_vertex(vertex)
            for neighbor in self._potential_neighbors(vertex):
                self._update_vertex(neighbor)
        return True

    def _heuristic(self, left: GridIndex, right: GridIndex) -> float:
        assert self._grid is not None
        return self._cost(self._grid.point(left), self._grid.point(right))

    def _cost(self, a: Point3, b: Point3) -> float:
        return flight_distance(a, b, self.config.vertical_cost_scale)

    def _value(self, values: dict[GridIndex, float], vertex: GridIndex) -> float:
        return values.get(vertex, math.inf)

    def _calculate_key(self, vertex: GridIndex) -> Key:
        assert self._start is not None
        value = min(self._value(self._g, vertex), self._value(self._rhs, vertex))
        return (value + self._heuristic(self._start, vertex) + self._km, value)

    def _push(self, vertex: GridIndex) -> None:
        key = self._calculate_key(vertex)
        self._open_keys[vertex] = key
        heapq.heappush(self._queue, (key[0], key[1], next(self._counter), vertex))

    def _remove(self, vertex: GridIndex) -> None:
        self._open_keys.pop(vertex, None)

    def _top_key(self) -> Key:
        while self._queue:
            first, second, _, vertex = self._queue[0]
            if self._open_keys.get(vertex) == (first, second):
                return (first, second)
            heapq.heappop(self._queue)
        return (math.inf, math.inf)

    def _pop(self) -> tuple[GridIndex, Key] | None:
        while self._queue:
            first, second, _, vertex = heapq.heappop(self._queue)
            old_key = (first, second)
            if self._open_keys.get(vertex) != old_key:
                continue
            self._open_keys.pop(vertex)
            return (vertex, old_key)
        return None

    def _potential_neighbors(self, vertex: GridIndex) -> list[GridIndex]:
        assert self._grid is not None
        neighbors: list[GridIndex] = []
        for delta in itertools.product((-1, 0, 1), repeat=3):
            if delta == (0, 0, 0):
                continue
            candidate = (
                vertex[0] + delta[0],
                vertex[1] + delta[1],
                vertex[2] + delta[2],
            )
            if self._grid.contains(candidate):
                neighbors.append(candidate)
        return neighbors

    def _edge(self, left: GridIndex, right: GridIndex) -> Edge:
        return (left, right) if left < right else (right, left)

    def _raw_edge_cost(self, left: GridIndex, right: GridIndex) -> float:
        assert self._grid is not None
        assert self._scene is not None
        left_point = self._grid.point(left)
        right_point = self._grid.point(right)
        if not segment_is_free(self._scene, left_point, right_point):
            return math.inf
        return self._cost(left_point, right_point)

    def _edge_cost(self, left: GridIndex, right: GridIndex) -> float:
        edge = self._edge(left, right)
        if edge not in self._edge_costs:
            self._edge_costs[edge] = self._raw_edge_cost(*edge)
        return self._edge_costs[edge]

    def _update_vertex(self, vertex: GridIndex) -> None:
        if vertex not in self._goal_terminal_costs:
            best = math.inf
            for successor in self._potential_neighbors(vertex):
                candidate = self._edge_cost(vertex, successor) + self._value(self._g, successor)
                best = min(best, candidate)
            self._rhs[vertex] = best
        self._remove(vertex)
        if self._value(self._g, vertex) != self._value(self._rhs, vertex):
            self._push(vertex)

    def _compute_shortest_path(self) -> bool:
        assert self._start is not None
        while self._top_key() < self._calculate_key(self._start) or self._value(
            self._rhs, self._start
        ) != self._value(self._g, self._start):
            if self._queue_pops_last >= self.config.max_queue_pops:
                return False
            popped = self._pop()
            if popped is None:
                break
            vertex, old_key = popped
            self._queue_pops_last += 1
            self._queue_pops_total += 1
            new_key = self._calculate_key(vertex)
            if old_key < new_key:
                self._push(vertex)
            elif self._value(self._g, vertex) > self._value(self._rhs, vertex):
                self._g[vertex] = self._value(self._rhs, vertex)
                for predecessor in self._potential_neighbors(vertex):
                    self._update_vertex(predecessor)
            else:
                self._g[vertex] = math.inf
                self._update_vertex(vertex)
                for predecessor in self._potential_neighbors(vertex):
                    self._update_vertex(predecessor)
        return True

    def _extract_path(self, scene: Scene) -> tuple[Point3, ...]:
        assert self._grid is not None
        assert self._start is not None
        if math.isinf(self._value(self._g, self._start)):
            return ()
        current = self._start
        indices = [current]
        seen = {current}
        vertex_limit = math.prod(self._grid.shape) + 1
        while current not in self._goal_terminal_costs and len(indices) <= vertex_limit:
            choices: list[tuple[float, GridIndex]] = []
            for successor in self._potential_neighbors(current):
                cost = self._edge_cost(current, successor)
                total = cost + self._value(self._g, successor)
                if math.isfinite(total):
                    choices.append((total, successor))
            if not choices:
                return ()
            _, current = min(choices, key=lambda item: (item[0], item[1]))
            if current in seen:
                return ()
            seen.add(current)
            indices.append(current)
        if current not in self._goal_terminal_costs:
            return ()
        self._selected_goal_last = current
        grid_path = [self._grid.point(index) for index in indices]
        return attach_exact_endpoints(scene, grid_path, scene.start, scene.goal)

    def _result(
        self,
        started: float,
        success: bool,
        path: tuple[Point3, ...],
        failure_reason: str | None,
        parameters: dict[str, Scalar],
        budget: PlanningBudget,
    ) -> PlanningResult:
        parameters.update(self._anchor_parameters())
        elapsed = (time.perf_counter() - started) * 1000.0
        termination = "goal-reached" if success else (failure_reason or "failed")
        return PlanningResult(
            self.algorithm_id,
            success,
            path,
            elapsed,
            expanded_nodes=self._queue_pops_last,
            generated_nodes=len(self._g) + len(self._rhs),
            iterations=self._queue_pops_last,
            failure_reason=failure_reason,
            parameters=parameters,
            search_ms=elapsed,
            budget=budget,
            budget_usage=BudgetUsage(self._queue_pops_last, elapsed, termination),
        )


__all__ = ["DStarLite3D", "DStarLiteConfig"]
