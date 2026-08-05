"""Deterministic finite-horizon A* over a 3D voxel-by-time graph."""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass
from itertools import pairwise
from typing import TYPE_CHECKING

from uav3d.collision import point_is_free
from uav3d.geometry import Point3, almost_equal, distance
from uav3d.planners.grid import GridIndex, VoxelGrid
from uav3d.predictive import (
    PredictivePlanningResult,
    PredictiveScalar,
    TimedAction,
    TimedPath,
    TimedWaypoint,
)

if TYPE_CHECKING:
    from uav3d.dynamic import DynamicScenario

SpaceTimeState = tuple[GridIndex, int]


def _point_is_free_at_time(
    scenario: DynamicScenario,
    point: Point3,
    time_s: float,
) -> bool:
    from uav3d.dynamic_collision import point_is_free_at_time

    return point_is_free_at_time(scenario, point, time_s)


def _spacetime_segment_is_free(
    scenario: DynamicScenario,
    start: Point3,
    end: Point3,
    start_time: float,
    end_time: float,
) -> bool:
    from uav3d.dynamic_collision import spacetime_segment_is_free

    return spacetime_segment_is_free(scenario, start, end, start_time, end_time)


@dataclass(frozen=True, slots=True)
class SpaceTimeAStarConfig:
    """Fixed motion and finite-search contract for :class:`SpaceTimeAStar3D`.

    Moving actions use a six-connected grid. ``resolution / cruise_speed`` must therefore be an
    integer number of time steps. Every moving action has exactly ``cruise_speed``; a separate wait
    action advances one time step without changing position.
    """

    resolution: float = 4.0
    time_step: float = 0.5
    cruise_speed: float = 8.0
    time_horizon: float = 60.0
    max_expansions: int = 120_000

    def __post_init__(self) -> None:
        numeric = (self.resolution, self.time_step, self.cruise_speed, self.time_horizon)
        if not all(math.isfinite(value) and value > 0 for value in numeric):
            raise ValueError("space-time resolution, clock, speed, and horizon must be positive")
        if self.max_expansions <= 0:
            raise ValueError("max_expansions must be positive")
        if not math.isclose(
            self.time_horizon / self.time_step,
            round(self.time_horizon / self.time_step),
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError("time_horizon must contain an integer number of time steps")
        if (
            not math.isclose(
                self.resolution / (self.cruise_speed * self.time_step),
                round(self.resolution / (self.cruise_speed * self.time_step)),
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            or round(self.resolution / (self.cruise_speed * self.time_step)) < 1
        ):
            raise ValueError(
                "resolution / cruise_speed must contain a positive integer number of time steps"
            )

    @property
    def horizon_steps(self) -> int:
        return round(self.time_horizon / self.time_step)

    @property
    def movement_steps(self) -> int:
        return round(self.resolution / (self.cruise_speed * self.time_step))


class SpaceTimeAStar3D:
    """Earliest-arrival search with explicit wait actions and exact dynamic edge checks."""

    algorithm_id = "space-time-astar-4d"

    def __init__(self, config: SpaceTimeAStarConfig | None = None) -> None:
        self.config = config or SpaceTimeAStarConfig()

    def plan(
        self,
        scenario: DynamicScenario,
        *,
        start_time: float = 0.0,
    ) -> PredictivePlanningResult:
        """Plan from the scenario's exact endpoints over ``[start_time, start_time + horizon]``."""

        if not math.isfinite(start_time) or start_time < 0:
            raise ValueError("start_time must be finite and non-negative")
        parameters = self._parameters(start_time)
        scene = scenario.static_scene
        if not _point_is_free_at_time(scenario, scene.start, start_time):
            return self._failure("invalid-start", 0, 0, parameters)
        # A dynamically occupied goal may become available later, but a statically invalid one never
        # can. Keep that distinction explicit in the failure contract.
        if not point_is_free(scene, scene.goal):
            return self._failure("invalid-goal", 0, 0, parameters)

        horizon_time = start_time + self.config.time_horizon
        direct = self._earliest_direct_path(scenario, start_time, horizon_time)
        if direct is not None and math.isclose(
            direct.arrival_time_s,
            start_time + distance(scene.start, scene.goal) / self.config.cruise_speed,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            # No trajectory can arrive before an immediately feasible Euclidean segment at the fixed
            # speed, so this case is globally earliest without expanding a voxel-time state.
            return self._success(scenario, direct, 0, 1, parameters)

        grid = VoxelGrid(scene, self.config.resolution)
        start_anchors = grid.anchor_indices(scene.start)
        goal_anchors = set(grid.anchor_indices(scene.goal))
        if not start_anchors or not goal_anchors:
            if direct is not None:
                return self._success(scenario, direct, 0, 1, parameters)
            return self._failure("no-free-grid-anchor", 0, 0, parameters)

        queue: list[tuple[float, int, GridIndex, int, SpaceTimeState]] = []
        counter = itertools.count()
        parents: dict[SpaceTimeState, SpaceTimeState | None] = {}
        prefixes: dict[SpaceTimeState, tuple[TimedWaypoint, ...]] = {}
        for state, prefix in self._initial_states(
            scenario,
            grid,
            start_anchors,
            start_time,
            horizon_time,
        ):
            if state in parents:
                continue
            parents[state] = None
            prefixes[state] = prefix
            heuristic = self._heuristic(grid.point(state[0]), scene.goal)
            heapq.heappush(
                queue,
                (
                    self._time(start_time, state[1]) + heuristic,
                    state[1],
                    state[0],
                    next(counter),
                    state,
                ),
            )

        if not queue:
            if direct is not None:
                return self._success(scenario, direct, 0, 1, parameters)
            return self._failure("time-horizon-exhausted", 0, 0, parameters)

        incumbent = direct
        expanded = 0
        closed: set[SpaceTimeState] = set()
        while queue:
            lower_bound, _, _, _, state = heapq.heappop(queue)
            if state in closed:
                continue
            if incumbent is not None and lower_bound >= incumbent.arrival_time_s - 1e-9:
                return self._success(scenario, incumbent, expanded, len(parents), parameters)
            if expanded >= self.config.max_expansions:
                return self._failure(
                    "expansion-budget-exhausted", expanded, len(parents), parameters
                )
            closed.add(state)
            expanded += 1

            goal_path = self._connect_goal(
                scenario,
                grid,
                state,
                goal_anchors,
                parents,
                prefixes,
                start_time,
                horizon_time,
            )
            if goal_path is not None and (
                incumbent is None or goal_path.arrival_time_s < incumbent.arrival_time_s - 1e-9
            ):
                incumbent = goal_path

            for successor in self._successors(scenario, grid, state, start_time):
                if successor in parents:
                    continue
                parents[successor] = state
                heuristic = self._heuristic(grid.point(successor[0]), scene.goal)
                heapq.heappush(
                    queue,
                    (
                        self._time(start_time, successor[1]) + heuristic,
                        successor[1],
                        successor[0],
                        next(counter),
                        successor,
                    ),
                )

        if incumbent is not None:
            return self._success(scenario, incumbent, expanded, len(parents), parameters)
        return self._failure("time-horizon-exhausted", expanded, len(parents), parameters)

    def _parameters(self, start_time: float) -> dict[str, PredictiveScalar]:
        return {
            "resolution": self.config.resolution,
            "time_step": self.config.time_step,
            "cruise_speed": self.config.cruise_speed,
            "time_horizon": self.config.time_horizon,
            "max_expansions": self.config.max_expansions,
            "start_time": start_time,
            "connectivity": 6,
            "movement_steps": self.config.movement_steps,
            "wait_action": True,
            "work_unit": "expanded-spacetime-states",
        }

    def _time(self, start_time: float, step: int) -> float:
        return start_time + step * self.config.time_step

    def _heuristic(self, point: Point3, goal: Point3) -> float:
        return distance(point, goal) / self.config.cruise_speed

    def _earliest_direct_path(
        self,
        scenario: DynamicScenario,
        start_time: float,
        horizon_time: float,
    ) -> TimedPath | None:
        scene = scenario.static_scene
        travel_time = distance(scene.start, scene.goal) / self.config.cruise_speed
        for wait_step in range(self.config.horizon_steps + 1):
            departure = self._time(start_time, wait_step)
            arrival = departure + travel_time
            if arrival > horizon_time + 1e-9:
                break
            if wait_step and not _spacetime_segment_is_free(
                scenario,
                scene.start,
                scene.start,
                start_time,
                departure,
            ):
                # Every later discrete departure includes this unsafe prefix.
                break
            if not _spacetime_segment_is_free(
                scenario,
                scene.start,
                scene.goal,
                departure,
                arrival,
            ):
                continue
            waypoints = [TimedWaypoint(start_time, scene.start, "start")]
            if wait_step:
                waypoints.append(TimedWaypoint(departure, scene.start, "wait"))
            waypoints.append(TimedWaypoint(arrival, scene.goal, "move"))
            return TimedPath(tuple(waypoints))
        return None

    def _initial_states(
        self,
        scenario: DynamicScenario,
        grid: VoxelGrid,
        anchors: list[GridIndex],
        start_time: float,
        horizon_time: float,
    ) -> list[tuple[SpaceTimeState, tuple[TimedWaypoint, ...]]]:
        start = scenario.static_scene.start
        candidates: list[tuple[SpaceTimeState, tuple[TimedWaypoint, ...]]] = []
        for anchor in anchors:
            point = grid.point(anchor)
            connector_length = distance(start, point)
            if connector_length <= 1e-12:
                candidates.append(((anchor, 0), (TimedWaypoint(start_time, start, "start"),)))
                continue
            travel_time = connector_length / self.config.cruise_speed
            for wait_step in range(self.config.horizon_steps + 1):
                departure = self._time(start_time, wait_step)
                arrival = departure + travel_time
                if arrival > horizon_time + 1e-9:
                    break
                if wait_step and not _spacetime_segment_is_free(
                    scenario,
                    start,
                    start,
                    start_time,
                    departure,
                ):
                    break
                if not _spacetime_segment_is_free(
                    scenario,
                    start,
                    point,
                    departure,
                    arrival,
                ):
                    continue
                aligned_step = math.ceil((arrival - start_time) / self.config.time_step - 1e-12)
                aligned_time = self._time(start_time, aligned_step)
                if aligned_step > self.config.horizon_steps:
                    break
                if aligned_time > arrival + 1e-12 and not _spacetime_segment_is_free(
                    scenario,
                    point,
                    point,
                    arrival,
                    aligned_time,
                ):
                    continue
                prefix = [TimedWaypoint(start_time, start, "start")]
                if wait_step:
                    prefix.append(TimedWaypoint(departure, start, "wait"))
                prefix.append(TimedWaypoint(arrival, point, "move"))
                if aligned_time > arrival + 1e-12:
                    prefix.append(TimedWaypoint(aligned_time, point, "wait"))
                candidates.append(((anchor, aligned_step), tuple(prefix)))
        candidates.sort(key=lambda item: (item[0][1], item[0][0], self._prefix_key(item[1])))
        return candidates

    def _prefix_key(
        self, prefix: tuple[TimedWaypoint, ...]
    ) -> tuple[tuple[float, Point3, str], ...]:
        return tuple((item.time_s, item.position, item.action) for item in prefix)

    def _axis_neighbors(self, grid: VoxelGrid, index: GridIndex) -> list[GridIndex]:
        candidates: list[GridIndex] = []
        for axis in range(3):
            for direction in (-1, 1):
                values = list(index)
                values[axis] += direction
                candidate: GridIndex = (values[0], values[1], values[2])
                if grid.contains(candidate):
                    candidates.append(candidate)
        candidates.sort()
        return candidates

    def _successors(
        self,
        scenario: DynamicScenario,
        grid: VoxelGrid,
        state: SpaceTimeState,
        start_time: float,
    ) -> list[SpaceTimeState]:
        index, step = state
        point = grid.point(index)
        successors: list[SpaceTimeState] = []

        wait_step = step + 1
        if wait_step <= self.config.horizon_steps and _spacetime_segment_is_free(
            scenario,
            point,
            point,
            self._time(start_time, step),
            self._time(start_time, wait_step),
        ):
            successors.append((index, wait_step))

        movement_step = step + self.config.movement_steps
        if movement_step <= self.config.horizon_steps:
            for neighbor in self._axis_neighbors(grid, index):
                if _spacetime_segment_is_free(
                    scenario,
                    point,
                    grid.point(neighbor),
                    self._time(start_time, step),
                    self._time(start_time, movement_step),
                ):
                    successors.append((neighbor, movement_step))
        successors.sort(key=lambda item: (item[1], item[0]))
        return successors

    def _connect_goal(
        self,
        scenario: DynamicScenario,
        grid: VoxelGrid,
        state: SpaceTimeState,
        goal_anchors: set[GridIndex],
        parents: dict[SpaceTimeState, SpaceTimeState | None],
        prefixes: dict[SpaceTimeState, tuple[TimedWaypoint, ...]],
        start_time: float,
        horizon_time: float,
    ) -> TimedPath | None:
        index, step = state
        if index not in goal_anchors:
            return None
        point = grid.point(index)
        goal = scenario.static_scene.goal
        departure = self._time(start_time, step)
        travel_time = distance(point, goal) / self.config.cruise_speed
        arrival = departure + travel_time
        if arrival > horizon_time + 1e-9:
            return None
        if travel_time > 1e-12 and not _spacetime_segment_is_free(
            scenario,
            point,
            goal,
            departure,
            arrival,
        ):
            return None
        if travel_time <= 1e-12 and not _point_is_free_at_time(scenario, goal, arrival):
            return None
        waypoints = self._path_to_state(state, grid, parents, prefixes, start_time)
        if travel_time > 1e-12:
            waypoints.append(TimedWaypoint(arrival, goal, "move"))
        return TimedPath(tuple(waypoints))

    def _path_to_state(
        self,
        state: SpaceTimeState,
        grid: VoxelGrid,
        parents: dict[SpaceTimeState, SpaceTimeState | None],
        prefixes: dict[SpaceTimeState, tuple[TimedWaypoint, ...]],
        start_time: float,
    ) -> list[TimedWaypoint]:
        chain = [state]
        current = state
        while parents[current] is not None:
            parent = parents[current]
            assert parent is not None
            chain.append(parent)
            current = parent
        chain.reverse()
        waypoints = list(prefixes[chain[0]])
        for previous, successor in pairwise(chain):
            previous_point = grid.point(previous[0])
            successor_point = grid.point(successor[0])
            action: TimedAction = (
                "wait" if almost_equal(previous_point, successor_point) else "move"
            )
            waypoints.append(
                TimedWaypoint(
                    self._time(start_time, successor[1]),
                    successor_point,
                    action,
                )
            )
        return waypoints

    def _success(
        self,
        scenario: DynamicScenario,
        path: TimedPath,
        expanded: int,
        generated: int,
        parameters: dict[str, PredictiveScalar],
    ) -> PredictivePlanningResult:
        scene = scenario.static_scene
        if path.start != scene.start or path.goal != scene.goal:
            raise AssertionError("predictive planners must preserve the exact supplied endpoints")
        if not path.is_safe(scenario):
            raise AssertionError("predictive planner produced an unsafe timed path")
        for previous, current in pairwise(path.waypoints):
            if current.action == "move":
                observed = distance(previous.position, current.position) / (
                    current.time_s - previous.time_s
                )
                if not math.isclose(
                    observed,
                    self.config.cruise_speed,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                ):
                    raise AssertionError(
                        "predictive moving actions must use the fixed cruise speed"
                    )
        return PredictivePlanningResult(
            self.algorithm_id,
            True,
            path,
            expanded,
            generated,
            parameters=parameters,
        )

    def _failure(
        self,
        reason: str,
        expanded: int,
        generated: int,
        parameters: dict[str, PredictiveScalar],
    ) -> PredictivePlanningResult:
        return PredictivePlanningResult(
            self.algorithm_id,
            False,
            None,
            expanded,
            generated,
            failure_reason=reason,
            parameters=parameters,
        )


__all__ = ["SpaceTimeAStar3D", "SpaceTimeAStarConfig", "SpaceTimeState"]
