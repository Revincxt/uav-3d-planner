"""Voxel-grid helpers shared by graph-search planners."""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass, field

from uav3d.collision import point_is_free, segment_is_free
from uav3d.geometry import Point3, almost_equal
from uav3d.scene import Scene

GridIndex = tuple[int, int, int]
_CACHE_LIMIT = 100_000


@dataclass(frozen=True, slots=True)
class VoxelGrid:
    scene: Scene
    resolution: float
    _shape: GridIndex = field(init=False, repr=False, compare=False)
    _point_cache: dict[GridIndex, bool] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )
    _edge_cache: dict[tuple[GridIndex, GridIndex], bool] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )
    _anchor_cache: dict[tuple[Point3, GridIndex], bool] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("grid resolution must be finite and positive")
        object.__setattr__(
            self,
            "_shape",
            tuple(
                math.floor((upper - lower) / self.resolution) + 1
                for lower, upper in zip(
                    self.scene.bounds.minimum, self.scene.bounds.maximum, strict=True
                )
            ),
        )

    @property
    def shape(self) -> GridIndex:
        return self._shape

    def point(self, index: GridIndex) -> Point3:
        return tuple(
            lower + component * self.resolution
            for lower, component in zip(self.scene.bounds.minimum, index, strict=True)
        )  # type: ignore[return-value]

    def nearest_index(self, point: Point3) -> GridIndex:
        shape = self.shape
        return tuple(
            max(0, min(size - 1, round((value - lower) / self.resolution)))
            for value, lower, size in zip(point, self.scene.bounds.minimum, shape, strict=True)
        )  # type: ignore[return-value]

    def anchor_indices(self, point: Point3) -> list[GridIndex]:
        """Return every visible free grid vertex in the local 3x3x3 anchor stencil."""

        origin = self.nearest_index(point)
        candidates: list[GridIndex] = []
        for delta in itertools.product((-1, 0, 1), repeat=3):
            candidate = (
                origin[0] + delta[0],
                origin[1] + delta[1],
                origin[2] + delta[2],
            )
            if not self.contains(candidate):
                continue
            if self.is_free(candidate) and self.visible_from(point, candidate):
                candidates.append(candidate)
        candidates.sort(key=lambda item: (math.dist(point, self.point(item)), item))
        if candidates:
            return candidates
        nearest = self.nearest_free_index(point)
        return [nearest] if nearest is not None else []

    def contains(self, index: GridIndex) -> bool:
        return all(0 <= value < size for value, size in zip(index, self.shape, strict=True))

    def nearest_free_index(self, point: Point3) -> GridIndex | None:
        origin = self.nearest_index(point)
        # Best-first lattice enumeration has exactly the old (squared index
        # distance, lexicographic index) order, but does not sort/materialize
        # every city voxel before checking a nearby free anchor. Each voxel has
        # a path from origin whose squared distance strictly increases.
        queue: list[tuple[int, GridIndex]] = [(0, origin)]
        seen = {origin}
        while queue:
            _, candidate = heapq.heappop(queue)
            if self.is_free(candidate) and self.visible_from(point, candidate):
                return candidate
            for axis in range(3):
                for step in (-1, 1):
                    coordinates = list(candidate)
                    coordinates[axis] += step
                    neighbor: GridIndex = (coordinates[0], coordinates[1], coordinates[2])
                    if neighbor in seen or not self.contains(neighbor):
                        continue
                    seen.add(neighbor)
                    squared_distance = sum((neighbor[i] - origin[i]) ** 2 for i in range(3))
                    heapq.heappush(queue, (squared_distance, neighbor))
        return None

    def is_free(self, index: GridIndex) -> bool:
        """Cache only this grid instance's immutable static Scene geometry.

        A dynamic simulation must construct a new Scene/grid for each snapshot;
        no timed collision predicate or cache is shared between planning calls.
        Cache capacity is bounded; a full cache simply evaluates uncached keys.
        """

        if index in self._point_cache:
            return self._point_cache[index]
        free = point_is_free(self.scene, self.point(index))
        if len(self._point_cache) < _CACHE_LIMIT:
            self._point_cache[index] = free
        return free

    def edge_is_free(self, first: GridIndex, second: GridIndex) -> bool:
        key = (first, second) if first <= second else (second, first)
        if key in self._edge_cache:
            return self._edge_cache[key]
        free = segment_is_free(self.scene, self.point(first), self.point(second))
        if len(self._edge_cache) < _CACHE_LIMIT:
            self._edge_cache[key] = free
        return free

    def visible_from(self, point: Point3, index: GridIndex) -> bool:
        key = (point, index)
        if key in self._anchor_cache:
            return self._anchor_cache[key]
        free = segment_is_free(self.scene, point, self.point(index))
        if len(self._anchor_cache) < _CACHE_LIMIT:
            self._anchor_cache[key] = free
        return free

    def neighbors(self, index: GridIndex) -> list[GridIndex]:
        valid: list[GridIndex] = []
        for delta in itertools.product((-1, 0, 1), repeat=3):
            if delta == (0, 0, 0):
                continue
            neighbor = (
                index[0] + delta[0],
                index[1] + delta[1],
                index[2] + delta[2],
            )
            if not self.contains(neighbor):
                continue
            if self.is_free(neighbor) and self.edge_is_free(index, neighbor):
                valid.append(neighbor)
        return valid


def attach_exact_endpoints(
    scene: Scene, grid_path: list[Point3], start: Point3, goal: Point3
) -> tuple[Point3, ...]:
    if not grid_path:
        return ()
    path = list(grid_path)
    if almost_equal(path[0], start):
        path[0] = start
    elif segment_is_free(scene, start, path[0]):
        path.insert(0, start)
    else:
        return ()
    if almost_equal(path[-1], goal):
        path[-1] = goal
    elif segment_is_free(scene, path[-1], goal):
        path.append(goal)
    else:
        return ()
    return tuple(path)
