"""Voxel-grid helpers shared by graph-search planners."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from uav3d.collision import point_is_free, segment_is_free
from uav3d.geometry import Point3, almost_equal
from uav3d.scene import Scene

GridIndex = tuple[int, int, int]


@dataclass(frozen=True, slots=True)
class VoxelGrid:
    scene: Scene
    resolution: float

    def __post_init__(self) -> None:
        if self.resolution <= 0:
            raise ValueError("grid resolution must be positive")

    @property
    def shape(self) -> GridIndex:
        return tuple(
            math.floor((upper - lower) / self.resolution) + 1
            for lower, upper in zip(
                self.scene.bounds.minimum, self.scene.bounds.maximum, strict=True
            )
        )  # type: ignore[return-value]

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
            candidate_point = self.point(candidate)
            if point_is_free(self.scene, candidate_point) and segment_is_free(
                self.scene, point, candidate_point
            ):
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
        candidates = itertools.product(*(range(size) for size in self.shape))
        ordered = sorted(
            candidates,
            key=lambda item: (
                sum((item[axis] - origin[axis]) ** 2 for axis in range(3)),
                item,
            ),
        )
        for candidate in ordered:
            typed_candidate: GridIndex = (candidate[0], candidate[1], candidate[2])
            candidate_point = self.point(typed_candidate)
            if point_is_free(self.scene, candidate_point) and segment_is_free(
                self.scene, point, candidate_point
            ):
                return typed_candidate
        return None

    def neighbors(self, index: GridIndex) -> list[GridIndex]:
        origin = self.point(index)
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
            point = self.point(neighbor)
            if point_is_free(self.scene, point) and segment_is_free(self.scene, origin, point):
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
