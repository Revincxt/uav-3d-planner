"""Reusable deterministic city geometry for the v0.5 predictive demonstrations."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TypeAlias

from uav3d.geometry import Point3
from uav3d.scene import AABB, Bounds3D, Cylinder, Scene

Footprint: TypeAlias = tuple[int, int, int, int]

CITY_BOUNDS = Bounds3D((0.0, 0.0, 0.0), (104.0, 88.0, 60.0))


# Every footprint coordinate is aligned to the public four-metre planning lattice.  Adjacent
# building bands leave at least twelve metres for streets; the patterns differ in block rhythm,
# open-space placement, and canyon orientation rather than only in building height.
ORTHOGONAL_GRID: tuple[Footprint, ...] = tuple(
    (x, y, 12, 12) for y in (4, 28, 52, 76) for x in (12, 36, 60, 84)
)

STAGGERED_MARKET: tuple[Footprint, ...] = tuple(
    (x, y, 12, 12)
    for row, y in enumerate((4, 28, 52, 76))
    for x in ((8, 32, 56, 80) if row % 2 == 0 else (16, 40, 64, 88))
)

CIVIC_COURTYARDS: tuple[Footprint, ...] = tuple(
    (x, y, 8, 12) for y in (4, 28, 52, 76) for x in (8, 28, 64, 84)
)

TRANSIT_BOULEVARD: tuple[Footprint, ...] = (
    (4, 4, 12, 8),
    (28, 4, 16, 8),
    (56, 4, 16, 8),
    (84, 4, 16, 8),
    (8, 24, 16, 12),
    (36, 24, 12, 12),
    (60, 24, 16, 12),
    (88, 24, 12, 12),
    (4, 52, 16, 12),
    (32, 52, 16, 12),
    (60, 52, 12, 12),
    (84, 52, 16, 12),
    (8, 76, 12, 12),
    (32, 76, 16, 12),
    (60, 76, 16, 12),
    (88, 76, 12, 12),
)

TERRACED_HEIGHTS: tuple[Footprint, ...] = (
    (8, 4, 12, 12),
    (32, 4, 16, 12),
    (60, 4, 12, 12),
    (84, 4, 16, 12),
    (4, 28, 16, 12),
    (32, 28, 12, 12),
    (56, 28, 16, 12),
    (84, 28, 12, 12),
    (8, 52, 16, 12),
    (36, 52, 12, 12),
    (60, 52, 16, 12),
    (88, 52, 12, 12),
    (4, 76, 12, 12),
    (28, 76, 16, 12),
    (56, 76, 12, 12),
    (80, 76, 16, 12),
)

MERGING_CANYONS: tuple[Footprint, ...] = tuple(
    (x, y, 8, depth) for x in (4, 24, 44, 64, 84) for y, depth in ((0, 24), (36, 24), (72, 16))
)

ROOFTOP_TOWERS: tuple[Footprint, ...] = tuple(
    (x, y, 12, 12) for y in (4, 28, 52, 76) for x in (12, 36, 60, 84)
)

CALIBRATION_BLOCKS: tuple[Footprint, ...] = (
    (48, 0, 8, 36),
    (48, 52, 8, 36),
    (12, 4, 16, 16),
    (12, 68, 16, 16),
    (32, 4, 12, 16),
    (32, 68, 12, 16),
    (68, 4, 16, 16),
    (68, 68, 16, 16),
)


def make_buildings(
    prefix: str,
    footprints: Sequence[Footprint],
    heights: Sequence[int],
) -> tuple[AABB, ...]:
    """Materialize one deterministic, lattice-aligned building collection."""

    if len(footprints) != len(heights):
        raise ValueError("every predictive-city footprint requires one height")
    buildings: list[AABB] = []
    for index, ((x, y, width, depth), height) in enumerate(
        zip(footprints, heights, strict=True), start=1
    ):
        values = (x, y, width, depth, height)
        if any(value % 4 != 0 for value in values):
            raise ValueError("predictive-city geometry must use four-metre increments")
        buildings.append(
            AABB(
                f"{prefix}-b{index:02d}",
                (float(x), float(y), 0.0),
                (float(x + width), float(y + depth), float(height)),
            )
        )
    return tuple(buildings)


def make_city_scene(
    scene_id: str,
    *,
    district: str,
    street_pattern: str,
    start: Point3,
    goal: Point3,
    footprints: Sequence[Footprint],
    heights: Sequence[int],
    static_zones: tuple[Cylinder, ...] = (),
) -> Scene:
    """Build a v0.5 scene while keeping selection independent of planner outcomes."""

    return Scene(
        scene_id=scene_id,
        name=scene_id.replace("-", " ").title(),
        bounds=CITY_BOUNDS,
        start=start,
        goal=goal,
        buildings=make_buildings(scene_id, footprints, heights),
        no_fly_zones=static_zones,
        drone_radius=0.5,
        safety_margin=0.5,
        metadata={
            "family": "predictive-urban-v0.5",
            "dataset": "predictive-urban-v0.5",
            "district": district,
            "street_pattern": street_pattern,
        },
    )


__all__ = [
    "CALIBRATION_BLOCKS",
    "CITY_BOUNDS",
    "CIVIC_COURTYARDS",
    "MERGING_CANYONS",
    "ORTHOGONAL_GRID",
    "ROOFTOP_TOWERS",
    "STAGGERED_MARKET",
    "TERRACED_HEIGHTS",
    "TRANSIT_BOULEVARD",
    "make_buildings",
    "make_city_scene",
]
