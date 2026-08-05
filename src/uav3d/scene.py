"""Scene contracts and reproducible static-city generators."""

from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from uav3d.geometry import Point3, as_point


@dataclass(frozen=True, slots=True)
class Bounds3D:
    minimum: Point3
    maximum: Point3

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (*self.minimum, *self.maximum)):
            raise ValueError("scene bounds must be finite")
        if any(lower >= upper for lower, upper in zip(self.minimum, self.maximum, strict=True)):
            raise ValueError("every lower scene bound must be smaller than its upper bound")

    def to_dict(self) -> dict[str, list[float]]:
        return {"minimum": list(self.minimum), "maximum": list(self.maximum)}


@dataclass(frozen=True, slots=True)
class AABB:
    obstacle_id: str
    minimum: Point3
    maximum: Point3

    def __post_init__(self) -> None:
        if not self.obstacle_id:
            raise ValueError("AABB obstacle IDs must not be empty")
        if not all(math.isfinite(value) for value in (*self.minimum, *self.maximum)):
            raise ValueError(f"AABB coordinates must be finite for {self.obstacle_id}")
        if any(lower >= upper for lower, upper in zip(self.minimum, self.maximum, strict=True)):
            raise ValueError(f"invalid AABB dimensions for {self.obstacle_id}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.obstacle_id,
            "minimum": list(self.minimum),
            "maximum": list(self.maximum),
        }


@dataclass(frozen=True, slots=True)
class Cylinder:
    zone_id: str
    center: tuple[float, float]
    radius: float
    z_min: float
    z_max: float

    def __post_init__(self) -> None:
        values = (*self.center, self.radius, self.z_min, self.z_max)
        if not self.zone_id:
            raise ValueError("cylindrical zone IDs must not be empty")
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"cylindrical zone coordinates must be finite for {self.zone_id}")
        if self.radius <= 0 or self.z_min >= self.z_max:
            raise ValueError(f"invalid cylindrical zone dimensions for {self.zone_id}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.zone_id,
            "center": list(self.center),
            "radius": self.radius,
            "z_min": self.z_min,
            "z_max": self.z_max,
        }


@dataclass(frozen=True, slots=True)
class Scene:
    scene_id: str
    name: str
    bounds: Bounds3D
    start: Point3
    goal: Point3
    buildings: tuple[AABB, ...] = ()
    no_fly_zones: tuple[Cylinder, ...] = ()
    drone_radius: float = 1.0
    safety_margin: float = 1.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        scalar_values = (*self.start, *self.goal, self.drone_radius, self.safety_margin)
        if not all(math.isfinite(value) for value in scalar_values):
            raise ValueError("scene endpoints and clearances must be finite")
        if self.drone_radius < 0 or self.safety_margin < 0:
            raise ValueError("drone radius and safety margin must be non-negative")
        obstacle_ids = [building.obstacle_id for building in self.buildings]
        obstacle_ids.extend(zone.zone_id for zone in self.no_fly_zones)
        if len(obstacle_ids) != len(set(obstacle_ids)):
            raise ValueError("building and no-fly-zone IDs must be unique within a scene")
        for label, point in (("start", self.start), ("goal", self.goal)):
            if not all(
                lower <= value <= upper
                for value, lower, upper in zip(
                    point, self.bounds.minimum, self.bounds.maximum, strict=True
                )
            ):
                raise ValueError(f"scene {label} must lie inside the declared bounds")

    @property
    def required_clearance(self) -> float:
        return self.drone_radius + self.safety_margin

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": "1.0",
            "id": self.scene_id,
            "name": self.name,
            "bounds": self.bounds.to_dict(),
            "start": list(self.start),
            "goal": list(self.goal),
            "drone_radius": self.drone_radius,
            "safety_margin": self.safety_margin,
            "buildings": [building.to_dict() for building in self.buildings],
            "no_fly_zones": [zone.to_dict() for zone in self.no_fly_zones],
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Scene:
        bounds_data = data["bounds"]
        return cls(
            scene_id=str(data["id"]),
            name=str(data.get("name", data["id"])),
            bounds=Bounds3D(as_point(bounds_data["minimum"]), as_point(bounds_data["maximum"])),
            start=as_point(data["start"]),
            goal=as_point(data["goal"]),
            buildings=tuple(
                AABB(str(item["id"]), as_point(item["minimum"]), as_point(item["maximum"]))
                for item in data.get("buildings", [])
            ),
            no_fly_zones=tuple(
                Cylinder(
                    str(item["id"]),
                    (float(item["center"][0]), float(item["center"][1])),
                    float(item["radius"]),
                    float(item["z_min"]),
                    float(item["z_max"]),
                )
                for item in data.get("no_fly_zones", [])
            ),
            drone_radius=float(data.get("drone_radius", 1.0)),
            safety_margin=float(data.get("safety_margin", 1.0)),
            metadata=dict(data.get("metadata", {})),
        )


def save_scene(scene: Scene, path: str | Path) -> None:
    Path(path).write_text(json.dumps(scene.to_dict(), indent=2) + "\n", encoding="utf-8")


def load_scene(path: str | Path) -> Scene:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("scene JSON must contain an object at the root")
    return Scene.from_dict(data)


def _box(obstacle_id: str, x: float, y: float, width: float, depth: float, height: float) -> AABB:
    return AABB(obstacle_id, (x, y, 0.0), (x + width, y + depth, height))


def _open_blocks() -> Scene:
    buildings = (
        _box("b01", 18, 12, 14, 18, 20),
        _box("b02", 18, 42, 14, 17, 32),
        _box("b03", 18, 72, 14, 16, 25),
        _box("b04", 44, 18, 15, 17, 35),
        _box("b05", 44, 50, 15, 18, 22),
        _box("b06", 70, 10, 16, 20, 28),
        _box("b07", 70, 43, 16, 16, 38),
        _box("b08", 70, 72, 16, 17, 24),
    )
    return Scene(
        "open-blocks",
        "Open blocks",
        Bounds3D((0, 0, 0), (100, 100, 55)),
        (6, 7, 6),
        (94, 92, 8),
        buildings,
        metadata={"family": "curated", "description": "Irregular open city blocks."},
    )


def _urban_canyon() -> Scene:
    buildings = (
        _box("west-1", 16, 0, 16, 62, 42),
        _box("west-2", 16, 75, 16, 25, 30),
        _box("mid-1", 42, 0, 17, 25, 30),
        _box("mid-2", 42, 39, 17, 61, 44),
        _box("east-1", 69, 0, 16, 62, 37),
        _box("east-2", 69, 76, 16, 24, 46),
    )
    return Scene(
        "urban-canyon",
        "Urban canyon",
        Bounds3D((0, 0, 0), (100, 100, 60)),
        (7, 8, 7),
        (93, 92, 7),
        buildings,
        metadata={"family": "curated", "description": "Alternating canyon gates."},
    )


def _restricted_core() -> Scene:
    buildings = (
        _box("north-west", 15, 55, 22, 24, 30),
        _box("north-east", 65, 58, 20, 26, 34),
        _box("south-west", 18, 15, 18, 23, 25),
        _box("south-east", 66, 16, 20, 22, 40),
        _box("core-a", 42, 31, 12, 14, 18),
        _box("core-b", 48, 63, 13, 15, 22),
    )
    zones = (Cylinder("nfz-central", (52, 50), 15, 0, 55),)
    return Scene(
        "restricted-core",
        "Restricted core",
        Bounds3D((0, 0, 0), (100, 100, 60)),
        (7, 50, 8),
        (93, 50, 8),
        buildings,
        zones,
        metadata={"family": "curated", "description": "Central full-height no-fly zone."},
    )


def _vertical_gate() -> Scene:
    buildings = (
        _box("wall-low", 40, 0, 20, 43, 22),
        _box("wall-high", 40, 57, 20, 43, 48),
        _box("left-a", 12, 20, 16, 19, 28),
        _box("left-b", 12, 57, 16, 22, 35),
        _box("right-a", 72, 18, 16, 22, 38),
        _box("right-b", 72, 58, 16, 20, 27),
    )
    zones = (Cylinder("nfz-gate", (50, 50), 9, 0, 31),)
    return Scene(
        "vertical-gate",
        "Vertical gate",
        Bounds3D((0, 0, 0), (100, 100, 60)),
        (7, 50, 7),
        (93, 50, 7),
        buildings,
        zones,
        metadata={"family": "curated", "description": "A low restricted gate rewards altitude."},
    )


BUILTIN_SCENES = {
    "open-blocks": _open_blocks,
    "urban-canyon": _urban_canyon,
    "restricted-core": _restricted_core,
    "vertical-gate": _vertical_gate,
}


def list_builtin_scenes() -> tuple[str, ...]:
    return tuple(BUILTIN_SCENES)


def load_builtin_scene(scene_id: str) -> Scene:
    try:
        return BUILTIN_SCENES[scene_id]()
    except KeyError as error:
        choices = ", ".join(BUILTIN_SCENES)
        raise ValueError(
            f"unknown built-in scene {scene_id!r}; choose one of: {choices}"
        ) from error


def generate_random_city(seed: int, building_count: int = 18) -> Scene:
    """Generate a deterministic city while preserving a broad diagonal flight corridor."""

    if building_count < 0:
        raise ValueError("building_count must be non-negative")
    rng = random.Random(seed)
    buildings: list[AABB] = []
    attempts = 0
    while len(buildings) < building_count and attempts < building_count * 80 + 1:
        attempts += 1
        width = rng.uniform(8, 16)
        depth = rng.uniform(8, 16)
        x = rng.uniform(8, 92 - width)
        y = rng.uniform(8, 92 - depth)
        center_x = x + width / 2
        center_y = y + depth / 2
        if abs(center_y - center_x) < 10:
            continue
        candidate = _box(
            f"random-{len(buildings) + 1:02d}", x, y, width, depth, rng.uniform(14, 44)
        )
        padded = AABB(
            candidate.obstacle_id,
            (candidate.minimum[0] - 3, candidate.minimum[1] - 3, candidate.minimum[2]),
            (candidate.maximum[0] + 3, candidate.maximum[1] + 3, candidate.maximum[2]),
        )
        overlaps = any(
            padded.minimum[0] < other.maximum[0]
            and padded.maximum[0] > other.minimum[0]
            and padded.minimum[1] < other.maximum[1]
            and padded.maximum[1] > other.minimum[1]
            for other in buildings
        )
        if not overlaps:
            buildings.append(candidate)
    if len(buildings) != building_count:
        raise RuntimeError("could not place the requested number of buildings")
    return Scene(
        f"random-city-{seed}",
        f"Random city · seed {seed}",
        Bounds3D((0, 0, 0), (100, 100, 55)),
        (5, 5, 6),
        (95, 95, 8),
        tuple(buildings),
        metadata={
            "family": "seeded-random",
            "seed": seed,
            "building_count": building_count,
            "generator": "diagonal-corridor-v1",
        },
    )
