"""Deterministic contracts for time-varying UAV planning scenarios."""

from __future__ import annotations

import hashlib
import json
import math
from bisect import bisect_right
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from uav3d.benchmark import problem_fingerprint
from uav3d.geometry import Point3, as_point, lerp
from uav3d.scene import AABB, Bounds3D, Cylinder, Scene


def _finite(values: tuple[float, ...], message: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(message)


@dataclass(frozen=True, slots=True)
class TemporaryCylinder:
    """A cylindrical exclusion zone active on ``[active_from, active_until)``."""

    zone_id: str
    center: tuple[float, float]
    radius: float
    z_min: float
    z_max: float
    active_from: float
    active_until: float

    def __post_init__(self) -> None:
        if not self.zone_id:
            raise ValueError("temporary-cylinder IDs must not be empty")
        _finite(
            (
                *self.center,
                self.radius,
                self.z_min,
                self.z_max,
                self.active_from,
                self.active_until,
            ),
            f"temporary-cylinder values must be finite for {self.zone_id}",
        )
        if self.radius <= 0 or self.z_min >= self.z_max:
            raise ValueError(f"invalid temporary-cylinder dimensions for {self.zone_id}")
        if self.active_from < 0 or self.active_from >= self.active_until:
            raise ValueError(f"invalid half-open active interval for {self.zone_id}")

    @property
    def active_start(self) -> float:
        """Compatibility alias for the inclusive interval boundary."""

        return self.active_from

    @property
    def active_end(self) -> float:
        """Compatibility alias for the exclusive interval boundary."""

        return self.active_until

    def is_active(self, time_s: float) -> bool:
        return self.active_from <= time_s < self.active_until

    def as_static(self) -> Cylinder:
        return Cylinder(self.zone_id, self.center, self.radius, self.z_min, self.z_max)

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.zone_id,
            "center": list(self.center),
            "radius": self.radius,
            "z_min": self.z_min,
            "z_max": self.z_max,
            "active_from": self.active_from,
            "active_until": self.active_until,
        }


@dataclass(frozen=True, slots=True)
class MovingSphere:
    """A spherical obstacle following a piecewise-linear, deterministic trajectory."""

    sphere_id: str
    radius: float
    keyframes: tuple[tuple[float, Point3], ...]

    def __post_init__(self) -> None:
        if not self.sphere_id:
            raise ValueError("moving-sphere IDs must not be empty")
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError(
                f"moving-sphere radius must be finite and positive for {self.sphere_id}"
            )
        if len(self.keyframes) < 2:
            raise ValueError(f"moving sphere {self.sphere_id} requires at least two keyframes")
        previous = -math.inf
        for time_s, position in self.keyframes:
            _finite((time_s, *position), f"keyframes must be finite for {self.sphere_id}")
            if time_s < 0 or time_s <= previous:
                raise ValueError(
                    "keyframe times must be non-negative and strictly increasing "
                    f"for {self.sphere_id}"
                )
            previous = time_s

    @property
    def obstacle_id(self) -> str:
        return self.sphere_id

    def position_at(self, time_s: float) -> Point3:
        """Interpolate at ``time_s``; hold the endpoint positions outside the keyframe span."""

        if not math.isfinite(time_s):
            raise ValueError("interpolation time must be finite")
        if time_s <= self.keyframes[0][0]:
            return self.keyframes[0][1]
        if time_s >= self.keyframes[-1][0]:
            return self.keyframes[-1][1]
        index = bisect_right(self.keyframes, time_s, key=lambda frame: frame[0]) - 1
        left_time, left = self.keyframes[index]
        right_time, right = self.keyframes[index + 1]
        return lerp(left, right, (time_s - left_time) / (right_time - left_time))

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.sphere_id,
            "radius": self.radius,
            "keyframes": [
                {"time": time_s, "position": list(position)} for time_s, position in self.keyframes
            ],
        }


@dataclass(frozen=True, slots=True)
class DynamicScenario:
    """A static planning problem augmented by deterministic obstacle schedules."""

    scenario_id: str
    name: str
    static_scene: Scene
    temporary_cylinders: tuple[TemporaryCylinder, ...] = ()
    moving_spheres: tuple[MovingSphere, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.scenario_id:
            raise ValueError("dynamic scenario IDs must not be empty")
        dynamic_ids = [zone.zone_id for zone in self.temporary_cylinders]
        dynamic_ids.extend(sphere.sphere_id for sphere in self.moving_spheres)
        static_ids = [building.obstacle_id for building in self.static_scene.buildings]
        static_ids.extend(zone.zone_id for zone in self.static_scene.no_fly_zones)
        all_ids = [*static_ids, *dynamic_ids]
        if len(all_ids) != len(set(all_ids)):
            raise ValueError("all static and dynamic obstacle IDs must be unique")

    @property
    def base_scene(self) -> Scene:
        return self.static_scene

    def snapshot_scene(self, time_s: float, start: Point3 | None = None) -> Scene:
        return snapshot_scene(self, time_s, start=start)

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": "dynamic-scenario-v1",
            "id": self.scenario_id,
            "name": self.name,
            "static_scene": self.static_scene.to_dict(),
            "temporary_cylinders": [zone.to_dict() for zone in self.temporary_cylinders],
            "moving_spheres": [sphere.to_dict() for sphere in self.moving_spheres],
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> DynamicScenario:
        static_data = data.get("static_scene")
        if not isinstance(static_data, dict):
            raise ValueError("dynamic scenario requires a static_scene object")
        temporary_data = data.get("temporary_cylinders", [])
        moving_data = data.get("moving_spheres", [])
        if not isinstance(temporary_data, list) or not isinstance(moving_data, list):
            raise ValueError("dynamic obstacle collections must be arrays")
        temporary: list[TemporaryCylinder] = []
        for raw in temporary_data:
            if not isinstance(raw, dict):
                raise ValueError("temporary cylinder records must be objects")
            center = raw["center"]
            temporary.append(
                TemporaryCylinder(
                    str(raw["id"]),
                    (float(center[0]), float(center[1])),
                    float(raw["radius"]),
                    float(raw["z_min"]),
                    float(raw["z_max"]),
                    float(raw["active_from"]),
                    float(raw["active_until"]),
                )
            )
        moving: list[MovingSphere] = []
        for raw in moving_data:
            if not isinstance(raw, dict):
                raise ValueError("moving sphere records must be objects")
            raw_keyframes = raw["keyframes"]
            if not isinstance(raw_keyframes, list):
                raise ValueError("moving-sphere keyframes must be an array")
            keyframes: list[tuple[float, Point3]] = []
            for keyframe in raw_keyframes:
                if not isinstance(keyframe, dict):
                    raise ValueError("moving-sphere keyframes must be objects")
                keyframes.append((float(keyframe["time"]), as_point(keyframe["position"])))
            moving.append(MovingSphere(str(raw["id"]), float(raw["radius"]), tuple(keyframes)))
        return cls(
            scenario_id=str(data["id"]),
            name=str(data.get("name", data["id"])),
            static_scene=Scene.from_dict(static_data),
            temporary_cylinders=tuple(temporary),
            moving_spheres=tuple(moving),
            metadata=dict(data.get("metadata", {})),
        )


def save_dynamic_scenario(scenario: DynamicScenario, path: str | Path) -> None:
    Path(path).write_text(json.dumps(scenario.to_dict(), indent=2) + "\n", encoding="utf-8")


def load_dynamic_scenario(path: str | Path) -> DynamicScenario:
    data: object = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("dynamic-scenario JSON must contain an object at the root")
    return DynamicScenario.from_dict(data)


def _canonical_number(value: float) -> str:
    normalized = 0.0 if float(value) == 0 else float(value)
    return normalized.hex()


def dynamic_scenario_fingerprint(scenario: DynamicScenario) -> str:
    """Hash planning semantics while excluding labels, IDs, metadata, and record order."""

    canonical = {
        "fingerprint_schema": "uav3d-dynamic-problem-v1",
        "static_problem": problem_fingerprint(scenario.static_scene),
        "temporary_cylinders": sorted(
            (
                {
                    "center": [_canonical_number(value) for value in zone.center],
                    "radius": _canonical_number(zone.radius),
                    "z_min": _canonical_number(zone.z_min),
                    "z_max": _canonical_number(zone.z_max),
                    "active_from": _canonical_number(zone.active_from),
                    "active_until": _canonical_number(zone.active_until),
                }
                for zone in scenario.temporary_cylinders
            ),
            key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":")),
        ),
        "moving_spheres": sorted(
            (
                {
                    "radius": _canonical_number(sphere.radius),
                    "keyframes": [
                        {
                            "time": _canonical_number(time_s),
                            "position": [_canonical_number(value) for value in position],
                        }
                        for time_s, position in sphere.keyframes
                    ],
                }
                for sphere in scenario.moving_spheres
            ),
            key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":")),
        ),
    }
    payload = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def snapshot_scene(
    scenario: DynamicScenario,
    time_s: float,
    *,
    start: Point3 | None = None,
    lookahead_s: float = 0.0,
) -> Scene:
    """Snapshot obstacles, optionally guarding their entire near-term swept volume.

    The default is the unchanged instantaneous snapshot. A requested guard also
    includes temporary airspace becoming active within its declared window.
    """

    if not math.isfinite(time_s) or time_s < 0:
        raise ValueError("snapshot time must be finite and non-negative")
    if not math.isfinite(lookahead_s) or lookahead_s < 0:
        raise ValueError("snapshot lookahead must be finite and non-negative")
    dynamic_zones = [
        zone.as_static()
        for zone in scenario.temporary_cylinders
        if (zone.active_from <= time_s + lookahead_s and zone.active_until > time_s)
    ]
    for sphere in scenario.moving_spheres:
        x, y, z = sphere.position_at(time_s)
        radius, z_min, z_max = sphere.radius, z - sphere.radius, z + sphere.radius
        if lookahead_s:
            samples = [
                sphere.position_at(time_s),
                sphere.position_at(time_s + lookahead_s),
                *(
                    position
                    for clock, position in sphere.keyframes
                    if time_s < clock < time_s + lookahead_s
                ),
            ]
            x = (min(point[0] for point in samples) + max(point[0] for point in samples)) / 2
            y = (min(point[1] for point in samples) + max(point[1] for point in samples)) / 2
            radius += max(math.hypot(point[0] - x, point[1] - y) for point in samples)
            z_min = min(point[2] for point in samples) - sphere.radius
            z_max = max(point[2] for point in samples) + sphere.radius
        dynamic_zones.append(
            Cylinder(
                sphere.sphere_id,
                (x, y),
                radius,
                z_min,
                z_max,
            )
        )
    scene_start = scenario.static_scene.start if start is None else start
    return replace(
        scenario.static_scene,
        scene_id=f"{scenario.scenario_id}@{time_s:g}",
        name=f"{scenario.name} at t={time_s:g}s",
        start=scene_start,
        no_fly_zones=(*scenario.static_scene.no_fly_zones, *dynamic_zones),
        metadata={
            "dynamic_scenario": scenario.scenario_id,
            "snapshot_time_s": time_s,
        },
    )


def _scene(
    scene_id: str,
    start: Point3 = (4.0, 20.0, 10.0),
    goal: Point3 = (56.0, 20.0, 10.0),
    buildings: tuple[AABB, ...] = (),
) -> Scene:
    return Scene(
        scene_id,
        scene_id.replace("-", " ").title(),
        Bounds3D((0.0, 0.0, 0.0), (60.0, 40.0, 30.0)),
        start,
        goal,
        buildings,
        drone_radius=0.5,
        safety_margin=0.5,
        metadata={"family": "dynamic-curated"},
    )


def _pop_up_nfz() -> DynamicScenario:
    return DynamicScenario(
        "pop-up-nfz",
        "Pop-up no-fly zone",
        _scene("pop-up-nfz-static"),
        temporary_cylinders=(TemporaryCylinder("popup", (30.0, 20.0), 6.0, 0.0, 24.0, 3.0, 30.0),),
        metadata={"event_family": "temporary-zone", "seed": 3101},
    )


def _crossing_traffic() -> DynamicScenario:
    return DynamicScenario(
        "crossing-traffic",
        "Crossing traffic",
        _scene("crossing-traffic-static"),
        moving_spheres=(
            MovingSphere(
                "traffic",
                3.0,
                (
                    (0.0, (30.0, 2.0, 10.0)),
                    (3.25, (30.0, 20.0, 10.0)),
                    (6.5, (30.0, 38.0, 10.0)),
                ),
            ),
        ),
        metadata={"event_family": "moving-obstacle", "seed": 3102},
    )


def _closing_gate() -> DynamicScenario:
    buildings = (
        AABB("gate-south", (25.0, 0.0, 0.0), (35.0, 14.0, 22.0)),
        AABB("gate-north", (25.0, 26.0, 0.0), (35.0, 40.0, 22.0)),
    )
    return DynamicScenario(
        "closing-gate",
        "Closing urban gate",
        _scene("closing-gate-static", buildings=buildings),
        temporary_cylinders=(
            TemporaryCylinder("gate-closure", (30.0, 20.0), 5.0, 0.0, 22.0, 3.0, 28.0),
        ),
        metadata={"event_family": "scheduled-closure", "seed": 3103},
    )


def _vertical_escape() -> DynamicScenario:
    buildings = (
        AABB("south-wall", (25.0, 0.0, 0.0), (35.0, 15.0, 24.0)),
        AABB("north-wall", (25.0, 25.0, 0.0), (35.0, 40.0, 24.0)),
    )
    return DynamicScenario(
        "vertical-escape",
        "Vertical escape",
        _scene("vertical-escape-static", buildings=buildings),
        temporary_cylinders=(
            TemporaryCylinder("low-altitude-block", (30.0, 20.0), 6.0, 0.0, 15.0, 2.0, 32.0),
        ),
        metadata={"event_family": "altitude-change", "seed": 3104},
    )


BUILTIN_DYNAMIC_SCENARIOS = {
    "pop-up-nfz": _pop_up_nfz,
    "crossing-traffic": _crossing_traffic,
    "closing-gate": _closing_gate,
    "vertical-escape": _vertical_escape,
}


def list_builtin_dynamic_scenarios() -> tuple[str, ...]:
    return tuple(BUILTIN_DYNAMIC_SCENARIOS)


def load_builtin_dynamic_scenario(scenario_id: str) -> DynamicScenario:
    try:
        return BUILTIN_DYNAMIC_SCENARIOS[scenario_id]()
    except KeyError as error:
        choices = ", ".join(BUILTIN_DYNAMIC_SCENARIOS)
        raise ValueError(
            f"unknown built-in dynamic scenario {scenario_id!r}; choose one of: {choices}"
        ) from error
