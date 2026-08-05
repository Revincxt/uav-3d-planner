"""Independent path audit used by the CLI, tests, and demo exporter."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from uav3d.collision import minimum_path_clearance, path_is_free
from uav3d.geometry import Point3, almost_equal, polyline_length
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class PathAudit:
    valid: bool
    collision_free: bool
    endpoints_match: bool
    length_m: float
    minimum_clearance_m: float
    waypoint_count: int

    def to_dict(self) -> dict[str, object]:
        return {
            "valid": self.valid,
            "collision_free": self.collision_free,
            "endpoints_match": self.endpoints_match,
            "length_m": self.length_m,
            "minimum_clearance_m": self.minimum_clearance_m,
            "waypoint_count": self.waypoint_count,
        }


def audit_path(scene: Scene, path: Sequence[Point3]) -> PathAudit:
    endpoints_match = (
        bool(path) and almost_equal(path[0], scene.start) and almost_equal(path[-1], scene.goal)
    )
    collision_free = path_is_free(scene, path)
    length = polyline_length(path)
    clearance = minimum_path_clearance(scene, path) if path else 0.0
    return PathAudit(
        valid=len(path) >= 2 and endpoints_match and collision_free,
        collision_free=collision_free,
        endpoints_match=endpoints_match,
        length_m=length,
        minimum_clearance_m=clearance,
        waypoint_count=len(path),
    )
