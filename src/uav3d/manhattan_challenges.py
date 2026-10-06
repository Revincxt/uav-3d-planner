"""Source-city encounter placement, without lifting traffic away from the mission.

The reference is an obstacle-free *static* plan, not a claimed flight prediction.
Events are declared from that reference before running any compared strategy.
Aircraft radius is a conservative separation envelope, not airframe dimensions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from itertools import pairwise
from typing import Any

from uav3d.collision import segment_is_free
from uav3d.dynamic import MovingSphere
from uav3d.geometry import Point3, distance, lerp, polyline_length
from uav3d.planners import AStar3D, AStarConfig
from uav3d.scene import Scene


@dataclass(frozen=True, slots=True)
class Encounter:
    position: Point3
    arrival_s: float
    entry: Point3
    exit: Point3
    radius_m: float
    patrol_loop: tuple[Point3, ...] = ()

    def aircraft(
        self,
        identifier: str,
        *,
        horizon: float,
        duration_s: float = 24.0,
    ) -> MovingSphere:
        """Constant-speed closed patrol, phase-aligned to the declared encounter.

        Every segment moves, including the beginning and final partial lap. The
        rounded return lane is certified during placement, not added just for display.
        """
        if horizon <= 0 or duration_s <= 0:
            raise ValueError("Invalid patrol horizon or speed")
        loop = self.patrol_loop or (self.entry, self.exit, self.entry)
        cumulative = [0.0]
        for a, b in pairwise(loop):
            cumulative.append(cumulative[-1] + distance(a, b))
        total = cumulative[-1]
        speed = distance(self.entry, self.exit) / duration_s
        origin = self.arrival_s - duration_s / 2

        def sample(clock: float) -> Point3:
            phase = ((clock - origin) * speed) % total
            for index, (a, b) in enumerate(pairwise(loop)):
                if phase <= cumulative[index + 1]:
                    return lerp(
                        a,
                        b,
                        (phase - cumulative[index]) / (cumulative[index + 1] - cumulative[index]),
                    )
            return loop[0]

        first_lap = math.floor(-origin * speed / total) - 1
        final_lap = math.ceil((horizon - origin) * speed / total) + 1
        frames: list[tuple[float, Point3]] = [(0.0, sample(0.0))]
        for lap in range(first_lap, final_lap + 1):
            for progress, point in zip(cumulative[:-1], loop[:-1], strict=True):
                clock = origin + (lap * total + progress) / speed
                if 1e-8 < clock < horizon - 1e-8:
                    frames.append((clock, point))
        frames.append((horizon, sample(horizon)))
        if any(b[0] <= a[0] for a, b in pairwise(frames)):
            raise ValueError("Aircraft schedule leaves its declared horizon")
        if any(distance(a[1], b[1]) < 1e-8 for a, b in pairwise(frames)):
            raise ValueError("Continuous patrol must not contain stationary intervals")
        return MovingSphere(identifier, self.radius_m, tuple(frames))

    def challenge(
        self, title: str, kind: str, *, lead_s: float = 22, tail_s: float = 45
    ) -> dict[str, Any]:
        return {
            "kind": kind,
            "title": title,
            "focusPosition": list(self.position),
            "startTimeS": max(0.0, self.arrival_s - lead_s),
            "endTimeS": self.arrival_s + tail_s,
            "source": "declared-static-reference-not-strategy-outcome",
        }


def static_reference(
    scene: Scene, tasks: list[dict[str, Any]], *, resolution: float = 50
) -> list[Point3]:
    planner = AStar3D(AStarConfig(resolution=resolution, max_expansions=40_000))
    points: list[Point3] = []
    targets = [scene.start, *(tuple(task["position"]) for task in tasks), scene.goal]
    for start, goal in pairwise(targets):
        result = planner.plan(replace(scene, start=start, goal=goal))
        if not result.success:
            raise ValueError(f"No static reference for {scene.scene_id}: {result.failure_reason}")
        points.extend(result.path if not points else result.path[1:])
    return points


def select_encounter(
    scene: Scene,
    path: list[Point3],
    tasks: list[dict[str, Any]],
    *,
    fraction: float,
    speed: float,
    kind: str = "crossing",
    radius_m: float = 24,
    anchor_clearance_m: float = 90,
    protected_anchors: tuple[Point3, ...] = (),
    continuous_patrol: bool = False,
    patrol_centres: tuple[Point3, ...] = (),
) -> Encounter:
    """Choose a mid-leg encounter in certified free air at the reference's height.

    No endpoint, required service roof or traffic endpoint is obstructed. If no
    appropriate corridor exists, fail instead of moving traffic above the flight.
    """
    if not 0 < fraction < 1 or speed <= 0 or radius_m <= 0:
        raise ValueError("Invalid encounter parameters")
    if kind not in ("crossing", "head-on", "leader"):
        raise ValueError("Unknown encounter geometry")
    length = polyline_length(path)
    anchors = [
        scene.start,
        scene.goal,
        *(tuple(task["position"]) for task in tasks),
        *protected_anchors,
    ]
    samples = []
    traversed, dwell = 0.0, 0.0
    for a, b in pairwise(path):
        span = distance(a, b)
        dx, dy = b[0] - a[0], b[1] - a[1]
        horizontal = math.hypot(dx, dy)
        if horizontal > 1e-8:
            for part in (0.25, 0.5, 0.75):
                position = lerp(a, b, part)
                progress = traversed + span * part
                arrival = progress / speed + dwell
                if (
                    arrival < 30
                    or min(
                        math.hypot(position[0] - anchor[0], position[1] - anchor[1])
                        for anchor in anchors
                    )
                    < anchor_clearance_m
                ):
                    continue
                direction = (
                    (-dy / horizontal, dx / horizontal)
                    if kind == "crossing"
                    else (dx / horizontal, dy / horizontal)
                )
                if kind == "head-on":
                    direction = (-direction[0], -direction[1])
                samples.append((abs(progress / length - fraction), position, arrival, direction))
        traversed += span
        dwell += sum(
            task["serviceDurationS"] for task in tasks if distance(b, task["position"]) < 1e-5
        )
    samples.sort(key=lambda sample: sample[0])
    # Prefer a full cross-block corridor over a cramped corridor at the exact
    # requested progress. This gives both aircraft room to separate horizontally.
    for half_span in (100.0, 80.0, 60.0):
        for _, position, arrival, direction in samples:
            if continuous_patrol and any(
                abs(position[2] - p[2]) < 2 * radius_m + scene.required_clearance
                and math.hypot(position[0] - p[0], position[1] - p[1]) < 310
                for p in patrol_centres
            ):
                continue
            entry = (
                position[0] - direction[0] * half_span,
                position[1] - direction[1] * half_span,
                position[2],
            )
            exit = (
                position[0] + direction[0] * half_span,
                position[1] + direction[1] * half_span,
                position[2],
            )
            if (
                min(
                    math.hypot(p[0] - anchor[0], p[1] - anchor[1])
                    for p in (entry, exit)
                    for anchor in anchors
                )
                < radius_m + 15
            ):
                continue
            if not segment_is_free(
                scene, entry, exit, clearance=radius_m + scene.required_clearance
            ):
                continue
            if not continuous_patrol:
                return Encounter(position, arrival, entry, exit, radius_m)
            # Two parallel lanes joined by sampled 18 m radius turns. Select a
            # certified side, never lift the patrol above the actual encounter.
            for side in (1, -1):
                normal = (-direction[1] * side, direction[0] * side)
                radius = 18.0
                loop = [entry, exit]
                for center, initial in ((exit, -math.pi / 2), (entry, math.pi / 2)):
                    if center == entry:
                        loop.append(
                            (
                                entry[0] + 2 * radius * normal[0],
                                entry[1] + 2 * radius * normal[1],
                                entry[2],
                            )
                        )
                    for step in range(1, 13):
                        angle = initial + step * math.pi / 12
                        loop.append(
                            (
                                center[0]
                                + radius * normal[0]
                                + radius
                                * (math.cos(angle) * direction[0] + math.sin(angle) * normal[0]),
                                center[1]
                                + radius * normal[1]
                                + radius
                                * (math.cos(angle) * direction[1] + math.sin(angle) * normal[1]),
                                center[2],
                            )
                        )
                loop[-1] = entry
                if min(
                    math.hypot(p[0] - anchor[0], p[1] - anchor[1])
                    for p in loop
                    for anchor in anchors
                ) >= radius_m + 45 and all(
                    segment_is_free(scene, a, b, clearance=radius_m + scene.required_clearance)
                    for a, b in pairwise(loop)
                ):
                    return Encounter(position, arrival, entry, exit, radius_m, tuple(loop))
    raise ValueError(f"No same-height certified {kind} corridor in {scene.scene_id}")
