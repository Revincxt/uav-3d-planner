"""District-scale simulated missions and verified rooftop anchors in the physical city."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace
from itertools import pairwise
from typing import TYPE_CHECKING, Any, cast

from uav3d.collision import point_is_free
from uav3d.geometry import Point3

if TYPE_CHECKING:
    from uav3d.dynamic import DynamicScenario
    from uav3d.manhattan_city import ManhattanCity, Ring
    from uav3d.replanning import DynamicFrame, DynamicRun

MISSION_SCALE_ID = "manhattan-island-multistop-v3"
MIN_MISSION_HORIZONTAL_DISTANCE_M = 4500.0
MIN_MISSION_AXIS_COVERAGE = 0.55
MAX_ANCHOR_SNAP_DISTANCE_M = 220.0

# Neighborhood service positions, not claimed real landing facilities. All resolve
# to measured NYC roof polygons; endpoints and required stops are different roles.
# Each tuple is (label, service, longitude, latitude).
TASK_ROUTES = {
    "medical-cross": (
        ("Civic Center relay", "Collect cold-chain parcel", -74.0035, 40.7140),
        ("Chinatown collection", "Collect specimen batch", -73.9990, 40.7200),
        ("NoHo relay", "Inspect cold-chain seal", -73.9925, 40.7280),
        ("Gramercy relay", "Collect specimen batch", -73.9832, 40.7384),
        ("Murray Hill clinic", "Deliver medical parcel", -73.9810, 40.7488),
        ("Midtown medical relay", "Verify receiving handoff", -73.9770, 40.7582),
    ),
    "west-delivery": (
        ("Tribeca west dispatch", "Deliver parcel", -74.0110, 40.7145),
        ("Hudson Square relay", "Collect return parcel", -74.0080, 40.7238),
        ("West Village depot", "Deliver parcel", -74.0048, 40.7325),
        ("Chelsea arts district", "Deliver parcel", -74.0024, 40.7416),
        ("High Line district", "Collect parcel", -74.0005, 40.7475),
        ("Hudson Yards relay", "Deliver parcel", -73.9984, 40.7533),
        ("Hell's Kitchen south", "Deliver parcel", -73.9959, 40.7584),
        ("Northside relay", "Verify receiving handoff", -73.9938, 40.7625),
    ),
    "hudson-inspection": (
        ("Battery waterfront service", "Inspect roof equipment", -74.0156, 40.7100),
        ("Tribeca waterfront service", "Inspect rooftop equipment", -74.0129, 40.7200),
        ("Hudson Square service", "Inspect facade from stand-off", -74.0106, 40.7293),
        ("Meatpacking service", "Inspect roof equipment", -74.0080, 40.7390),
        ("Chelsea Piers service", "Inspect rooftop equipment", -74.0062, 40.7483),
        ("Javits waterfront service", "Record final inspection", -74.0018, 40.7573),
    ),
    "medical-north": (
        ("Lower East Side clinic", "Collect specimen batch", -73.9908, 40.7205),
        ("East Village clinic", "Deliver medical parcel", -73.9843, 40.7301),
        ("Stuyvesant medical relay", "Inspect cold-chain seal", -73.9786, 40.7392),
        ("Murray Hill relay", "Collect specimen batch", -73.9761, 40.7500),
        ("Theater district clinic", "Deliver medical parcel", -73.9816, 40.7580),
        ("Columbus south relay", "Verify receiving handoff", -73.9860, 40.7634),
    ),
    "east-logistics": (
        ("Two Bridges logistics", "Deliver equipment", -73.9940, 40.7133),
        ("Lower East Side depot", "Collect return equipment", -73.9848, 40.7214),
        ("Alphabet City service", "Deliver equipment", -73.9779, 40.7280),
        ("East Village north", "Deliver equipment", -73.9769, 40.7335),
        ("Stuyvesant service", "Collect equipment", -73.9746, 40.7388),
        ("Kips Bay equipment", "Deliver equipment", -73.9739, 40.7450),
        ("Tudor City service", "Inspect equipment handoff", -73.9717, 40.7506),
        ("Turtle Bay receiving", "Deliver equipment", -73.9694, 40.7555),
    ),
    "riverfront-logistics": (
        ("Civic Center east supply", "Collect supply batch", -74.0010, 40.7138),
        ("SoHo supply relay", "Deliver supplies", -74.0020, 40.7247),
        ("Greenwich Village supply", "Collect supply batch", -73.9970, 40.7329),
        ("Flatiron west relay", "Deliver supplies", -73.9924, 40.7410),
        ("NoMad service", "Deliver supplies", -73.9867, 40.7462),
        ("Bryant Park relay", "Collect return batch", -73.9834, 40.7530),
        ("Grand Central north", "Deliver supplies", -73.9776, 40.7564),
        ("Turtle Bay receiving", "Verify receiving handoff", -73.9728, 40.7588),
    ),
    # Return collections have different customers and corridors, not the same
    # outbound checkpoints reversed. All requests resolve onto actual source roofs.
    "west-backhaul": (
        ("Hell's Kitchen north collection", "Collect return parcel", -73.9919, 40.7650),
        ("Garment district west collection", "Collect return parcel", -73.9938, 40.7544),
        ("Chelsea interior collection", "Collect reusable container", -73.9961, 40.7455),
        ("Greenwich return depot", "Collect return parcel", -73.9999, 40.7353),
        ("West SoHo collection", "Collect reusable container", -74.0035, 40.7279),
        ("Tribeca interior depot", "Collect return parcel", -74.0069, 40.7211),
        ("World Trade district", "Consolidate return parcels", -74.0105, 40.7133),
        ("Financial west depot", "Verify depot handoff", -74.0113, 40.7080),
    ),
    "medical-south": (
        ("Sutton district clinic relay", "Collect clinical equipment", -73.9725, 40.7605),
        ("Grand Central clinic relay", "Collect clinical equipment", -73.9795, 40.7514),
        ("Kips Bay west medical relay", "Inspect cold-chain seal", -73.9810, 40.7423),
        ("East Village north clinic", "Collect medical return", -73.9812, 40.7328),
        ("Lower East Side north clinic", "Deliver medical return", -73.9855, 40.7232),
        ("Two Bridges clinic", "Verify receiving handoff", -73.9939, 40.7140),
    ),
    "east-backhaul": (
        ("Bryant Park west return", "Collect reusable equipment", -73.9870, 40.7540),
        ("Flatiron collection", "Collect supply container", -73.9880, 40.7442),
        ("Stuyvesant west collection", "Collect reusable equipment", -73.9820, 40.7352),
        ("Greenwich Village east collection", "Collect supply container", -73.9930, 40.7290),
        ("Lower East Side return relay", "Collect reusable equipment", -73.9938, 40.7212),
        ("Civic Center return depot", "Collect supply container", -74.0030, 40.7186),
        ("Financial district collection", "Consolidate supply return", -74.0060, 40.7124),
        ("Seaport receiving depot", "Verify depot handoff", -73.9999, 40.7078),
    ),
    "riverfront-backhaul": (
        ("Tudor City priority", "Collect spare parts", -73.9730, 40.7509),
        ("Flatiron east priority", "Deliver spare parts", -73.9860, 40.7429),
        ("Stuyvesant priority", "Deliver spare parts", -73.9842, 40.7340),
        ("Greenwich Village east priority", "Collect spare parts", -73.9938, 40.7311),
        ("SoHo priority", "Deliver spare parts", -74.0010, 40.7242),
        ("Tribeca west relay", "Collect spare parts", -74.0070, 40.7173),
        ("Civic Center priority", "Deliver spare parts", -74.0077, 40.7102),
        ("Financial east receiving", "Verify priority handoff", -74.0040, 40.7040),
    ),
}


def _inside_ring(point: tuple[float, float], ring: Ring) -> bool:
    inside = False
    x, y = point
    for start, end in pairwise(ring):
        if (start[1] > y) != (end[1] > y):
            crossing = start[0] + (y - start[1]) * (end[0] - start[0]) / (end[1] - start[1])
            if x < crossing:
                inside = not inside
    return inside


def point_on_roof(point: tuple[float, float], rings: tuple[Ring, ...]) -> bool:
    """A real polygon interior, never the empty center of a concave roof or courtyard."""
    return _inside_ring(point, rings[0]) and not any(
        _inside_ring(point, ring) for ring in rings[1:]
    )


@dataclass(frozen=True, slots=True)
class RooftopAnchor:
    position: Point3
    building_id: str
    requested_wgs84: tuple[float, float]
    snap_distance_m: float

    def to_metadata(self) -> dict[str, Any]:
        return {
            "position": list(self.position),
            "buildingId": self.building_id,
            "requestedWgs84": list(self.requested_wgs84),
            "snapDistanceM": self.snap_distance_m,
            "selection": (
                "Interior of a nearby official low/mid-rise roof; "
                "simulated pad, not a real helipad."
            ),
        }


def resolve_rooftop_anchor(
    city: ManhattanCity,
    longitude: float,
    latitude: float,
    *,
    preferred_altitude_m: float = 0.0,
    roof_clearance_m: float = 18.0,
) -> RooftopAnchor:
    """Resolve a district landmark to a source-backed roof without arbitrary endpoint drift."""
    from uav3d.manhattan_city import make_city_scene

    east, north, _ = city.project(longitude, latitude)
    candidates = [
        (box, footprint)
        for box, footprint in zip(city.buildings, city.footprints, strict=True)
        if 12 <= box.maximum[2] <= 180 and not footprint.height_assumed
    ]
    candidates.sort(
        key=lambda entry: math.hypot(
            (entry[0].minimum[0] + entry[0].maximum[0]) / 2 - east,
            (entry[0].minimum[1] + entry[0].maximum[1]) / 2 - north,
        )
    )
    fractions = (0.5, 0.25, 0.75, 0.125, 0.875, 0.375, 0.625)
    for box, footprint in candidates[:80]:
        points = [
            (
                box.minimum[0] + fx * (box.maximum[0] - box.minimum[0]),
                box.minimum[1] + fy * (box.maximum[1] - box.minimum[1]),
            )
            for fx in fractions
            for fy in fractions
        ]
        # Prefer a real point directly under the anchor if it lies within a source roof.
        points.insert(0, (east, north))
        points = [point for point in points if point_on_roof(point, footprint.rings)]
        if not points:
            continue
        x, y = min(points, key=lambda point: math.hypot(point[0] - east, point[1] - north))
        separation = math.hypot(x - east, y - north)
        if separation > MAX_ANCHOR_SNAP_DISTANCE_M:
            continue
        roof = max(
            candidate.maximum[2]
            for candidate in city.buildings
            if candidate.minimum[0] - 6 <= x <= candidate.maximum[0] + 6
            and candidate.minimum[1] - 6 <= y <= candidate.maximum[1] + 6
        )
        position = (x, y, max(preferred_altitude_m, roof + roof_clearance_m))
        if position[2] >= city.bounds.maximum[2] - 8:
            continue
        probe = make_city_scene(
            city,
            "roof-anchor-probe",
            name="Rooftop anchor probe",
            start=position,
            goal=(x + 1, y, position[2]),
            safety_margin=5.0,
        )
        if point_is_free(probe, position):
            return RooftopAnchor(position, box.obstacle_id, (longitude, latitude), separation)
    raise ValueError(
        f"No certified source roof within {MAX_ANCHOR_SNAP_DISTANCE_M} m of {(longitude, latitude)}"
    )


def district_mission_evidence(
    city: ManhattanCity,
    start: RooftopAnchor,
    goal: RooftopAnchor,
) -> dict[str, Any]:
    dx, dy = goal.position[0] - start.position[0], goal.position[1] - start.position[1]
    width = city.bounds.maximum[0] - city.bounds.minimum[0]
    depth = city.bounds.maximum[1] - city.bounds.minimum[1]
    horizontal_distance = math.hypot(dx, dy)
    coverage = max(abs(dx) / width, abs(dy) / depth)
    if (
        horizontal_distance < MIN_MISSION_HORIZONTAL_DISTANCE_M
        or coverage < MIN_MISSION_AXIS_COVERAGE
    ):
        raise ValueError("Island missions must span at least 4.5 km and over half one city axis")
    return {
        "id": MISSION_SCALE_ID,
        "horizontalDistanceM": horizontal_distance,
        "cityAxisCoverage": coverage,
        "startAnchor": start.to_metadata(),
        "goalAnchor": goal.to_metadata(),
        "routeRule": (
            "Visit all ordered service roofs; plan every leg on the complete physical city. "
            "Optimization must retain each stop."
        ),
    }


def mission_task_points(
    city: ManhattanCity, route: str, *, preferred_altitude_m: float = 100.0
) -> list[dict[str, Any]]:
    return [
        {
            "id": f"{route}-{index + 1}",
            "order": index + 1,
            "label": label,
            "action": "Fly-through checkpoint",
            "visitMode": "fly-through",
            "serviceDurationS": 0.0,
            **resolve_rooftop_anchor(
                city, lon, lat, preferred_altitude_m=preferred_altitude_m
            ).to_metadata(),
        }
        for index, (label, _action, lon, lat) in enumerate(TASK_ROUTES[route])
    ]


def mission_leg_event(
    start: Point3, goal: Point3, tasks: list[dict[str, Any]], *, speed: float, fraction: float
) -> tuple[Point3, float, tuple[float, float]]:
    """Place simulated traffic between service roofs, including earlier dwell times."""
    from uav3d.geometry import distance, lerp

    points = [start, *(cast(Point3, tuple(task["position"])) for task in tasks), goal]
    lengths = [distance(a, b) for a, b in pairwise(points)]
    candidates = [index for index, length in enumerate(lengths) if length >= 280]
    if not candidates:
        raise ValueError("A mission needs a cross-block corridor for its simulated traffic")
    index = min(candidates, key=lambda index: abs((index + 0.5) / len(lengths) - fraction))
    a, b = points[index : index + 2]
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = math.hypot(dx, dy)
    arrival = (sum(lengths[:index]) + lengths[index] / 2) / speed + sum(
        task["serviceDurationS"] for task in tasks[:index]
    )
    return lerp(a, b, 0.5), arrival, (-dy / norm, dx / norm)


def audit_task_visits(
    positions: Sequence[Sequence[float]],
    tasks: list[dict[str, Any]],
    *,
    times: Sequence[float] | None = None,
    tolerance: float = 1e-5,
) -> None:
    """Independent ordered hard-stop audit, including service time when available."""
    from uav3d.geometry import distance

    cursor = 0
    for task in tasks:
        while (
            cursor < len(positions)
            and distance(cast(Point3, positions[cursor]), task["position"]) > tolerance
        ):
            cursor += 1
        if cursor == len(positions):
            raise ValueError(f"Required task {task['id']} was omitted or visited out of order")
        if times is not None:
            departure = cursor
            while (
                departure + 1 < len(positions)
                and distance(cast(Point3, positions[departure + 1]), task["position"]) <= tolerance
            ):
                departure += 1
            if times[departure] - times[cursor] < task["serviceDurationS"] - tolerance:
                raise ValueError(f"Required service time missing at {task['id']}")
            if task.get("visitMode") == "fly-through" and (
                times[departure] - times[cursor] > tolerance
            ):
                raise ValueError(f"Fly-through task must not contain a dwell: {task['id']}")
            cursor = departure
        cursor += 1


def simulate_mission_replanning(
    scenario: DynamicScenario, algorithm: str, tasks: list[dict[str, Any]], **options: Any
) -> DynamicRun:
    """Run each required leg on one absolute clock; reset goal-dependent search per leg."""
    from uav3d.dynamic import dynamic_scenario_fingerprint
    from uav3d.dynamic_collision import spacetime_segment_is_free
    from uav3d.geometry import distance
    from uav3d.predictive import TimedPath, TimedWaypoint
    from uav3d.replanning import DynamicRun, simulate_replanning

    if not tasks:
        return simulate_replanning(scenario, algorithm, **options)
    frames: list[DynamicFrame] = []
    trace: list[TimedWaypoint] = []
    runs = []
    start = scenario.static_scene.start
    clock = 0.0
    targets = [cast(Point3, tuple(task["position"])) for task in tasks] + [
        scenario.static_scene.goal
    ]
    departure_heading = None
    for index, goal in enumerate(targets):
        leg = replace(scenario, static_scene=replace(scenario.static_scene, start=start, goal=goal))
        leg_options = dict(options)
        if options.get("smooth_turns"):
            axes = range(3 if options.get("spatial_curves") else 2)
            leg_options["initial_heading"] = departure_heading
            if index + 1 < len(targets):
                following = targets[index + 1]
                vectors = [
                    tuple(goal[i] - start[i] for i in axes),
                    tuple(following[i] - goal[i] for i in axes),
                ]
                units = [
                    tuple(value / math.hypot(*v) for value in v)
                    for v in vectors
                    if math.hypot(*v) > 1e-8
                ]
                leg_options["arrival_heading"] = tuple(sum(v[i] for v in units) for i in axes)
        run = simulate_replanning(leg, algorithm, start_time=clock, **leg_options)
        if run.execution_timed_path is not None:
            trace.extend(
                run.execution_timed_path.waypoints
                if not trace
                else run.execution_timed_path.waypoints[1:]
            )
        if options.get("smooth_turns") and len(run.frames) >= 2:
            path = run.frames[-2].planned_path
            departure_heading = next(
                (
                    tuple(b[i] - a[i] for i in range(3 if options.get("spatial_curves") else 2))
                    for a, b in reversed(list(pairwise(path)))
                    if math.hypot(*(b[i] - a[i] for i in axes)) > 1e-8
                ),
                None,
            )
        if frames and run.frames and abs(frames[-1].time_s - run.frames[0].time_s) < 1e-10:
            # A fly-through leg seam has one real timestamp, not a zero-time hold
            # or an intermediate mission-complete event. Keep the outgoing plan.
            frames[-1] = run.frames[0]
            frames.extend(run.frames[1:])
        else:
            frames.extend(run.frames)
        runs.append(run)
        if not run.metrics.success:
            break
        clock = run.frames[-1].time_s
        if index < len(tasks) and tasks[index]["serviceDurationS"] > 0:
            departure = clock + tasks[index]["serviceDurationS"]
            if departure >= options.get("max_time", 180.0) or not spacetime_segment_is_free(
                scenario, goal, goal, clock, departure
            ):
                raise ValueError(f"No safe service window at {tasks[index]['id']}")
            frames[-1] = replace(frames[-1], status="hold")
            clock = departure
            if trace:
                trace.append(TimedWaypoint(departure, goal, "wait"))
        start = goal
    metrics = replace(
        runs[-1].metrics,
        success=len(runs) == len(targets) and runs[-1].metrics.success,
        executed_path_length_m=sum(run.metrics.executed_path_length_m for run in runs),
        direct_distance_m=distance(scenario.static_scene.start, scenario.static_scene.goal),
        replans=sum(run.metrics.replans for run in runs),
        failed_replans=sum(run.metrics.failed_replans for run in runs),
        holds=sum(run.metrics.holds for run in runs)
        + sum(task["serviceDurationS"] > 0 for task in tasks[: len(runs) - 1]),
        safety_gate_activations=sum(run.metrics.safety_gate_activations for run in runs),
        collision_count=sum(run.metrics.collision_count for run in runs),
        total_planning_work=sum(run.metrics.total_planning_work for run in runs),
        total_changed_edges=sum(run.metrics.total_changed_edges for run in runs),
    )
    metrics = replace(
        metrics,
        path_excess_ratio=metrics.executed_path_length_m / metrics.direct_distance_m - 1
        if metrics.success
        else None,
    )
    if metrics.success:
        audit_task_visits(
            [frame.position for frame in frames], tasks, times=[frame.time_s for frame in frames]
        )
    parameters = {
        **runs[0].parameters,
        "task_point_count": len(tasks),
        "leg_count": len(targets),
        "service_duration_total_s": sum(task["serviceDurationS"] for task in tasks),
    }
    return DynamicRun(
        scenario.scenario_id,
        dynamic_scenario_fingerprint(scenario),
        algorithm,
        parameters,
        tuple(frames),
        metrics,
        TimedPath(tuple(trace)) if trace else None,
    )
