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

MISSION_SCALE_ID = "cross-district-multistop-v2"
MIN_MISSION_HORIZONTAL_DISTANCE_M = 2400.0
MIN_MISSION_AXIS_COVERAGE = 0.55
MAX_ANCHOR_SNAP_DISTANCE_M = 220.0

# Neighborhood service positions, not claimed real landing facilities. All resolve
# to measured NYC roof polygons; endpoints and required stops are different roles.
# Each tuple is (label, service, longitude, latitude).
TASK_ROUTES = {
    "medical-cross": (
        ("Chelsea collection", "Collect cold-chain parcel", -74.0008, 40.7409),
        ("Flatiron relay", "Collect specimen batch", -73.9950, 40.7408),
        ("NoMad west", "Inspect cold-chain seal", -73.9900, 40.7420),
        ("NoMad east", "Collect specimen batch", -73.9853, 40.7419),
        ("Kips Bay relay", "Deliver medical parcel", -73.9809, 40.7423),
        ("Medical district relay", "Verify receiving handoff", -73.9765, 40.7424),
    ),
    "west-delivery": (
        ("Chelsea arts district", "Deliver parcel", -74.0038, 40.7423),
        ("High Line district", "Collect return parcel", -74.0018, 40.7463),
        ("Hudson Yards south", "Deliver parcel", -73.9991, 40.7508),
        ("West 34th relay", "Deliver parcel", -73.9973, 40.7540),
        ("Javits north relay", "Collect parcel", -73.9982, 40.7568),
        ("Hell's Kitchen south", "Deliver parcel", -73.9949, 40.7590),
        ("Hell's Kitchen central", "Deliver parcel", -73.9918, 40.7616),
        ("Northside relay", "Verify receiving handoff", -73.9935, 40.7642),
    ),
    "hudson-inspection": (
        ("Hudson waterfront roof", "Inspect roof equipment", -74.0060, 40.7480),
        ("West 30th service", "Inspect rooftop equipment", -74.0044, 40.7511),
        ("Javits waterfront service", "Inspect facade from stand-off", -74.0028, 40.7545),
        ("West 40th service", "Inspect roof equipment", -74.0002, 40.7580),
        ("De Witt Clinton service", "Inspect rooftop equipment", -73.9974, 40.7614),
        ("West 52nd service", "Record final inspection", -73.9956, 40.7640),
    ),
    "medical-north": (
        ("Kips Bay collection", "Collect specimen batch", -73.9776, 40.7460),
        ("Murray Hill relay", "Deliver medical parcel", -73.9761, 40.7500),
        ("Grand Central east", "Inspect cold-chain seal", -73.9782, 40.7524),
        ("Midtown central relay", "Collect specimen batch", -73.9820, 40.7572),
        ("Theater district relay", "Deliver medical parcel", -73.9846, 40.7605),
        ("Columbus south relay", "Verify receiving handoff", -73.9858, 40.7634),
    ),
    "east-logistics": (
        ("Flatiron north", "Deliver equipment", -73.9884, 40.7412),
        ("NoMad service roof", "Collect return equipment", -73.9879, 40.7451),
        ("Murray Hill west", "Deliver equipment", -73.9831, 40.7470),
        ("Murray Hill east", "Deliver equipment", -73.9782, 40.7483),
        ("Grand Central east", "Collect equipment", -73.9749, 40.7520),
        ("Turtle Bay south", "Deliver equipment", -73.9709, 40.7538),
        ("Turtle Bay service", "Inspect equipment handoff", -73.9689, 40.7560),
        ("Midtown East relay", "Deliver equipment", -73.9719, 40.7575),
    ),
    "riverfront-logistics": (
        ("West Chelsea service", "Collect supply batch", -74.0038, 40.7477),
        ("Hudson Yards service", "Deliver supplies", -73.9991, 40.7508),
        ("Penn west relay", "Collect supply batch", -73.9947, 40.7502),
        ("Herald Square relay", "Deliver supplies", -73.9899, 40.7500),
        ("Bryant Park district", "Deliver supplies", -73.9851, 40.7539),
        ("Grand Central relay", "Collect return batch", -73.9787, 40.7523),
        ("Turtle Bay south", "Deliver supplies", -73.9743, 40.7545),
        ("East Midtown relay", "Verify receiving handoff", -73.9741, 40.7558),
    ),
    # Return collections have different customers and corridors, not the same
    # outbound checkpoints reversed. All requests resolve onto actual source roofs.
    "west-backhaul": (
        ("Theater district collection", "Collect return parcel", -73.9858, 40.7629),
        ("Seventh Avenue return", "Collect return parcel", -73.9877, 40.7599),
        ("Garment district north", "Collect reusable container", -73.9904, 40.7565),
        ("Penn district north", "Collect return parcel", -73.9930, 40.7534),
        ("Penn district south", "Collect reusable container", -73.9956, 40.7501),
        ("Eighth Avenue Chelsea", "Collect return parcel", -73.9981, 40.7469),
        ("Chelsea interior depot", "Consolidate return parcels", -74.0006, 40.7439),
        ("South Chelsea collection", "Verify depot handoff", -74.0030, 40.7405),
    ),
    "medical-south": (
        ("Plaza medical dispatch", "Collect clinical equipment", -73.9773, 40.7640),
        ("East 55th clinic relay", "Collect clinical equipment", -73.9731, 40.7600),
        ("East 50th medical relay", "Inspect cold-chain seal", -73.9706, 40.7554),
        ("Turtle Bay east clinic", "Collect medical return", -73.9700, 40.7516),
        ("First Avenue medical relay", "Deliver medical return", -73.9706, 40.7479),
        ("Kips Bay east receiving", "Verify receiving handoff", -73.9725, 40.7449),
    ),
    "east-backhaul": (
        ("Rockefeller supply collection", "Collect reusable equipment", -73.9789, 40.7585),
        ("Fifth Avenue collection", "Collect supply container", -73.9814, 40.7558),
        ("Library district return", "Collect reusable equipment", -73.9841, 40.7526),
        ("Herald Square east return", "Collect supply container", -73.9872, 40.7496),
        ("NoMad Broadway collection", "Collect reusable equipment", -73.9900, 40.7468),
        ("Madison Square west depot", "Collect supply container", -73.9924, 40.7439),
        ("Flatiron west collection", "Consolidate supply return", -73.9946, 40.7417),
        ("Chelsea east return depot", "Verify depot handoff", -73.9942, 40.7397),
    ),
    "riverfront-backhaul": (
        ("Turtle Bay north priority", "Collect spare parts", -73.9754, 40.7590),
        ("Sixth Avenue north relay", "Deliver spare parts", -73.9811, 40.7610),
        ("Theater west priority relay", "Deliver spare parts", -73.9868, 40.7607),
        ("Hell's Kitchen east relay", "Collect spare parts", -73.9923, 40.7589),
        ("West 39th priority relay", "Deliver spare parts", -73.9970, 40.7570),
        ("Javits inland dispatch", "Collect spare parts", -73.9994, 40.7539),
        ("Hudson Yards west relay", "Deliver spare parts", -74.0026, 40.7512),
        ("West Chelsea priority receiving", "Verify priority handoff", -74.0048, 40.7484),
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
        raise ValueError("District missions must span at least 2.4 km and over half one city axis")
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
    from uav3d.replanning import DynamicRun, simulate_replanning

    if not tasks:
        return simulate_replanning(scenario, algorithm, **options)
    frames: list[DynamicFrame] = []
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
            leg_options["initial_heading"] = departure_heading
            if index + 1 < len(targets):
                following = targets[index + 1]
                vectors = [
                    (goal[0] - start[0], goal[1] - start[1]),
                    (following[0] - goal[0], following[1] - goal[1]),
                ]
                units = [
                    (v[0] / math.hypot(*v), v[1] / math.hypot(*v))
                    for v in vectors
                    if math.hypot(*v) > 1e-8
                ]
                leg_options["arrival_heading"] = (
                    sum(v[0] for v in units),
                    sum(v[1] for v in units),
                )
        run = simulate_replanning(leg, algorithm, start_time=clock, **leg_options)
        if options.get("smooth_turns") and len(run.frames) >= 2:
            path = run.frames[-2].planned_path
            departure_heading = next(
                (
                    (b[0] - a[0], b[1] - a[1])
                    for a, b in reversed(list(pairwise(path)))
                    if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-8
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
    )
