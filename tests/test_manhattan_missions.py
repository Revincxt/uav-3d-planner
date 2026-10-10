"""Required city service stops are planning constraints, not decorative markers."""

from __future__ import annotations

import json
import runpy
import unittest
from dataclasses import replace
from itertools import pairwise
from pathlib import Path

from uav3d.benchmark import scene_fingerprint
from uav3d.collision import point_is_free
from uav3d.dynamic import DynamicScenario, MovingSphere, TemporaryCylinder, snapshot_scene
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.manhattan_city import build_manhattan_city
from uav3d.manhattan_missions import (
    audit_task_visits,
    mission_task_points,
    point_on_roof,
    resolve_rooftop_anchor,
    simulate_mission_replanning,
)
from uav3d.manhattan_predictive import build_manhattan_missions
from uav3d.replanning import REPLANNING_ALGORITHMS, _candidate_traversal, simulate_replanning
from uav3d.scene import Bounds3D, Scene

ROOT = Path(__file__).resolve().parents[1]


class RequiredStopTests(unittest.TestCase):
    def test_fingerprint_includes_stop_order_positions_and_dwell_but_not_labels(self):
        scene = Scene(
            "hash-stops",
            "Hash stops",
            Bounds3D((0, 0, 0), (100, 100, 60)),
            (10, 10, 20),
            (70, 10, 20),
        )
        tasks = [
            {"position": (30, 10, 20), "serviceDurationS": 6, "label": "One"},
            {"position": (50, 10, 20), "serviceDurationS": 8, "label": "Two"},
        ]
        mission = replace(scene, metadata={"missionTaskPoints": tasks})
        fingerprint = scene_fingerprint(mission)
        self.assertNotEqual(scene_fingerprint(scene), fingerprint)
        self.assertNotEqual(
            scene_fingerprint(replace(mission, metadata={"missionTaskPoints": tasks[::-1]})),
            fingerprint,
        )
        for change in ({"position": (31, 10, 20)}, {"serviceDurationS": 7}):
            changed = [{**tasks[0], **change}, tasks[1]]
            self.assertNotEqual(
                scene_fingerprint(replace(mission, metadata={"missionTaskPoints": changed})),
                fingerprint,
            )
        renamed = [{**task, "label": "Renamed"} for task in tasks]
        self.assertEqual(
            scene_fingerprint(
                replace(mission, metadata={"missionTaskPoints": renamed, "description": "New text"})
            ),
            fingerprint,
        )
        self.assertEqual(
            scene_fingerprint(replace(scene, metadata={"description": "Legacy"})),
            scene_fingerprint(scene),
        )

    def test_guard_covers_future_airspace_and_piecewise_motion_extrema(self):
        scene = Scene(
            "guard", "Guard", Bounds3D((0, 0, 0), (100, 100, 100)), (5, 5, 20), (95, 95, 20)
        )
        future = TemporaryCylinder("future", (30, 30), 4, 0, 50, 3, 10)
        sphere = MovingSphere(
            "zigzag", 2, ((0, (10, 10, 20)), (2, (40, 30, 40)), (4, (20, 20, 20)))
        )
        scenario = DynamicScenario(
            "guard", "Guard", scene, temporary_cylinders=(future,), moving_spheres=(sphere,)
        )
        instantaneous = snapshot_scene(scenario, 0)
        guarded = snapshot_scene(scenario, 0, lookahead_s=4)
        self.assertEqual(len(instantaneous.no_fly_zones), 1)
        self.assertEqual(len(guarded.no_fly_zones), 2)
        zone = next(zone for zone in guarded.no_fly_zones if zone.zone_id == "zigzag")
        for _, point in sphere.keyframes:
            self.assertLessEqual(
                ((point[0] - zone.center[0]) ** 2 + (point[1] - zone.center[1]) ** 2) ** 0.5
                + sphere.radius,
                zone.radius + 1e-9,
            )
            self.assertLessEqual(zone.z_min, point[2] - sphere.radius)
            self.assertGreaterEqual(zone.z_max, point[2] + sphere.radius)
        for lookahead in (-1, float("nan")):
            with self.assertRaisesRegex(ValueError, "lookahead"):
                snapshot_scene(scenario, 0, lookahead_s=lookahead)

    def test_polygon_interior_excludes_courtyards_and_concave_empty_areas(self):
        outer = ((0, 0), (10, 0), (10, 10), (0, 10), (0, 0))
        hole = ((3, 3), (7, 3), (7, 7), (3, 7), (3, 3))
        self.assertTrue(point_on_roof((2, 2), (outer, hole)))
        self.assertFalse(point_on_roof((5, 5), (outer, hole)))
        concave = ((0, 0), (10, 0), (10, 2), (2, 2), (2, 10), (0, 10), (0, 0))
        self.assertFalse(point_on_roof((5, 5), (concave,)))

    def test_audit_rejects_skipped_reordered_and_insufficient_service(self):
        tasks = [
            {"id": "one", "position": (1, 0, 0), "serviceDurationS": 6},
            {"id": "two", "position": (2, 0, 0), "serviceDurationS": 6},
        ]
        audit_task_visits(
            [(0, 0, 0), (1, 0, 0), (1, 0, 0), (2, 0, 0), (2, 0, 0)], tasks, times=[0, 1, 7, 8, 14]
        )
        for path in [[(0, 0, 0), (2, 0, 0)], [(2, 0, 0), (1, 0, 0)]]:
            with self.assertRaisesRegex(ValueError, "omitted|order"):
                audit_task_visits(path, tasks)
        with self.assertRaisesRegex(ValueError, "service time"):
            audit_task_visits(
                [(1, 0, 0), (1, 0, 0), (2, 0, 0), (2, 0, 0)], tasks, times=[0, 5, 6, 12]
            )

    def test_fly_through_legs_have_no_dwell_or_duplicate_clock_seams(self):
        scene = Scene(
            "fly-through",
            "Fly-through",
            Bounds3D((0, 0, 0), (100, 100, 60)),
            (10, 10, 20),
            (70, 10, 20),
            drone_radius=0,
            safety_margin=0,
        )
        scenario = DynamicScenario("fly-through", "Fly-through", scene)
        tasks = [
            {
                "id": str(i),
                "position": (x, 10, 20),
                "serviceDurationS": 0,
                "visitMode": "fly-through",
            }
            for i, x in enumerate((30, 50))
        ]
        for algorithm in REPLANNING_ALGORITHMS:
            run = simulate_mission_replanning(
                scenario,
                algorithm,
                tasks,
                time_step=1,
                replan_interval=4,
                cruise_speed=10,
                max_time=80,
                resolution=5,
                max_expansions=10000,
            )
            self.assertTrue(run.metrics.success)
            self.assertEqual(run.metrics.holds, 0)
            self.assertTrue(
                all(
                    b.time_s > a.time_s and a.position != b.position
                    for a, b in pairwise(run.frames)
                )
            )
            audit_task_visits(
                [f.position for f in run.frames], tasks, times=[f.time_s for f in run.frames]
            )
        with self.assertRaisesRegex(ValueError, "must not contain a dwell"):
            audit_task_visits([(30, 10, 20), (30, 10, 20), (50, 10, 20)], tasks, times=[0, 1, 2])

    def test_reactive_legs_keep_absolute_clock_order_and_real_work(self):
        scene = Scene(
            "stops",
            "Stops",
            Bounds3D((0, 0, 0), (100, 100, 60)),
            (10, 10, 20),
            (70, 10, 20),
            drone_radius=0,
            safety_margin=0,
        )
        hazard = TemporaryCylinder("later-leg", (60, 10), 3, 0, 60, 14, 30)
        scenario = DynamicScenario("stops", "Stops", scene, temporary_cylinders=(hazard,))
        tasks = [
            {"id": "one", "position": (30, 10, 20), "serviceDurationS": 6},
            {"id": "two", "position": (50, 10, 20), "serviceDurationS": 6},
        ]
        for algorithm in REPLANNING_ALGORITHMS:
            with self.subTest(algorithm=algorithm):
                run = simulate_mission_replanning(
                    scenario,
                    algorithm,
                    tasks,
                    time_step=1,
                    replan_interval=4,
                    cruise_speed=10,
                    max_time=80,
                    resolution=5,
                    max_expansions=10000,
                )
                self.assertTrue(run.metrics.success)
                self.assertEqual(
                    run.metrics.total_planning_work,
                    sum(frame.planning_work for frame in run.frames),
                )
                self.assertEqual(
                    run.metrics.total_changed_edges,
                    sum(frame.changed_edges for frame in run.frames),
                )
                self.assertEqual(run.parameters["leg_count"], 3)
                audit_task_visits(
                    [frame.position for frame in run.frames],
                    tasks,
                    times=[frame.time_s for frame in run.frames],
                )
                for a, b in pairwise(run.frames):
                    self.assertGreater(b.time_s, a.time_s)
                    if a.position == b.position:
                        self.assertTrue(
                            spacetime_segment_is_free(
                                scenario, a.position, b.position, a.time_s, b.time_s
                            )
                        )
                    else:
                        traversals, _, _ = _candidate_traversal(
                            a.planned_path, a.time_s, b.time_s - a.time_s, 10
                        )
                        self.assertEqual(traversals[-1].end, b.position)
                        for segment in traversals:
                            self.assertTrue(
                                spacetime_segment_is_free(
                                    scenario,
                                    segment.start,
                                    segment.end,
                                    segment.start_time,
                                    segment.end_time,
                                )
                            )

    def test_service_is_fail_closed_if_hazard_appears_during_required_dwell(self):
        scene = Scene(
            "unsafe-service",
            "Unsafe service",
            Bounds3D((0, 0, 0), (100, 100, 60)),
            (10, 10, 20),
            (70, 10, 20),
            drone_radius=0,
            safety_margin=0,
        )
        hazard = TemporaryCylinder("service-conflict", (30, 10), 3, 0, 60, 4, 8)
        scenario = DynamicScenario(
            "unsafe-service", "Unsafe service", scene, temporary_cylinders=(hazard,)
        )
        tasks = [{"id": "one", "position": (30, 10, 20), "serviceDurationS": 6}]
        with self.assertRaisesRegex(ValueError, "safe service window"):
            simulate_mission_replanning(
                scenario, "repeated-astar-3d", tasks, cruise_speed=10, max_time=60, resolution=5
            )

    def test_default_single_leg_contract_unchanged_and_absolute_clock_validated(self):
        scene = Scene(
            "single", "Single", Bounds3D((0, 0, 0), (100, 100, 60)), (10, 10, 20), (30, 10, 20)
        )
        scenario = DynamicScenario("single", "Single", scene)
        default = simulate_replanning(scenario, "repeated-astar-3d", resolution=5)
        self.assertEqual(
            default, simulate_mission_replanning(scenario, "repeated-astar-3d", [], resolution=5)
        )
        self.assertNotIn("start_time", default.parameters)
        for start_time in [-1, float("nan"), 180]:
            with self.assertRaisesRegex(ValueError, "start_time"):
                simulate_replanning(scenario, "repeated-astar-3d", start_time=start_time)


class CityMissionStopEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.city = build_manhattan_city()

    def test_new_missions_have_distinct_endpoints_and_new_service_roofs_not_reversed_stops(self):
        static = runpy.run_path(str(ROOT / "scripts/export_manhattan_static_dynamic.py"))[
            "MISSIONS"
        ]
        predictive = runpy.run_path(str(ROOT / "src/uav3d/manhattan_predictive.py"))["MISSIONS"]
        for definitions in (static, predictive):
            starts = [m["start"] if isinstance(m, dict) else m.start_lon_lat for m in definitions]
            goals = [m["goal"] if isinstance(m, dict) else m.goal_lon_lat for m in definitions]
            routes = [m["route"] if isinstance(m, dict) else m.route for m in definitions]
            self.assertEqual(len(starts), 8)
            self.assertEqual(len(set(starts)), 8)
            self.assertEqual(len(set(goals)), 8)
            self.assertTrue(set(starts[4:]).isdisjoint(starts[:4] + goals[:4]))
            self.assertTrue(set(goals[4:]).isdisjoint(starts[:4] + goals[:4]))
            start_roofs = [resolve_rooftop_anchor(self.city, *p).building_id for p in starts]
            goal_roofs = [resolve_rooftop_anchor(self.city, *p).building_id for p in goals]
            self.assertEqual(len(set(start_roofs)), 8)
            self.assertEqual(len(set(goal_roofs)), 8)
            self.assertTrue(set(start_roofs[4:]).isdisjoint(start_roofs[:4] + goal_roofs[:4]))
            self.assertTrue(set(goal_roofs[4:]).isdisjoint(start_roofs[:4] + goal_roofs[:4]))
            old_roofs = {
                task["buildingId"]
                for route in routes[:4]
                for task in mission_task_points(self.city, route)
            }
            new_roofs = set()
            for route in routes[4:]:
                tasks = mission_task_points(self.city, route)
                roofs = {task["buildingId"] for task in tasks}
                self.assertTrue(roofs.isdisjoint(old_roofs))
                self.assertTrue(roofs.isdisjoint(new_roofs))
                new_roofs.update(roofs)
                self.assertIn(len(tasks), (6, 7, 8))
                self.assertTrue(all(t["visitMode"] == "fly-through" for t in tasks))

    def test_every_mission_has_six_to_eight_distinct_source_roof_stops_and_large_span(self):
        static = runpy.run_path(str(ROOT / "scripts/export_manhattan_static_dynamic.py"))[
            "mission_scenes"
        ](self.city)
        predictive = build_manhattan_missions(self.city)
        missions = [(scene, mission) for mission, scene in static] + [
            (scenario.static_scene, scenario.metadata["mission"]) for scenario in predictive
        ]
        footprints = {footprint.building_id: footprint for footprint in self.city.footprints}
        for scene, mission in missions:
            with self.subTest(mission=scene.scene_id):
                tasks = mission["taskPoints"]
                self.assertIn(len(tasks), (6, 7, 8))
                self.assertEqual(tasks, scene.metadata["missionTaskPoints"])
                self.assertEqual(len({tuple(task["position"]) for task in tasks}), len(tasks))
                self.assertGreaterEqual(mission["planningScale"]["horizontalDistanceM"], 4500)
                self.assertGreaterEqual(mission["planningScale"]["cityAxisCoverage"], 0.55)
                for index, task in enumerate(tasks):
                    self.assertEqual(task["order"], index + 1)
                    footprint = footprints[task["buildingId"]]
                    self.assertFalse(footprint.height_assumed)
                    self.assertTrue(point_on_roof(tuple(task["position"][:2]), footprint.rings))
                    self.assertTrue(point_is_free(scene, tuple(task["position"])))
                    self.assertEqual(task["visitMode"], "fly-through")
                    self.assertEqual(task["serviceDurationS"], 0)

    def test_published_tracks_retain_all_hard_stops_in_every_evidence_layer(self):
        for track in ("demo", "dynamic", "predictive"):
            bundle = json.loads((ROOT / "web/public" / f"{track}-data.json").read_text())
            for scenario in bundle["scenarios"]:
                tasks = scenario["mission"]["taskPoints"]
                self.assertIn(len(tasks), (6, 7, 8))
                for run in scenario.get("results", scenario.get("runs")):
                    self.assertEqual(run["status"], "success")
                    if track == "demo":
                        for path in run["paths"].values():
                            audit_task_visits(path, tasks)
                    elif track == "dynamic":
                        audit_task_visits(
                            [frame["vehicle"] for frame in run["frames"]],
                            tasks,
                            times=[frame["timeS"] for frame in run["frames"]],
                        )
                    else:
                        for layer in ("rawTimedPath", "geometryTimedPath", "executionTimedPath"):
                            path = run[layer]
                            audit_task_visits(
                                [point["position"] for point in path],
                                tasks,
                                times=[point["timeS"] for point in path],
                            )

    def test_crossing_traffic_regression_does_not_trap_reactive_planners(self):
        exporter = runpy.run_path(str(ROOT / "scripts/export_manhattan_static_dynamic.py"))
        mission, scene = exporter["mission_scenes"](self.city)[1]
        baseline = exporter["static_result"](scene, "astar-3d", 18)["paths"]["raw"]
        scenario = exporter["dynamic_scenario"](mission, scene, baseline, 1)
        for algorithm in REPLANNING_ALGORITHMS:
            with self.subTest(algorithm=algorithm):
                run = simulate_mission_replanning(
                    scenario,
                    algorithm,
                    mission["taskPoints"],
                    time_step=2,
                    replan_interval=10,
                    cruise_speed=14,
                    max_time=900,
                    resolution=50,
                    max_expansions=20000,
                    shortcut_paths=True,
                    preserve_altitude=True,
                    planning_guard_s=10,
                )
                self.assertTrue(run.metrics.success)
                self.assertEqual(run.metrics.collision_count, 0)


if __name__ == "__main__":
    unittest.main()
