"""City challenge types must encode real interacting, certified physical obstacles."""

import json
import runpy
import unittest
from itertools import combinations, pairwise
from pathlib import Path

from uav3d.collision import point_is_free, segment_is_free
from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import distance
from uav3d.manhattan_city import build_manhattan_city
from uav3d.manhattan_predictive import build_manhattan_missions

ROOT = Path(__file__).resolve().parents[1]


class ManhattanChallengeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.city = build_manhattan_city()
        cls.exporter = runpy.run_path(str(ROOT / "scripts/export_manhattan_static_dynamic.py"))
        cls.missions = cls.exporter["mission_scenes"](cls.city)
        cls.static = json.loads((ROOT / "web/public/demo-data.json").read_text())

    def test_static_constraints_are_shared_and_service_roofs_stay_free(self):
        self.assertEqual([len(scene.no_fly_zones) for _, scene in self.missions], [3] * 8)
        self.assertTrue(
            all(
                scene.no_fly_zones is self.missions[0][1].no_fly_zones for _, scene in self.missions
            )
        )
        self.assertEqual(len({m["sharedWorld"]["fingerprint"] for m, _ in self.missions}), 1)
        self.assertEqual(len({mission["challenge"]["title"] for mission, _ in self.missions}), 8)
        for mission, scene in self.missions:
            self.assertGreaterEqual(mission["challenge"]["blockedDirectLegs"], 2)
            self.assertTrue(
                all(point_is_free(scene, tuple(task["position"])) for task in mission["taskPoints"])
            )

    def test_all_dynamic_tasks_have_same_height_interacting_cargo_traffic(self):
        baselines = [s["results"][0]["paths"]["raw"] for s in self.static["scenarios"]]
        scenarios = self.exporter["shared_dynamic_scenarios"](self.missions, baselines)
        modes = []
        for (_mission, scene), scenario in zip(self.missions, scenarios, strict=True):
            self.assertEqual(len(scenario.moving_spheres), 7)
            self.assertIs(scenario.moving_spheres, scenarios[0].moving_spheres)
            self.assertIs(scenario.temporary_cylinders, scenarios[0].temporary_cylinders)
            challenge = scenario.metadata["challenge"]
            arrival = challenge["startTimeS"] + 22
            point = tuple(challenge["focusPosition"])
            aircraft = next(
                (
                    a
                    for a in scenario.moving_spheres
                    if a.sphere_id == f"{scene.scene_id}-{scenario.metadata['trafficMode']}-cargo"
                ),
                None,
            )
            if aircraft is not None:
                self.assertEqual(aircraft.radius, 24)
                self.assertLess(distance(aircraft.position_at(arrival), point), 1e-6)
                # Ignoring the obstacle at the declared encounter is genuinely unsafe.
                self.assertFalse(
                    spacetime_segment_is_free(scenario, point, point, arrival, arrival + 0.01)
                )
            for aircraft in scenario.moving_spheres:
                for (_, a), (_, b) in pairwise(aircraft.keyframes):
                    self.assertTrue(
                        segment_is_free(
                            scene, a, b, clearance=aircraft.radius + scene.required_clearance
                        )
                    )
            modes.append(scenario.metadata["trafficMode"])
        self.assertEqual(
            modes,
            [
                "crossing",
                "head-on",
                "leader",
                "crossing",
                "head-on",
                "crossing",
                "head-on",
                "crossing",
            ],
        )
        self.assert_continuous_traffic(scenarios[0], 600)

    def test_predictive_has_eight_task_queries_and_certified_shared_traffic(self):
        scenarios = build_manhattan_missions(self.city)
        self.assertEqual([len(s.temporary_cylinders) for s in scenarios], [4] * 8)
        self.assertEqual([len(s.moving_spheres) for s in scenarios], [7] * 8)
        self.assertTrue(all(s.moving_spheres is scenarios[0].moving_spheres for s in scenarios))
        self.assertTrue(
            all(s.temporary_cylinders is scenarios[0].temporary_cylinders for s in scenarios)
        )
        self.assertEqual(len({s.metadata["mission"]["challenge"]["title"] for s in scenarios}), 8)
        self.assert_continuous_traffic(scenarios[0], 900)
        for scenario in scenarios:
            scene = scenario.static_scene
            for aircraft in scenario.moving_spheres:
                for (_, a), (_, b) in pairwise(aircraft.keyframes):
                    self.assertTrue(
                        segment_is_free(
                            scene, a, b, clearance=aircraft.radius + scene.required_clearance
                        )
                    )
            for zone in scenario.temporary_cylinders:
                for task in scenario.metadata["mission"]["taskPoints"]:
                    self.assertGreater(
                        (
                            (task["position"][0] - zone.center[0]) ** 2
                            + (task["position"][1] - zone.center[1]) ** 2
                        )
                        ** 0.5,
                        zone.radius + scene.required_clearance,
                    )

    def assert_continuous_traffic(self, scenario, horizon):
        for aircraft in scenario.moving_spheres:
            self.assertEqual(aircraft.keyframes[0][0], 0)
            self.assertEqual(aircraft.keyframes[-1][0], horizon)
            velocities = []
            for (t0, a), (t1, b) in pairwise(aircraft.keyframes):
                self.assertGreater(t1, t0)
                self.assertGreater(distance(a, b), 1e-8)
                self.assertEqual(a[2], b[2])
                velocities.append(distance(a, b) / (t1 - t0))
            self.assertLess(max(velocities) - min(velocities), 1e-6)
        # Both patrols are piecewise linear on this union of knot times: the
        # relative closest point certifies the complete interval, not samples.
        for left, right in combinations(scenario.moving_spheres, 2):
            clocks = sorted({t for aircraft in (left, right) for t, _ in aircraft.keyframes})
            for start, end in pairwise(clocks):
                a = tuple(
                    x - y
                    for x, y in zip(left.position_at(start), right.position_at(start), strict=True)
                )
                b = tuple(
                    x - y
                    for x, y in zip(left.position_at(end), right.position_at(end), strict=True)
                )
                delta = tuple(y - x for x, y in zip(a, b, strict=True))
                squared = sum(v * v for v in delta)
                fraction = (
                    max(0, min(1, -sum(x * v for x, v in zip(a, delta, strict=True)) / squared))
                    if squared
                    else 0
                )
                separation = (
                    sum((x + fraction * v) ** 2 for x, v in zip(a, delta, strict=True)) ** 0.5
                )
                self.assertGreater(separation, left.radius + right.radius)
