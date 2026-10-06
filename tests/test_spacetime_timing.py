"""Scheduling repairs preserve geometry and fail closed on unsafe waiting."""

import unittest
from dataclasses import replace

from uav3d.dynamic import DynamicScenario, TemporaryCylinder
from uav3d.kinematics import DiscreteExecutionEnvelope, qualify_timed_path_execution
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.scene import Bounds3D, Scene
from uav3d.spacetime_timing import insert_braking_holds, schedule_safe_departures
from uav3d.trajectory_timing import retime_timed_path


class SpaceTimeSchedulingTests(unittest.TestCase):
    def setUp(self):
        self.scene = Scene(
            "schedule",
            "Schedule",
            Bounds3D((0, 0, 0), (100, 100, 100)),
            (10, 50, 20),
            (90, 50, 20),
            drone_radius=0,
            safety_margin=0,
        )
        self.envelope = DiscreteExecutionEnvelope(max_speed_mps=10, max_execution_time_s=100)
        self.path = TimedPath(
            (
                TimedWaypoint(0, (10, 50, 20), "start"),
                TimedWaypoint(10, (50, 50, 20), "move"),
                TimedWaypoint(16, (50, 50, 20), "wait"),
                TimedWaypoint(26, (90, 50, 20), "move"),
            )
        )

    def test_no_conflict_leaves_timestamps_positions_and_service_unchanged(self):
        scenario = DynamicScenario("clear", "Clear", self.scene)
        self.assertEqual(schedule_safe_departures(scenario, self.path, self.envelope), self.path)

    def test_wait_before_departure_then_preserve_all_knots_and_original_dwell(self):
        zone = TemporaryCylinder("reservation", (30, 50), 6, 0, 40, 0, 12)
        scenario = DynamicScenario("slot", "Slot", self.scene, temporary_cylinders=(zone,))
        result = schedule_safe_departures(scenario, self.path, self.envelope)
        self.assertIsNotNone(result)
        self.assertTrue(result.is_safe(scenario))
        self.assertTrue(qualify_timed_path_execution(result, self.envelope).qualified)
        self.assertEqual(result.waypoints[1].position, self.path.waypoints[0].position)
        self.assertEqual(result.waypoints[1].action, "wait")
        self.assertGreater(result.wait_time_s, self.path.wait_time_s)
        self.assertEqual(result.waypoints[2].position, (50, 50, 20))
        self.assertAlmostEqual(result.waypoints[3].time_s - result.waypoints[2].time_s, 6)

    def test_unsafe_additional_wait_exposes_no_candidate(self):
        obstacle = TemporaryCylinder("blocking", (30, 50), 6, 0, 40, 0, 50)
        source = TemporaryCylinder("unsafe-hold", (10, 50), 2, 0, 40, 1, 100)
        scenario = DynamicScenario(
            "unsafe", "Unsafe", self.scene, temporary_cylinders=(obstacle, source)
        )
        self.assertIsNone(schedule_safe_departures(scenario, self.path, self.envelope))

    def test_schedule_whole_service_leg_not_unsafe_intermediate_wait(self):
        # The destination service roof stays safe. An intermediate planning hold
        # becomes unsafe, so departure must be delayed at the source instead.
        scene = replace(
            self.scene,
            metadata={"missionTaskPoints": [{"position": (90, 50, 20), "serviceDurationS": 6}]},
        )
        path = TimedPath(
            (
                TimedWaypoint(0, (10, 50, 20), "start"),
                TimedWaypoint(10, (50, 50, 20), "move"),
                TimedWaypoint(12, (50, 50, 20), "wait"),
                TimedWaypoint(22, (90, 50, 20), "move"),
                TimedWaypoint(28, (90, 50, 20), "wait"),
            )
        )
        zone = TemporaryCylinder("middle", (50, 50), 6, 0, 40, 8, 30)
        scenario = DynamicScenario("leg", "Leg", scene, temporary_cylinders=(zone,))
        result = schedule_safe_departures(scenario, path, self.envelope)
        self.assertIsNotNone(result)
        self.assertTrue(result.is_safe(scenario))
        self.assertGreater(result.waypoints[1].time_s, 20)
        self.assertEqual(result.waypoints[1].position, scene.start)

    def test_retained_vertical_reversal_requires_real_braking_and_qualification(self):
        path = TimedPath(
            (
                TimedWaypoint(0, (10, 50, 30), "start"),
                TimedWaypoint(2, (11, 50, 20), "move"),
                TimedWaypoint(4, (12, 50, 30), "move"),
            )
        )
        held = insert_braking_holds(path, self.envelope)
        self.assertEqual(
            [w.position for w in held.waypoints],
            [(10, 50, 30), (11, 50, 20), (11, 50, 20), (12, 50, 30)],
        )
        result = retime_timed_path(held, self.envelope)
        self.assertTrue(result.qualification.qualified)
        self.assertEqual(result.qualification.diagnostics.reversal_count, 0)
        self.assertGreater(result.candidate_duration_s, path.duration_s)

    def test_horizon_exhaustion_and_invalid_delay_grid_fail_closed(self):
        scenario = DynamicScenario("clear", "Clear", self.scene)
        self.assertIsNone(
            schedule_safe_departures(
                scenario, self.path, replace(self.envelope, max_execution_time_s=5)
            )
        )
        for invalid in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                schedule_safe_departures(scenario, self.path, self.envelope, time_step_s=invalid)
