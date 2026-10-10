"""Do not conceal reused routes with height offsets or a sampled crossing count."""

from __future__ import annotations

import runpy
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
AUDIT = runpy.run_path(str(ROOT / "scripts/audit_route_diversity.py"))


class RouteDiversityTests(unittest.TestCase):
    def overlap(self, left, right):
        samples = AUDIT["horizontal_samples"]
        return AUDIT["directed_overlap"](samples(left), samples(right))

    def test_detects_same_corridor_in_both_directions_and_at_different_altitudes(self):
        left = [(0, 0, 40), (1200, 0, 40)]
        for other in (left, left[::-1], [(0, 0, 200), (1200, 0, 200)]):
            self.assertEqual(self.overlap(left, other), 1.0)

    def test_distinct_parallel_corridors_and_perpendicular_crossings_are_not_reuse(self):
        left = [(0, 0, 40), (1200, 0, 40)]
        self.assertEqual(self.overlap(left, [(0, 200, 40), (1200, 200, 40)]), 0.0)
        self.assertEqual(self.overlap(left, [(600, -600, 40), (600, 600, 40)]), 0.0)
        self.assertEqual(self.overlap(left, [(0, 20, 40), (1200, 20, 40)]), 1.0)

    def test_omits_only_the_declared_terminal_approach_not_the_route_interior(self):
        left = [(0, 0, 40), (1200, 0, 40)]
        terminal_only = [(0, 0, 40), (50, 0, 40), (50, 600, 40), (1200, 600, 40)]
        self.assertEqual(self.overlap(left, terminal_only), 0.0)
        interior = [(0, 600, 40), (200, 0, 40), (1000, 0, 40), (1200, 600, 40)]
        self.assertGreater(self.overlap(left, interior), 0.7)
        self.assertEqual(AUDIT["horizontal_samples"]([(0, 0, 40), (0, 0, 140)]), [])

    def test_every_published_planner_has_eight_distinct_corridors(self):
        for study in AUDIT["audit_public"](ROOT / "web/public"):
            with self.subTest(study=study["study"], planner=study["planner"]):
                self.assertEqual(study["missionCount"], 8)
                # Efficient low-altitude urban flights may share short street links.
                # Keep the full XY diagnostic (including reverse traffic), but
                # do not force extra flight distance just to separate two drawings.
                # Bound both a local shared corridor and cohort-wide repetition;
                # this layout policy is not an inter-vehicle safety certificate.
                self.assertLess(study["worstPairFraction"], 0.15)
                self.assertLess(study["meanPairFraction"], 0.01)
                self.assertEqual(len(study["pairs"]), 28)


if __name__ == "__main__":
    unittest.main()
