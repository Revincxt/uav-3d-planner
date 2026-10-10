"""Displayed city flights must retain certified, smoothly sampled horizontal turns."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

PUBLIC = Path(__file__).resolve().parents[1] / "web" / "public"


def max_horizontal_heading_step(points: list[list[float]]) -> float:
    maximum = 0.0
    for a, b, c in zip(points, points[1:], points[2:], strict=False):
        u, v = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
        lengths = math.hypot(*u), math.hypot(*v)
        # A vertical segment has no XY heading. Ignore sub-millimetre noise.
        if min(lengths) <= 1e-4:
            continue
        cosine = (u[0] * v[0] + u[1] * v[1]) / math.prod(lengths)
        maximum = max(maximum, math.degrees(math.acos(max(-1, min(1, cosine)))))
    return maximum


@pytest.mark.parametrize("study", ["demo", "dynamic", "predictive"])
def test_every_displayed_mission_has_smooth_horizontal_turns(study: str) -> None:
    bundle = json.loads((PUBLIC / f"{study}-data.json").read_text())
    for scene in bundle["scenarios"]:
        for run in scene.get("results", scene.get("runs", [])):
            if study == "demo":
                points = run["paths"]["smoothed"]
                assert run["parameters"]["vertical_cost_scale"] == 5
            elif study == "dynamic":
                points = [p["position"] for p in run["executionTimedPath"]]
                assert run["parameters"]["verticalCostScale"] == 5
                assert run["parameters"]["maxClimbRateMps"] == 3
            else:
                assert run["smoothing"]["execution"]["qualified"]
                points = [p["position"] for p in run["executionTimedPath"]]
            assert max_horizontal_heading_step(points) <= 3.1, (scene["id"], run["plannerId"])
