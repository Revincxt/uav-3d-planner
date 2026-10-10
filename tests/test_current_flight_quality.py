"""Inspect displayed XYZ curves without treating a vertical leg's XY bearing as motion."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

PUBLIC = Path(__file__).resolve().parents[1] / "web" / "public"


def spatial_heading_steps(points: list[list[float]]) -> list[float]:
    angles = []
    for a, b, c in zip(points, points[1:], points[2:], strict=False):
        u = tuple(b[axis] - a[axis] for axis in range(3))
        v = tuple(c[axis] - b[axis] for axis in range(3))
        lengths = math.hypot(*u), math.hypot(*v)
        # Real holds have no spatial tangent. Ignore sub-millimetre noise.
        if min(lengths) <= 1e-4:
            continue
        cosine = sum(x * y for x, y in zip(u, v, strict=True)) / math.prod(lengths)
        angles.append(math.degrees(math.acos(max(-1, min(1, cosine)))))
    return angles


def test_spatial_diagnostic_detects_a_vertical_corner() -> None:
    assert spatial_heading_steps([[0, 0, 0], [0, 0, 20], [20, 0, 20]]) == [90]


@pytest.mark.parametrize("study", ["demo", "dynamic", "predictive"])
def test_every_displayed_mission_has_smooth_spatial_turns(study: str) -> None:
    bundle = json.loads((PUBLIC / f"{study}-data.json").read_text())
    for scene in bundle["scenarios"]:
        for run in scene.get("results", scene.get("runs", [])):
            if study == "demo":
                points = run["paths"]["smoothed"]
                assert run["parameters"]["vertical_cost_scale"] == 5
                assert run["smoothing"]["optimizationAxes"] == ["x", "y", "z"]
            elif study == "dynamic":
                points = [p["position"] for p in run["executionTimedPath"]]
                assert run["parameters"]["verticalCostScale"] == 5
                assert run["parameters"]["maxClimbRateMps"] == 3
                assert run["parameters"]["curveDimensions"] == 3
                assert run["metrics"]["collisionCount"] == 0
            else:
                assert run["smoothing"]["execution"]["qualified"]
                points = [p["position"] for p in run["executionTimedPath"]]
                assert run["smoothing"]["optimizationAxes"] == ["x", "y", "z"]
            angles = spatial_heading_steps(points)
            context = (scene["id"], run["plannerId"])
            if study == "dynamic":
                # Reactive, two-second control boundaries may retain a safe chord
                # when no collision-free tangent connector exists. Do not claim a
                # global C2/flight-dynamics certificate for that online trace.
                # At least 99% of its spatial joins must be densely rounded;
                # remaining safety fallbacks cannot introduce a right-angle turn.
                assert sum(angle > 3.1 for angle in angles) <= len(angles) * 0.01, context
                assert max(angles, default=0) < 90, context
            else:
                assert max(angles, default=0) <= 3.1, context
