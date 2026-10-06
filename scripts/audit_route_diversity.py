"""Measure near-parallel horizontal route reuse, independently of display and altitude.

This is a sampled layout diagnostic, not a collision or inter-vehicle safety certificate.
Crossings are excluded by heading; only the first/last 100 m of each route are omitted
to avoid penalizing a shared depot approach. Reverse traffic still counts as reuse.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from itertools import combinations, pairwise
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPACING_M = 20.0
DISTANCE_M = 35.0
TERMINAL_TRIM_M = 100.0
HEADING_COSINE = math.cos(math.radians(25))


def horizontal_samples(points):
    segments = []
    cumulative = 0.0
    for a, b in pairwise(points):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length > 1e-9:
            segments.append((cumulative, cumulative + length, a, dx / length, dy / length))
            cumulative += length
    samples = []
    index = 0
    progress = TERMINAL_TRIM_M + SPACING_M / 2
    while progress < cumulative - TERMINAL_TRIM_M:
        while segments[index][1] < progress:
            index += 1
        start, _, a, ux, uy = segments[index]
        samples.append((a[0] + ux * (progress - start), a[1] + uy * (progress - start), ux, uy))
        progress += SPACING_M
    return samples


def directed_overlap(samples, other):
    bins = defaultdict(list)
    for point in other:
        bins[(math.floor(point[0] / DISTANCE_M), math.floor(point[1] / DISTANCE_M))].append(point)
    count = 0
    for x, y, ux, uy in samples:
        bx, by = math.floor(x / DISTANCE_M), math.floor(y / DISTANCE_M)
        matches = (
            point
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for point in bins.get((bx + dx, by + dy), ())
        )
        if any(
            math.hypot(x - ox, y - oy) <= DISTANCE_M and abs(ux * vx + uy * vy) >= HEADING_COSINE
            for ox, oy, vx, vy in matches
        ):
            count += 1
    return count / len(samples) if samples else 0.0


def route_report(routes):
    sampled = [(mission_id, horizontal_samples(points)) for mission_id, points in routes]
    pairs = []
    for (left, a), (right, b) in combinations(sampled, 2):
        forward, reverse = directed_overlap(a, b), directed_overlap(b, a)
        pairs.append(
            {
                "missions": [left, right],
                "leftFraction": round(forward, 6),
                "rightFraction": round(reverse, 6),
                "maxFraction": round(max(forward, reverse), 6),
            }
        )
    return {
        "missionCount": len(routes),
        "worstPairFraction": max((pair["maxFraction"] for pair in pairs), default=0.0),
        "meanPairFraction": round(
            math.fsum(pair["maxFraction"] for pair in pairs) / max(1, len(pairs)), 6
        ),
        "pairs": sorted(pairs, key=lambda pair: pair["maxFraction"], reverse=True),
    }


def audit_public(directory):
    studies = []
    for study, filename in (
        ("static", "demo-data.json"),
        ("dynamic", "dynamic-data.json"),
        ("predictive", "predictive-data.json"),
    ):
        bundle = json.loads((directory / filename).read_text())
        scenarios = bundle["scenarios"]
        key = "results" if study == "static" else "runs"
        for planner in (run["plannerId"] for run in scenarios[0][key]):
            routes = []
            for scenario in scenarios:
                run = next(run for run in scenario[key] if run["plannerId"] == planner)
                if run["status"] != "success":
                    raise ValueError(f"Cannot audit failed route {scenario['id']}: {planner}")
                if study == "static":
                    points = run["paths"]["smoothed"]
                elif study == "dynamic":
                    points = run["frames"][-1]["executedPath"]
                else:
                    points = [knot["position"] for knot in run["executionTimedPath"]]
                routes.append((scenario["id"], points))
            studies.append({"study": study, "planner": planner, **route_report(routes)})
    return studies


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--public", type=Path, default=ROOT / "web/public")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "data/manhattan/route-diversity-audit.json"
    )
    args = parser.parse_args()
    report = {
        "kind": "sampled-horizontal-near-parallel-route-reuse-not-safety-certification",
        "sampleSpacingM": SPACING_M,
        "proximityM": DISTANCE_M,
        "headingToleranceDeg": 25,
        "terminalTrimM": TERMINAL_TRIM_M,
        "altitudeIgnored": True,
        "studies": audit_public(args.public),
    }
    if args.baseline:
        report["baselineStudies"] = audit_public(args.baseline)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    for study in report["studies"]:
        print(json.dumps({key: value for key, value in study.items() if key != "pairs"}))


if __name__ == "__main__":
    main()
