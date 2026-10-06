"""Measure route geometry independently of the demo's recorded planner metrics."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from itertools import pairwise
from pathlib import Path


def geometry(points: list[list[float]]) -> dict[str, float | int]:
    cleaned = []
    for point in points:
        if not cleaned or math.dist(cleaned[-1], point) > 1e-7:
            cleaned.append(point)
    lengths = [math.dist(a, b) for a, b in pairwise(cleaned)]
    angles = []
    radii = []
    for a, b, c in zip(cleaned, cleaned[1:], cleaned[2:], strict=False):
        incoming = [right - left for left, right in zip(a, b, strict=True)]
        outgoing = [right - left for left, right in zip(b, c, strict=True)]
        left_length, right_length = math.dist(a, b), math.dist(b, c)
        cosine = sum(x * y for x, y in zip(incoming, outgoing, strict=True)) / (
            left_length * right_length
        )
        angle = math.acos(max(-1, min(1, cosine)))
        angles.append(math.degrees(angle))
        if 1 < math.degrees(angle) < 179:
            radii.append(math.dist(a, c) / (2 * math.sin(angle)))
    length = sum(lengths)
    direct = math.dist(cleaned[0], cleaned[-1]) if cleaned else 0
    return {
        "waypoints": len(cleaned),
        "lengthM": length,
        "excessPct": 100 * (length / direct - 1) if direct else 0,
        "totalTurnDeg": sum(angles),
        "maxTurnDeg": max(angles, default=0),
        "turnsOver30Deg": sum(angle > 30 for angle in angles),
        "minObservedCircumradiusM": min(radii, default=0),
        "verticalTravelM": sum(abs(b[2] - a[2]) for a, b in pairwise(cleaned)),
    }


def report(public_dir: Path) -> dict[str, object]:
    records = []
    for kind, filename in (
        ("static", "demo-data.json"),
        ("reactive", "dynamic-data.json"),
        ("predictive", "predictive-data.json"),
    ):
        bundle = json.loads((public_dir / filename).read_text())
        for scenario in bundle["scenarios"]:
            for run in scenario.get("runs", scenario.get("results", [])):
                if kind == "static":
                    domains = {key: (path, None) for key, path in run["paths"].items()}
                elif kind == "reactive":
                    domains = {
                        "execution": (
                            run["frames"][-1]["executedPath"],
                            run["metrics"]["completionTimeS"],
                        )
                    }
                else:
                    domains = {}
                    for key in ("raw", "geometry", "execution"):
                        timed = run[f"{key}TimedPath"]
                        if timed:
                            domains[key] = (
                                [point["position"] for point in timed],
                                timed[-1]["timeS"] - timed[0]["timeS"],
                            )
                for domain, (points, duration) in domains.items():
                    measurements = geometry(points)
                    records.append(
                        {
                            "study": kind,
                            "scenario": scenario["id"],
                            "planner": run["plannerId"],
                            "domain": domain,
                            "durationS": duration,
                            **measurements,
                        }
                    )
    groups = defaultdict(list)
    for record in records:
        groups[(record["study"], record["planner"], record["domain"])].append(record)
    summaries = []
    for (study, planner, domain), rows in groups.items():
        summary = {"study": study, "planner": planner, "domain": domain, "runs": len(rows)}
        for key in (
            "lengthM",
            "excessPct",
            "totalTurnDeg",
            "maxTurnDeg",
            "turnsOver30Deg",
            "verticalTravelM",
            "durationS",
        ):
            values = [row[key] for row in rows if row[key] is not None]
            summary[f"median{key[0].upper()}{key[1:]}"] = (
                round(statistics.median(values), 3) if values else None
            )
        summaries.append(summary)
    return {
        "scope": "independent-geometry-observations-not-algorithm-ranking",
        "summaries": summaries,
        "records": records,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--public-dir", type=Path, default=Path(__file__).resolve().parents[1] / "web" / "public"
    )
    arguments = parser.parse_args()
    print(json.dumps(report(arguments.public_dir), separators=(",", ":")))
