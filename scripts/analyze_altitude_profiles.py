"""Independently inspect altitude preservation across the three Manhattan demo studies.

Static normalized 3D arc-length alignment is an observation, not a preservation certificate:
changing XY changes this parameterization. Static certificates instead use the exported original
progress parameter and also compare turning heights, bounds, and total vertical travel.
Predictive geometry has absolute timestamps,
so a union-of-knots comparison exactly measures the piecewise-linear difference in z(t).
Retiming changes that clock; geometry/execution comparison therefore checks positions rather
than asserting absolute-time altitude equality. All numbers are extracted from serialized data.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import statistics
from itertools import pairwise
from pathlib import Path

_EPSILON = 1e-6


def retained_geometry_knots(geometry: list[dict], execution: list[dict]) -> bool:
    """All original knots in order, permitting only additional stationary copies."""
    if (
        not geometry
        or not execution
        or math.dist(geometry[0]["position"], execution[0]["position"]) > _EPSILON
    ):
        return False
    cursor = 0
    for point in geometry[1:]:
        while (
            cursor + 1 < len(execution)
            and math.dist(execution[cursor + 1]["position"], point["position"]) > _EPSILON
        ):
            if (
                math.dist(execution[cursor + 1]["position"], execution[cursor]["position"])
                > _EPSILON
            ):
                return False
            cursor += 1
        cursor += 1
        if cursor >= len(execution):
            return False
    return cursor == len(execution) - 1


def altitude(points: list[list[float]]) -> dict[str, object]:
    heights = [float(point[2]) for point in points]
    if not heights:
        return {"minM": None, "maxM": None, "verticalTravelM": None, "criticalHeightsM": []}
    distinct = [heights[0]]
    for height in heights[1:]:
        if abs(height - distinct[-1]) > _EPSILON:
            distinct.append(height)
    critical = distinct[:1]
    for previous, current, following in zip(distinct, distinct[1:], distinct[2:], strict=False):
        if (current - previous) * (following - current) < 0:
            critical.append(current)
    if len(distinct) > 1:
        critical.append(distinct[-1])
    return {
        "minM": min(heights),
        "maxM": max(heights),
        "verticalTravelM": math.fsum(abs(b - a) for a, b in pairwise(heights)),
        "criticalHeightsM": critical,
    }


def _height_at(knots: list[tuple[float, float]], value: float) -> float:
    index = min(max(bisect.bisect_right([item[0] for item in knots], value) - 1, 0), len(knots) - 1)
    if index == len(knots) - 1:
        return knots[index][1]
    left, right = knots[index], knots[index + 1]
    span = right[0] - left[0]
    fraction = 0.0 if span <= 1e-12 else (value - left[0]) / span
    return left[1] + fraction * (right[1] - left[1])


def max_height_difference(
    left: list[tuple[float, float]], right: list[tuple[float, float]]
) -> float:
    """Exact maximum for two piecewise-linear profiles sharing the same parameter domain."""

    if abs(left[0][0] - right[0][0]) > _EPSILON or abs(left[-1][0] - right[-1][0]) > _EPSILON:
        raise ValueError("height profiles must share parameter endpoints")
    knots = sorted({item[0] for item in left + right})
    return max(abs(_height_at(left, value) - _height_at(right, value)) for value in knots)


def _arc_profile(points: list[list[float]]) -> list[tuple[float, float]]:
    cumulative = [0.0]
    for previous, following in pairwise(points):
        cumulative.append(cumulative[-1] + math.dist(previous, following))
    if cumulative[-1] <= 1e-12:
        return [(0.0, points[0][2]), (1.0, points[-1][2])]
    return [
        (value / cumulative[-1], point[2]) for value, point in zip(cumulative, points, strict=True)
    ]


def _close_sequence(left: list[float], right: list[float]) -> bool:
    return len(left) == len(right) and all(
        abs(a - b) <= _EPSILON for a, b in zip(left, right, strict=True)
    )


def _invariants(raw: dict[str, object], output: dict[str, object]) -> dict[str, object]:
    return {
        "criticalHeightsPreserved": _close_sequence(
            raw["criticalHeightsM"], output["criticalHeightsM"]
        ),
        "altitudeBoundsPreserved": abs(raw["minM"] - output["minM"]) <= _EPSILON
        and abs(raw["maxM"] - output["maxM"]) <= _EPSILON,
        "noVerticalTravelIncrease": output["verticalTravelM"] <= raw["verticalTravelM"] + _EPSILON,
        "verticalTravelDeltaM": output["verticalTravelM"] - raw["verticalTravelM"],
    }


def _fingerprint(value: object) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def report(public_dir: Path) -> dict[str, object]:
    records = []
    for study, filename in (
        ("static", "demo-data.json"),
        ("reactive", "dynamic-data.json"),
        ("predictive", "predictive-data.json"),
    ):
        bundle = json.loads((public_dir / filename).read_text())
        for scenario in bundle["scenarios"]:
            for run in scenario.get("results", scenario.get("runs", [])):
                record = {"study": study, "scenario": scenario["id"], "planner": run["plannerId"]}
                if study == "static":
                    raw_points, output_points = run["paths"]["raw"], run["paths"]["smoothed"]
                    if not raw_points or not output_points:
                        record["scope"] = "unavailable-empty-raw-or-geometry-path"
                        records.append(record)
                        continue
                    record.update(
                        rawPathSHA256=_fingerprint(raw_points),
                        raw=altitude(raw_points),
                        geometry=altitude(output_points),
                        arcAlignmentObservationMaxZDifferenceM=max_height_difference(
                            _arc_profile(raw_points), _arc_profile(output_points)
                        ),
                    )
                    record.update(_invariants(record["raw"], record["geometry"]))
                    explicit_progress = run.get("smoothing", {}).get("altitudeProfileProgress")
                    if explicit_progress is not None:
                        if len(explicit_progress) != len(output_points):
                            raise ValueError("static altitude progress must match output points")
                        explicit_difference = max_height_difference(
                            _arc_profile(raw_points),
                            [
                                (value, point[2])
                                for value, point in zip(
                                    explicit_progress, output_points, strict=True
                                )
                            ],
                        )
                        record["explicitProgressMaxZDifferenceM"] = explicit_difference
                        record["explicitProgressAltitudePreserved"] = (
                            explicit_difference <= _EPSILON
                        )
                elif study == "predictive":
                    raw_timed, geometry_timed = run["rawTimedPath"], run["geometryTimedPath"]
                    execution_timed = run["executionTimedPath"]
                    if not raw_timed or not geometry_timed:
                        record["scope"] = "unavailable-empty-raw-or-geometry-timed-path"
                        records.append(record)
                        continue
                    raw_points = [item["position"] for item in raw_timed]
                    output_points = [item["position"] for item in geometry_timed]
                    max_difference = max_height_difference(
                        [(item["timeS"], item["position"][2]) for item in raw_timed],
                        [(item["timeS"], item["position"][2]) for item in geometry_timed],
                    )
                    record.update(
                        rawPathSHA256=_fingerprint(raw_timed),
                        raw=altitude(raw_points),
                        geometry=altitude(output_points),
                        absoluteTimeMaxZDifferenceM=max_difference,
                        absoluteTimeAltitudePreserved=max_difference <= _EPSILON,
                        executionGeometryKnotsPreserved=retained_geometry_knots(
                            geometry_timed, execution_timed
                        )
                        if execution_timed
                        else None,
                        executionGeometryPositionSequencePreserved=(
                            len(geometry_timed) == len(execution_timed)
                            and all(
                                math.dist(left["position"], right["position"]) <= _EPSILON
                                for left, right in zip(
                                    geometry_timed, execution_timed, strict=False
                                )
                            )
                        )
                        if execution_timed
                        else None,
                        execution=altitude([item["position"] for item in execution_timed])
                        if execution_timed
                        else None,
                    )
                    record.update(_invariants(record["raw"], record["geometry"]))
                else:
                    record["execution"] = altitude(run["frames"][-1]["executedPath"])
                    record["scope"] = "execution-only-no-serialized-raw-altitude-reference"
                records.append(record)
    summaries = []
    for study in ("static", "reactive", "predictive"):
        rows = [row for row in records if row["study"] == study]
        summary = {"study": study, "runs": len(rows)}
        if study != "reactive":
            rows = [row for row in rows if "raw" in row]
            summary["availableRuns"] = len(rows)
            observation = (
                "arcAlignmentObservationMaxZDifferenceM"
                if study == "static"
                else "absoluteTimeMaxZDifferenceM"
            )
            summary.update(
                maxZDifferenceScope="normalized-3D-arc-observation-only"
                if study == "static"
                else "exact-piecewise-linear-absolute-time",
                medianMaxZDifferenceM=statistics.median(row[observation] for row in rows)
                if rows
                else None,
                maxZDifferenceM=max(row[observation] for row in rows) if rows else None,
                medianRawVerticalTravelM=statistics.median(
                    row["raw"]["verticalTravelM"] for row in rows
                )
                if rows
                else None,
                medianGeometryVerticalTravelM=statistics.median(
                    row["geometry"]["verticalTravelM"] for row in rows
                )
                if rows
                else None,
                criticalHeightsPreservedRuns=sum(row["criticalHeightsPreserved"] for row in rows),
                altitudeBoundsPreservedRuns=sum(row["altitudeBoundsPreserved"] for row in rows),
                noVerticalTravelIncreaseRuns=sum(row["noVerticalTravelIncrease"] for row in rows),
            )
        if study == "predictive":
            summary.update(
                absoluteTimeAltitudePreservedRuns=sum(
                    row["absoluteTimeAltitudePreserved"] for row in rows
                ),
                executionGeometryPositionSequencePreservedRuns=sum(
                    row["executionGeometryPositionSequencePreserved"] is True for row in rows
                ),
                executionGeometryKnotsPreservedRuns=sum(
                    row["executionGeometryKnotsPreserved"] is True for row in rows
                ),
            )
        if study == "static":
            summary["explicitProgressAltitudePreservedRuns"] = sum(
                row.get("explicitProgressAltitudePreserved", False) for row in rows
            )
        summaries.append(summary)
    return {
        "scope": "serialized-altitude-profiles-not-a-continuous-flight-dynamics-certificate",
        "toleranceM": _EPSILON,
        "summaries": summaries,
        "records": records,
    }


def compare_with_baseline(
    analysis: dict[str, object], baseline: dict[str, object]
) -> list[dict[str, object]]:
    previous = {(row["study"], row["scenario"], row["planner"]): row for row in baseline["records"]}
    comparisons = []
    for row in analysis["records"]:
        key = (row["study"], row["scenario"], row["planner"])
        before = previous.get(key)
        if before is None:
            continue
        comparison = {"study": key[0], "scenario": key[1], "planner": key[2]}
        if "rawPathSHA256" in before and "rawPathSHA256" in row:
            comparison["rawReferenceUnchanged"] = before["rawPathSHA256"] == row["rawPathSHA256"]
        for metric in ("arcAlignmentObservationMaxZDifferenceM", "absoluteTimeMaxZDifferenceM"):
            if metric in before and metric in row:
                comparison[metric] = {"before": before[metric], "after": row[metric]}
        for domain in ("raw", "geometry", "execution"):
            if before.get(domain) and row.get(domain):
                comparison[f"{domain}VerticalTravelM"] = {
                    "before": before[domain]["verticalTravelM"],
                    "after": row[domain]["verticalTravelM"],
                }
        comparisons.append(comparison)
    return comparisons


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--public-dir", type=Path, default=Path(__file__).resolve().parents[1] / "web" / "public"
    )
    parser.add_argument("--output", type=Path, help="Optional generated JSON analysis artifact.")
    parser.add_argument("--baseline", type=Path, help="Compare against a prior generated report.")
    arguments = parser.parse_args()
    analysis = report(arguments.public_dir)
    if arguments.baseline is not None:
        analysis["baselineComparison"] = compare_with_baseline(
            analysis, json.loads(arguments.baseline.read_text())
        )
    if arguments.output is not None:
        arguments.output.write_text(json.dumps(analysis, indent=2) + "\n")
    print(json.dumps(analysis, separators=(",", ":")))
