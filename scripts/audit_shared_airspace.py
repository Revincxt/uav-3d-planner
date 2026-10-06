"""Recompute traffic-free controls without changing buildings or reservation windows."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from itertools import pairwise
from pathlib import Path

from export_manhattan_static_dynamic import PROTOCOL, mission_scenes, shared_dynamic_scenarios

from uav3d.dynamic_collision import spacetime_segment_is_free
from uav3d.geometry import almost_equal, lerp
from uav3d.manhattan_city import build_manhattan_city
from uav3d.manhattan_missions import simulate_mission_replanning
from uav3d.manhattan_predictive import (
    build_manhattan_missions,
    run_multistop_episode,
    study_runtime,
)
from uav3d.predictive import TimedPath, TimedWaypoint
from uav3d.replanning import REPLANNING_ALGORITHMS

ROOT = Path(__file__).resolve().parents[1]


def trace(frames):
    points = [(frames[0]["timeS"], frames[0]["vehicle"])]
    for left, right in pairwise(frames):
        portion = right["executedPath"][len(left["executedPath"]) - 1 :]
        lengths = [math.dist(a, b) for a, b in pairwise(portion)]
        total = math.fsum(lengths)
        if not total:
            points.append((right["timeS"], right["vehicle"]))
            continue
        distance = 0.0
        for endpoint, length in zip(portion[1:], lengths, strict=True):
            distance += length
            clock = left["timeS"] + distance / total * (right["timeS"] - left["timeS"])
            points.append((clock, endpoint))
    unique = []
    for t, p in points:
        if unique and abs(t - unique[-1][0]) < 1e-10:
            if not almost_equal(tuple(p), tuple(unique[-1][1])):
                raise ValueError("Different positions at the same instant")
            continue
        unique.append((t, p))
    return timed([{"timeS": t, "position": p} for t, p in unique])


def timed(records):
    return TimedPath(
        tuple(
            TimedWaypoint(
                w["timeS"],
                tuple(w["position"]),
                "start"
                if i == 0
                else (
                    "wait"
                    if almost_equal(tuple(records[i - 1]["position"]), tuple(w["position"]))
                    else "move"
                ),
            )
            for i, w in enumerate(records)
        )
    )


def position(path, clock):
    for a, b in pairwise(path.waypoints):
        if clock <= b.time_s:
            return lerp(a.position, b.position, max(0, (clock - a.time_s) / (b.time_s - a.time_s)))
    return path.goal


def evidence(scenario, baseline, actual):
    if not actual.is_safe(scenario):
        raise ValueError("Actual shared-world trajectory is unsafe")
    conflicts = sum(
        not spacetime_segment_is_free(scenario, a.position, b.position, a.time_s, b.time_s)
        for a, b in pairwise(baseline.waypoints)
    )
    duration = max(baseline.waypoints[-1].time_s, actual.waypoints[-1].time_s)
    deviations = []
    for i in range(math.ceil(duration / 2) + 1):
        a, b = position(baseline, i * 2), position(actual, i * 2)
        deviations.append(math.hypot(a[0] - b[0], a[1] - b[1]))
    return {
        "counterfactualUnsafeSegments": conflicts,
        "actualSharedWorldSafe": True,
        "maxHorizontalReplayDifferenceM": round(max(deviations), 3),
        "arrivalDifferenceS": round(actual.waypoints[-1].time_s - baseline.waypoints[-1].time_s, 3),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--public", type=Path, default=ROOT / "web/public")
    parser.add_argument(
        "--output", type=Path, default=ROOT / "data/manhattan/shared-airspace-audit.json"
    )
    args = parser.parse_args()
    static, dynamic, predictive = [
        json.loads((args.public / name).read_text())
        for name in ("demo-data.json", "dynamic-data.json", "predictive-data.json")
    ]
    for bundle, static_key in (
        (static, "noFlyZones"),
        (dynamic, "staticNoFlyZones"),
        (predictive, "staticNoFlyZones"),
    ):
        first = bundle["scenarios"][0]
        for case in bundle["scenarios"]:
            for key in ("bounds", "buildings", static_key, "temporaryNoFlyZones", "movingSpheres"):
                if case.get(key) != first.get(key):
                    raise ValueError("Mission queries contain different physical worlds")
            if case["mission"]["sharedWorld"] != first["mission"]["sharedWorld"]:
                raise ValueError("Shared world declarations differ")
    city, runtime = build_manhattan_city(), study_runtime()
    missions = mission_scenes(city)
    baselines = [case["results"][0]["paths"]["raw"] for case in static["scenarios"]]
    scenarios = shared_dynamic_scenarios(missions, baselines)
    options = dict(
        time_step=PROTOCOL["timeStepS"],
        replan_interval=PROTOCOL["replanIntervalS"],
        cruise_speed=PROTOCOL["cruiseSpeedMps"],
        max_time=PROTOCOL["maxTimeS"],
        resolution=PROTOCOL["resolutionM"],
        max_expansions=PROTOCOL["maxExpansions"],
        shortcut_paths=True,
        preserve_altitude=True,
        planning_guard_s=PROTOCOL["planningGuardS"],
        smooth_turns=True,
        turn_scale_m=60,
        curve_sample_spacing_m=2,
        allow_horizontal_escape=True,
    )
    rows = []
    for (mission, _), scenario, case in zip(missions, scenarios, dynamic["scenarios"], strict=True):
        for algorithm in REPLANNING_ALGORITHMS:
            control = simulate_mission_replanning(
                replace(scenario, moving_spheres=()), algorithm, mission["taskPoints"], **options
            )
            if not control.metrics.success:
                raise ValueError("Traffic-free dynamic control failed")
            baseline = runtime._reactive_timed_path(control)
            actual = trace(next(r for r in case["runs"] if r["plannerId"] == algorithm)["frames"])
            row = {
                "study": "dynamic",
                "mission": scenario.scenario_id,
                "planner": algorithm,
                **evidence(scenario, baseline, actual),
            }
            rows.append(row)
            print(json.dumps(row), flush=True)
    for scenario, case in zip(build_manhattan_missions(city), predictive["scenarios"], strict=True):
        algorithm = "space-time-astar-4d"
        control = run_multistop_episode(replace(scenario, moving_spheres=()), algorithm, runtime)
        actual = timed(next(r for r in case["runs"] if r["plannerId"] == algorithm)["rawTimedPath"])
        row = {
            "study": "predictive",
            "mission": scenario.scenario_id,
            "planner": algorithm,
            **evidence(scenario, control.raw_timed_path, actual),
        }
        rows.append(row)
        print(json.dumps(row), flush=True)
    for study in ("dynamic", "predictive"):
        cohort = [row for row in rows if row["study"] == study]
        if not any(row["counterfactualUnsafeSegments"] > 0 for row in cohort) or not any(
            row["maxHorizontalReplayDifferenceM"] > 1e-3 for row in cohort
        ):
            raise ValueError(f"Shared aircraft have no demonstrated planning effect in {study}")
    report = {
        "contract": "one-physical-world-one-clock-per-study",
        "missionCountPerWorld": len(static["scenarios"]),
        "control": "remove-aircraft-only-retain-city-fixed-zones-and-all-reservation-windows",
        "sourceCommit": dynamic["sourceCommit"],
        "jointMissionDeconfliction": False,
        "rows": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
