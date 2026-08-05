import { describe, expect, it, vi } from "vitest";

import {
  buildPredictiveComparisonRows,
  loadPredictiveBundle,
  validatePredictiveBundle,
} from "../src/predictive-data";
import type {
  PredictiveBundleV2,
  PredictiveFrame,
  PredictiveRun,
  TimedWaypoint,
  Vec3,
} from "../src/predictive-schema";

const start: Vec3 = [0, 0, 10];
const goal: Vec3 = [30, 40, 10];
const directDistance = 50;

function movingState(timeS: number): PredictiveFrame["movingSpheres"] {
  return [{ id: "traffic-1", position: [50, 10 + timeS, 10], radiusM: 2 }];
}

function reactiveTimedPath(): TimedWaypoint[] {
  return [
    { timeS: 0, position: [...start] },
    { timeS: 2, position: [6, 8, 10] },
    { timeS: 4, position: [6, 8, 10] },
    { timeS: 6, position: [14, 18.666666666666668, 10] },
    { timeS: 10, position: [...goal] },
  ];
}

function predictiveRawPath(): TimedWaypoint[] {
  return [
    { timeS: 0, position: [...start] },
    { timeS: 4, position: [0, 20, 10] },
    { timeS: 10, position: [30, 20, 10] },
    { timeS: 14, position: [...goal] },
  ];
}

function predictiveTimedPath(): TimedWaypoint[] {
  return [
    { timeS: 0, position: [...start] },
    { timeS: 2, position: [6, 8, 10] },
    { timeS: 4, position: [12, 16, 10] },
    { timeS: 6, position: [18, 24, 10] },
    { timeS: 8, position: [24, 32, 10] },
    { timeS: 10, position: [...goal] },
  ];
}

function frames(waypoints: TimedWaypoint[], predictive: boolean): PredictiveFrame[] {
  return waypoints.map((waypoint, index) => {
    const event: PredictiveFrame["event"] =
      index === 0
        ? predictive
          ? { kind: "prediction-update", label: "Initial forecast", subjectId: "traffic-1" }
          : { kind: "replan", label: "Initial reactive plan", subjectId: null }
        : index === waypoints.length - 1
          ? { kind: "goal-reached", label: "Goal reached", subjectId: null }
          : !predictive && waypoint.timeS === 2
            ? { kind: "wait-start", label: "Hold for crossing traffic", subjectId: "traffic-1" }
            : !predictive && waypoint.timeS === 4
              ? { kind: "wait-end", label: "Resume flight", subjectId: "traffic-1" }
              : null;
    return {
      timeS: waypoint.timeS,
      vehicle: [...waypoint.position],
      activeTemporaryZoneIds:
        waypoint.timeS >= 2 && waypoint.timeS < 6 ? ["popup-zone"] : [],
      movingSpheres: movingState(waypoint.timeS),
      event,
    };
  });
}

function run(plannerId: string, predictive: boolean): PredictiveRun {
  const rawTimedPath = predictive ? predictiveRawPath() : reactiveTimedPath();
  const timedPath = predictive ? predictiveTimedPath() : reactiveTimedPath();
  return {
    runId: `sha256:${predictive ? "2".repeat(64) : "1".repeat(64)}`,
    plannerId,
    predictive,
    status: "success",
    failureReason: null,
    parameters: { resolutionM: 4, predictionHorizonS: predictive ? 12 : 0 },
    rawTimedPath,
    timedPath,
    smoothing: predictive
      ? {
          method: "certified-rounded-corners-v1",
          applied: true,
          certified: true,
          rawWaypointCount: rawTimedPath.length,
          outputWaypointCount: timedPath.length,
          roundedCornerCount: 2,
          requestedTurnRadiusM: 4,
          appliedTurnRadiusM: 4,
          sampleSpacingM: 1,
          maxTurnAngleBeforeDeg: 90,
          maxTurnAngleAfterDeg: 0,
        }
      : {
          method: "certified-raw-fallback",
          applied: false,
          certified: true,
          rawWaypointCount: rawTimedPath.length,
          outputWaypointCount: timedPath.length,
          roundedCornerCount: 0,
          requestedTurnRadiusM: 4,
          appliedTurnRadiusM: null,
          sampleSpacingM: 1,
          maxTurnAngleBeforeDeg: 0,
          maxTurnAngleAfterDeg: 0,
        },
    waitIntervals: predictive
      ? []
      : [
          {
            startTimeS: 2,
            endTimeS: 4,
            position: [6, 8, 10],
            reason: "moving obstacle crossing",
          },
        ],
    metrics: {
      success: true,
      failureReason: null,
      arrivalTimeS: 10,
      travelTimeS: predictive ? 10 : 8,
      waitTimeS: predictive ? 0 : 2,
      executedPathLengthM: 50,
      directDistanceM: directDistance,
      pathExcessPct: 0,
      replans: 1,
      expandedStates: predictive ? 1800 : 900,
      workUnit: predictive ? "expanded-spacetime-states" : "expanded-nodes",
      minimumSeparationM: predictive ? 6.5 : 4.5,
      safetyViolations: 0,
    },
    frames: frames(timedPath, predictive),
  };
}

function fixture(): PredictiveBundleV2 {
  return {
    schemaVersion: 2,
    generatedAt: "2026-08-05T08:00:00Z",
    sourceCommit: "0123456789abcdef0123456789abcdef01234567",
    verificationStatus: "PREDICTIVE_DEMO_NON_CONFIRMATORY",
    protocol: {
      id: "predictive-space-time-v2",
      timeStepS: 1,
      cruiseSpeedMps: 8,
      maxTimeS: 120,
      resolutionM: 4,
      timeResolutionS: 1,
      planningHorizonS: 60,
      predictionHorizonS: 12,
      reactiveMaxWorkPerReplan: 120_000,
      predictiveMaxExpandedStatesPerMission: 120_000,
      trajectoryPostprocessor: "certified-rounded-corners-v1",
    },
    planners: [
      { id: "repeated-astar-3d", label: "Repeated 3D A*", predictive: false },
      { id: "space-time-astar-4d", label: "Space–time A*", predictive: true },
    ],
    scenarios: [
      {
        id: "crossing-traffic",
        label: "Crossing traffic",
        description: "A moving obstacle crosses the nominal route.",
        fingerprint: `sha256:${"a".repeat(64)}`,
        cohort: "demo",
        bounds: { min: [0, 0, 0], max: [100, 100, 50] },
        start: [...start],
        goal: [...goal],
        constraints: { vehicleRadiusM: 0.5, safetyMarginM: 0.5 },
        environment: {
          district: "Mixed-use research district",
          streetPattern: "Staggered urban grid",
          buildingCount: 1,
          hazardCount: 3,
        },
        buildings: [{ id: "tower", min: [60, 60, 0], max: [70, 70, 30] }],
        staticNoFlyZones: [
          { id: "static-zone", center: [80, 20], radiusM: 5, zMinM: 0, zMaxM: 20 },
        ],
        temporaryNoFlyZones: [
          {
            id: "popup-zone",
            center: [20, 20],
            radiusM: 4,
            zMinM: 0,
            zMaxM: 30,
            activeFromS: 2,
            activeUntilS: 6,
          },
        ],
        movingSpheres: [
          {
            id: "traffic-1",
            radiusM: 2,
            keyframes: [
              { timeS: 0, position: [50, 10, 10] },
              { timeS: 10, position: [50, 20, 10] },
            ],
          },
        ],
        runs: [run("repeated-astar-3d", false), run("space-time-astar-4d", true)],
      },
    ],
    downloads: {
      recordsCsv: {
        path: "predictive-records.csv",
        sha256: `sha256:${"b".repeat(64)}`,
        bytes: 120,
      },
      scenarioManifest: {
        path: "predictive-scenario-manifest.json",
        sha256: `sha256:${"c".repeat(64)}`,
        bytes: 240,
      },
    },
  };
}

describe("predictive bundle v2", () => {
  it("accepts O(N) event frames with raw and certified trajectories", () => {
    const parsed = validatePredictiveBundle(fixture());
    expect(parsed.schemaVersion).toBe(2);
    expect(parsed.scenarios[0]!.runs[0]!.frames[0]).not.toHaveProperty("executedPath");
    expect(parsed.scenarios[0]!.runs[1]!.smoothing.certified).toBe(true);
    expect(parsed.scenarios[0]!.environment.hazardCount).toBe(3);
  });

  it("loads and validates predictive-data.json through an injected fetch implementation", async () => {
    const fetcher = vi.fn(async () => new Response(JSON.stringify(fixture()), { status: 200 }));
    const parsed = await loadPredictiveBundle(fetcher as typeof fetch);
    expect(parsed.verificationStatus).toBe("PREDICTIVE_DEMO_NON_CONFIRMATORY");
    expect(fetcher).toHaveBeenCalledWith(expect.stringContaining("predictive-data.json"));
  });

  it("builds comparison rows in declared order with post-processing evidence", () => {
    const value = validatePredictiveBundle(fixture());
    const rows = buildPredictiveComparisonRows(value, value.scenarios[0]!);
    expect(rows.map((row) => row.plannerId)).toEqual([
      "repeated-astar-3d",
      "space-time-astar-4d",
    ]);
    expect(rows.map((row) => row.workUnit)).toEqual([
      "expanded-nodes",
      "expanded-spacetime-states",
    ]);
    expect(rows[1]!.smoothingApplied).toBe(true);
    expect(rows[1]!.maxTurnAngleAfterDeg).toBe(0);
  });

  it("rejects non-monotonic timed paths and event-frame times", () => {
    const timed = fixture();
    timed.scenarios[0]!.runs[0]!.timedPath[2]!.timeS = 1;
    expect(() => validatePredictiveBundle(timed)).toThrow(/timedPath must be strictly increasing/);

    const frame = fixture();
    frame.scenarios[0]!.runs[0]!.frames[2]!.timeS = 2;
    frame.scenarios[0]!.runs[0]!.frames[2]!.movingSpheres[0]!.position = [50, 12, 10];
    expect(() => validatePredictiveBundle(frame)).toThrow(/strictly increasing/);
  });

  it("rejects overlapping waits and wait metrics that disagree with intervals", () => {
    const overlap = fixture();
    overlap.scenarios[0]!.runs[0]!.waitIntervals.push({
      startTimeS: 2.5,
      endTimeS: 3.5,
      position: [6, 8, 10],
      reason: "duplicate hold",
    });
    expect(() => validatePredictiveBundle(overlap)).toThrow(/sorted and non-overlapping/);

    const total = fixture();
    total.scenarios[0]!.runs[0]!.metrics.waitTimeS = 3;
    total.scenarios[0]!.runs[0]!.metrics.travelTimeS = 7;
    expect(() => validatePredictiveBundle(total)).toThrow(/waitTimeS disagrees/);
  });

  it("rejects planner flags and work units that disagree with declarations", () => {
    const flag = fixture();
    flag.scenarios[0]!.runs[1]!.predictive = false;
    expect(() => validatePredictiveBundle(flag)).toThrow(/predictive disagrees/);

    const unit = fixture();
    unit.scenarios[0]!.runs[1]!.metrics.workUnit = "expanded-nodes";
    expect(() => validatePredictiveBundle(unit)).toThrow(/workUnit disagrees/);
  });

  it("rejects obstacle states or active-zone sets that contradict the shared schedule", () => {
    const moving = fixture();
    moving.scenarios[0]!.runs[0]!.frames[1]!.movingSpheres[0]!.position = [50, 13, 10];
    expect(() => validatePredictiveBundle(moving)).toThrow(/declared keyframes/);

    const active = fixture();
    active.scenarios[0]!.runs[0]!.frames[0]!.activeTemporaryZoneIds = ["popup-zone"];
    expect(() => validatePredictiveBundle(active)).toThrow(/half-open schedules/);
  });

  it("rejects inconsistent path metrics and duplicate run identities", () => {
    const metric = fixture();
    metric.scenarios[0]!.runs[0]!.metrics.executedPathLengthM = 49;
    expect(() => validatePredictiveBundle(metric)).toThrow(/path lengths disagree/);

    const duplicate = fixture();
    duplicate.scenarios[0]!.runs[1]!.runId = duplicate.scenarios[0]!.runs[0]!.runId;
    expect(() => validatePredictiveBundle(duplicate)).toThrow(/runId values must be unique/);
  });

  it("rejects smoothing evidence that disagrees with exported trajectory geometry", () => {
    const count = fixture();
    count.scenarios[0]!.runs[1]!.smoothing.outputWaypointCount = 5;
    expect(() => validatePredictiveBundle(count)).toThrow(/waypoint counts disagree/);

    const uncertified = fixture();
    uncertified.scenarios[0]!.runs[1]!.smoothing.certified = false;
    expect(() => validatePredictiveBundle(uncertified)).toThrow(/applied output must be certified/);

    const worse = fixture();
    worse.scenarios[0]!.runs[1]!.smoothing.maxTurnAngleAfterDeg = 100;
    expect(() => validatePredictiveBundle(worse)).toThrow(/cannot increase/);
  });

  it("rejects environment counts and legacy quadratic frame paths", () => {
    const environment = fixture();
    environment.scenarios[0]!.environment.buildingCount = 2;
    expect(() => validatePredictiveBundle(environment)).toThrow(/environment counts disagree/);

    const legacy = fixture();
    const frame = legacy.scenarios[0]!.runs[0]!.frames[0] as unknown as Record<string, unknown>;
    frame.executedPath = [[...start]];
    expect(() => validatePredictiveBundle(legacy)).toThrow(/must not|executedPath/);
  });
});
