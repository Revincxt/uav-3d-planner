import { describe, expect, it, vi } from "vitest";

import {
  buildPredictiveComparisonRows,
  loadPredictiveBundle,
  stationaryDuration,
  validatePredictiveBundle,
} from "../src/predictive-data";
import type {
  PredictiveBundleV3,
  PredictiveDiscreteKinematicDiagnostics,
  PredictiveExecutionEnvelope,
  PredictiveFrame,
  PredictiveRun,
  PredictiveRunMetrics,
  TimedWaypoint,
  Vec3,
  WaitInterval,
} from "../src/predictive-schema";

const start: Vec3 = [0, 0, 10];
const goal: Vec3 = [30, 40, 10];
const directDistance = 50;

describe("origin-independent wait classification", () => {
  it.each([-10_000, 0, 1500, 1_000_000])("does not label sub-millimetre motion as a wait at origin %s", origin => {
    const path: TimedWaypoint[] = [
      { timeS: 0, position: [origin, origin, 180] },
      { timeS: 0.0000599279, position: [origin + 0.0006, origin + 0.0006, 180] },
    ];
    expect(stationaryDuration(path)).toBe(0);
  });
  it("counts actual waits but not short horizontal or vertical motion", () => {
    const path: TimedWaypoint[] = [
      { timeS: 0, position: [1500, 3000, 180] },
      { timeS: 2, position: [1500, 3000, 180] },
      { timeS: 3, position: [1500.0001, 3000, 180] },
      { timeS: 4, position: [1500.0001, 3000, 180.0001] },
    ];
    expect(stationaryDuration(path)).toBe(2);
  });
  it("uses the same absolute numerical tolerance as the Python planner", () => {
    expect(stationaryDuration([
      { timeS: 0, position: [0, 0, 0] },
      { timeS: 2, position: [5e-10, 0, 0] },
    ])).toBe(2);
  });
});

const executionEnvelope: PredictiveExecutionEnvelope = {
  model: "discrete-segment-average-envelope-v1",
  maxSpeedMps: 8,
  maxAbsClimbRateMps: 3,
  maxDiscreteAccelerationProxyMps2: 4,
  reversalThresholdDeg: 150,
  allowReversals: false,
  maxExecutionTimeS: 90,
  continuousDynamicsCertified: false,
};

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

function predictiveGeometryPath(): TimedWaypoint[] {
  return [
    { timeS: 0, position: [...start] },
    { timeS: 2, position: [6, 8, 10] },
    { timeS: 4, position: [12, 16, 10] },
    { timeS: 6, position: [18, 24, 10] },
    { timeS: 8, position: [24, 32, 10] },
    { timeS: 10, position: [...goal] },
  ];
}

function executionPath(path: TimedWaypoint[], predictive: boolean): TimedWaypoint[] {
  const times = predictive ? [0, 2.5, 5, 7.5, 10, 12.5] : [0, 3, 5, 8, 13];
  return path.map((waypoint, index) => ({
    timeS: times[index]!,
    position: [...waypoint.position],
  }));
}

function waits(path: TimedWaypoint[], predictive: boolean): WaitInterval[] {
  if (predictive) return [];
  return [
    {
      startTimeS: path[1]!.timeS,
      endTimeS: path[2]!.timeS,
      position: [...path[2]!.position],
      reason: "reactive safety hold",
    },
  ];
}

function frames(path: TimedWaypoint[], predictive: boolean): PredictiveFrame[] {
  return path.map((waypoint, index) => {
    const event: PredictiveFrame["event"] =
      index === 0
        ? predictive
          ? { kind: "prediction-update", label: "Initial forecast", subjectId: "traffic-1" }
          : { kind: "replan", label: "Initial reactive plan", subjectId: null }
        : index === path.length - 1
          ? { kind: "goal-reached", label: "Goal reached", subjectId: null }
          : !predictive && index === 1
            ? { kind: "wait-start", label: "Hold for crossing traffic", subjectId: "traffic-1" }
            : !predictive && index === 2
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

function pathLength(path: TimedWaypoint[]): number {
  let total = 0;
  for (let index = 1; index < path.length; index += 1) {
    total += Math.hypot(
      path[index]!.position[0] - path[index - 1]!.position[0],
      path[index]!.position[1] - path[index - 1]!.position[1],
      path[index]!.position[2] - path[index - 1]!.position[2],
    );
  }
  return total;
}

function metricsFor(
  path: TimedWaypoint[],
  plannerId: string,
  waitTimeS: number,
): PredictiveRunMetrics {
  const executedPathLengthM = pathLength(path);
  return {
    success: true,
    failureReason: null,
    arrivalTimeS: path.at(-1)!.timeS,
    travelTimeS: path.at(-1)!.timeS - waitTimeS,
    waitTimeS,
    executedPathLengthM,
    directDistanceM: directDistance,
    pathExcessPct: (executedPathLengthM / directDistance - 1) * 100,
    replans: 1,
    expandedStates: plannerId === "space-time-astar-4d" ? 1800 : 900,
    workUnit:
      plannerId === "space-time-astar-4d" ? "expanded-spacetime-states" : "expanded-nodes",
    minimumSeparationM: null,
    minimumSeparationWitness: null,
    safetyViolations: 0,
  };
}

function kinematicDiagnostics(
  segmentCount: number,
  movementSegmentCount: number,
): PredictiveDiscreteKinematicDiagnostics {
  return {
    status: "discrete-diagnostic-only",
    continuousDynamicsCertified: false,
    segmentCount,
    movementSegmentCount,
    reversalCount: 0,
    reversalThresholdDeg: 150,
    maxSpeedMps: 6,
    maxDiscreteVelocityChangeMps: 5,
    maxDiscreteAccelerationProxyMps2: 3,
    maxAbsClimbRateMps: 0,
  };
}

function run(plannerId: string, predictive: boolean): PredictiveRun {
  const rawTimedPath = predictive ? predictiveRawPath() : reactiveTimedPath();
  const geometryTimedPath = predictive ? predictiveGeometryPath() : reactiveTimedPath();
  const executionTimedPath = executionPath(geometryTimedPath, predictive);
  const geometryWaitIntervals = waits(geometryTimedPath, predictive);
  const executionWaitIntervals = waits(executionTimedPath, predictive);
  const plannerWaitTime = predictive ? 0 : 2;
  const executionQualification = kinematicDiagnostics(
    executionTimedPath.length - 1,
    executionTimedPath.length - 1 - (predictive ? 0 : 1),
  );
  return {
    runId: `sha256:${predictive ? "2".repeat(64) : "1".repeat(64)}`,
    plannerId,
    predictive,
    status: "success",
    failureReason: null,
    parameters: { resolutionM: 4, predictionHorizonS: predictive ? 12 : 0 },
    rawTimedPath,
    geometryTimedPath,
    executionTimedPath,
    smoothing: {
      method: predictive ? "certified-rounded-corners-v1" : "certified-raw-fallback",
      applied: predictive,
      certified: true,
      collisionCertified: true,
      collisionCertificationScope: "dense-piecewise-linear-space-time-path",
      rawWaypointCount: rawTimedPath.length,
      outputWaypointCount: geometryTimedPath.length,
      roundedCornerCount: predictive ? 2 : 0,
      requestedTurnRadiusM: 4,
      appliedTurnRadiusM: predictive ? 4 : null,
      sampleSpacingM: 1,
      maxTurnAngleBeforeDeg: predictive ? 90 : 0,
      maxTurnAngleAfterDeg: 0,
      kinematicDiagnostics: {
        status: "discrete-diagnostic-only",
        continuousDynamicsCertified: false,
        raw: kinematicDiagnostics(
          rawTimedPath.length - 1,
          rawTimedPath.length - 1 - (predictive ? 0 : 1),
        ),
        output: kinematicDiagnostics(
          geometryTimedPath.length - 1,
          geometryTimedPath.length - 1 - (predictive ? 0 : 1),
        ),
      },
      execution: {
        status: "qualified",
        qualified: true,
        collisionCertified: true,
        collisionCertificationScope: "dense-piecewise-linear-space-time-path",
        continuousDynamicsCertified: false,
        envelope: { ...executionEnvelope },
        qualification: {
          status: "qualified",
          qualified: true,
          continuousDynamicsCertified: false,
          diagnostics: executionQualification,
          boundaryAwareMaxDiscreteAccelerationProxyMps2: 3,
          violations: [],
        },
        timingIterations: 2,
        originalDurationS: geometryTimedPath.at(-1)!.timeS,
        candidateDurationS: executionTimedPath.at(-1)!.timeS,
        addedDurationS:
          executionTimedPath.at(-1)!.timeS - geometryTimedPath.at(-1)!.timeS,
      },
    },
    geometryWaitIntervals,
    executionWaitIntervals,
    plannerMetrics: metricsFor(rawTimedPath, plannerId, plannerWaitTime),
    geometryMetrics: metricsFor(geometryTimedPath, plannerId, plannerWaitTime),
    executionMetrics: metricsFor(executionTimedPath, plannerId, plannerWaitTime),
    geometryFrames: frames(geometryTimedPath, predictive),
    executionFrames: frames(executionTimedPath, predictive),
  };
}

function fixture(): PredictiveBundleV3 {
  return {
    schemaVersion: 3,
    generatedAt: "2026-08-08T08:00:00Z",
    sourceCommit: "0123456789abcdef0123456789abcdef01234567",
    verificationStatus: "PREDICTIVE_DEMO_NON_CONFIRMATORY",
    protocol: {
      id: "predictive-space-time-v4",
      timeStepS: 1,
      cruiseSpeedMps: 8,
      maxTimeS: 90,
      resolutionM: 4,
      timeResolutionS: 0.5,
      planningHorizonS: 90,
      predictionHorizonS: 90,
      reactiveMaxWorkPerReplan: 240_000,
      predictiveMaxExpandedStatesPerMission: 240_000,
      trajectoryPostprocessor: "certified-fillet-plus-discrete-execution-envelope-v2",
      executionEnvelope: { ...executionEnvelope },
      continuousDynamicsCertified: false,
      metricDomains: {
        plannerMetrics: "raw planner or simulator output only",
        geometryMetrics: "common collision-certified geometric post-processing",
        executionMetrics: "optional discrete-envelope-qualified retimed candidate",
      },
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
              { timeS: 90, position: [50, 100, 10] },
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

function addValidWitness(metrics: PredictiveRunMetrics): void {
  const center: Vec3 = [50, 10, 10];
  const centerDistance = Math.hypot(50, 10);
  const obstaclePosition: Vec3 = [
    center[0] - (2 * 50) / centerDistance,
    center[1] - (2 * 10) / centerDistance,
    center[2],
  ];
  const separationM = centerDistance - 2 - 0.5;
  metrics.minimumSeparationM = separationM;
  metrics.minimumSeparationWitness = {
    separationM,
    timeS: 0,
    vehiclePosition: [...start],
    obstacleId: "traffic-1",
    obstacleKind: "moving-sphere",
    obstaclePosition,
    declaredSafetyMarginM: 0.5,
    method: "exact-relative-linear-motion",
    exact: true,
  };
}

describe("predictive bundle v3", () => {
  it("accepts distinct planner, geometry, and qualified execution evidence", () => {
    const parsed = validatePredictiveBundle(fixture());
    expect(parsed.schemaVersion).toBe(3);
    expect(parsed.protocol.id).toBe("predictive-space-time-v4");
    expect(parsed.scenarios[0]!.runs[0]!.geometryFrames[0]).not.toHaveProperty(
      "executedPath",
    );
    expect(parsed.scenarios[0]!.runs[1]!.smoothing.execution.status).toBe("qualified");
    expect(parsed.scenarios[0]!.runs[1]!.executionTimedPath).not.toBeNull();
    expect(parsed.protocol.continuousDynamicsCertified).toBe(false);
  });

  it("loads and validates predictive-data.json through an injected fetch implementation", async () => {
    const fetcher = vi.fn(async () => new Response(JSON.stringify(fixture()), { status: 200 }));
    const parsed = await loadPredictiveBundle(fetcher as typeof fetch);
    expect(parsed.verificationStatus).toBe("PREDICTIVE_DEMO_NON_CONFIRMATORY");
    expect(fetcher).toHaveBeenCalledWith(expect.stringContaining("predictive-data.json"));
  });

  it("builds comparison rows from planner metrics rather than post-processed evidence", () => {
    const input = fixture();
    input.scenarios[0]!.runs[1]!.plannerMetrics.expandedStates = 4321;
    input.scenarios[0]!.runs[1]!.geometryMetrics.expandedStates = 4321;
    input.scenarios[0]!.runs[1]!.executionMetrics!.expandedStates = 4321;
    const value = validatePredictiveBundle(input);
    const rows = buildPredictiveComparisonRows(value, value.scenarios[0]!);
    expect(rows.map((row) => row.plannerId)).toEqual([
      "repeated-astar-3d",
      "space-time-astar-4d",
    ]);
    expect(rows[1]!.expandedStates).toBe(4321);
    expect(rows[1]!.executedPathLengthM).toBe(70);
    expect(rows[1]!.smoothingApplied).toBe(true);
    expect(rows[1]!.executionQualified).toBe(true);
    expect(rows[1]!.executionStatus).toBe("qualified");
  });

  it("rejects schema/protocol drift and false continuous-dynamics claims", () => {
    const schema = fixture() as unknown as Record<string, unknown>;
    schema.schemaVersion = 2;
    expect(() => validatePredictiveBundle(schema)).toThrow(/schemaVersion must be 3/);

    const protocol = fixture();
    protocol.protocol.id = "predictive-space-time-v3";
    expect(() => validatePredictiveBundle(protocol)).toThrow(/predictive-space-time-v4/);

    const dynamics = fixture();
    (dynamics.protocol as unknown as Record<string, unknown>).continuousDynamicsCertified = true;
    expect(() => validatePredictiveBundle(dynamics)).toThrow(
      /continuousDynamicsCertified must remain false/,
    );
  });

  it("rejects execution envelopes that drift from the frozen v0.7 declaration", () => {
    const protocol = fixture();
    protocol.protocol.executionEnvelope.maxSpeedMps = 9;
    expect(() => validatePredictiveBundle(protocol)).toThrow(/frozen v0.7 execution envelope/);

    const runEnvelope = fixture();
    runEnvelope.scenarios[0]!.runs[0]!.smoothing.execution.envelope.maxExecutionTimeS = 91;
    expect(() => validatePredictiveBundle(runEnvelope)).toThrow(
      /frozen v0.7 execution envelope/,
    );
  });

  it("rejects non-monotonic paths and evidence-frame times", () => {
    const path = fixture();
    path.scenarios[0]!.runs[0]!.geometryTimedPath[2]!.timeS = 1;
    expect(() => validatePredictiveBundle(path)).toThrow(
      /geometryTimedPath must be strictly increasing/,
    );

    const frame = fixture();
    frame.scenarios[0]!.runs[0]!.geometryFrames[2]!.timeS = 2;
    frame.scenarios[0]!.runs[0]!.geometryFrames[2]!.movingSpheres = movingState(2);
    expect(() => validatePredictiveBundle(frame)).toThrow(/strictly increasing/);
  });

  it("rejects overlapping waits and metric domains paired with the wrong path", () => {
    const overlap = fixture();
    overlap.scenarios[0]!.runs[0]!.geometryWaitIntervals.push({
      startTimeS: 2.5,
      endTimeS: 3.5,
      position: [6, 8, 10],
      reason: "duplicate hold",
    });
    expect(() => validatePredictiveBundle(overlap)).toThrow(/sorted and non-overlapping/);

    const plannerMetric = fixture();
    plannerMetric.scenarios[0]!.runs[1]!.plannerMetrics.executedPathLengthM = 50;
    plannerMetric.scenarios[0]!.runs[1]!.plannerMetrics.pathExcessPct = 0;
    expect(() => validatePredictiveBundle(plannerMetric)).toThrow(
      /plannerMetrics path lengths disagree with its evidence path geometry/,
    );
  });

  it("requires execution paths, waits, metrics, and frames to be jointly present", () => {
    const missing = fixture();
    missing.scenarios[0]!.runs[0]!.executionMetrics = null;
    expect(() => validatePredictiveBundle(missing)).toThrow(
      /execution evidence fields must exist exactly/,
    );

    const failed = fixture();
    const target = failed.scenarios[0]!.runs[0]!;
    target.smoothing.execution.status = "reversal-not-allowed";
    target.smoothing.execution.qualified = false;
    target.smoothing.execution.collisionCertified = false;
    target.smoothing.execution.qualification!.status = "not-qualified";
    target.smoothing.execution.qualification!.qualified = false;
    target.smoothing.execution.qualification!.diagnostics.reversalCount = 1;
    target.smoothing.execution.qualification!.violations = ["reversal-not-allowed"];
    target.executionTimedPath = null;
    target.executionWaitIntervals = null;
    target.executionMetrics = null;
    target.executionFrames = null;
    expect(validatePredictiveBundle(failed).scenarios[0]!.runs[0]!.executionTimedPath).toBeNull();

    const collision = fixture();
    const collisionTarget = collision.scenarios[0]!.runs[1]!;
    collisionTarget.smoothing.execution.status = "dynamic-collision-after-retiming";
    collisionTarget.smoothing.execution.qualified = false;
    collisionTarget.smoothing.execution.collisionCertified = false;
    collisionTarget.executionTimedPath = null;
    collisionTarget.executionWaitIntervals = null;
    collisionTarget.executionMetrics = null;
    collisionTarget.executionFrames = null;
    expect(
      validatePredictiveBundle(collision).scenarios[0]!.runs[1]!.smoothing.execution.status,
    ).toBe("dynamic-collision-after-retiming");
  });

  it("rejects retiming that changes geometry, shortens a move, or changes a wait duration", () => {
    const geometry = fixture();
    geometry.scenarios[0]!.runs[1]!.executionTimedPath![1]!.position = [7, 8, 10];
    geometry.scenarios[0]!.runs[1]!.executionFrames![1]!.vehicle = [7, 8, 10];
    expect(() => validatePredictiveBundle(geometry)).toThrow(/waypoint sequence exactly/);

    const shorter = fixture();
    shorter.scenarios[0]!.runs[1]!.executionTimedPath![1]!.timeS = 1.5;
    shorter.scenarios[0]!.runs[1]!.executionFrames![1]!.timeS = 1.5;
    shorter.scenarios[0]!.runs[1]!.executionFrames![1]!.activeTemporaryZoneIds = [];
    shorter.scenarios[0]!.runs[1]!.executionFrames![1]!.movingSpheres = movingState(1.5);
    expect(() => validatePredictiveBundle(shorter)).toThrow(/must not shorten any/);

    const wait = fixture();
    const reactive = wait.scenarios[0]!.runs[0]!;
    reactive.executionTimedPath![2]!.timeS = 5.5;
    reactive.executionTimedPath![3]!.timeS = 8.5;
    reactive.executionTimedPath![4]!.timeS = 13.5;
    reactive.executionWaitIntervals![0]!.endTimeS = 5.5;
    reactive.executionMetrics!.arrivalTimeS = 13.5;
    reactive.executionMetrics!.travelTimeS = 11;
    reactive.executionMetrics!.waitTimeS = 2.5;
    reactive.executionFrames = frames(reactive.executionTimedPath!, false);
    reactive.smoothing.execution.candidateDurationS = 13.5;
    reactive.smoothing.execution.addedDurationS = 3.5;
    expect(() => validatePredictiveBundle(wait)).toThrow(/wait-segment duration/);
  });

  it("rejects qualification and timing claims that contradict execution evidence", () => {
    const qualification = fixture();
    qualification.scenarios[0]!.runs[0]!.smoothing.execution.qualification!
      .boundaryAwareMaxDiscreteAccelerationProxyMps2 = 5;
    expect(() => validatePredictiveBundle(qualification)).toThrow(/claims qualification outside/);

    const duration = fixture();
    duration.scenarios[0]!.runs[0]!.smoothing.execution.candidateDurationS = 12;
    duration.scenarios[0]!.runs[0]!.smoothing.execution.addedDurationS = 2;
    expect(() => validatePredictiveBundle(duration)).toThrow(/durations disagree/);
  });

  it("rejects planner flags, work units, and planning counters that drift across domains", () => {
    const flag = fixture();
    flag.scenarios[0]!.runs[1]!.predictive = false;
    expect(() => validatePredictiveBundle(flag)).toThrow(/predictive disagrees/);

    const unit = fixture();
    unit.scenarios[0]!.runs[1]!.plannerMetrics.workUnit = "expanded-nodes";
    expect(() => validatePredictiveBundle(unit)).toThrow(/workUnit disagrees/);

    const accounting = fixture();
    accounting.scenarios[0]!.runs[0]!.geometryMetrics.expandedStates += 1;
    expect(() => validatePredictiveBundle(accounting)).toThrow(/preserve planner work accounting/);
  });

  it("rejects obstacle states or active-zone sets that contradict the shared schedule", () => {
    const moving = fixture();
    moving.scenarios[0]!.runs[0]!.geometryFrames[1]!.movingSpheres[0]!.position = [50, 13, 10];
    expect(() => validatePredictiveBundle(moving)).toThrow(/declared keyframes/);

    const active = fixture();
    active.scenarios[0]!.runs[0]!.geometryFrames[0]!.activeTemporaryZoneIds = ["popup-zone"];
    expect(() => validatePredictiveBundle(active)).toThrow(/half-open schedules/);
  });

  it("retains strict closest-approach witness validation in each metric domain", () => {
    const distance = fixture();
    addValidWitness(distance.scenarios[0]!.runs[0]!.geometryMetrics);
    distance.scenarios[0]!.runs[0]!.geometryMetrics.minimumSeparationWitness!.separationM -= 1;
    expect(() => validatePredictiveBundle(distance)).toThrow(/witness is inconsistent/);

    const obstacle = fixture();
    addValidWitness(obstacle.scenarios[0]!.runs[0]!.plannerMetrics);
    obstacle.scenarios[0]!.runs[0]!.plannerMetrics.minimumSeparationWitness!.obstacleId = "unknown";
    expect(() => validatePredictiveBundle(obstacle)).toThrow(/moving-sphere witness/);
  });

  it("rejects smoothing evidence, environment counts, and legacy quadratic frame paths", () => {
    const count = fixture();
    count.scenarios[0]!.runs[1]!.smoothing.outputWaypointCount = 5;
    expect(() => validatePredictiveBundle(count)).toThrow(/segment counts|waypoint counts/);

    const environment = fixture();
    environment.scenarios[0]!.environment.buildingCount = 2;
    expect(() => validatePredictiveBundle(environment)).toThrow(/environment counts disagree/);

    const legacy = fixture();
    const frame = legacy.scenarios[0]!.runs[0]!.geometryFrames[0] as unknown as Record<
      string,
      unknown
    >;
    frame.executedPath = [[...start]];
    expect(() => validatePredictiveBundle(legacy)).toThrow(/must not duplicate/);
  });
});
