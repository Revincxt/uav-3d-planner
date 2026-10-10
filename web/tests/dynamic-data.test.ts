import { describe, expect, it, vi } from "vitest";

import {
  buildDynamicComparisonRows,
  loadDynamicBundle,
  validateDynamicBundle,
} from "../src/dynamic-data";
import type { DynamicBundleV1, DynamicFrame, DynamicPlannerId, DynamicRun } from "../src/dynamic-schema";

const start = [4, 5, 6] as [number, number, number];
const midpoint = [25, 20, 12] as [number, number, number];
const goal = [46, 45, 8] as [number, number, number];
const directDistance = Math.hypot(42, 40, 2);
const executedLength = Math.hypot(21, 15, 6) + Math.hypot(21, 25, -4);

const frames = (): DynamicFrame[] => [
  {
    timeS: 0,
    vehicle: [...start],
    path: [[...start], [...midpoint], [...goal]],
    executedPath: [[...start]],
    activeTemporaryZoneIds: [],
    movingSpheres: [{ id: "traffic-1", position: [12, 30, 12], radiusM: 2 }],
    event: { kind: "replan", label: "Initial plan", subjectId: null },
    replanned: true,
    replanReason: "initial",
    plannerSuccess: true,
    planningTimeMs: null,
    workUsed: 120,
    changedEdges: 0,
  },
  {
    timeS: 8,
    vehicle: [...goal],
    path: [[...goal]],
    executedPath: [[...start], [...midpoint], [...goal]],
    activeTemporaryZoneIds: ["popup-zone"],
    movingSpheres: [{ id: "traffic-1", position: [32, 30, 12], radiusM: 2 }],
    event: { kind: "goal-reached", label: "Goal reached", subjectId: null },
    replanned: false,
    replanReason: null,
    plannerSuccess: null,
    planningTimeMs: null,
    workUsed: 0,
    changedEdges: 0,
  },
];

const run = (plannerId: DynamicPlannerId): DynamicRun => ({
  runId: `sha256:${
    plannerId === "repeated-astar-3d"
      ? "1".repeat(64)
      : plannerId === "repeated-lazy-theta-star"
        ? "2".repeat(64)
        : "3".repeat(64)
  }`,
  plannerId,
  status: "success",
  failureReason: null,
  parameters: { resolutionM: 4, maxExpansions: 120_000 },
  metrics: {
    success: true,
    failureReason: null,
    completionTimeS: 8,
    executedPathLengthM: executedLength,
    directDistanceM: directDistance,
    pathExcessPct: (executedLength / directDistance - 1) * 100,
    replans: 1,
    failedReplans: 0,
    holds: 0,
    safetyGateActivations: 0,
    collisionCount: 0,
    totalPlanningWork: 120,
    workUnit: plannerId === "dstar-lite-3d" ? "queue-pops" : "expanded-nodes",
    totalChangedEdges: 0,
    deadlineMisses: 0,
    minimumClearanceM: null,
  },
  frames: frames(),
});

const fixture = (): DynamicBundleV1 => ({
  schemaVersion: 1,
  sourceCommit: "0123456789abcdef0123456789abcdef01234567",
  generatedAt: "2026-08-05T08:00:00Z",
  verificationStatus: "DYNAMIC_DEMO_NON_CONFIRMATORY",
  protocol: {
    id: "dynamic-replanning-v1",
    timeStepS: 1,
    replanIntervalS: 4,
    cruiseSpeedMps: 8,
    maxTimeS: 180,
    resolutionM: 4,
    maxExpansions: 120_000,
  },
  planners: [
    { id: "repeated-astar-3d", label: "Repeated 3D A*" },
    { id: "repeated-lazy-theta-star", label: "Repeated Lazy Theta*" },
    { id: "dstar-lite-3d", label: "3D D* Lite" },
  ],
  scenarios: [
    {
      id: "crossing-traffic",
      label: "Crossing traffic",
      description: "A moving obstacle crosses the nominal route.",
      fingerprint: `sha256:${"a".repeat(64)}`,
      bounds: { min: [0, 0, 0], max: [50, 50, 30] },
      start: [...start],
      goal: [...goal],
      constraints: { vehicleRadiusM: 0.5, safetyMarginM: 0.5 },
      buildings: [],
      staticNoFlyZones: [],
      temporaryNoFlyZones: [
        {
          id: "popup-zone",
          center: [25, 25],
          radiusM: 4,
          zMinM: 2,
          zMaxM: 20,
          activeFromS: 4,
          activeUntilS: 10,
        },
      ],
      movingSpheres: [
        {
          id: "traffic-1",
          radiusM: 2,
          keyframes: [
            { timeS: 0, position: [12, 30, 12] },
            { timeS: 8, position: [32, 30, 12] },
          ],
        },
      ],
      runs: [
        run("repeated-astar-3d"),
        run("repeated-lazy-theta-star"),
        run("dstar-lite-3d"),
      ],
    },
  ],
  downloads: {
    recordsCsv: {
      path: "dynamic-records.csv",
      sha256: `sha256:${"b".repeat(64)}`,
      bytes: 120,
    },
    scenarioManifest: {
      path: "dynamic-scenario-manifest.json",
      sha256: `sha256:${"c".repeat(64)}`,
      bytes: 240,
    },
  },
});

describe("dynamic bundle v1", () => {
  it("accepts complete time-indexed runs for the declared three-planner comparison", () => {
    const parsed = validateDynamicBundle(fixture());
    expect(parsed.scenarios[0]!.runs).toHaveLength(3);
    expect(parsed.scenarios[0]!.runs[0]!.frames[1]!.timeS).toBe(8);
  });

  it("loads and validates the public bundle through an injected fetch implementation", async () => {
    const fetcher = vi.fn(async () => new Response(JSON.stringify(fixture()), { status: 200 }));
    const parsed = await loadDynamicBundle(fetcher as typeof fetch);
    expect(parsed.verificationStatus).toBe("DYNAMIC_DEMO_NON_CONFIRMATORY");
    expect(fetcher).toHaveBeenCalledWith(expect.stringContaining("dynamic-data.json"));
  });

  it("builds one ordered comparison row per planner without equating work units", () => {
    const value = validateDynamicBundle(fixture());
    const rows = buildDynamicComparisonRows(value, value.scenarios[0]!);
    expect(rows.map((row) => row.plannerId)).toEqual([
      "repeated-astar-3d",
      "repeated-lazy-theta-star",
      "dstar-lite-3d",
    ]);
    expect(rows.map((row) => row.workUnit)).toEqual([
      "expanded-nodes",
      "expanded-nodes",
      "queue-pops",
    ]);
  });

  it("rejects frames that do not advance monotonically", () => {
    const value = fixture();
    value.scenarios[0]!.runs[0]!.frames[1]!.timeS = 0;
    expect(() => validateDynamicBundle(value)).toThrow(/strictly increasing|half-open schedules/);
  });

  it("rejects a current plan that does not start at the vehicle", () => {
    const value = fixture();
    value.scenarios[0]!.runs[0]!.frames[0]!.path[0] = [5, 5, 6];
    expect(() => validateDynamicBundle(value)).toThrow(/path endpoints/);
  });

  it("rejects obstacle states that disagree with schedules or keyframes", () => {
    const movingState = fixture();
    movingState.scenarios[0]!.runs[0]!.frames[1]!.movingSpheres[0]!.position = [30, 30, 12];
    expect(() => validateDynamicBundle(movingState)).toThrow(/declared keyframes/);

    const activeZone = fixture();
    activeZone.scenarios[0]!.runs[0]!.frames[0]!.activeTemporaryZoneIds = ["popup-zone"];
    expect(() => validateDynamicBundle(activeZone)).toThrow(/half-open schedules/);
  });

  it("rejects aggregate metrics that disagree with the final frame", () => {
    const value = fixture();
    value.scenarios[0]!.runs[0]!.metrics.executedPathLengthM += 1;
    expect(() => validateDynamicBundle(value)).toThrow(/path lengths disagree/);
  });

  it("validates and retains an exact variable-speed execution clock", () => {
    const value = fixture() as unknown as { scenarios: { runs: Record<string, unknown>[] }[] };
    const recorded = value.scenarios[0]!.runs[0]!;
    recorded.parameters = { resolutionM: 4, maxExpansions: 120_000, cruiseSpeedMps: 8, maxClimbRateMps: 3 };
    recorded.executionTimedPath = [
      { time: 0, position: start, action: "start" },
      { time: 3.5, position: midpoint, action: "move" },
      { time: 8, position: goal, action: "move" },
    ];
    const parsed = validateDynamicBundle(value).scenarios[0]!.runs[0]!.executionTimedPath!;
    expect(parsed.map(p => p.timeS)).toEqual([0, 3.5, 8]);
    (recorded.executionTimedPath as { time: number }[])[1]!.time = 0.1;
    expect(() => validateDynamicBundle(value)).toThrow(/motion contract/);
  });

  it("refuses to fabricate timing when a climb-constrained trace is missing", () => {
    const value = fixture();
    value.scenarios[0]!.runs[0]!.parameters.maxClimbRateMps = 3;
    expect(() => validateDynamicBundle(value)).toThrow(/exact climb-constrained/);
  });

  it("rejects placeholder provenance and duplicate planner runs", () => {
    const placeholder = fixture();
    placeholder.sourceCommit = "worktree-v0.3";
    expect(() => validateDynamicBundle(placeholder)).toThrow(/Git object ID/);

    const duplicate = fixture();
    duplicate.scenarios[0]!.runs[1]!.plannerId = "repeated-astar-3d";
    expect(() => validateDynamicBundle(duplicate)).toThrow(/one run for each declared planner/);

    const duplicateRunId = fixture();
    duplicateRunId.scenarios[0]!.runs[1]!.runId = duplicateRunId.scenarios[0]!.runs[0]!.runId;
    expect(() => validateDynamicBundle(duplicateRunId)).toThrow(/runId values must be unique/);
  });
});
