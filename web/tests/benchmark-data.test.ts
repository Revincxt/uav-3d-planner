import { describe, expect, it } from "vitest";

import { validateBenchmarkBundle } from "../src/benchmark-data";
import type {
  BenchmarkBundleV2,
  Conditioning,
  DistributionSummary,
  PlannerId,
} from "../src/benchmark-schema";

const distribution = (
  median: number | null,
  conditioning: Conditioning,
  successes = 16,
  runs = 24,
): DistributionSummary => ({
  estimator: "median-of-scene-medians",
  median,
  q1: median === null ? null : median - 1,
  q3: median === null ? null : median + 1,
  ci95Low: median === null ? null : median - 2,
  ci95High: median === null ? null : median + 2,
  conditioning,
  nScenes: 8,
  nDefinedScenes: median === null ? 0 : 8,
  nRuns: runs,
  nSuccesses: successes,
});

const summary = (plannerId: PlannerId, budgetId: string, value: number) => {
  const qualityRuns = plannerId === "rrt-star" ? 24 : 8;
  const successes = plannerId === "rrt-star" ? 16 : 6;
  return {
    plannerId,
    budgetId,
    successRate: {
      estimator: "scene-weighted-mean" as const,
      value: 2 / 3,
      ci95Low: 0.5,
      ci95High: 0.8,
      nScenes: 8,
      nDefinedScenes: 8,
      nRuns: qualityRuns,
      nSuccesses: successes,
    },
    planningTimeMs: distribution(value * 10, "all-runs", 40, 40),
    rawPathExcessPct: distribution(value, "successful-runs", successes, qualityRuns),
    smoothedPathExcessPct: distribution(
      value - 0.5,
      "successful-runs",
      successes,
      qualityRuns,
    ),
    minimumClearanceM: distribution(
      3 + value / 100,
      "successful-runs",
      successes,
      qualityRuns,
    ),
  };
};

const fixture = (): BenchmarkBundleV2 => ({
  schemaVersion: 2,
  generatedAt: "2026-08-05T00:00:00Z",
  evidenceLabel: "DESCRIPTIVE_BENCHMARK",
  sourceCommit: "0123456789abcdef0123456789abcdef01234567",
  protocol: {
    id: "benchmark-v2",
    rawPathPrimary: true,
    confidenceLevel: 0.95,
    pathExcessDefinition: "(path_length / euclidean_start_goal - 1) * 100",
    bootstrap: { method: "scene-clustered-percentile", resamples: 2000, seed: 41 },
    timing: { isolatedProcesses: true, repetitionsPerCell: 5 },
    quality: {
      deterministicRunsPerScene: 1,
      rrtPlannerSeeds: [11, 23, 37],
    },
  },
  dataset: {
    id: "held-out-cities-v2",
    label: "Held-out random cities",
    split: "held-out",
    attemptedScenes: 10,
    acceptedScenes: 8,
    rejectedScenes: 2,
    manifestSha256: `sha256:${"a".repeat(64)}`,
  },
  planners: [
    { id: "astar-3d", label: "3D A*" },
    { id: "lazy-theta-star", label: "Lazy Theta*" },
    { id: "rrt-star", label: "RRT*" },
  ],
  budgets: [
    {
      id: "astar-nominal",
      kind: "voxel",
      plannerId: "astar-3d",
      voxelResolutionM: 4,
      maxExpansions: 120000,
      wallClockLimitMs: null,
    },
    {
      id: "theta-nominal",
      kind: "voxel",
      plannerId: "lazy-theta-star",
      voxelResolutionM: 4,
      maxExpansions: 120000,
      wallClockLimitMs: null,
    },
    {
      id: "rrt-nominal",
      kind: "samples",
      plannerId: "rrt-star",
      sampleBudget: 3000,
      wallClockLimitMs: null,
    },
  ],
  nominalBudgetSetId: "nominal-v2",
  summaries: [
    summary("astar-3d", "astar-nominal", 8),
    summary("lazy-theta-star", "theta-nominal", 6),
    summary("rrt-star", "rrt-nominal", 12),
  ],
  sensitivity: {
    resolution: [
      {
        plannerId: "astar-3d",
        voxelResolutionM: 2,
        rawPathExcessPct: distribution(7, "successful-runs", 6, 8),
      },
      {
        plannerId: "astar-3d",
        voxelResolutionM: 4,
        rawPathExcessPct: distribution(9, "successful-runs", 6, 8),
      },
      {
        plannerId: "lazy-theta-star",
        voxelResolutionM: 2,
        rawPathExcessPct: distribution(5, "successful-runs", 6, 8),
      },
      {
        plannerId: "lazy-theta-star",
        voxelResolutionM: 4,
        rawPathExcessPct: distribution(6, "successful-runs", 6, 8),
      },
    ],
    rrtBudget: [
      { sampleBudget: 1000, rawPathExcessPct: distribution(16, "successful-runs") },
      { sampleBudget: 3000, rawPathExcessPct: distribution(12, "successful-runs") },
      { sampleBudget: 10000, rawPathExcessPct: distribution(9, "successful-runs") },
    ],
  },
  downloads: {
    recordsCsv: {
      path: "benchmark-records.csv",
      sha256: `sha256:${"b".repeat(64)}`,
      bytes: 100,
    },
    summariesCsv: {
      path: "benchmark-summary.csv",
      sha256: `sha256:${"c".repeat(64)}`,
      bytes: 100,
    },
    datasetManifest: {
      path: "dataset-manifest.json",
      sha256: `sha256:${"a".repeat(64)}`,
      bytes: 100,
    },
    timingManifest: {
      path: "timing-manifest.json",
      sha256: `sha256:${"d".repeat(64)}`,
      bytes: 100,
    },
  },
});

describe("benchmark bundle v2", () => {
  it("accepts one complete three-planner summary and both sensitivity sweeps", () => {
    const value = fixture();
    expect(validateBenchmarkBundle(value)).toBe(value);
  });

  it("preserves null path-quality estimates when a sweep cell has no successes", () => {
    const value = fixture();
    value.sensitivity.rrtBudget[0]!.rawPathExcessPct = distribution(
      null,
      "successful-runs",
      0,
    );
    const parsed = validateBenchmarkBundle(value);
    expect(parsed.sensitivity.rrtBudget[0]!.rawPathExcessPct.median).toBeNull();
  });

  it("rejects an interval that does not contain its median", () => {
    const value = fixture();
    value.summaries[0]!.rawPathExcessPct.q1 = 99;
    expect(() => validateBenchmarkBundle(value)).toThrow(/IQR must contain the median/);
  });

  it("rejects duplicate planner summaries instead of rendering fewer than three rows", () => {
    const value = fixture();
    value.summaries[1]!.plannerId = "astar-3d";
    expect(() => validateBenchmarkBundle(value)).toThrow(/wrong budget|duplicate summary/);
  });

  it("rejects placeholder provenance and malformed artifact digests", () => {
    const placeholder = fixture();
    placeholder.sourceCommit = "worktree-v0.2.0";
    expect(() => validateBenchmarkBundle(placeholder)).toThrow(/Git object ID/);

    const malformedDigest = fixture();
    malformedDigest.downloads.recordsCsv.sha256 = "sha256:not-a-digest";
    expect(() => validateBenchmarkBundle(malformedDigest)).toThrow(/sha256 is invalid/);
  });

  it("rejects path and timing counts that disagree with the declared protocol", () => {
    const pathCounts = fixture();
    pathCounts.summaries[0]!.rawPathExcessPct.nRuns += 1;
    expect(() => validateBenchmarkBundle(pathCounts)).toThrow(/sample counts/);

    const timingCounts = fixture();
    timingCounts.summaries[2]!.planningTimeMs.nRuns += 1;
    expect(() => validateBenchmarkBundle(timingCounts)).toThrow(/timing sample counts/);
  });
});
