import type { PlannerId } from "./schema";

export type { PlannerId };
export type GraphPlannerId = Exclude<PlannerId, "rrt-star">;
export type Conditioning = "all-runs" | "successful-runs";

export interface SampleCounts {
  nScenes: number;
  nDefinedScenes: number;
  nRuns: number;
  nSuccesses: number;
}

export interface ProportionSummary extends SampleCounts {
  estimator: "scene-weighted-mean";
  value: number;
  ci95Low: number;
  ci95High: number;
}

export interface DistributionSummary extends SampleCounts {
  estimator: "median-of-scene-medians";
  median: number | null;
  q1: number | null;
  q3: number | null;
  ci95Low: number | null;
  ci95High: number | null;
  conditioning: Conditioning;
}

export interface VoxelBudget {
  id: string;
  kind: "voxel";
  plannerId: GraphPlannerId;
  voxelResolutionM: number;
  maxExpansions: number;
  wallClockLimitMs: number | null;
}

export interface SampleBudget {
  id: string;
  kind: "samples";
  plannerId: "rrt-star";
  sampleBudget: number;
  wallClockLimitMs: number | null;
}

export type PlannerBudget = VoxelBudget | SampleBudget;

export interface PlannerSummary {
  plannerId: PlannerId;
  budgetId: string;
  successRate: ProportionSummary;
  planningTimeMs: DistributionSummary;
  rawPathExcessPct: DistributionSummary;
  smoothedPathExcessPct: DistributionSummary;
  minimumClearanceM: DistributionSummary;
}

export interface ResolutionSweepPoint {
  plannerId: GraphPlannerId;
  voxelResolutionM: number;
  rawPathExcessPct: DistributionSummary;
}

export interface RrtBudgetSweepPoint {
  sampleBudget: number;
  rawPathExcessPct: DistributionSummary;
}

export interface ArtifactReference {
  path: string;
  sha256: `sha256:${string}`;
  bytes: number;
}

export interface BenchmarkBundleV2 {
  schemaVersion: 2;
  generatedAt: string;
  evidenceLabel: "DESCRIPTIVE_BENCHMARK";
  sourceCommit: string;
  protocol: {
    id: string;
    rawPathPrimary: true;
    confidenceLevel: 0.95;
    pathExcessDefinition: "(path_length / euclidean_start_goal - 1) * 100";
    bootstrap: {
      method: "scene-clustered-percentile";
      resamples: number;
      seed: number;
    };
    timing: {
      isolatedProcesses: true;
      repetitionsPerCell: number;
    };
    quality: {
      deterministicRunsPerScene: 1;
      rrtPlannerSeeds: number[];
    };
  };
  dataset: {
    id: string;
    label: string;
    split: "held-out" | "diagnostic";
    attemptedScenes: number;
    acceptedScenes: number;
    rejectedScenes: number;
    manifestSha256: `sha256:${string}`;
  };
  planners: Array<{ id: PlannerId; label: string }>;
  budgets: PlannerBudget[];
  nominalBudgetSetId: string;
  summaries: PlannerSummary[];
  sensitivity: {
    resolution: ResolutionSweepPoint[];
    rrtBudget: RrtBudgetSweepPoint[];
  };
  downloads: {
    recordsCsv: ArtifactReference;
    summariesCsv: ArtifactReference;
    datasetManifest: ArtifactReference;
    timingManifest: ArtifactReference;
  };
}
