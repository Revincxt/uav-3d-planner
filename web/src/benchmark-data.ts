import type {
  BenchmarkBundleV2,
  Conditioning,
  DistributionSummary,
  PlannerBudget,
  PlannerId,
  PlannerSummary,
  ProportionSummary,
  SampleCounts,
} from "./benchmark-schema";

const PLANNER_IDS = ["astar-3d", "lazy-theta-star", "rrt-star"] as const;
const GRAPH_PLANNERS = new Set<PlannerId>(["astar-3d", "lazy-theta-star"]);

function fail(message: string): never {
  throw new Error(`benchmark-data.json: ${message}`);
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function finite(value: unknown, label: string): number {
  if (typeof value !== "number" || !Number.isFinite(value)) fail(`${label} must be finite`);
  return value;
}

function positive(value: unknown, label: string): number {
  const result = finite(value, label);
  if (result <= 0) fail(`${label} must be positive`);
  return result;
}

function positiveInteger(value: unknown, label: string): number {
  const result = positive(value, label);
  if (!Number.isInteger(result)) fail(`${label} must be an integer`);
  return result;
}

function integer(value: unknown, label: string): number {
  const result = finite(value, label);
  if (!Number.isInteger(result) || result < 0) fail(`${label} must be a non-negative integer`);
  return result;
}

function nonEmptyString(value: unknown, label: string): string {
  if (typeof value !== "string" || value.trim() === "") fail(`${label} must be a string`);
  return value;
}

function plannerId(value: unknown, label: string): PlannerId {
  if (!PLANNER_IDS.includes(value as PlannerId)) fail(`${label} is not a supported planner`);
  return value as PlannerId;
}

function validateCounts(value: SampleCounts, label: string): void {
  if (!isRecord(value)) fail(`${label} must be an object`);
  const nScenes = integer(value.nScenes, `${label}.nScenes`);
  const nDefinedScenes = integer(value.nDefinedScenes, `${label}.nDefinedScenes`);
  const nRuns = integer(value.nRuns, `${label}.nRuns`);
  const nSuccesses = integer(value.nSuccesses, `${label}.nSuccesses`);
  if (nScenes > nRuns) fail(`${label}.nScenes cannot exceed nRuns`);
  if (nDefinedScenes > nScenes) fail(`${label}.nDefinedScenes cannot exceed nScenes`);
  if (nSuccesses > nRuns) fail(`${label}.nSuccesses cannot exceed nRuns`);
}

function validateProportion(value: ProportionSummary, label: string): void {
  if (!isRecord(value)) fail(`${label} must be an object`);
  validateCounts(value, label);
  if (value.estimator !== "scene-weighted-mean") fail(`${label} has an invalid estimator`);
  const estimate = finite(value.value, `${label}.value`);
  const low = finite(value.ci95Low, `${label}.ci95Low`);
  const high = finite(value.ci95High, `${label}.ci95High`);
  if (low < 0 || high > 1 || low > estimate || estimate > high) {
    fail(`${label} confidence interval must contain a proportion in [0, 1]`);
  }
}

function validateDistribution(value: DistributionSummary, label: string): void {
  if (!isRecord(value)) fail(`${label} must be an object`);
  validateCounts(value, label);
  if (value.estimator !== "median-of-scene-medians") {
    fail(`${label} has an invalid estimator`);
  }
  const conditioning: Conditioning = value.conditioning;
  if (conditioning !== "all-runs" && conditioning !== "successful-runs") {
    fail(`${label} has invalid conditioning`);
  }
  const interval = [value.median, value.q1, value.q3, value.ci95Low, value.ci95High];
  const allNull = interval.every((item) => item === null);
  const allFinite = interval.every((item) => typeof item === "number" && Number.isFinite(item));
  if (!allNull && !allFinite) fail(`${label} interval must be entirely finite or entirely null`);
  if (value.nDefinedScenes === 0 && !allNull) fail(`${label} must be null without defined scenes`);
  if (value.nDefinedScenes > 0 && allNull) fail(`${label} cannot be null with defined scenes`);
  if (allFinite) {
    const median = value.median as number;
    const q1 = value.q1 as number;
    const q3 = value.q3 as number;
    const low = value.ci95Low as number;
    const high = value.ci95High as number;
    if (q1 > median || median > q3) fail(`${label} IQR must contain the median`);
    if (low > median || median > high) fail(`${label} confidence interval must contain the median`);
  }
}

function validateNonNegativeDistribution(value: DistributionSummary, label: string): void {
  validateDistribution(value, label);
  for (const item of [value.median, value.q1, value.q3, value.ci95Low, value.ci95High]) {
    if (item !== null && item < 0) fail(`${label} cannot contain negative values`);
  }
}

function validateBudget(value: PlannerBudget, label: string): void {
  if (!isRecord(value)) fail(`${label} must be an object`);
  nonEmptyString(value.id, `${label}.id`);
  const id = plannerId(value.plannerId, `${label}.plannerId`);
  if (value.wallClockLimitMs !== null) positive(value.wallClockLimitMs, `${label}.wallClockLimitMs`);
  if (value.kind === "voxel") {
    if (!GRAPH_PLANNERS.has(id)) fail(`${label} voxel budget must belong to a graph planner`);
    positive(value.voxelResolutionM, `${label}.voxelResolutionM`);
    positiveInteger(value.maxExpansions, `${label}.maxExpansions`);
  } else if (value.kind === "samples") {
    if (id !== "rrt-star") fail(`${label} sample budget must belong to RRT*`);
    positiveInteger(value.sampleBudget, `${label}.sampleBudget`);
  } else {
    fail(`${label} has an invalid kind`);
  }
}

function validateSummary(
  value: PlannerSummary,
  label: string,
  budgets: Map<string, PlannerBudget>,
): void {
  if (!isRecord(value)) fail(`${label} must be an object`);
  const id = plannerId(value.plannerId, `${label}.plannerId`);
  const budget = budgets.get(nonEmptyString(value.budgetId, `${label}.budgetId`));
  if (!budget || budget.plannerId !== id) fail(`${label} references the wrong budget`);
  validateProportion(value.successRate, `${label}.successRate`);
  validateNonNegativeDistribution(value.planningTimeMs, `${label}.planningTimeMs`);
  validateNonNegativeDistribution(value.rawPathExcessPct, `${label}.rawPathExcessPct`);
  validateNonNegativeDistribution(value.smoothedPathExcessPct, `${label}.smoothedPathExcessPct`);
  validateNonNegativeDistribution(value.minimumClearanceM, `${label}.minimumClearanceM`);
  if (value.planningTimeMs.conditioning !== "all-runs") {
    fail(`${label}.planningTimeMs must include all runs`);
  }
  for (const metric of [
    value.rawPathExcessPct,
    value.smoothedPathExcessPct,
    value.minimumClearanceM,
  ]) {
    if (metric.conditioning !== "successful-runs") {
      fail(`${label} path metrics must be conditioned on successful runs`);
    }
  }
}

export function validateBenchmarkBundle(value: unknown): BenchmarkBundleV2 {
  if (!isRecord(value)) fail("root must be an object");
  const bundle = value as unknown as BenchmarkBundleV2;
  if (bundle.schemaVersion !== 2) fail("schemaVersion must be 2");
  if (bundle.evidenceLabel !== "DESCRIPTIVE_BENCHMARK") {
    fail("evidenceLabel must be DESCRIPTIVE_BENCHMARK");
  }
  nonEmptyString(bundle.sourceCommit, "sourceCommit");
  if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(bundle.sourceCommit)) {
    fail("sourceCommit must be a full lowercase Git object ID");
  }
  if (typeof bundle.generatedAt !== "string" || Number.isNaN(Date.parse(bundle.generatedAt))) {
    fail("generatedAt must be an ISO date-time");
  }
  if (!isRecord(bundle.protocol)) fail("protocol must be an object");
  nonEmptyString(bundle.protocol.id, "protocol.id");
  if (bundle.protocol.rawPathPrimary !== true) fail("rawPathPrimary must be true");
  if (bundle.protocol.confidenceLevel !== 0.95) fail("confidenceLevel must be 0.95");
  if (
    bundle.protocol.pathExcessDefinition !==
    "(path_length / euclidean_start_goal - 1) * 100"
  ) {
    fail("pathExcessDefinition is unsupported");
  }
  if (!isRecord(bundle.protocol.bootstrap)) fail("protocol.bootstrap must be an object");
  if (bundle.protocol.bootstrap.method !== "scene-clustered-percentile") {
    fail("bootstrap method must cluster by scene");
  }
  if (integer(bundle.protocol.bootstrap.resamples, "protocol.bootstrap.resamples") < 1000) {
    fail("protocol.bootstrap.resamples must be at least 1000");
  }
  integer(bundle.protocol.bootstrap.seed, "protocol.bootstrap.seed");
  if (!isRecord(bundle.protocol.timing) || bundle.protocol.timing.isolatedProcesses !== true) {
    fail("timing must use isolated processes");
  }
  positiveInteger(
    bundle.protocol.timing.repetitionsPerCell,
    "protocol.timing.repetitionsPerCell",
  );
  if (
    !isRecord(bundle.protocol.quality) ||
    bundle.protocol.quality.deterministicRunsPerScene !== 1
  ) {
    fail("quality.deterministicRunsPerScene must be 1");
  }
  if (
    !Array.isArray(bundle.protocol.quality.rrtPlannerSeeds) ||
    bundle.protocol.quality.rrtPlannerSeeds.length === 0
  ) {
    fail("quality.rrtPlannerSeeds must be non-empty");
  }
  const rrtSeeds = bundle.protocol.quality.rrtPlannerSeeds.map((seed, index) =>
    integer(seed, `protocol.quality.rrtPlannerSeeds[${index}]`),
  );
  if (new Set(rrtSeeds).size !== rrtSeeds.length) {
    fail("quality.rrtPlannerSeeds must be unique");
  }

  if (!isRecord(bundle.dataset)) fail("dataset must be an object");
  nonEmptyString(bundle.dataset.id, "dataset.id");
  nonEmptyString(bundle.dataset.label, "dataset.label");
  if (bundle.dataset.split !== "held-out" && bundle.dataset.split !== "diagnostic") {
    fail("dataset.split is invalid");
  }
  const attempted = integer(bundle.dataset.attemptedScenes, "dataset.attemptedScenes");
  const accepted = integer(bundle.dataset.acceptedScenes, "dataset.acceptedScenes");
  const rejected = integer(bundle.dataset.rejectedScenes, "dataset.rejectedScenes");
  if (accepted + rejected !== attempted) fail("acceptedScenes + rejectedScenes must equal attemptedScenes");
  if (!/^sha256:[0-9a-f]{64}$/.test(bundle.dataset.manifestSha256)) {
    fail("dataset.manifestSha256 is invalid");
  }

  if (!Array.isArray(bundle.planners) || bundle.planners.length !== PLANNER_IDS.length) {
    fail("planners must contain exactly three entries");
  }
  const plannerIds = new Set<PlannerId>();
  bundle.planners.forEach((planner, index) => {
    if (!isRecord(planner)) fail(`planners[${index}] must be an object`);
    const id = plannerId(planner.id, `planners[${index}].id`);
    if (plannerIds.has(id)) fail(`duplicate planner ${id}`);
    plannerIds.add(id);
    nonEmptyString(planner.label, `planners[${index}].label`);
  });
  if (PLANNER_IDS.some((id) => !plannerIds.has(id))) fail("planner catalog is incomplete");

  if (!Array.isArray(bundle.budgets) || bundle.budgets.length < 3) {
    fail("budgets must contain at least one budget per planner");
  }
  const budgets = new Map<string, PlannerBudget>();
  bundle.budgets.forEach((budget, index) => {
    validateBudget(budget, `budgets[${index}]`);
    if (budgets.has(budget.id)) fail(`duplicate budget ${budget.id}`);
    budgets.set(budget.id, budget);
  });
  nonEmptyString(bundle.nominalBudgetSetId, "nominalBudgetSetId");

  if (!Array.isArray(bundle.summaries) || bundle.summaries.length !== PLANNER_IDS.length) {
    fail("summaries must contain exactly three planner rows");
  }
  const summaryIds = new Set<PlannerId>();
  bundle.summaries.forEach((summary, index) => {
    validateSummary(summary, `summaries[${index}]`, budgets);
    if (summaryIds.has(summary.plannerId)) fail(`duplicate summary ${summary.plannerId}`);
    summaryIds.add(summary.plannerId);
  });
  if (PLANNER_IDS.some((id) => !summaryIds.has(id))) fail("summary table is incomplete");
  bundle.summaries.forEach((summary) => {
    const expectedQualityRuns =
      accepted *
      (summary.plannerId === "rrt-star"
        ? rrtSeeds.length
        : bundle.protocol.quality.deterministicRunsPerScene);
    const expectedTimingRuns = accepted * bundle.protocol.timing.repetitionsPerCell;
    if (
      summary.successRate.nScenes !== accepted ||
      summary.successRate.nRuns !== expectedQualityRuns
    ) {
      fail(`${summary.plannerId} quality sample counts do not match the protocol`);
    }
    if (
      summary.planningTimeMs.nScenes !== accepted ||
      summary.planningTimeMs.nRuns !== expectedTimingRuns
    ) {
      fail(`${summary.plannerId} timing sample counts do not match the protocol`);
    }
    for (const metric of [
      summary.rawPathExcessPct,
      summary.smoothedPathExcessPct,
      summary.minimumClearanceM,
    ]) {
      if (
        metric.nScenes !== summary.successRate.nScenes ||
        metric.nRuns !== summary.successRate.nRuns ||
        metric.nSuccesses !== summary.successRate.nSuccesses
      ) {
        fail(`${summary.plannerId} path metric sample counts are inconsistent`);
      }
    }
  });

  if (!isRecord(bundle.sensitivity)) fail("sensitivity must be an object");
  if (!Array.isArray(bundle.sensitivity.resolution) || bundle.sensitivity.resolution.length < 2) {
    fail("resolution sensitivity must contain at least two points");
  }
  const resolutionKeys = new Set<string>();
  const resolutionsByPlanner = new Map<PlannerId, Set<number>>();
  bundle.sensitivity.resolution.forEach((point, index) => {
    if (!isRecord(point)) fail(`sensitivity.resolution[${index}] must be an object`);
    const id = plannerId(point.plannerId, `sensitivity.resolution[${index}].plannerId`);
    if (!GRAPH_PLANNERS.has(id)) fail("resolution sensitivity only supports graph planners");
    const resolution = positive(
      point.voxelResolutionM,
      `sensitivity.resolution[${index}].voxelResolutionM`,
    );
    const key = `${id}:${resolution}`;
    if (resolutionKeys.has(key)) fail(`duplicate resolution point ${key}`);
    resolutionKeys.add(key);
    const plannerResolutions = resolutionsByPlanner.get(id) ?? new Set<number>();
    plannerResolutions.add(resolution);
    resolutionsByPlanner.set(id, plannerResolutions);
    validateNonNegativeDistribution(
      point.rawPathExcessPct,
      `sensitivity.resolution[${index}].rawPathExcessPct`,
    );
    if (point.rawPathExcessPct.conditioning !== "successful-runs") {
      fail("resolution path quality must be conditioned on success");
    }
    if (
      point.rawPathExcessPct.nScenes !== accepted ||
      point.rawPathExcessPct.nRuns !==
        accepted * bundle.protocol.quality.deterministicRunsPerScene
    ) {
      fail("resolution sensitivity sample counts do not match the protocol");
    }
  });
  for (const id of ["astar-3d", "lazy-theta-star"] as const) {
    if ((resolutionsByPlanner.get(id)?.size ?? 0) < 2) {
      fail(`resolution sensitivity requires at least two ${id} settings`);
    }
  }

  if (!Array.isArray(bundle.sensitivity.rrtBudget) || bundle.sensitivity.rrtBudget.length < 2) {
    fail("RRT* budget sensitivity must contain at least two points");
  }
  const sampleBudgets = new Set<number>();
  bundle.sensitivity.rrtBudget.forEach((point, index) => {
    if (!isRecord(point)) fail(`sensitivity.rrtBudget[${index}] must be an object`);
    const samples = positiveInteger(
      point.sampleBudget,
      `sensitivity.rrtBudget[${index}].sampleBudget`,
    );
    if (sampleBudgets.has(samples)) fail(`duplicate RRT* sample budget ${samples}`);
    sampleBudgets.add(samples);
    validateNonNegativeDistribution(
      point.rawPathExcessPct,
      `sensitivity.rrtBudget[${index}].rawPathExcessPct`,
    );
    if (point.rawPathExcessPct.conditioning !== "successful-runs") {
      fail("RRT* path quality must be conditioned on success");
    }
    if (
      point.rawPathExcessPct.nScenes !== accepted ||
      point.rawPathExcessPct.nRuns !== accepted * rrtSeeds.length
    ) {
      fail("RRT* sensitivity sample counts do not match the protocol");
    }
  });

  if (!isRecord(bundle.downloads)) fail("downloads must be an object");
  const expectedDownloads = {
    recordsCsv: "benchmark-records.csv",
    summariesCsv: "benchmark-summary.csv",
    datasetManifest: "dataset-manifest.json",
    timingManifest: "timing-manifest.json",
  } as const;
  Object.entries(expectedDownloads).forEach(([key, expectedPath]) => {
    const reference = bundle.downloads[key as keyof typeof expectedDownloads];
    if (!isRecord(reference)) fail(`downloads.${key} must be an object`);
    if (reference.path !== expectedPath) fail(`downloads.${key}.path is invalid`);
    if (!/^sha256:[0-9a-f]{64}$/.test(reference.sha256)) {
      fail(`downloads.${key}.sha256 is invalid`);
    }
    positiveInteger(reference.bytes, `downloads.${key}.bytes`);
  });

  return bundle;
}

export async function loadBenchmarkBundle(): Promise<BenchmarkBundleV2> {
  const response = await fetch(`${import.meta.env.BASE_URL}benchmark-data.json`);
  if (!response.ok) throw new Error(`Could not load benchmark data (${response.status})`);
  return validateBenchmarkBundle(await response.json());
}
