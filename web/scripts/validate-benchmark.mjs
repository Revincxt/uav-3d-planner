import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";

const input = globalThis.process?.argv?.[2];
const source = input
  ? pathToFileURL(resolve(input))
  : new URL("../public/benchmark-data.json", import.meta.url);
const data = JSON.parse(await readFile(source, "utf8"));

const fail = (message) => {
  throw new Error(`benchmark-data.json: ${message}`);
};
const finite = (value) => typeof value === "number" && Number.isFinite(value);
const nonNegativeInteger = (value) => Number.isInteger(value) && value >= 0;
const plannerIds = new Set(["astar-3d", "lazy-theta-star", "rrt-star"]);

const counts = (value, label) => {
  if (
    !value ||
    !nonNegativeInteger(value.nScenes) ||
    !nonNegativeInteger(value.nDefinedScenes) ||
    !nonNegativeInteger(value.nRuns) ||
    !nonNegativeInteger(value.nSuccesses) ||
    value.nScenes > value.nRuns ||
    value.nDefinedScenes > value.nScenes ||
    value.nSuccesses > value.nRuns
  ) {
    fail(`${label} has invalid counts`);
  }
};

const distribution = (value, label) => {
  counts(value, label);
  if (value.estimator !== "median-of-scene-medians") fail(`${label} estimator is invalid`);
  if (!new Set(["all-runs", "successful-runs"]).has(value.conditioning)) {
    fail(`${label} conditioning is invalid`);
  }
  const interval = [value.median, value.q1, value.q3, value.ci95Low, value.ci95High];
  const allNull = interval.every((item) => item === null);
  const allFinite = interval.every(finite);
  if (!allNull && !allFinite) fail(`${label} interval must be entirely finite or null`);
  if (value.nDefinedScenes === 0 && !allNull) fail(`${label} must be null without defined scenes`);
  if (value.nDefinedScenes > 0 && allNull) fail(`${label} cannot be null with defined scenes`);
  if (allFinite) {
    if (value.q1 > value.median || value.median > value.q3) fail(`${label} IQR is invalid`);
    if (value.ci95Low > value.median || value.median > value.ci95High) {
      fail(`${label} confidence interval is invalid`);
    }
    if (interval.some((item) => item < 0)) fail(`${label} cannot contain negative values`);
  }
};

if (data.schemaVersion !== 2) fail("schemaVersion must be 2");
if (data.evidenceLabel !== "DESCRIPTIVE_BENCHMARK") fail("evidenceLabel is invalid");
if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(data.sourceCommit ?? "")) {
  fail("sourceCommit must be a full lowercase Git object ID");
}
if (!data.protocol || data.protocol.bootstrap?.method !== "scene-clustered-percentile") {
  fail("scene-clustered bootstrap protocol is required");
}
if (data.protocol.rawPathPrimary !== true || data.protocol.confidenceLevel !== 0.95) {
  fail("the declared analysis contract is invalid");
}
const rrtSeeds = data.protocol.quality?.rrtPlannerSeeds;
if (
  data.protocol.quality?.deterministicRunsPerScene !== 1 ||
  !Array.isArray(rrtSeeds) ||
  rrtSeeds.length === 0 ||
  !rrtSeeds.every(nonNegativeInteger) ||
  new Set(rrtSeeds).size !== rrtSeeds.length
) {
  fail("the path-quality replication contract is invalid");
}
if (!data.dataset || data.dataset.acceptedScenes + data.dataset.rejectedScenes !== data.dataset.attemptedScenes) {
  fail("dataset acceptance counts are inconsistent");
}
if (!/^sha256:[0-9a-f]{64}$/.test(data.dataset.manifestSha256 ?? "")) {
  fail("dataset manifest fingerprint is invalid");
}
if (!Array.isArray(data.planners) || data.planners.length !== 3) fail("expected three planners");
if (new Set(data.planners.map((planner) => planner.id)).size !== 3) fail("planner IDs must be unique");
if (!data.planners.every((planner) => plannerIds.has(planner.id))) fail("planner ID is unsupported");

if (!Array.isArray(data.budgets) || data.budgets.length < 3) fail("expected planner budgets");
const budgets = new Map();
for (const budget of data.budgets) {
  if (!budget.id || budgets.has(budget.id)) fail("budget IDs must be non-empty and unique");
  if (!plannerIds.has(budget.plannerId)) fail(`${budget.id} has an invalid planner`);
  if (budget.kind === "voxel") {
    if (
      budget.plannerId === "rrt-star" ||
      !finite(budget.voxelResolutionM) ||
      budget.voxelResolutionM <= 0 ||
      !Number.isInteger(budget.maxExpansions) ||
      budget.maxExpansions <= 0
    ) {
      fail(`${budget.id} has an invalid voxel budget`);
    }
  } else if (budget.kind === "samples") {
    if (
      budget.plannerId !== "rrt-star" ||
      !Number.isInteger(budget.sampleBudget) ||
      budget.sampleBudget <= 0
    ) {
      fail(`${budget.id} has an invalid sample budget`);
    }
  } else {
    fail(`${budget.id} has an invalid budget kind`);
  }
  budgets.set(budget.id, budget);
}

if (!Array.isArray(data.summaries) || data.summaries.length !== 3) {
  fail("expected exactly three summary rows");
}
const summaryIds = new Set();
for (const summary of data.summaries) {
  if (summaryIds.has(summary.plannerId)) fail(`duplicate summary ${summary.plannerId}`);
  summaryIds.add(summary.plannerId);
  if (budgets.get(summary.budgetId)?.plannerId !== summary.plannerId) {
    fail(`${summary.plannerId} references the wrong budget`);
  }
  counts(summary.successRate, `${summary.plannerId}.successRate`);
  if (
    summary.successRate.estimator !== "scene-weighted-mean" ||
    !finite(summary.successRate.value) ||
    !finite(summary.successRate.ci95Low) ||
    !finite(summary.successRate.ci95High) ||
    summary.successRate.ci95Low < 0 ||
    summary.successRate.ci95High > 1 ||
    summary.successRate.ci95Low > summary.successRate.value ||
    summary.successRate.value > summary.successRate.ci95High
  ) {
    fail(`${summary.plannerId} success interval is invalid`);
  }
  for (const metric of [
    "planningTimeMs",
    "rawPathExcessPct",
    "smoothedPathExcessPct",
    "minimumClearanceM",
  ]) {
    distribution(summary[metric], `${summary.plannerId}.${metric}`);
  }
  if (summary.planningTimeMs.conditioning !== "all-runs") {
    fail(`${summary.plannerId} timing must include all runs`);
  }
  const expectedQualityRuns =
    data.dataset.acceptedScenes * (summary.plannerId === "rrt-star" ? rrtSeeds.length : 1);
  const expectedTimingRuns =
    data.dataset.acceptedScenes * data.protocol.timing.repetitionsPerCell;
  if (
    summary.successRate.nScenes !== data.dataset.acceptedScenes ||
    summary.successRate.nRuns !== expectedQualityRuns ||
    summary.planningTimeMs.nScenes !== data.dataset.acceptedScenes ||
    summary.planningTimeMs.nRuns !== expectedTimingRuns
  ) {
    fail(`${summary.plannerId} path/timing sample counts do not match the protocol`);
  }
  for (const metric of [
    summary.rawPathExcessPct,
    summary.smoothedPathExcessPct,
    summary.minimumClearanceM,
  ]) {
    if (metric.conditioning !== "successful-runs") {
      fail(`${summary.plannerId} path metrics must be conditioned on successful runs`);
    }
    if (
      metric.nScenes !== summary.successRate.nScenes ||
      metric.nRuns !== summary.successRate.nRuns ||
      metric.nSuccesses !== summary.successRate.nSuccesses
    ) {
      fail(`${summary.plannerId} path metric sample counts are inconsistent`);
    }
  }
}

if (!Array.isArray(data.sensitivity?.resolution) || data.sensitivity.resolution.length < 4) {
  fail("resolution sensitivity is incomplete");
}
const resolutionsByPlanner = new Map([
  ["astar-3d", new Set()],
  ["lazy-theta-star", new Set()],
]);
for (const point of data.sensitivity.resolution) {
  if (point.plannerId === "rrt-star" || !finite(point.voxelResolutionM) || point.voxelResolutionM <= 0) {
    fail("resolution sensitivity point is invalid");
  }
  resolutionsByPlanner.get(point.plannerId)?.add(point.voxelResolutionM);
  distribution(point.rawPathExcessPct, `resolution:${point.plannerId}:${point.voxelResolutionM}`);
  if (
    point.rawPathExcessPct.nScenes !== data.dataset.acceptedScenes ||
    point.rawPathExcessPct.nRuns !== data.dataset.acceptedScenes
  ) {
    fail("resolution sensitivity sample counts do not match the protocol");
  }
}
for (const [planner, values] of resolutionsByPlanner) {
  if (values.size < 2) fail(`resolution sensitivity requires at least two ${planner} settings`);
}
if (!Array.isArray(data.sensitivity.rrtBudget) || data.sensitivity.rrtBudget.length < 2) {
  fail("RRT* budget sensitivity is incomplete");
}
for (const point of data.sensitivity.rrtBudget) {
  if (!Number.isInteger(point.sampleBudget) || point.sampleBudget <= 0) {
    fail("RRT* sample budget is invalid");
  }
  distribution(point.rawPathExcessPct, `rrt-budget:${point.sampleBudget}`);
  if (
    point.rawPathExcessPct.nScenes !== data.dataset.acceptedScenes ||
    point.rawPathExcessPct.nRuns !== data.dataset.acceptedScenes * rrtSeeds.length
  ) {
    fail("RRT* sensitivity sample counts do not match the protocol");
  }
}

const expectedDownloads = {
  recordsCsv: "benchmark-records.csv",
  summariesCsv: "benchmark-summary.csv",
  datasetManifest: "dataset-manifest.json",
  timingManifest: "timing-manifest.json",
};
if (!data.downloads || typeof data.downloads !== "object") fail("downloads are required");
for (const [key, expectedPath] of Object.entries(expectedDownloads)) {
  const reference = data.downloads[key];
  if (
    !reference ||
    reference.path !== expectedPath ||
    !/^sha256:[0-9a-f]{64}$/.test(reference.sha256 ?? "") ||
    !Number.isInteger(reference.bytes) ||
    reference.bytes <= 0
  ) {
    fail(`downloads.${key} is invalid`);
  }
  const bytes = await readFile(new URL(reference.path, source));
  if (bytes.byteLength !== reference.bytes) fail(`downloads.${key} byte size does not match`);
  const digest = `sha256:${createHash("sha256").update(bytes).digest("hex")}`;
  if (digest !== reference.sha256) fail(`downloads.${key} digest does not match`);
}
if (data.downloads.datasetManifest.sha256 !== data.dataset.manifestSha256) {
  fail("dataset manifest digests disagree");
}

console.log(
  `validated ${data.summaries.length} planner summaries, ` +
    `${data.sensitivity.resolution.length} resolution points, and ` +
    `${data.sensitivity.rrtBudget.length} RRT* budget points`,
);
