import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";

const input = globalThis.process?.argv?.[2];
const source = input
  ? pathToFileURL(resolve(input))
  : new URL("../public/dynamic-data.json", import.meta.url);
const data = JSON.parse(await readFile(source, "utf8"));
const tolerance = 1e-6;
const declaredPlanners = [
  ["repeated-astar-3d", "Repeated 3D A*"],
  ["repeated-lazy-theta-star", "Repeated Lazy Theta*"],
  ["dstar-lite-3d", "3D D* Lite"],
];

const fail = (message) => {
  throw new Error(`dynamic-data.json: ${message}`);
};
const object = (value, label) => {
  if (!value || typeof value !== "object" || Array.isArray(value)) fail(`${label} must be an object`);
  return value;
};
const array = (value, label) => {
  if (!Array.isArray(value)) fail(`${label} must be an array`);
  return value;
};
const text = (value, label) => {
  if (typeof value !== "string" || value.trim() === "") fail(`${label} must be a string`);
  return value;
};
const finite = (value, label) => {
  if (typeof value !== "number" || !Number.isFinite(value)) fail(`${label} must be finite`);
  return value;
};
const nonNegative = (value, label) => {
  const parsed = finite(value, label);
  if (parsed < 0) fail(`${label} cannot be negative`);
  return parsed;
};
const positive = (value, label) => {
  const parsed = finite(value, label);
  if (parsed <= 0) fail(`${label} must be positive`);
  return parsed;
};
const nonNegativeInteger = (value, label) => {
  const parsed = nonNegative(value, label);
  if (!Number.isInteger(parsed)) fail(`${label} must be an integer`);
  return parsed;
};
const positiveInteger = (value, label) => {
  const parsed = positive(value, label);
  if (!Number.isInteger(parsed)) fail(`${label} must be an integer`);
  return parsed;
};
const bool = (value, label) => {
  if (typeof value !== "boolean") fail(`${label} must be boolean`);
  return value;
};
const nullableText = (value, label) => value === null ? null : text(value, label);
const vector = (value, dimensions, label) => {
  const parsed = array(value, label);
  if (parsed.length !== dimensions) fail(`${label} must contain ${dimensions} coordinates`);
  return parsed.map((coordinate, index) => finite(coordinate, `${label}[${index}]`));
};
const sameNumber = (left, right) =>
  Math.abs(left - right) <= tolerance * Math.max(1, Math.abs(left), Math.abs(right));
const samePoint = (left, right) => left.every((coordinate, index) => sameNumber(coordinate, right[index]));
const inBounds = (point, bounds) => point.every(
  (coordinate, index) =>
    coordinate >= bounds.min[index] - tolerance && coordinate <= bounds.max[index] + tolerance,
);
const uniqueObjects = (value, label) => {
  const items = array(value, label).map((entry, index) => object(entry, `${label}[${index}]`));
  const ids = new Set();
  for (const [index, item] of items.entries()) {
    const id = text(item.id, `${label}[${index}].id`);
    if (ids.has(id)) fail(`${label} contains duplicate ID ${id}`);
    ids.add(id);
  }
  return items;
};
const length = (path) => {
  let total = 0;
  for (let index = 1; index < path.length; index += 1) {
    total += Math.hypot(...path[index].map((coordinate, axis) => coordinate - path[index - 1][axis]));
  }
  return total;
};
const parseCsv = (value) => {
  const rows = [];
  let row = [];
  let field = "";
  let quoted = false;
  for (let index = 0; index < value.length; index += 1) {
    const character = value[index];
    if (quoted) {
      if (character === '"') {
        if (value[index + 1] === '"') {
          field += '"';
          index += 1;
        } else quoted = false;
      } else field += character;
    } else if (character === '"' && field === "") quoted = true;
    else if (character === ",") {
      row.push(field);
      field = "";
    } else if (character === "\n") {
      row.push(field);
      rows.push(row);
      row = [];
      field = "";
    } else field += character;
  }
  if (quoted) fail("dynamic-records.csv contains an unterminated quote");
  if (field !== "" || row.length > 0) {
    row.push(field);
    rows.push(row);
  }
  return rows;
};

if (data.schemaVersion !== 1) fail("schemaVersion must be 1");
if (data.verificationStatus !== "DYNAMIC_DEMO_NON_CONFIRMATORY") {
  fail("verificationStatus must be DYNAMIC_DEMO_NON_CONFIRMATORY");
}
if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(data.sourceCommit ?? "")) {
  fail("sourceCommit must be a full lowercase Git object ID");
}
if (typeof data.generatedAt !== "string" || !Number.isFinite(Date.parse(data.generatedAt))) {
  fail("generatedAt must be an ISO timestamp");
}
const protocol = object(data.protocol, "protocol");
text(protocol.id, "protocol.id");
for (const key of [
  "timeStepS", "replanIntervalS", "cruiseSpeedMps", "maxTimeS", "resolutionM",
]) positive(protocol[key], `protocol.${key}`);
positiveInteger(protocol.maxExpansions, "protocol.maxExpansions");
if (
  protocol.timeStepS !== 1 || protocol.replanIntervalS !== 4 ||
  protocol.cruiseSpeedMps !== 8 || protocol.maxTimeS !== 180 ||
  protocol.resolutionM !== 4 || protocol.maxExpansions !== 120_000
) fail("protocol does not match the declared deterministic replay configuration");

const planners = uniqueObjects(data.planners, "planners");
if (
  planners.length !== declaredPlanners.length ||
  planners.some((planner, index) =>
    planner.id !== declaredPlanners[index][0] || planner.label !== declaredPlanners[index][1])
) {
  fail("planners must contain the declared three-planner comparison in protocol order");
}
const plannerIds = new Set(declaredPlanners.map(([id]) => id));
const scenarios = array(data.scenarios, "scenarios");
if (scenarios.length === 0) fail("at least one scenario is required");
const scenarioIds = new Set();
const fingerprints = new Set();
let runCount = 0;
let frameCount = 0;
const runIds = new Set();
const auditedRuns = [];

for (const [sceneIndex, rawScene] of scenarios.entries()) {
  const sceneLabel = `scenarios[${sceneIndex}]`;
  const scene = object(rawScene, sceneLabel);
  const sceneId = text(scene.id, `${sceneLabel}.id`);
  if (scenarioIds.has(sceneId)) fail(`duplicate scenario ID ${sceneId}`);
  scenarioIds.add(sceneId);
  text(scene.label, `${sceneLabel}.label`);
  text(scene.description, `${sceneLabel}.description`);
  const fingerprint = text(scene.fingerprint, `${sceneLabel}.fingerprint`);
  if (!/^sha256:[0-9a-f]{64}$/.test(fingerprint) || fingerprints.has(fingerprint)) {
    fail(`${sceneLabel}.fingerprint must be a unique SHA-256 digest`);
  }
  fingerprints.add(fingerprint);

  const rawBounds = object(scene.bounds, `${sceneLabel}.bounds`);
  const bounds = {
    min: vector(rawBounds.min, 3, `${sceneLabel}.bounds.min`),
    max: vector(rawBounds.max, 3, `${sceneLabel}.bounds.max`),
  };
  if (bounds.min.some((coordinate, index) => coordinate >= bounds.max[index])) {
    fail(`${sceneLabel}.bounds must have positive extent`);
  }
  const start = vector(scene.start, 3, `${sceneLabel}.start`);
  const goal = vector(scene.goal, 3, `${sceneLabel}.goal`);
  if (!inBounds(start, bounds) || !inBounds(goal, bounds) || samePoint(start, goal)) {
    fail(`${sceneLabel} endpoints are invalid`);
  }
  const constraints = object(scene.constraints, `${sceneLabel}.constraints`);
  const vehicleRadius = nonNegative(
    constraints.vehicleRadiusM,
    `${sceneLabel}.constraints.vehicleRadiusM`,
  );
  const safetyMargin = nonNegative(
    constraints.safetyMarginM,
    `${sceneLabel}.constraints.safetyMarginM`,
  );
  if (vehicleRadius + safetyMargin <= 0) fail(`${sceneLabel}.constraints are invalid`);

  const obstacleIds = [];
  for (const [index, building] of uniqueObjects(scene.buildings, `${sceneLabel}.buildings`).entries()) {
    const min = vector(building.min, 3, `${sceneLabel}.buildings[${index}].min`);
    const max = vector(building.max, 3, `${sceneLabel}.buildings[${index}].max`);
    if (
      min.some((coordinate, axis) => coordinate >= max[axis]) ||
      !inBounds(min, bounds) || !inBounds(max, bounds)
    ) fail(`${sceneLabel}.buildings[${index}] is invalid`);
    obstacleIds.push(building.id);
  }
  const parseZone = (rawZone, label) => {
    const zone = object(rawZone, label);
    const center = vector(zone.center, 2, `${label}.center`);
    const radius = positive(zone.radiusM, `${label}.radiusM`);
    const zMin = finite(zone.zMinM, `${label}.zMinM`);
    const zMax = finite(zone.zMaxM, `${label}.zMaxM`);
    if (
      zMin >= zMax || center[0] - radius < bounds.min[0] - tolerance ||
      center[0] + radius > bounds.max[0] + tolerance ||
      center[1] - radius < bounds.min[1] - tolerance ||
      center[1] + radius > bounds.max[1] + tolerance ||
      zMin < bounds.min[2] - tolerance || zMax > bounds.max[2] + tolerance
    ) fail(`${label} is outside bounds or has invalid extent`);
    return zone;
  };
  for (const [index, zone] of uniqueObjects(
    scene.staticNoFlyZones,
    `${sceneLabel}.staticNoFlyZones`,
  ).entries()) {
    parseZone(zone, `${sceneLabel}.staticNoFlyZones[${index}]`);
    obstacleIds.push(zone.id);
  }
  const temporaryZones = uniqueObjects(
    scene.temporaryNoFlyZones,
    `${sceneLabel}.temporaryNoFlyZones`,
  ).map((zone, index) => {
    const label = `${sceneLabel}.temporaryNoFlyZones[${index}]`;
    parseZone(zone, label);
    const activeFromS = nonNegative(zone.activeFromS, `${label}.activeFromS`);
    const activeUntilS = positive(zone.activeUntilS, `${label}.activeUntilS`);
    if (activeFromS >= activeUntilS) fail(`${label} has an invalid half-open active interval`);
    obstacleIds.push(zone.id);
    return zone;
  });
  const temporaryIds = new Set(temporaryZones.map((zone) => zone.id));

  const movingSpheres = uniqueObjects(scene.movingSpheres, `${sceneLabel}.movingSpheres`).map(
    (sphere, index) => {
      const label = `${sceneLabel}.movingSpheres[${index}]`;
      positive(sphere.radiusM, `${label}.radiusM`);
      const keyframes = array(sphere.keyframes, `${label}.keyframes`).map((rawKeyframe, keyIndex) => {
        const keyframe = object(rawKeyframe, `${label}.keyframes[${keyIndex}]`);
        const timeS = nonNegative(keyframe.timeS, `${label}.keyframes[${keyIndex}].timeS`);
        const position = vector(
          keyframe.position,
          3,
          `${label}.keyframes[${keyIndex}].position`,
        );
        if (!inBounds(position, bounds)) fail(`${label}.keyframes[${keyIndex}] is outside bounds`);
        return { timeS, position };
      });
      if (keyframes.length < 2) fail(`${label} requires at least two keyframes`);
      for (let keyIndex = 1; keyIndex < keyframes.length; keyIndex += 1) {
        if (keyframes[keyIndex].timeS <= keyframes[keyIndex - 1].timeS) {
          fail(`${label}.keyframes must be strictly increasing`);
        }
      }
      obstacleIds.push(sphere.id);
      return { ...sphere, keyframes };
    },
  );
  if (new Set(obstacleIds).size !== obstacleIds.length) {
    fail(`${sceneLabel} obstacle IDs must be unique across geometry types`);
  }
  const movingById = new Map(movingSpheres.map((sphere) => [sphere.id, sphere]));
  const movingPosition = (sphere, timeS) => {
    if (timeS <= sphere.keyframes[0].timeS) return sphere.keyframes[0].position;
    if (timeS >= sphere.keyframes.at(-1).timeS) return sphere.keyframes.at(-1).position;
    for (let index = 1; index < sphere.keyframes.length; index += 1) {
      const left = sphere.keyframes[index - 1];
      const right = sphere.keyframes[index];
      if (timeS <= right.timeS) {
        const fraction = (timeS - left.timeS) / (right.timeS - left.timeS);
        return left.position.map(
          (coordinate, axis) => coordinate + (right.position[axis] - coordinate) * fraction,
        );
      }
    }
    return sphere.keyframes.at(-1).position;
  };

  const runs = array(scene.runs, `${sceneLabel}.runs`);
  const seenRunPlanners = new Set();
  for (const [runIndex, rawRun] of runs.entries()) {
    const runLabel = `${sceneLabel}.runs[${runIndex}]`;
    const run = object(rawRun, runLabel);
    const runId = text(run.runId, `${runLabel}.runId`);
    if (!/^sha256:[0-9a-f]{64}$/.test(runId) || runIds.has(runId)) {
      fail(`${runLabel}.runId must be a unique SHA-256 digest`);
    }
    runIds.add(runId);
    const plannerId = text(run.plannerId, `${runLabel}.plannerId`);
    if (!plannerIds.has(plannerId) || seenRunPlanners.has(plannerId)) {
      fail(`${runLabel}.plannerId is unknown or duplicated`);
    }
    seenRunPlanners.add(plannerId);
    if (!["success", "no-path", "timeout", "invalid"].includes(run.status)) {
      fail(`${runLabel}.status is unsupported`);
    }
    const failureReason = nullableText(run.failureReason, `${runLabel}.failureReason`);
    if ((run.status === "success") !== (failureReason === null)) {
      fail(`${runLabel}.failureReason disagrees with status`);
    }
    const parameters = object(run.parameters, `${runLabel}.parameters`);
    if (Object.keys(parameters).length === 0) fail(`${runLabel}.parameters cannot be empty`);
    for (const [key, value] of Object.entries(parameters)) finite(value, `${runLabel}.parameters.${key}`);

    const metrics = object(run.metrics, `${runLabel}.metrics`);
    const success = bool(metrics.success, `${runLabel}.metrics.success`);
    const metricFailure = nullableText(metrics.failureReason, `${runLabel}.metrics.failureReason`);
    const completionTime = metrics.completionTimeS === null
      ? null
      : nonNegative(metrics.completionTimeS, `${runLabel}.metrics.completionTimeS`);
    const pathExcess = metrics.pathExcessPct === null
      ? null
      : finite(metrics.pathExcessPct, `${runLabel}.metrics.pathExcessPct`);
    const executedLength = nonNegative(
      metrics.executedPathLengthM,
      `${runLabel}.metrics.executedPathLengthM`,
    );
    const directDistance = positive(metrics.directDistanceM, `${runLabel}.metrics.directDistanceM`);
    if (!new Set(["expanded-nodes", "queue-pops"]).has(metrics.workUnit)) {
      fail(`${runLabel}.metrics.workUnit is unsupported`);
    }
    const expectedWorkUnit = plannerId === "dstar-lite-3d" ? "queue-pops" : "expanded-nodes";
    if (metrics.workUnit !== expectedWorkUnit) {
      fail(`${runLabel}.metrics.workUnit does not match the planner`);
    }
    for (const key of [
      "replans", "failedReplans", "holds", "safetyGateActivations", "collisionCount",
      "totalPlanningWork", "totalChangedEdges",
    ]) nonNegativeInteger(metrics[key], `${runLabel}.metrics.${key}`);
    if (metrics.deadlineMisses !== undefined) {
      nonNegativeInteger(metrics.deadlineMisses, `${runLabel}.metrics.deadlineMisses`);
    }
    if (metrics.minimumClearanceM !== undefined && metrics.minimumClearanceM !== null) {
      nonNegative(metrics.minimumClearanceM, `${runLabel}.metrics.minimumClearanceM`);
    }
    if (
      success !== (run.status === "success") || metricFailure !== failureReason ||
      metrics.failedReplans > metrics.replans || metrics.safetyGateActivations > metrics.replans
    ) fail(`${runLabel}.metrics status or replanning counts are inconsistent`);

    const frames = array(run.frames, `${runLabel}.frames`);
    if (frames.length === 0) fail(`${runLabel}.frames cannot be empty`);
    let previousTime = -1;
    let previousExecuted = [];
    let lastFrame = null;
    let workTotal = 0;
    let changedTotal = 0;
    let holdCount = 0;
    let replanFrameCount = 0;
    for (const [frameIndex, rawFrame] of frames.entries()) {
      const frameLabel = `${runLabel}.frames[${frameIndex}]`;
      const frame = object(rawFrame, frameLabel);
      const timeS = nonNegative(frame.timeS, `${frameLabel}.timeS`);
      if ((frameIndex === 0 && timeS !== 0) || timeS <= previousTime) {
        fail(`${runLabel}.frames must start at zero and be strictly increasing`);
      }
      previousTime = timeS;
      const vehicle = vector(frame.vehicle, 3, `${frameLabel}.vehicle`);
      if (!inBounds(vehicle, bounds)) fail(`${frameLabel}.vehicle is outside bounds`);
      const parsePath = (rawPath, label) => array(rawPath, label).map((point, pointIndex) => {
        const parsed = vector(point, 3, `${label}[${pointIndex}]`);
        if (!inBounds(parsed, bounds)) fail(`${label}[${pointIndex}] is outside bounds`);
        return parsed;
      });
      const planned = parsePath(frame.path, `${frameLabel}.path`);
      const executed = parsePath(frame.executedPath, `${frameLabel}.executedPath`);
      if (
        planned.length > 0 &&
        (!samePoint(planned[0], vehicle) || !samePoint(planned.at(-1), goal))
      ) fail(`${frameLabel}.path endpoints are inconsistent`);
      if (
        executed.length === 0 || !samePoint(executed[0], start) ||
        !samePoint(executed.at(-1), vehicle) || previousExecuted.length > executed.length ||
        previousExecuted.some((point, index) => !samePoint(point, executed[index]))
      ) fail(`${frameLabel}.executedPath endpoints or prefix are inconsistent`);
      previousExecuted = executed;

      const activeIds = array(frame.activeTemporaryZoneIds, `${frameLabel}.activeTemporaryZoneIds`);
      const expectedActive = new Set(temporaryZones
        .filter((zone) => zone.activeFromS <= timeS && timeS < zone.activeUntilS)
        .map((zone) => zone.id));
      if (
        new Set(activeIds).size !== activeIds.length || activeIds.length !== expectedActive.size ||
        activeIds.some((id) => !temporaryIds.has(id) || !expectedActive.has(id))
      ) fail(`${frameLabel}.activeTemporaryZoneIds disagrees with schedules`);

      const states = array(frame.movingSpheres, `${frameLabel}.movingSpheres`);
      const stateIds = new Set();
      for (const [stateIndex, rawState] of states.entries()) {
        const stateLabel = `${frameLabel}.movingSpheres[${stateIndex}]`;
        const state = object(rawState, stateLabel);
        const id = text(state.id, `${stateLabel}.id`);
        const definition = movingById.get(id);
        if (!definition || stateIds.has(id)) fail(`${stateLabel}.id is invalid`);
        stateIds.add(id);
        const position = vector(state.position, 3, `${stateLabel}.position`);
        if (!samePoint(position, movingPosition(definition, timeS))) {
          fail(`${stateLabel}.position disagrees with keyframes`);
        }
        if (!sameNumber(positive(state.radiusM, `${stateLabel}.radiusM`), definition.radiusM)) {
          fail(`${stateLabel}.radiusM disagrees with its definition`);
        }
      }
      if (stateIds.size !== movingById.size) fail(`${frameLabel} must report every moving sphere`);

      if (frame.event !== null) {
        const event = object(frame.event, `${frameLabel}.event`);
        if (![
          "none", "temporary-zone-activated", "temporary-zone-deactivated", "replan",
          "wait", "goal-reached", "no-path",
        ].includes(event.kind)) fail(`${frameLabel}.event.kind is unsupported`);
        text(event.label, `${frameLabel}.event.label`);
        if (event.subjectId !== null) text(event.subjectId, `${frameLabel}.event.subjectId`);
        if (event.kind === "wait") holdCount += 1;
        if (event.kind === "goal-reached" && !samePoint(vehicle, goal)) {
          fail(`${frameLabel}.event goal does not match scenario goal`);
        }
      }
      const replanned = bool(frame.replanned, `${frameLabel}.replanned`);
      const replanReason = nullableText(frame.replanReason, `${frameLabel}.replanReason`);
      const plannerSuccess = frame.plannerSuccess === null
        ? null
        : bool(frame.plannerSuccess, `${frameLabel}.plannerSuccess`);
      const planningTime = frame.planningTimeMs === null
        ? null
        : nonNegative(frame.planningTimeMs, `${frameLabel}.planningTimeMs`);
      const workUsed = nonNegativeInteger(frame.workUsed, `${frameLabel}.workUsed`);
      const changedEdges = nonNegativeInteger(frame.changedEdges, `${frameLabel}.changedEdges`);
      if (replanned) {
        if (replanReason === null || plannerSuccess === null) {
          fail(`${frameLabel} replanning outcome is incomplete`);
        }
        replanFrameCount += 1;
      } else if (
        replanReason !== null || plannerSuccess !== null || planningTime !== null ||
        workUsed !== 0 || changedEdges !== 0
      ) fail(`${frameLabel} non-replanning frame contains planner outcomes`);
      workTotal += workUsed;
      changedTotal += changedEdges;
      lastFrame = { ...frame, timeS, vehicle, executedPath: executed };
      frameCount += 1;
    }

    if (success) {
      if (
        completionTime === null || pathExcess === null ||
        lastFrame.event?.kind !== "goal-reached" || !samePoint(lastFrame.vehicle, goal) ||
        !sameNumber(completionTime, lastFrame.timeS)
      ) fail(`${runLabel} successful outcome is incomplete`);
    } else if (completionTime !== null || pathExcess !== null) {
      fail(`${runLabel} failed outcome cannot define completion time or path excess`);
    }
    const directFromGeometry = Math.hypot(...goal.map((coordinate, index) => coordinate - start[index]));
    const executedFromGeometry = length(lastFrame.executedPath);
    if (
      !sameNumber(directDistance, directFromGeometry) ||
      !sameNumber(executedLength, executedFromGeometry) ||
      (pathExcess !== null && !sameNumber(pathExcess, (executedLength / directDistance - 1) * 100))
    ) fail(`${runLabel}.metrics path values disagree with geometry`);
    if (
      metrics.holds !== holdCount || metrics.totalPlanningWork !== workTotal ||
      metrics.totalChangedEdges !== changedTotal || metrics.replans < replanFrameCount
    ) fail(`${runLabel}.metrics do not aggregate the recorded frames`);
    auditedRuns.push({ scene, run, metrics, frameCount: frames.length });
    runCount += 1;
  }
  if (
    runs.length !== plannerIds.size || seenRunPlanners.size !== plannerIds.size ||
    [...plannerIds].some((id) => !seenRunPlanners.has(id))
  ) fail(`${sceneLabel} must contain exactly one run per planner`);
}

const downloads = object(data.downloads, "downloads");
if (
  Object.keys(downloads).length !== 2 || !("recordsCsv" in downloads) ||
  !("scenarioManifest" in downloads)
) fail("downloads must contain exactly recordsCsv and scenarioManifest");
const artifactBytes = new Map();
for (const [key, expectedPath] of [
  ["recordsCsv", "dynamic-records.csv"],
  ["scenarioManifest", "dynamic-scenario-manifest.json"],
]) {
  const reference = object(downloads[key], `downloads.${key}`);
  if (
    reference.path !== expectedPath || !/^sha256:[0-9a-f]{64}$/.test(reference.sha256 ?? "") ||
    !Number.isInteger(reference.bytes) || reference.bytes <= 0
  ) fail(`downloads.${key} is invalid`);
  const bytes = await readFile(new URL(reference.path, source));
  artifactBytes.set(key, bytes);
  if (bytes.byteLength !== reference.bytes) fail(`downloads.${key} byte size does not match`);
  const digest = `sha256:${createHash("sha256").update(bytes).digest("hex")}`;
  if (digest !== reference.sha256) fail(`downloads.${key} digest does not match`);
}

const manifest = JSON.parse(new TextDecoder().decode(artifactBytes.get("scenarioManifest")));
if (
  manifest.schemaVersion !== 1 || manifest.sourceCommit !== data.sourceCommit ||
  manifest.generatedAt !== data.generatedAt || manifest.protocolId !== data.protocol.id ||
  manifest.scenarioCount !== scenarios.length || manifest.runCount !== runCount ||
  !Array.isArray(manifest.scenarios) || manifest.scenarios.length !== scenarios.length
) fail("dynamic-scenario-manifest.json provenance or counts disagree with the bundle");
for (const [index, scene] of scenarios.entries()) {
  const entry = manifest.scenarios[index];
  if (
    entry?.id !== scene.id || entry?.label !== scene.label ||
    entry?.fingerprint !== scene.fingerprint || entry?.selected !== true
  ) fail(`dynamic-scenario-manifest.json scenario ${index} disagrees with the bundle`);
}

const csvText = new TextDecoder().decode(artifactBytes.get("recordsCsv"));
if (csvText.includes("\r")) fail("dynamic-records.csv must use LF line endings");
const csvRows = parseCsv(csvText);
const expectedHeader = [
  "source_commit", "protocol_id", "generated_at", "run_id", "scenario_id",
  "scenario_fingerprint", "planner_id", "status", "failure_reason", "success",
  "completion_time_s", "executed_path_length_m", "direct_distance_m", "path_excess_pct",
  "replans", "failed_replans", "holds", "safety_gate_activations", "collision_count",
  "total_planning_work", "work_unit", "total_changed_edges", "deadline_misses",
  "frame_count", "parameters_json",
];
if (
  csvRows.length !== runCount + 1 ||
  csvRows[0].length !== expectedHeader.length ||
  csvRows[0].some((column, index) => column !== expectedHeader[index])
) fail("dynamic-records.csv header or row count is invalid");
for (const [index, audit] of auditedRuns.entries()) {
  const values = csvRows[index + 1];
  const row = Object.fromEntries(expectedHeader.map((column, columnIndex) => [column, values[columnIndex]]));
  if (
    row.source_commit !== data.sourceCommit || row.protocol_id !== data.protocol.id ||
    row.generated_at !== data.generatedAt || row.run_id !== audit.run.runId ||
    row.scenario_id !== audit.scene.id || row.scenario_fingerprint !== audit.scene.fingerprint ||
    row.planner_id !== audit.run.plannerId || row.status !== audit.run.status ||
    row.success !== String(audit.metrics.success) || row.work_unit !== audit.metrics.workUnit ||
    Number(row.total_planning_work) !== audit.metrics.totalPlanningWork ||
    Number(row.frame_count) !== audit.frameCount
  ) fail(`dynamic-records.csv row ${index + 2} disagrees with the bundle`);
  const csvParameters = JSON.parse(row.parameters_json);
  if (JSON.stringify(csvParameters) !== JSON.stringify(audit.run.parameters)) {
    fail(`dynamic-records.csv row ${index + 2} parameters disagree with the bundle`);
  }
}

console.log(
  `validated ${scenarios.length} dynamic scenarios, ${runCount} runs, and ${frameCount} frames`,
);
