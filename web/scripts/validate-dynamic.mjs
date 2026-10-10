import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { readStudyData } from './study-reader.mjs';

const input = globalThis.process?.argv?.[2];
const source = input
  ? pathToFileURL(resolve(input))
  : new URL("../public/dynamic-data.json", import.meta.url);
const data = await readStudyData(source);
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
if (typeof data.generatedAt !== "string" || !Number.isFinite(Date.parse(data.generatedAt))) {
  fail("generatedAt must be an ISO timestamp");
}
const protocol = object(data.protocol, "protocol");
text(protocol.id, "protocol.id");
for (const key of [
  "timeStepS", "replanIntervalS", "cruiseSpeedMps", "maxTimeS", "resolutionM",
]) positive(protocol[key], `protocol.${key}`);
positiveInteger(protocol.maxExpansions, "protocol.maxExpansions");
const replayProtocols = {
  "dynamic-replanning-demo-v1": [1, 4, 8, 180, 4, 120_000],
  "dynamic-replanning-v1": [1, 4, 8, 180, 4, 120_000],
  "manhattan-reactive-demo-v5": [2, 10, 14, 900, 50, 20_000],
};
const MANHATTAN_PROTOCOL_ID = "manhattan-reactive-demo-v5";
const expectedProtocol = replayProtocols[protocol.id];
const actualProtocol = [protocol.timeStepS, protocol.replanIntervalS, protocol.cruiseSpeedMps,
  protocol.maxTimeS, protocol.resolutionM, protocol.maxExpansions];
if (!expectedProtocol || actualProtocol.some((value, index) => value !== expectedProtocol[index])) {
  fail("protocol does not match the declared deterministic replay configuration");
}
if (protocol.verticalCostScale !== undefined) {
  if (positive(protocol.verticalCostScale, "verticalCostScale") < 1) fail("verticalCostScale must be at least one");
  positive(protocol.maxClimbRateMps, "maxClimbRateMps");
}
if ((protocol.id === MANHATTAN_PROTOCOL_ID) && protocol.pathShortcut !== 1) {
  fail("protocol.pathShortcut must be 1 for current Manhattan replay");
}
if ((protocol.id === MANHATTAN_PROTOCOL_ID) && protocol.preserveAltitude !== 1) {
  fail("protocol.preserveAltitude must be 1 for current Manhattan replay");
}
if ((protocol.id === MANHATTAN_PROTOCOL_ID) &&
    (protocol.smoothTurns !== undefined || protocol.turnScaleM !== undefined || protocol.curveSampleSpacingM !== undefined)) {
  if (protocol.smoothTurns !== 1) fail("protocol.smoothTurns must be 1 for local B-spline curves");
  positive(protocol.turnScaleM, "protocol.turnScaleM");
  positive(protocol.curveSampleSpacingM, "protocol.curveSampleSpacingM");
}
if (protocol.id === "manhattan-reactive-demo-v5" && (protocol.curveDimensions !== 3 || protocol.smoothTurns !== 1))
  fail("protocol.curveDimensions must be 3 with smoothTurns for Manhattan v5");
if ((protocol.id === MANHATTAN_PROTOCOL_ID)) {
  if (!/^local-snapshot:sha256:[0-9a-f]{64}$/.test(data.sourceCommit ?? "")) {
    fail("Manhattan sourceCommit must identify a local source snapshot");
  }
  const provenance = object(data.sourceProvenance, "sourceProvenance");
  if (provenance.kind !== "local-snapshot" || provenance.sha256 !== data.sourceCommit.slice(15)) {
    fail("sourceProvenance disagrees with the local source snapshot");
  }
  const files = array(provenance.files, "sourceProvenance.files");
  if (files.length === 0) fail("sourceProvenance.files cannot be empty");
  const paths = new Set();
  for (const [index, rawFile] of files.entries()) {
    const file = object(rawFile, `sourceProvenance.files[${index}]`);
    const path = text(file.path, `sourceProvenance.files[${index}].path`);
    if (path.startsWith("/") || path.split("/").includes("..") || paths.has(path)
      || !/^sha256:[0-9a-f]{64}$/.test(file.sha256 ?? "")) {
      fail("sourceProvenance.files must contain unique relative paths and SHA-256 digests");
    }
    paths.add(path);
  }
  const digest = `sha256:${createHash("sha256").update(JSON.stringify(files)).digest("hex")}`;
  if (digest !== provenance.sha256) fail("source snapshot manifest digest disagrees");
} else if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(data.sourceCommit ?? "")) {
  fail("sourceCommit must be a full lowercase Git object ID");
}

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
if ((protocol.id === MANHATTAN_PROTOCOL_ID) && scenarios.length !== 8) {
  fail("the Manhattan protocol requires the eight declared simulation missions");
}
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
  if ((protocol.id === MANHATTAN_PROTOCOL_ID)) {
    const width = bounds.max[0] - bounds.min[0], depth = bounds.max[1] - bounds.min[1];
    const regionId = scene.city?.planningRegion?.id;
    const completeExtent = regionId === "manhattan-south-expanded-v4" &&
      width >= 4400 && width <= 4800 && depth >= 7400 && depth <= 8000;
    if (scene.city?.sourceKind !== "nyc-open-data" || scene.city.collisionModel !== "conservative-aabb"
      || !/^(?:sha256:)?[0-9a-f]{64}$/.test(scene.city.sourceSha256 ?? "")
      || !scene.city.sourceUrl?.startsWith("https://services6.arcgis.com/")
      || !Array.isArray(scene.buildings) || scene.buildings.length < 1000
      || scene.city.buildingCount !== scene.buildings.length
      || !completeExtent) {
      fail(`${sceneLabel} must retain the full physical Manhattan district and source provenance`);
    }
  }
  for (const [index, building] of uniqueObjects(scene.buildings, `${sceneLabel}.buildings`).entries()) {
    const min = vector(building.min, 3, `${sceneLabel}.buildings[${index}].min`);
    const max = vector(building.max, 3, `${sceneLabel}.buildings[${index}].max`);
    if (
      min.some((coordinate, axis) => coordinate >= max[axis]) ||
      !inBounds(min, bounds) || !inBounds(max, bounds)
    ) fail(`${sceneLabel}.buildings[${index}] is invalid`);
    if (building.footprint !== undefined || (protocol.id === MANHATTAN_PROTOCOL_ID)) {
      const footprint = array(building.footprint, `${sceneLabel}.buildings[${index}].footprint`);
      if (footprint.length === 0) fail(`${sceneLabel}.buildings[${index}] requires an exterior ring`);
      for (const [ringIndex, rawRing] of footprint.entries()) {
        const ring = array(rawRing, `${sceneLabel}.buildings[${index}].footprint[${ringIndex}]`);
        if (ring.length < 4 || ring.some((point) => !Array.isArray(point) || point.length !== 2
          || point.some((coordinate, axis) => !Number.isFinite(coordinate)
            || coordinate < min[axis] - 1e-5 || coordinate > max[axis] + 1e-5))
          || ring[0].some((value, axis) => value !== ring.at(-1)[axis])) {
          fail(`${sceneLabel}.buildings[${index}] has an invalid source footprint ring`);
        }
      }
    }
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
    if (protocol.verticalCostScale !== undefined &&
        (parameters.verticalCostScale !== protocol.verticalCostScale || parameters.maxClimbRateMps !== protocol.maxClimbRateMps || !Array.isArray(run.executionTimedPath))) {
      fail(`${runLabel} flight-aware parameters and exact clock must match the protocol`);
    }
    if ((protocol.id === MANHATTAN_PROTOCOL_ID) && parameters.pathShortcut !== 1) {
      fail(`${runLabel}.parameters.pathShortcut must be 1 for current Manhattan replay`);
    }
    if ((protocol.id === MANHATTAN_PROTOCOL_ID) && parameters.preserveAltitude !== 1) {
      fail(`${runLabel}.parameters.preserveAltitude must be 1 for current Manhattan replay`);
    }
    if (protocol.curveDimensions === 3 && parameters.curveDimensions !== 3)
      fail(`${runLabel}.parameters.curveDimensions must match protocol`);
    if ((protocol.id === MANHATTAN_PROTOCOL_ID) && protocol.smoothTurns === 1 &&
        (parameters.smoothTurns !== 1 || parameters.turnScaleM !== protocol.turnScaleM ||
         parameters.curveSampleSpacingM !== protocol.curveSampleSpacingM)) {
      fail(`${runLabel}.parameters must match the local B-spline curve protocol`);
    }

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
      const curvedEscape = parameters.verticalCostScale !== undefined && Array.isArray(run.executionTimedPath);
      const localEscape = protocol.horizontalEscape === 1 && parameters.horizontalEscape === 1 &&
        planned.length >= 2 && frame.replanReason === "safety-gate" &&
        planned.every(p => Math.abs(p[2] - planned[0][2]) <= 1e-5) &&
        (curvedEscape ? length(planned) <= protocol.cruiseSpeedMps * protocol.timeStepS * 2 + 1e-5 :
          planned.length === 2 && length(planned) <= protocol.cruiseSpeedMps * protocol.timeStepS + 1e-5);
      if (
        planned.length > 0 &&
        (!samePoint(planned[0], vehicle) || (!localEscape && ![goal, ...(scene.mission?.taskPoints ?? []).map(task => task.position)].some(target => samePoint(planned.at(-1), target))))
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
    if (run.executionTimedPath != null) {
      const knots = array(run.executionTimedPath, `${runLabel}.executionTimedPath`).map((p, i) => ({
        time: nonNegative(p.time, `executionTimedPath[${i}].time`),
        position: vector(p.position, 3, `executionTimedPath[${i}].position`), action: p.action,
      }));
      if (!knots.length || knots[0].time !== 0 || knots[0].action !== "start" ||
          !samePoint(knots[0].position, start) || !samePoint(knots.at(-1).position, lastFrame.vehicle) ||
          !sameNumber(knots.at(-1).time, lastFrame.timeS) || !sameNumber(length(knots.map(p => p.position)), executedFromGeometry)) fail("Exact execution clock disagrees with telemetry");
      for (let i = 1; i < knots.length; i++) {
        const a = knots[i - 1], b = knots[i], dt = b.time - a.time;
        const span = Math.hypot(...b.position.map((v, axis) => v - a.position[axis]));
        if (dt <= 0 || !["move", "wait"].includes(b.action) || (b.action === "wait") !== (span <= 1e-9) ||
            span > parameters.cruiseSpeedMps * dt + 1e-6 ||
            Math.abs(b.position[2] - a.position[2]) > parameters.maxClimbRateMps * dt + 1e-6) fail("Invalid climb-constrained execution segment");
      }
    } else if (parameters.maxClimbRateMps !== undefined) fail("Missing exact execution clock");
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
