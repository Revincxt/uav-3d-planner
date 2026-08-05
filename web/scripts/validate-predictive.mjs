import { createHash } from "node:crypto";
import { readFile, stat } from "node:fs/promises";
import { dirname, resolve } from "node:path";

const TOLERANCE = 1e-6;
const inputPath = resolve(process.cwd(), process.argv[2] ?? "public/predictive-data.json");

function fail(message) {
  throw new Error(`predictive-data.json: ${message}`);
}

function object(value, label) {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    fail(`${label} must be an object`);
  }
  return value;
}

function list(value, label) {
  if (!Array.isArray(value)) fail(`${label} must be an array`);
  return value;
}

function text(value, label) {
  if (typeof value !== "string" || value.trim() === "") fail(`${label} must be a string`);
  return value;
}

function nullableText(value, label) {
  return value === null ? null : text(value, label);
}

function finite(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) fail(`${label} must be finite`);
  return value;
}

function nonNegative(value, label) {
  const parsed = finite(value, label);
  if (parsed < 0) fail(`${label} cannot be negative`);
  return parsed;
}

function positive(value, label) {
  const parsed = finite(value, label);
  if (parsed <= 0) fail(`${label} must be positive`);
  return parsed;
}

function integer(value, label) {
  const parsed = nonNegative(value, label);
  if (!Number.isInteger(parsed)) fail(`${label} must be an integer`);
  return parsed;
}

function positiveInteger(value, label) {
  const parsed = positive(value, label);
  if (!Number.isInteger(parsed)) fail(`${label} must be an integer`);
  return parsed;
}

function bool(value, label) {
  if (typeof value !== "boolean") fail(`${label} must be boolean`);
  return value;
}

function vec3(value, label) {
  const coordinates = list(value, label);
  if (coordinates.length !== 3) fail(`${label} must contain three coordinates`);
  return coordinates.map((coordinate, index) => finite(coordinate, `${label}[${index}]`));
}

function vec2(value, label) {
  const coordinates = list(value, label);
  if (coordinates.length !== 2) fail(`${label} must contain two coordinates`);
  return coordinates.map((coordinate, index) => finite(coordinate, `${label}[${index}]`));
}

function sameNumber(left, right) {
  return Math.abs(left - right) <= TOLERANCE * Math.max(1, Math.abs(left), Math.abs(right));
}

function samePoint(left, right) {
  return left.every((coordinate, index) => sameNumber(coordinate, right[index]));
}

function inBounds(point, bounds) {
  return point.every(
    (coordinate, index) =>
      coordinate >= bounds.min[index] - TOLERANCE &&
      coordinate <= bounds.max[index] + TOLERANCE,
  );
}

function pathLength(points) {
  let total = 0;
  for (let index = 1; index < points.length; index += 1) {
    total += Math.hypot(
      points[index][0] - points[index - 1][0],
      points[index][1] - points[index - 1][1],
      points[index][2] - points[index - 1][2],
    );
  }
  return total;
}

function uniqueIdObjects(value, label) {
  const entries = list(value, label).map((entry, index) => object(entry, `${label}[${index}]`));
  const ids = new Set();
  entries.forEach((entry, index) => {
    const id = text(entry.id, `${label}[${index}].id`);
    if (ids.has(id)) fail(`${label} contains duplicate ID ${id}`);
    ids.add(id);
  });
  return entries;
}

function parseBounds(value, label) {
  const entry = object(value, label);
  const bounds = { min: vec3(entry.min, `${label}.min`), max: vec3(entry.max, `${label}.max`) };
  if (bounds.min.some((coordinate, index) => coordinate >= bounds.max[index])) {
    fail(`${label}.min must be strictly below max`);
  }
  return bounds;
}

function parseProtocol(value) {
  const entry = object(value, "protocol");
  const protocol = {
    id: text(entry.id, "protocol.id"),
    timeStepS: positive(entry.timeStepS, "protocol.timeStepS"),
    cruiseSpeedMps: positive(entry.cruiseSpeedMps, "protocol.cruiseSpeedMps"),
    maxTimeS: positive(entry.maxTimeS, "protocol.maxTimeS"),
    resolutionM: positive(entry.resolutionM, "protocol.resolutionM"),
    timeResolutionS: positive(entry.timeResolutionS, "protocol.timeResolutionS"),
    planningHorizonS: positive(entry.planningHorizonS, "protocol.planningHorizonS"),
    predictionHorizonS: positive(entry.predictionHorizonS, "protocol.predictionHorizonS"),
    reactiveMaxWorkPerReplan: positiveInteger(
      entry.reactiveMaxWorkPerReplan,
      "protocol.reactiveMaxWorkPerReplan",
    ),
    predictiveMaxExpandedStatesPerMission: positiveInteger(
      entry.predictiveMaxExpandedStatesPerMission,
      "protocol.predictiveMaxExpandedStatesPerMission",
    ),
  };
  if (protocol.timeResolutionS > protocol.planningHorizonS) {
    fail("protocol.timeResolutionS cannot exceed planningHorizonS");
  }
  if (protocol.predictionHorizonS > protocol.planningHorizonS) {
    fail("protocol.predictionHorizonS cannot exceed planningHorizonS");
  }
  if (protocol.planningHorizonS > protocol.maxTimeS) {
    fail("protocol.planningHorizonS cannot exceed maxTimeS");
  }
  return protocol;
}

function staticZone(value, label, bounds) {
  const entry = object(value, label);
  const zone = {
    id: text(entry.id, `${label}.id`),
    center: vec2(entry.center, `${label}.center`),
    radiusM: positive(entry.radiusM, `${label}.radiusM`),
    zMinM: finite(entry.zMinM, `${label}.zMinM`),
    zMaxM: finite(entry.zMaxM, `${label}.zMaxM`),
  };
  if (zone.zMinM >= zone.zMaxM) fail(`${label} must have positive height`);
  if (
    zone.center[0] - zone.radiusM < bounds.min[0] - TOLERANCE ||
    zone.center[0] + zone.radiusM > bounds.max[0] + TOLERANCE ||
    zone.center[1] - zone.radiusM < bounds.min[1] - TOLERANCE ||
    zone.center[1] + zone.radiusM > bounds.max[1] + TOLERANCE ||
    zone.zMinM < bounds.min[2] - TOLERANCE ||
    zone.zMaxM > bounds.max[2] + TOLERANCE
  ) {
    fail(`${label} must be contained by scenario bounds`);
  }
  return zone;
}

function temporaryZone(value, label, bounds) {
  const zone = staticZone(value, label, bounds);
  const activeFromS = nonNegative(value.activeFromS, `${label}.activeFromS`);
  const activeUntilS = positive(value.activeUntilS, `${label}.activeUntilS`);
  if (activeFromS >= activeUntilS) fail(`${label} has an invalid half-open active interval`);
  return { ...zone, activeFromS, activeUntilS };
}

function movingSphere(value, label, bounds) {
  const entry = object(value, label);
  const keyframes = list(entry.keyframes, `${label}.keyframes`).map((raw, index) => {
    const keyframe = object(raw, `${label}.keyframes[${index}]`);
    const position = vec3(keyframe.position, `${label}.keyframes[${index}].position`);
    if (!inBounds(position, bounds)) fail(`${label}.keyframes[${index}].position is out of bounds`);
    return {
      timeS: nonNegative(keyframe.timeS, `${label}.keyframes[${index}].timeS`),
      position,
    };
  });
  if (keyframes.length < 2) fail(`${label} requires at least two keyframes`);
  for (let index = 1; index < keyframes.length; index += 1) {
    if (keyframes[index].timeS <= keyframes[index - 1].timeS) {
      fail(`${label}.keyframes must be strictly increasing in time`);
    }
  }
  return {
    id: text(entry.id, `${label}.id`),
    radiusM: positive(entry.radiusM, `${label}.radiusM`),
    keyframes,
  };
}

function interpolate(points, timeS) {
  const first = points[0];
  const last = points.at(-1);
  if (timeS <= first.timeS) return first.position;
  if (timeS >= last.timeS) return last.position;
  for (let index = 1; index < points.length; index += 1) {
    const right = points[index];
    const left = points[index - 1];
    if (timeS <= right.timeS) {
      const fraction = (timeS - left.timeS) / (right.timeS - left.timeS);
      return left.position.map(
        (coordinate, axis) => coordinate + (right.position[axis] - coordinate) * fraction,
      );
    }
  }
  return last.position;
}

function spatialPath(value, label, bounds) {
  return list(value, label).map((raw, index) => {
    const point = vec3(raw, `${label}[${index}]`);
    if (!inBounds(point, bounds)) fail(`${label}[${index}] is outside scenario bounds`);
    return point;
  });
}

function timedPath(value, label, bounds) {
  const points = list(value, label).map((raw, index) => {
    const entry = object(raw, `${label}[${index}]`);
    const position = vec3(entry.position, `${label}[${index}].position`);
    if (!inBounds(position, bounds)) fail(`${label}[${index}].position is outside bounds`);
    return { timeS: nonNegative(entry.timeS, `${label}[${index}].timeS`), position };
  });
  if (points.length === 0 || points[0].timeS !== 0) {
    fail(`${label} must be non-empty and start at time 0`);
  }
  for (let index = 1; index < points.length; index += 1) {
    if (points[index].timeS <= points[index - 1].timeS) {
      fail(`${label} must be strictly increasing in time`);
    }
  }
  return points;
}

function waitIntervals(value, label, bounds, points) {
  const waits = list(value, label).map((raw, index) => {
    const entry = object(raw, `${label}[${index}]`);
    const interval = {
      startTimeS: nonNegative(entry.startTimeS, `${label}[${index}].startTimeS`),
      endTimeS: positive(entry.endTimeS, `${label}[${index}].endTimeS`),
      position: vec3(entry.position, `${label}[${index}].position`),
      reason: text(entry.reason, `${label}[${index}].reason`),
    };
    if (interval.startTimeS >= interval.endTimeS) fail(`${label}[${index}] must have positive duration`);
    if (!inBounds(interval.position, bounds)) fail(`${label}[${index}].position is outside bounds`);
    if (interval.endTimeS > points.at(-1).timeS + TOLERANCE) {
      fail(`${label}[${index}] extends beyond timedPath`);
    }
    if (
      !samePoint(interpolate(points, interval.startTimeS), interval.position) ||
      !samePoint(interpolate(points, interval.endTimeS), interval.position) ||
      points.some(
        (point) =>
          point.timeS > interval.startTimeS &&
          point.timeS < interval.endTimeS &&
          !samePoint(point.position, interval.position),
      )
    ) {
      fail(`${label}[${index}] is not stationary in timedPath`);
    }
    return interval;
  });
  for (let index = 1; index < waits.length; index += 1) {
    if (waits[index].startTimeS < waits[index - 1].endTimeS - TOLERANCE) {
      fail(`${label} must be sorted and non-overlapping`);
    }
  }
  return waits;
}

function parseEvent(value, label) {
  if (value === null) return null;
  const entry = object(value, label);
  const kinds = new Set([
    "none",
    "temporary-zone-activated",
    "temporary-zone-deactivated",
    "replan",
    "wait",
    "wait-start",
    "wait-end",
    "prediction-update",
    "goal-reached",
    "no-path",
  ]);
  if (!kinds.has(entry.kind)) fail(`${label}.kind is unsupported`);
  return {
    kind: entry.kind,
    label: text(entry.label, `${label}.label`),
    subjectId: entry.subjectId === null ? null : text(entry.subjectId, `${label}.subjectId`),
  };
}

function frame(value, label, scenario) {
  const entry = object(value, label);
  const timeS = nonNegative(entry.timeS, `${label}.timeS`);
  const vehicle = vec3(entry.vehicle, `${label}.vehicle`);
  if (!inBounds(vehicle, scenario.bounds)) fail(`${label}.vehicle is outside bounds`);
  const path = spatialPath(entry.path, `${label}.path`, scenario.bounds);
  const executedPath = spatialPath(entry.executedPath, `${label}.executedPath`, scenario.bounds);
  if (executedPath.length === 0 || !samePoint(executedPath[0], scenario.start)) {
    fail(`${label}.executedPath must start at the scenario start`);
  }
  if (!samePoint(executedPath.at(-1), vehicle)) {
    fail(`${label}.executedPath must end at the vehicle`);
  }
  if (path.length > 0 && (!samePoint(path[0], vehicle) || !samePoint(path.at(-1), scenario.goal))) {
    fail(`${label}.path endpoints must be the vehicle and goal`);
  }

  const activeTemporaryZoneIds = list(
    entry.activeTemporaryZoneIds,
    `${label}.activeTemporaryZoneIds`,
  ).map((id, index) => text(id, `${label}.activeTemporaryZoneIds[${index}]`));
  const expectedActive = new Set(
    scenario.temporaryNoFlyZones
      .filter((zone) => zone.activeFromS <= timeS && timeS < zone.activeUntilS)
      .map((zone) => zone.id),
  );
  if (
    new Set(activeTemporaryZoneIds).size !== activeTemporaryZoneIds.length ||
    activeTemporaryZoneIds.length !== expectedActive.size ||
    activeTemporaryZoneIds.some((id) => !expectedActive.has(id))
  ) {
    fail(`${label}.activeTemporaryZoneIds disagrees with half-open schedules`);
  }

  const movingSpheres = list(entry.movingSpheres, `${label}.movingSpheres`).map((raw, index) => {
    const state = object(raw, `${label}.movingSpheres[${index}]`);
    const id = text(state.id, `${label}.movingSpheres[${index}].id`);
    const definition = scenario.movingSphereMap.get(id);
    if (!definition) fail(`${label}.movingSpheres[${index}].id is undeclared`);
    const position = vec3(state.position, `${label}.movingSpheres[${index}].position`);
    if (!samePoint(position, interpolate(definition.keyframes, timeS))) {
      fail(`${label}.movingSpheres[${index}].position disagrees with declared keyframes`);
    }
    const radiusM = positive(state.radiusM, `${label}.movingSpheres[${index}].radiusM`);
    if (!sameNumber(radiusM, definition.radiusM)) {
      fail(`${label}.movingSpheres[${index}].radiusM disagrees with its definition`);
    }
    return { id, position, radiusM };
  });
  if (
    movingSpheres.length !== scenario.movingSphereMap.size ||
    new Set(movingSpheres.map((state) => state.id)).size !== movingSpheres.length
  ) {
    fail(`${label}.movingSpheres must report each definition exactly once`);
  }
  const event = parseEvent(entry.event, `${label}.event`);
  if (event?.kind === "goal-reached" && !samePoint(vehicle, scenario.goal)) {
    fail(`${label} goal-reached event must be at the goal`);
  }
  return { timeS, vehicle, path, executedPath, activeTemporaryZoneIds, movingSpheres, event };
}

function metrics(value, label) {
  const entry = object(value, label);
  const workUnits = new Set(["expanded-nodes", "queue-pops", "expanded-spacetime-states"]);
  if (!workUnits.has(entry.workUnit)) fail(`${label}.workUnit is unsupported`);
  return {
    success: bool(entry.success, `${label}.success`),
    failureReason: nullableText(entry.failureReason, `${label}.failureReason`),
    arrivalTimeS: entry.arrivalTimeS === null ? null : nonNegative(entry.arrivalTimeS, `${label}.arrivalTimeS`),
    travelTimeS: entry.travelTimeS === null ? null : nonNegative(entry.travelTimeS, `${label}.travelTimeS`),
    waitTimeS: nonNegative(entry.waitTimeS, `${label}.waitTimeS`),
    executedPathLengthM: nonNegative(entry.executedPathLengthM, `${label}.executedPathLengthM`),
    directDistanceM: positive(entry.directDistanceM, `${label}.directDistanceM`),
    pathExcessPct: entry.pathExcessPct === null ? null : finite(entry.pathExcessPct, `${label}.pathExcessPct`),
    replans: integer(entry.replans, `${label}.replans`),
    expandedStates: integer(entry.expandedStates, `${label}.expandedStates`),
    workUnit: entry.workUnit,
    minimumSeparationM:
      entry.minimumSeparationM === null
        ? null
        : nonNegative(entry.minimumSeparationM, `${label}.minimumSeparationM`),
    safetyViolations: integer(entry.safetyViolations, `${label}.safetyViolations`),
  };
}

function run(value, label, scenario, planner) {
  const entry = object(value, label);
  const statuses = new Set(["success", "no-path", "timeout", "invalid"]);
  if (!statuses.has(entry.status)) fail(`${label}.status is unsupported`);
  const failureReason = nullableText(entry.failureReason, `${label}.failureReason`);
  if ((entry.status === "success") !== (failureReason === null)) {
    fail(`${label}.failureReason is inconsistent with status`);
  }
  if (text(entry.plannerId, `${label}.plannerId`) !== planner.id) {
    fail(`${label}.plannerId disagrees with its declared planner`);
  }
  const predictive = bool(entry.predictive, `${label}.predictive`);
  if (predictive !== planner.predictive) fail(`${label}.predictive disagrees with its planner`);
  const parameters = object(entry.parameters, `${label}.parameters`);
  if (Object.keys(parameters).length === 0) fail(`${label}.parameters cannot be empty`);
  Object.entries(parameters).forEach(([key, value]) => finite(value, `${label}.parameters.${key}`));

  const points = timedPath(entry.timedPath, `${label}.timedPath`, scenario.bounds);
  if (!samePoint(points[0].position, scenario.start)) fail(`${label}.timedPath must start at scenario start`);
  const waits = waitIntervals(entry.waitIntervals, `${label}.waitIntervals`, scenario.bounds, points);
  const frames = list(entry.frames, `${label}.frames`).map((raw, index) =>
    frame(raw, `${label}.frames[${index}]`, scenario),
  );
  if (frames.length === 0 || frames[0].timeS !== 0) {
    fail(`${label}.frames must be non-empty and start at time 0`);
  }
  frames.forEach((current, index) => {
    if (!samePoint(current.vehicle, interpolate(points, current.timeS))) {
      fail(`${label}.frames[${index}].vehicle disagrees with timedPath`);
    }
    if (index === 0) return;
    const previous = frames[index - 1];
    if (current.timeS <= previous.timeS) fail(`${label}.frames must be strictly increasing in time`);
    if (
      previous.executedPath.length > current.executedPath.length ||
      previous.executedPath.some((point, pointIndex) => !samePoint(point, current.executedPath[pointIndex]))
    ) {
      fail(`${label}.executedPath must grow monotonically`);
    }
  });

  const outcome = metrics(entry.metrics, `${label}.metrics`);
  const expectedWorkUnit =
    planner.id === "space-time-astar-4d"
      ? "expanded-spacetime-states"
      : planner.id === "dstar-lite-reset-3d" || planner.id === "dstar-lite-reuse-3d"
        ? "queue-pops"
        : planner.id === "repeated-astar-3d"
          ? "expanded-nodes"
          : null;
  if (expectedWorkUnit !== null && outcome.workUnit !== expectedWorkUnit) {
    fail(`${label}.metrics.workUnit disagrees with its planner`);
  }
  const runId = text(entry.runId, `${label}.runId`);
  if (!/^sha256:[0-9a-f]{64}$/.test(runId)) fail(`${label}.runId must be a SHA-256 digest`);
  const succeeded = entry.status === "success";
  if (outcome.success !== succeeded || outcome.failureReason !== failureReason) {
    fail(`${label}.metrics status fields disagree with the run`);
  }
  const finalPoint = points.at(-1);
  const finalFrame = frames.at(-1);
  if (!sameNumber(finalPoint.timeS, finalFrame.timeS)) {
    fail(`${label}.frames and timedPath must end at the same time`);
  }
  if (succeeded) {
    if (
      outcome.arrivalTimeS === null ||
      outcome.travelTimeS === null ||
      outcome.pathExcessPct === null ||
      !samePoint(finalPoint.position, scenario.goal) ||
      !samePoint(finalFrame.vehicle, scenario.goal) ||
      finalFrame.event?.kind !== "goal-reached"
    ) {
      fail(`${label} successful outcome is incomplete`);
    }
    if (
      !sameNumber(outcome.arrivalTimeS, finalPoint.timeS) ||
      !sameNumber(outcome.travelTimeS, outcome.arrivalTimeS - outcome.waitTimeS)
    ) {
      fail(`${label}.metrics arrival, travel, and stationary times are inconsistent`);
    }
  } else if (
    outcome.arrivalTimeS !== null ||
    outcome.travelTimeS !== null ||
    outcome.pathExcessPct !== null
  ) {
    fail(`${label} failed outcome cannot define arrival, travel, or path excess`);
  }
  const directDistance = Math.hypot(
    scenario.goal[0] - scenario.start[0],
    scenario.goal[1] - scenario.start[1],
    scenario.goal[2] - scenario.start[2],
  );
  const executedLength = pathLength(points.map((point) => point.position));
  if (!sameNumber(outcome.directDistanceM, directDistance) || !sameNumber(outcome.executedPathLengthM, executedLength)) {
    fail(`${label}.metrics path lengths disagree with timedPath geometry`);
  }
  if (
    outcome.pathExcessPct !== null &&
    !sameNumber(outcome.pathExcessPct, (outcome.executedPathLengthM / outcome.directDistanceM - 1) * 100)
  ) {
    fail(`${label}.metrics.pathExcessPct is inconsistent`);
  }
  const stationaryTime = waits.reduce(
    (total, interval) => total + interval.endTimeS - interval.startTimeS,
    0,
  );
  if (!sameNumber(outcome.waitTimeS, stationaryTime)) {
    fail(`${label}.metrics.waitTimeS disagrees with waitIntervals`);
  }
  const visiblePlanningEvents = frames.filter(
    (item) => item.event?.kind === "replan" || item.event?.kind === "prediction-update",
  ).length;
  if (outcome.replans < visiblePlanningEvents) {
    fail(`${label}.metrics.replans is smaller than recorded planning events`);
  }
  return { runId, plannerId: planner.id };
}

function scenario(value, label, planners) {
  const entry = object(value, label);
  const bounds = parseBounds(entry.bounds, `${label}.bounds`);
  const start = vec3(entry.start, `${label}.start`);
  const goal = vec3(entry.goal, `${label}.goal`);
  if (!inBounds(start, bounds) || !inBounds(goal, bounds) || samePoint(start, goal)) {
    fail(`${label} endpoints are invalid`);
  }
  const constraints = object(entry.constraints, `${label}.constraints`);
  const vehicleRadiusM = nonNegative(constraints.vehicleRadiusM, `${label}.constraints.vehicleRadiusM`);
  const safetyMarginM = nonNegative(constraints.safetyMarginM, `${label}.constraints.safetyMarginM`);
  if (vehicleRadiusM + safetyMarginM <= 0) fail(`${label}.constraints require positive clearance`);

  const buildings = uniqueIdObjects(entry.buildings, `${label}.buildings`).map((building, index) => {
    const min = vec3(building.min, `${label}.buildings[${index}].min`);
    const max = vec3(building.max, `${label}.buildings[${index}].max`);
    if (min.some((coordinate, axis) => coordinate >= max[axis]) || !inBounds(min, bounds) || !inBounds(max, bounds)) {
      fail(`${label}.buildings[${index}] is invalid`);
    }
    return { id: building.id };
  });
  const staticNoFlyZones = uniqueIdObjects(entry.staticNoFlyZones, `${label}.staticNoFlyZones`).map(
    (zone, index) => staticZone(zone, `${label}.staticNoFlyZones[${index}]`, bounds),
  );
  const temporaryNoFlyZones = uniqueIdObjects(
    entry.temporaryNoFlyZones,
    `${label}.temporaryNoFlyZones`,
  ).map((zone, index) => temporaryZone(zone, `${label}.temporaryNoFlyZones[${index}]`, bounds));
  const movingSpheres = uniqueIdObjects(entry.movingSpheres, `${label}.movingSpheres`).map(
    (sphere, index) => movingSphere(sphere, `${label}.movingSpheres[${index}]`, bounds),
  );
  const obstacleIds = [
    ...buildings,
    ...staticNoFlyZones,
    ...temporaryNoFlyZones,
    ...movingSpheres,
  ].map((obstacle) => obstacle.id);
  if (new Set(obstacleIds).size !== obstacleIds.length) {
    fail(`${label} obstacle IDs must be unique across geometry types`);
  }

  const fingerprint = text(entry.fingerprint, `${label}.fingerprint`);
  if (!/^sha256:[0-9a-f]{64}$/.test(fingerprint)) fail(`${label}.fingerprint must be a digest`);
  const parsedScenario = {
    id: text(entry.id, `${label}.id`),
    fingerprint,
    bounds,
    start,
    goal,
    temporaryNoFlyZones,
    movingSphereMap: new Map(movingSpheres.map((sphere) => [sphere.id, sphere])),
  };
  text(entry.label, `${label}.label`);
  text(entry.description, `${label}.description`);
  if (entry.cohort !== undefined) text(entry.cohort, `${label}.cohort`);
  const seen = new Set();
  const runs = list(entry.runs, `${label}.runs`).map((raw, index) => {
    const runEntry = object(raw, `${label}.runs[${index}]`);
    const plannerId = text(runEntry.plannerId, `${label}.runs[${index}].plannerId`);
    const planner = planners.get(plannerId);
    if (!planner) fail(`${label}.runs[${index}].plannerId is undeclared`);
    if (seen.has(plannerId)) fail(`${label} contains duplicate run for ${plannerId}`);
    seen.add(plannerId);
    return run(runEntry, `${label}.runs[${index}]`, parsedScenario, planner);
  });
  if (runs.length !== planners.size || [...planners.keys()].some((id) => !seen.has(id))) {
    fail(`${label} must contain exactly one run for every declared planner`);
  }
  return { id: parsedScenario.id, fingerprint, runs };
}

function artifact(value, label, expectedPath) {
  const entry = object(value, label);
  if (entry.path !== expectedPath) fail(`${label}.path must be ${expectedPath}`);
  const sha256 = text(entry.sha256, `${label}.sha256`);
  if (!/^sha256:[0-9a-f]{64}$/.test(sha256)) fail(`${label}.sha256 is invalid`);
  return { path: expectedPath, sha256, bytes: positiveInteger(entry.bytes, `${label}.bytes`) };
}

function validateBundle(value) {
  const root = object(value, "root");
  if (root.schemaVersion !== 1) fail("schemaVersion must be 1");
  if (root.verificationStatus !== "PREDICTIVE_DEMO_NON_CONFIRMATORY") {
    fail("verificationStatus must be PREDICTIVE_DEMO_NON_CONFIRMATORY");
  }
  const generatedAt = text(root.generatedAt, "generatedAt");
  if (!Number.isFinite(Date.parse(generatedAt))) fail("generatedAt must be an ISO timestamp");
  const sourceCommit = text(root.sourceCommit, "sourceCommit");
  if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(sourceCommit)) {
    fail("sourceCommit must be a full lowercase Git object ID");
  }
  parseProtocol(root.protocol);

  const plannerEntries = uniqueIdObjects(root.planners, "planners").map((planner, index) => ({
    id: planner.id,
    label: text(planner.label, `planners[${index}].label`),
    predictive: bool(planner.predictive, `planners[${index}].predictive`),
  }));
  const expectedPlannerIds = [
    "repeated-astar-3d",
    "dstar-lite-reset-3d",
    "dstar-lite-reuse-3d",
    "space-time-astar-4d",
  ];
  if (
    plannerEntries.length !== expectedPlannerIds.length ||
    expectedPlannerIds.some((id) => !plannerEntries.some((planner) => planner.id === id))
  ) {
    fail("the public protocol requires exactly four declared planner conditions");
  }
  if (!plannerEntries.some((planner) => planner.predictive) || plannerEntries.every((planner) => planner.predictive)) {
    fail("planners must include predictive and non-predictive baselines");
  }
  const planners = new Map(plannerEntries.map((planner) => [planner.id, planner]));
  const scenarios = list(root.scenarios, "scenarios").map((entry, index) =>
    scenario(entry, `scenarios[${index}]`, planners),
  );
  const expectedScenarioIds = [
    "wait-then-straight",
    "closing-window",
    "periodic-traffic",
    "chained-restrictions",
    "multi-obstacle",
    "vertical-time-window",
  ];
  if (
    scenarios.length !== expectedScenarioIds.length ||
    expectedScenarioIds.some((id) => !scenarios.some((entry) => entry.id === id))
  ) {
    fail("the public protocol requires exactly six registered scenarios");
  }
  if (new Set(scenarios.map((entry) => entry.id)).size !== scenarios.length) fail("scenario IDs must be unique");
  if (new Set(scenarios.map((entry) => entry.fingerprint)).size !== scenarios.length) {
    fail("scenario fingerprints must be unique");
  }
  const runIds = scenarios.flatMap((entry) => entry.runs.map((record) => record.runId));
  if (runIds.length !== 24) fail("the public protocol requires exactly 24 planner runs");
  if (new Set(runIds).size !== runIds.length) fail("runId values must be unique across the bundle");

  const downloads = object(root.downloads, "downloads");
  if (
    Object.keys(downloads).length !== 2 ||
    !("recordsCsv" in downloads) ||
    !("scenarioManifest" in downloads)
  ) {
    fail("downloads must contain exactly recordsCsv and scenarioManifest");
  }
  return {
    scenarioCount: scenarios.length,
    runCount: runIds.length,
    artifacts: [
      artifact(downloads.recordsCsv, "downloads.recordsCsv", "predictive-records.csv"),
      artifact(
        downloads.scenarioManifest,
        "downloads.scenarioManifest",
        "predictive-scenario-manifest.json",
      ),
    ],
  };
}

async function verifyArtifact(reference) {
  const artifactPath = resolve(dirname(inputPath), reference.path);
  const [contents, metadata] = await Promise.all([readFile(artifactPath), stat(artifactPath)]);
  const digest = `sha256:${createHash("sha256").update(contents).digest("hex")}`;
  if (metadata.size !== reference.bytes) {
    fail(`${reference.path} size is ${metadata.size}, expected ${reference.bytes}`);
  }
  if (digest !== reference.sha256) {
    fail(`${reference.path} digest is ${digest}, expected ${reference.sha256}`);
  }
}

const source = await readFile(inputPath, "utf8");
let value;
try {
  value = JSON.parse(source);
} catch (error) {
  fail(`invalid JSON (${error instanceof Error ? error.message : String(error)})`);
}
const result = validateBundle(value);
await Promise.all(result.artifacts.map(verifyArtifact));
console.log(
  `Validated ${result.scenarioCount} predictive scenarios, ${result.runCount} runs, and ${result.artifacts.length} referenced artifacts.`,
);
