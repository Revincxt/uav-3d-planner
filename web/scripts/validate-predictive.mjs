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
    trajectoryPostprocessor: text(
      entry.trajectoryPostprocessor,
      "protocol.trajectoryPostprocessor",
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

function sameTimedPath(left, right) {
  return (
    left.length === right.length &&
    left.every(
      (point, index) =>
        sameNumber(point.timeS, right[index].timeS) && samePoint(point.position, right[index].position),
    )
  );
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
  if ("path" in entry || "executedPath" in entry) {
    fail(`${label} must not duplicate O(N) path geometry`);
  }
  const timeS = nonNegative(entry.timeS, `${label}.timeS`);
  const vehicle = vec3(entry.vehicle, `${label}.vehicle`);
  if (!inBounds(vehicle, scenario.bounds)) fail(`${label}.vehicle is outside bounds`);
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
  return { timeS, vehicle, event };
}

function discreteKinematicDiagnostics(value, label) {
  const entry = object(value, label);
  if (entry.status !== "discrete-diagnostic-only") {
    fail(`${label}.status must be discrete-diagnostic-only`);
  }
  if (entry.continuousDynamicsCertified !== false) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  const parsed = {
    status: entry.status,
    continuousDynamicsCertified: false,
    segmentCount: integer(entry.segmentCount, `${label}.segmentCount`),
    movementSegmentCount: integer(entry.movementSegmentCount, `${label}.movementSegmentCount`),
    reversalCount: integer(entry.reversalCount, `${label}.reversalCount`),
    reversalThresholdDeg: positive(entry.reversalThresholdDeg, `${label}.reversalThresholdDeg`),
    maxSpeedMps: nonNegative(entry.maxSpeedMps, `${label}.maxSpeedMps`),
    maxDiscreteVelocityChangeMps: nonNegative(
      entry.maxDiscreteVelocityChangeMps,
      `${label}.maxDiscreteVelocityChangeMps`,
    ),
    maxDiscreteAccelerationProxyMps2: nonNegative(
      entry.maxDiscreteAccelerationProxyMps2,
      `${label}.maxDiscreteAccelerationProxyMps2`,
    ),
    maxAbsClimbRateMps: nonNegative(
      entry.maxAbsClimbRateMps,
      `${label}.maxAbsClimbRateMps`,
    ),
  };
  if (parsed.reversalThresholdDeg > 180 + TOLERANCE) {
    fail(`${label}.reversalThresholdDeg cannot exceed 180 degrees`);
  }
  if (parsed.movementSegmentCount > parsed.segmentCount) {
    fail(`${label}.movementSegmentCount cannot exceed segmentCount`);
  }
  if (parsed.reversalCount > Math.max(0, parsed.movementSegmentCount - 1)) {
    fail(`${label}.reversalCount is inconsistent with movementSegmentCount`);
  }
  return parsed;
}

function kinematicDiagnostics(value, label) {
  const entry = object(value, label);
  if (entry.status !== "discrete-diagnostic-only") {
    fail(`${label}.status must be discrete-diagnostic-only`);
  }
  if (entry.continuousDynamicsCertified !== false) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  return {
    status: entry.status,
    continuousDynamicsCertified: false,
    raw: discreteKinematicDiagnostics(entry.raw, `${label}.raw`),
    output: discreteKinematicDiagnostics(entry.output, `${label}.output`),
  };
}

function smoothing(value, label) {
  const entry = object(value, label);
  const before =
    entry.maxTurnAngleBeforeDeg === null
      ? null
      : nonNegative(entry.maxTurnAngleBeforeDeg, `${label}.maxTurnAngleBeforeDeg`);
  const after =
    entry.maxTurnAngleAfterDeg === null
      ? null
      : nonNegative(entry.maxTurnAngleAfterDeg, `${label}.maxTurnAngleAfterDeg`);
  if ((before !== null && before > 180 + TOLERANCE) || (after !== null && after > 180 + TOLERANCE)) {
    fail(`${label} turn angles cannot exceed 180 degrees`);
  }
  const parsed = {
    method: text(entry.method, `${label}.method`),
    applied: bool(entry.applied, `${label}.applied`),
    certified: bool(entry.certified, `${label}.certified`),
    collisionCertified: bool(entry.collisionCertified, `${label}.collisionCertified`),
    collisionCertificationScope:
      entry.collisionCertificationScope === "dense-piecewise-linear-space-time-path"
        ? entry.collisionCertificationScope
        : fail(`${label}.collisionCertificationScope is unsupported`),
    rawWaypointCount: positiveInteger(entry.rawWaypointCount, `${label}.rawWaypointCount`),
    outputWaypointCount: positiveInteger(entry.outputWaypointCount, `${label}.outputWaypointCount`),
    roundedCornerCount: integer(entry.roundedCornerCount, `${label}.roundedCornerCount`),
    requestedTurnRadiusM: positive(entry.requestedTurnRadiusM, `${label}.requestedTurnRadiusM`),
    appliedTurnRadiusM:
      entry.appliedTurnRadiusM === null
        ? null
        : positive(entry.appliedTurnRadiusM, `${label}.appliedTurnRadiusM`),
    sampleSpacingM: positive(entry.sampleSpacingM, `${label}.sampleSpacingM`),
    before,
    after,
    kinematicDiagnostics: kinematicDiagnostics(
      entry.kinematicDiagnostics,
      `${label}.kinematicDiagnostics`,
    ),
  };
  if (parsed.collisionCertified !== parsed.certified) {
    fail(`${label}.collisionCertified must agree with certified`);
  }
  if (parsed.applied && !parsed.certified) fail(`${label} applied output must be certified`);
  if (parsed.applied !== (parsed.appliedTurnRadiusM !== null)) {
    fail(`${label}.appliedTurnRadiusM must be present exactly when smoothing is applied`);
  }
  if (
    parsed.appliedTurnRadiusM !== null &&
    parsed.appliedTurnRadiusM > parsed.requestedTurnRadiusM + TOLERANCE
  ) {
    fail(`${label}.appliedTurnRadiusM cannot exceed requestedTurnRadiusM`);
  }
  if (!parsed.applied && parsed.roundedCornerCount !== 0) {
    fail(`${label}.roundedCornerCount must be zero when smoothing is not applied`);
  }
  // acos is ill-conditioned at a 180-degree reversal; permit only micro-degree drift.
  if (before !== null && after !== null && after > before + 1e-5) {
    fail(`${label} cannot increase maximum turn angle`);
  }
  if (
    parsed.kinematicDiagnostics.raw.segmentCount !== parsed.rawWaypointCount - 1 ||
    parsed.kinematicDiagnostics.output.segmentCount !== parsed.outputWaypointCount - 1
  ) {
    fail(`${label}.kinematicDiagnostics segment counts disagree with waypoint counts`);
  }
  return parsed;
}

function minimumSeparationWitness(value, label) {
  if (value === null) return null;
  const entry = object(value, label);
  if (entry.obstacleKind !== "moving-sphere" && entry.obstacleKind !== "temporary-cylinder") {
    fail(`${label}.obstacleKind is unsupported`);
  }
  return {
    separationM: finite(entry.separationM, `${label}.separationM`),
    timeS: nonNegative(entry.timeS, `${label}.timeS`),
    vehiclePosition: vec3(entry.vehiclePosition, `${label}.vehiclePosition`),
    obstacleId: text(entry.obstacleId, `${label}.obstacleId`),
    obstacleKind: entry.obstacleKind,
    obstaclePosition: vec3(entry.obstaclePosition, `${label}.obstaclePosition`),
    declaredSafetyMarginM: nonNegative(
      entry.declaredSafetyMarginM,
      `${label}.declaredSafetyMarginM`,
    ),
    method: text(entry.method, `${label}.method`),
    exact: bool(entry.exact, `${label}.exact`),
  };
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
        : finite(entry.minimumSeparationM, `${label}.minimumSeparationM`),
    minimumSeparationWitness: minimumSeparationWitness(
      entry.minimumSeparationWitness,
      `${label}.minimumSeparationWitness`,
    ),
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
  Object.entries(parameters).forEach(([key, parameter]) => finite(parameter, `${label}.parameters.${key}`));

  const rawPoints = timedPath(entry.rawTimedPath, `${label}.rawTimedPath`, scenario.bounds);
  const points = timedPath(entry.timedPath, `${label}.timedPath`, scenario.bounds);
  if (!samePoint(rawPoints[0].position, scenario.start) || !samePoint(points[0].position, scenario.start)) {
    fail(`${label} paths must start at scenario start`);
  }
  const postprocess = smoothing(entry.smoothing, `${label}.smoothing`);
  if (
    postprocess.rawWaypointCount !== rawPoints.length ||
    postprocess.outputWaypointCount !== points.length
  ) {
    fail(`${label}.smoothing waypoint counts disagree with exported paths`);
  }
  if (!postprocess.applied && !sameTimedPath(rawPoints, points)) {
    fail(`${label} unapplied smoothing must preserve the raw path exactly`);
  }
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
    if (index > 0 && current.timeS <= frames[index - 1].timeS) {
      fail(`${label}.frames must be strictly increasing in time`);
    }
  });

  const outcome = metrics(entry.metrics, `${label}.metrics`);
  const separationWitness = outcome.minimumSeparationWitness;
  if ((outcome.minimumSeparationM === null) !== (separationWitness === null)) {
    fail(`${label}.metrics minimum separation and witness must be present together`);
  }
  if (separationWitness !== null) {
    if (
      !sameNumber(outcome.minimumSeparationM, separationWitness.separationM) ||
      !sameNumber(separationWitness.declaredSafetyMarginM, scenario.constraints.safetyMarginM)
    ) {
      fail(`${label}.metrics minimum-separation witness is inconsistent`);
    }
    if (
      separationWitness.timeS > points.at(-1).timeS + TOLERANCE ||
      !inBounds(separationWitness.vehiclePosition, scenario.bounds) ||
      !inBounds(separationWitness.obstaclePosition, scenario.bounds) ||
      !samePoint(separationWitness.vehiclePosition, interpolate(points, separationWitness.timeS))
    ) {
      fail(`${label}.metrics minimum-separation witness is outside the executed trajectory`);
    }
    const witnessSurfaceDistance = Math.hypot(
      separationWitness.vehiclePosition[0] - separationWitness.obstaclePosition[0],
      separationWitness.vehiclePosition[1] - separationWitness.obstaclePosition[1],
      separationWitness.vehiclePosition[2] - separationWitness.obstaclePosition[2],
    );
    if (
      !sameNumber(
        separationWitness.separationM,
        witnessSurfaceDistance - scenario.constraints.vehicleRadiusM,
      )
    ) {
      fail(`${label}.metrics minimum-separation witness geometry is inconsistent`);
    }
    if (separationWitness.obstacleKind === "moving-sphere") {
      const definition = scenario.movingSphereMap.get(separationWitness.obstacleId);
      const center = definition && interpolate(definition.keyframes, separationWitness.timeS);
      if (
        definition === undefined ||
        center === undefined ||
        !separationWitness.exact ||
        !sameNumber(
          Math.hypot(
            separationWitness.obstaclePosition[0] - center[0],
            separationWitness.obstaclePosition[1] - center[1],
            separationWitness.obstaclePosition[2] - center[2],
          ),
          definition.radiusM,
        ) ||
        !sameNumber(
          Math.hypot(
            separationWitness.vehiclePosition[0] - center[0],
            separationWitness.vehiclePosition[1] - center[1],
            separationWitness.vehiclePosition[2] - center[2],
          ),
          witnessSurfaceDistance + definition.radiusM,
        )
      ) {
        fail(`${label}.metrics moving-sphere witness is inconsistent`);
      }
    } else if (
      separationWitness.exact ||
      !scenario.temporaryNoFlyZones.some((zone) => zone.id === separationWitness.obstacleId)
    ) {
      fail(`${label}.metrics temporary-cylinder witness is inconsistent`);
    }
    if (
      outcome.safetyViolations === 0 &&
      separationWitness.separationM + TOLERANCE < scenario.constraints.safetyMarginM
    ) {
      fail(`${label}.metrics safe run violates its declared dynamic safety margin`);
    }
  }
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
      !postprocess.certified ||
      outcome.arrivalTimeS === null ||
      outcome.travelTimeS === null ||
      outcome.pathExcessPct === null ||
      !samePoint(finalPoint.position, scenario.goal) ||
      !samePoint(rawPoints.at(-1).position, scenario.goal) ||
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
  if (
    !sameNumber(outcome.directDistanceM, directDistance) ||
    !sameNumber(outcome.executedPathLengthM, executedLength)
  ) {
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
    if (
      min.some((coordinate, axis) => coordinate >= max[axis]) ||
      !inBounds(min, bounds) ||
      !inBounds(max, bounds)
    ) {
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

  const environmentEntry = object(entry.environment, `${label}.environment`);
  const environment = {
    district: text(environmentEntry.district, `${label}.environment.district`),
    streetPattern: text(environmentEntry.streetPattern, `${label}.environment.streetPattern`),
    buildingCount: integer(environmentEntry.buildingCount, `${label}.environment.buildingCount`),
    hazardCount: integer(environmentEntry.hazardCount, `${label}.environment.hazardCount`),
  };
  const hazardCount = staticNoFlyZones.length + temporaryNoFlyZones.length + movingSpheres.length;
  if (environment.buildingCount !== buildings.length || environment.hazardCount !== hazardCount) {
    fail(`${label}.environment counts disagree with declared geometry`);
  }
  const fingerprint = text(entry.fingerprint, `${label}.fingerprint`);
  if (!/^sha256:[0-9a-f]{64}$/.test(fingerprint)) fail(`${label}.fingerprint must be a digest`);
  const cohort = entry.cohort === undefined ? null : text(entry.cohort, `${label}.cohort`);
  const parsedScenario = {
    id: text(entry.id, `${label}.id`),
    fingerprint,
    cohort,
    bounds,
    start,
    goal,
    constraints: { vehicleRadiusM, safetyMarginM },
    buildings,
    staticNoFlyZones,
    temporaryNoFlyZones,
    movingSpheres,
    movingSphereMap: new Map(movingSpheres.map((sphere) => [sphere.id, sphere])),
    environment,
  };
  text(entry.label, `${label}.label`);
  text(entry.description, `${label}.description`);
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
  const dynamicHazards = temporaryNoFlyZones.length + movingSpheres.length;
  return {
    id: parsedScenario.id,
    fingerprint,
    cohort,
    buildingCount: buildings.length,
    staticZoneCount: staticNoFlyZones.length,
    dynamicHazards,
    runs,
  };
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
  if (root.schemaVersion !== 2) fail("schemaVersion must be 2");
  if (root.verificationStatus !== "PREDICTIVE_DEMO_NON_CONFIRMATORY") {
    fail("verificationStatus must be PREDICTIVE_DEMO_NON_CONFIRMATORY");
  }
  const generatedAt = text(root.generatedAt, "generatedAt");
  if (!Number.isFinite(Date.parse(generatedAt))) fail("generatedAt must be an ISO timestamp");
  const sourceCommit = text(root.sourceCommit, "sourceCommit");
  if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(sourceCommit)) {
    fail("sourceCommit must be a full lowercase Git object ID");
  }
  const protocol = parseProtocol(root.protocol);
  if (protocol.id !== "predictive-space-time-v3") {
    fail("protocol.id must be predictive-space-time-v3");
  }

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
    fail("the public protocol requires the four declared planner conditions");
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
    "urban-canyon-merge",
    "rooftop-transfer",
    "braided-skyway",
    "harbor-switchback",
  ];
  if (
    scenarios.length !== expectedScenarioIds.length ||
    expectedScenarioIds.some((id) => !scenarios.some((entry) => entry.id === id))
  ) {
    fail("the public v0.6 protocol requires the ten declared scenarios");
  }
  if (new Set(scenarios.map((entry) => entry.id)).size !== scenarios.length) fail("scenario IDs must be unique");
  if (new Set(scenarios.map((entry) => entry.fingerprint)).size !== scenarios.length) {
    fail("scenario fingerprints must be unique");
  }
  const complexDemo = scenarios.find(
    (entry) =>
      entry.cohort === "demo" &&
      entry.buildingCount >= 14 &&
      entry.staticZoneCount >= 1 &&
      entry.dynamicHazards >= 2,
  );
  if (!complexDemo) {
    fail("at least one demo scenario must have 14 buildings, a static NFZ, and two dynamic hazards");
  }
  const runIds = scenarios.flatMap((entry) => entry.runs.map((record) => record.runId));
  if (runIds.length !== 40) fail("the public v0.6 protocol requires exactly 40 planner runs");
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
    complexDemoId: complexDemo.id,
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
  `Validated ${result.scenarioCount} predictive scenarios, ${result.runCount} runs, complex demo ${result.complexDemoId}, and ${result.artifacts.length} referenced artifacts.`,
);
