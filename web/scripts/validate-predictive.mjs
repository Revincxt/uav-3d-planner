import { createHash } from "node:crypto";
import { readFile, stat } from "node:fs/promises";
import { readStudyData } from './study-reader.mjs';
import { dirname, resolve } from "node:path";
import { validateExecutionSequence } from "../shared/execution-sequence.mjs";

const TOLERANCE = 1e-6;
const inputPath = resolve(process.cwd(), process.argv[2] ?? "public/predictive-data.json");
const MANHATTAN_V4_PROTOCOL_ID = "manhattan-space-time-v4";
const MANHATTAN_SPATIAL_POSTPROCESSOR = "spatial-local-quintic-bspline-bounded-altitude-envelope-v6";
const FROZEN_ENVELOPE = { maxSpeedMps: 8, maxAbsClimbRateMps: 3, maxDiscreteAccelerationProxyMps2: 4, reversalThresholdDeg: 150, maxExecutionTimeS: 90 };
const MANHATTAN_ENVELOPE = { ...FROZEN_ENVELOPE, maxSpeedMps: 15, maxExecutionTimeS: 900 };
const MANHATTAN_IDS = ["manhattan-westside-delivery", "manhattan-medical-transfer", "manhattan-midtown-rooftops", "manhattan-riverfront-logistics", "manhattan-westside-backhaul", "manhattan-medical-return", "manhattan-eastside-backhaul", "manhattan-riverfront-return"];
const matchesEnvelope = (value, expected) => Object.entries(expected).every(([key, entry]) => sameNumber(value[key], entry));

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

function sameStationaryPosition(left, right) {
  // Match Python geometry.almost_equal, not coordinate-scaled display tolerance.
  return Math.hypot(...left.map((coordinate, index) => coordinate - right[index])) <= 1e-9;
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
  if (entry.id !== "predictive-space-time-v4" && !(entry.id === MANHATTAN_V4_PROTOCOL_ID)) {
    fail("protocol.id must be predictive-space-time-v4 or manhattan-space-time-v4");
  }
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
    executionEnvelope: executionEnvelope(entry.executionEnvelope, "protocol.executionEnvelope"),
    continuousDynamicsCertified: bool(
      entry.continuousDynamicsCertified,
      "protocol.continuousDynamicsCertified",
    ),
  };
  if ((protocol.id === MANHATTAN_V4_PROTOCOL_ID)) {
    if (entry.spaceTimeConnectivity !== 26) fail("protocol.spaceTimeConnectivity must be 26 for current Manhattan v4");
    if (entry.trajectoryShortcut !== true) fail("protocol.trajectoryShortcut must be true for current Manhattan v4");
    if (protocol.trajectoryPostprocessor !== MANHATTAN_SPATIAL_POSTPROCESSOR) {
      fail("protocol.trajectoryPostprocessor disagrees with the declared Manhattan postprocessor");
    }
    protocol.spaceTimeConnectivity = 26;
    protocol.trajectoryShortcut = true;
  }
  if (protocol.continuousDynamicsCertified) {
    fail("protocol.continuousDynamicsCertified must remain false");
  }
  if (protocol.id === MANHATTAN_V4_PROTOCOL_ID) {
    if (entry.trajectoryPreserveAltitude !== false) fail("protocol.trajectoryPreserveAltitude must be false for Manhattan v4");
    if (entry.trajectoryCurveDegree !== 5) fail("protocol.trajectoryCurveDegree must be 5");
    if (entry.trajectoryCurveDimensions !== 3) fail("protocol.trajectoryCurveDimensions must be 3");
    if (entry.trajectoryAltitudeDeviationLimitM !== 12) fail("protocol.trajectoryAltitudeDeviationLimitM must be 12");
    if (entry.trajectoryDynamicScheduling !== "certified-move-block-departures") fail("Unsupported dynamic execution scheduling");
    protocol.trajectoryPreserveAltitude = false;
    protocol.trajectoryCurveDegree = 5;
    protocol.trajectoryCurveDimensions = 3;
    protocol.trajectoryAltitudeDeviationLimitM = 12;
    protocol.trajectoryDynamicScheduling = entry.trajectoryDynamicScheduling;
  }
  if (protocol.timeResolutionS > protocol.planningHorizonS) {
    fail("protocol.timeResolutionS cannot exceed planningHorizonS");
  }
  if (protocol.predictionHorizonS > protocol.planningHorizonS) {
    fail("protocol.predictionHorizonS cannot exceed planningHorizonS");
  }
  if (protocol.planningHorizonS > protocol.maxTimeS) {
    fail("protocol.planningHorizonS cannot exceed maxTimeS");
  }
  const expectedEnvelope = (protocol.id === MANHATTAN_V4_PROTOCOL_ID) ? MANHATTAN_ENVELOPE : FROZEN_ENVELOPE;
  if (!matchesEnvelope(protocol.executionEnvelope, expectedEnvelope)) {
    fail("protocol.executionEnvelope disagrees with the declared protocol");
  }
  if (!sameNumber(protocol.cruiseSpeedMps, protocol.executionEnvelope.maxSpeedMps) ||
      !sameNumber(protocol.maxTimeS, protocol.executionEnvelope.maxExecutionTimeS)) {
    fail("protocol speed and mission horizon must agree with its execution envelope");
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

function validateAltitudeProfile(raw, geometry, label, maxDeviationM) {
  if (Math.abs(raw.at(-1).timeS - geometry.at(-1).timeS) > TOLERANCE) {
    fail(`${label} altitude preservation requires the same absolute-time domain`);
  }
  // Checking both profiles' knots is exact for the serialized piecewise-linear z(t).
  // Use an absolute 1e-6 metre tolerance, never the relative coordinate comparator.
  const times = new Set([...raw, ...geometry].map((point) => point.timeS));
  for (const timeS of times) {
    if (Math.abs(interpolate(raw, timeS)[2] - interpolate(geometry, timeS)[2]) > maxDeviationM + TOLERANCE) {
      fail(`${label} geometry exceeds its declared altitude deviation at a profile knot`);
    }
  }
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
      !sameStationaryPosition(interpolate(points, interval.startTimeS), interval.position) ||
      !sameStationaryPosition(interpolate(points, interval.endTimeS), interval.position) ||
      points.some(
        (point) =>
          point.timeS > interval.startTimeS &&
          point.timeS < interval.endTimeS &&
          !sameStationaryPosition(point.position, interval.position),
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

function executionEnvelope(value, label) {
  const entry = object(value, label);
  const parsed = {
    model: text(entry.model, `${label}.model`),
    maxSpeedMps: positive(entry.maxSpeedMps, `${label}.maxSpeedMps`),
    maxAbsClimbRateMps: positive(
      entry.maxAbsClimbRateMps,
      `${label}.maxAbsClimbRateMps`,
    ),
    maxDiscreteAccelerationProxyMps2: positive(
      entry.maxDiscreteAccelerationProxyMps2,
      `${label}.maxDiscreteAccelerationProxyMps2`,
    ),
    reversalThresholdDeg: positive(
      entry.reversalThresholdDeg,
      `${label}.reversalThresholdDeg`,
    ),
    allowReversals: bool(entry.allowReversals, `${label}.allowReversals`),
    maxExecutionTimeS: positive(entry.maxExecutionTimeS, `${label}.maxExecutionTimeS`),
    continuousDynamicsCertified: bool(
      entry.continuousDynamicsCertified,
      `${label}.continuousDynamicsCertified`,
    ),
  };
  if (parsed.model !== "discrete-segment-average-envelope-v1") {
    fail(`${label}.model is unsupported`);
  }
  if (parsed.reversalThresholdDeg > 180 + TOLERANCE) {
    fail(`${label}.reversalThresholdDeg cannot exceed 180 degrees`);
  }
  if (parsed.allowReversals || parsed.continuousDynamicsCertified) {
    fail(`${label} must forbid reversals and must not claim continuous-dynamics certification`);
  }
  if (!matchesEnvelope(parsed, FROZEN_ENVELOPE) && !matchesEnvelope(parsed, MANHATTAN_ENVELOPE)) {
    fail(`${label} disagrees with v0.7 or the Manhattan execution envelope`);
  }
  return parsed;
}

function executionQualification(value, label) {
  if (value === null) return null;
  const entry = object(value, label);
  if (entry.status !== "qualified" && entry.status !== "not-qualified") {
    fail(`${label}.status is unsupported`);
  }
  const violations = list(entry.violations, `${label}.violations`).map((value, index) =>
    text(value, `${label}.violations[${index}]`),
  );
  const allowedViolations = new Set([
    "speed-limit-exceeded",
    "climb-rate-limit-exceeded",
    "acceleration-proxy-limit-exceeded",
    "reversal-not-allowed",
    "execution-time-limit-exceeded",
  ]);
  if (
    new Set(violations).size !== violations.length ||
    violations.some((violation) => !allowedViolations.has(violation))
  ) {
    fail(`${label}.violations contains an unsupported or duplicate value`);
  }
  const parsed = {
    status: entry.status,
    qualified: bool(entry.qualified, `${label}.qualified`),
    continuousDynamicsCertified: bool(
      entry.continuousDynamicsCertified,
      `${label}.continuousDynamicsCertified`,
    ),
    diagnostics: discreteKinematicDiagnostics(entry.diagnostics, `${label}.diagnostics`),
    boundaryAwareMaxDiscreteAccelerationProxyMps2: nonNegative(
      entry.boundaryAwareMaxDiscreteAccelerationProxyMps2,
      `${label}.boundaryAwareMaxDiscreteAccelerationProxyMps2`,
    ),
    violations,
  };
  if (
    parsed.continuousDynamicsCertified ||
    parsed.qualified !== (parsed.status === "qualified") ||
    parsed.qualified !== (violations.length === 0)
  ) {
    fail(`${label} qualification verdict is inconsistent`);
  }
  return parsed;
}

function executionMetadata(value, label) {
  const entry = object(value, label);
  const statuses = new Set([
    "not-evaluated",
    "qualified",
    "reversal-not-allowed",
    "execution-time-limit-exceeded",
    "time-parameterization-did-not-converge",
    "dynamic-collision-after-retiming",
  ]);
  if (!statuses.has(entry.status)) fail(`${label}.status is unsupported`);
  const nullableDuration = (candidate, field) =>
    candidate === null ? null : nonNegative(candidate, `${label}.${field}`);
  const parsed = {
    status: entry.status,
    qualified: bool(entry.qualified, `${label}.qualified`),
    collisionCertified: bool(entry.collisionCertified, `${label}.collisionCertified`),
    collisionCertificationScope:
      entry.collisionCertificationScope === "dense-piecewise-linear-space-time-path"
        ? entry.collisionCertificationScope
        : fail(`${label}.collisionCertificationScope is unsupported`),
    continuousDynamicsCertified: bool(
      entry.continuousDynamicsCertified,
      `${label}.continuousDynamicsCertified`,
    ),
    envelope: executionEnvelope(entry.envelope, `${label}.envelope`),
    qualification: executionQualification(entry.qualification, `${label}.qualification`),
    timingIterations: integer(entry.timingIterations, `${label}.timingIterations`),
    originalDurationS: nullableDuration(entry.originalDurationS, "originalDurationS"),
    candidateDurationS: nullableDuration(entry.candidateDurationS, "candidateDurationS"),
    addedDurationS: nullableDuration(entry.addedDurationS, "addedDurationS"),
  };
  if (parsed.continuousDynamicsCertified) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  const durationFields = [
    parsed.originalDurationS,
    parsed.candidateDurationS,
    parsed.addedDurationS,
  ];
  if (durationFields.some((item) => item === null) && !durationFields.every((item) => item === null)) {
    fail(`${label} duration fields must be all numeric or all null`);
  }
  if (parsed.status === "not-evaluated") {
    if (
      parsed.qualified ||
      parsed.collisionCertified ||
      parsed.qualification !== null ||
      parsed.timingIterations !== 0 ||
      !durationFields.every((item) => item === null)
    ) {
      fail(`${label} not-evaluated metadata is inconsistent`);
    }
    return parsed;
  }
  if (parsed.qualification === null || durationFields.some((item) => item === null)) {
    fail(`${label} evaluated status requires qualification and durations`);
  }
  if (
    parsed.candidateDurationS + TOLERANCE < parsed.originalDurationS ||
    !sameNumber(parsed.addedDurationS, parsed.candidateDurationS - parsed.originalDurationS)
  ) {
    fail(`${label} retiming must not shorten the path and addedDurationS must be exact`);
  }
  if (parsed.status === "qualified") {
    if (!parsed.qualified || !parsed.collisionCertified || !parsed.qualification.qualified) {
      fail(`${label} qualified status requires both discrete and collision qualification`);
    }
  } else if (parsed.qualified || parsed.collisionCertified) {
    fail(`${label} failed status cannot expose a qualified or collision-certified candidate`);
  } else if (
    parsed.status === "dynamic-collision-after-retiming" &&
    !parsed.qualification.qualified
  ) {
    fail(`${label} collision-after-retiming requires a discretely qualified timing result`);
  }
  return parsed;
}

function smoothing(value, label, allowShortcut, spatialCurves) {
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
        // A quintic's legacy trim/tan(theta/2) scale tends to zero at a
        // reversal; canonicalization can round it to 0 despite a safe curve.
        : spatialCurves && entry.method === "spacetime-shortcut-plus-local-quintic-bspline"
          ? nonNegative(entry.appliedTurnRadiusM, `${label}.appliedTurnRadiusM`)
        : positive(entry.appliedTurnRadiusM, `${label}.appliedTurnRadiusM`),
    sampleSpacingM: positive(entry.sampleSpacingM, `${label}.sampleSpacingM`),
    before,
    after,
    kinematicDiagnostics: kinematicDiagnostics(
      entry.kinematicDiagnostics,
      `${label}.kinematicDiagnostics`,
    ),
    execution: executionMetadata(entry.execution, `${label}.execution`),
  };
  if (spatialCurves) {
    const axes = list(entry.optimizationAxes, `${label}.optimizationAxes`);
    if (axes.length !== 3 || axes[0] !== "x" || axes[1] !== "y" || axes[2] !== "z")
      fail(`${label}.optimizationAxes must be exactly ['x', 'y', 'z']`);
    if (entry.altitudePolicy !== "bounded-spatial-spline-v1") fail(`${label}.altitudePolicy must declare bounded XYZ smoothing`);
    if (entry.altitudeDeviationLimitM !== 12) fail(`${label}.altitudeDeviationLimitM must be 12`);
    parsed.optimizationAxes = ["x", "y", "z"];
    parsed.altitudePolicy = "bounded-spatial-spline-v1";
    parsed.altitudeDeviationLimitM = 12;
  }
  if (parsed.collisionCertified !== parsed.certified) {
    fail(`${label}.collisionCertified must agree with certified`);
  }
  if (parsed.applied && !parsed.certified) fail(`${label} applied output must be certified`);
  const shortcutOnly = parsed.method === "spacetime-shortcut" || parsed.method === "spacetime-shortcut-fillet-fallback";
  const shortcutCurve = parsed.method === "spacetime-shortcut-plus-local-quintic-bspline";
  if (parsed.method === "spacetime-shortcut-plus-local-quintic-bspline" && !spatialCurves) {
    fail(`${label} local B-spline curves require the XYZ protocol`);
  }
  if ((shortcutOnly || shortcutCurve) && !allowShortcut) {
    fail(`${label} shortcut methods require the current Manhattan v4 protocol`);
  }
  if (allowShortcut && !shortcutOnly && !shortcutCurve && parsed.method !== "not-run-uncertified-raw-path") {
    fail(`${label}.method is unsupported for the current Manhattan v4 postprocessor`);
  }
  if (shortcutOnly) {
    if (parsed.appliedTurnRadiusM !== null || parsed.roundedCornerCount !== 0) {
      fail(`${label} LOS-only shortcut method requires no fillet radius or rounded corners`);
    }
  } else if (shortcutCurve) {
    if (!parsed.applied || parsed.appliedTurnRadiusM === null || parsed.roundedCornerCount === 0) {
      fail(`${label} shortcut-plus-curve method requires an applied curve, trim scale and rounded corners`);
    }
  } else if (parsed.applied !== (parsed.appliedTurnRadiusM !== null)) {
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
  if (!(allowShortcut && (shortcutOnly || shortcutCurve)) && before !== null && after !== null && after > before + 1e-5) {
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

function stationarySegments(points) {
  const waits = [];
  for (let index = 1; index < points.length; index += 1) {
    if (sameStationaryPosition(points[index - 1].position, points[index].position)) {
      waits.push({
        startTimeS: points[index - 1].timeS,
        endTimeS: points[index].timeS,
        position: points[index].position,
      });
    }
  }
  return waits;
}

function stationaryDuration(points) {
  return stationarySegments(points).reduce(
    (total, interval) => total + interval.endTimeS - interval.startTimeS,
    0,
  );
}

function framesForPath(value, label, scenario, points) {
  const frames = list(value, label).map((raw, index) =>
    frame(raw, `${label}[${index}]`, scenario),
  );
  if (frames.length === 0 || frames.length > points.length || frames[0].timeS !== 0) {
    fail(`${label} must be a non-empty compact path subset starting at time 0`);
  }
  frames.forEach((current, index) => {
    if (
      current.event === null ||
      current.event.kind === "none" ||
      !samePoint(current.vehicle, interpolate(points, current.timeS))
    ) {
      fail(`${label}[${index}] must be a semantic event on its declared path`);
    }
    if (index > 0 && current.timeS <= frames[index - 1].timeS) {
      fail(`${label} must be strictly increasing in time`);
    }
  });
  if (!sameNumber(frames.at(-1).timeS, points.at(-1).timeS)) {
    fail(`${label} must anchor the final path waypoint`);
  }
  return frames;
}

function validateWitness(outcome, label, scenario, points) {
  const witness = outcome.minimumSeparationWitness;
  if ((outcome.minimumSeparationM === null) !== (witness === null)) {
    fail(`${label} minimum separation and witness must be present together`);
  }
  if (witness === null) return;
  if (
    !sameNumber(outcome.minimumSeparationM, witness.separationM) ||
    !sameNumber(witness.declaredSafetyMarginM, scenario.constraints.safetyMarginM)
  ) {
    fail(`${label} minimum-separation witness is inconsistent`);
  }
  if (
    witness.timeS > points.at(-1).timeS + TOLERANCE ||
    !inBounds(witness.vehiclePosition, scenario.bounds) ||
    !inBounds(witness.obstaclePosition, scenario.bounds) ||
    !samePoint(witness.vehiclePosition, interpolate(points, witness.timeS))
  ) {
    fail(`${label} minimum-separation witness is outside its metric-domain path`);
  }
  const witnessSurfaceDistance = Math.hypot(
    witness.vehiclePosition[0] - witness.obstaclePosition[0],
    witness.vehiclePosition[1] - witness.obstaclePosition[1],
    witness.vehiclePosition[2] - witness.obstaclePosition[2],
  );
  if (
    !sameNumber(
      witness.separationM,
      witnessSurfaceDistance - scenario.constraints.vehicleRadiusM,
    )
  ) {
    fail(`${label} minimum-separation witness geometry is inconsistent`);
  }
  if (witness.obstacleKind === "moving-sphere") {
    const definition = scenario.movingSphereMap.get(witness.obstacleId);
    const center = definition && interpolate(definition.keyframes, witness.timeS);
    if (
      definition === undefined ||
      center === undefined ||
      !witness.exact ||
      !sameNumber(
        Math.hypot(
          witness.obstaclePosition[0] - center[0],
          witness.obstaclePosition[1] - center[1],
          witness.obstaclePosition[2] - center[2],
        ),
        definition.radiusM,
      ) ||
      !sameNumber(
        Math.hypot(
          witness.vehiclePosition[0] - center[0],
          witness.vehiclePosition[1] - center[1],
          witness.vehiclePosition[2] - center[2],
        ),
        witnessSurfaceDistance + definition.radiusM,
      )
    ) {
      fail(`${label} moving-sphere witness is inconsistent`);
    }
  } else if (
    witness.exact ||
    !scenario.temporaryNoFlyZones.some((zone) => zone.id === witness.obstacleId)
  ) {
    fail(`${label} temporary-cylinder witness is inconsistent`);
  }
  if (
    outcome.safetyViolations === 0 &&
    witness.separationM + TOLERANCE < scenario.constraints.safetyMarginM
  ) {
    fail(`${label} safe trajectory violates its declared dynamic safety margin`);
  }
}

function validateMetricsAgainstPath(value, label, scenario, points, frames = null) {
  const outcome = metrics(value, label);
  const directDistance = Math.hypot(
    scenario.goal[0] - scenario.start[0],
    scenario.goal[1] - scenario.start[1],
    scenario.goal[2] - scenario.start[2],
  );
  const executedLength = pathLength(points.map((point) => point.position));
  const waitTime = stationaryDuration(points);
  if (
    !sameNumber(outcome.directDistanceM, directDistance) ||
    !sameNumber(outcome.executedPathLengthM, executedLength) ||
    !sameNumber(outcome.waitTimeS, waitTime)
  ) {
    fail(`${label} path length or stationary time disagrees with its metric-domain path`);
  }
  validateWitness(outcome, label, scenario, points);
  if (outcome.success) {
    if (
      outcome.failureReason !== null ||
      outcome.arrivalTimeS === null ||
      outcome.travelTimeS === null ||
      outcome.pathExcessPct === null ||
      !samePoint(points.at(-1).position, scenario.goal) ||
      !sameNumber(outcome.arrivalTimeS, points.at(-1).timeS) ||
      !sameNumber(outcome.travelTimeS, outcome.arrivalTimeS - outcome.waitTimeS)
    ) {
      fail(`${label} successful metrics are incomplete or inconsistent`);
    }
  } else if (
    outcome.failureReason === null ||
    outcome.arrivalTimeS !== null ||
    outcome.travelTimeS !== null ||
    outcome.pathExcessPct !== null
  ) {
    fail(`${label} failed metrics cannot define arrival, travel, or path excess`);
  }
  if (
    outcome.pathExcessPct !== null &&
    !sameNumber(
      outcome.pathExcessPct,
      (outcome.executedPathLengthM / outcome.directDistanceM - 1) * 100,
    )
  ) {
    fail(`${label}.pathExcessPct is inconsistent`);
  }
  if (frames !== null) {
    const finalFrame = frames.at(-1);
    if (
      !samePoint(finalFrame.vehicle, points.at(-1).position) ||
      finalFrame.event?.kind !== (outcome.success ? "goal-reached" : "no-path")
    ) {
      fail(`${label} final event disagrees with metric success`);
    }
  }
  return outcome;
}

function run(value, label, scenario, planner, allowShortcut, scheduled, spatialCurves) {
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
  Object.entries(parameters).forEach(([key, parameter]) =>
    finite(parameter, `${label}.parameters.${key}`),
  );

  const rawPoints = timedPath(entry.rawTimedPath, `${label}.rawTimedPath`, scenario.bounds);
  const geometryPoints = timedPath(
    entry.geometryTimedPath,
    `${label}.geometryTimedPath`,
    scenario.bounds,
  );
  if (
    !samePoint(rawPoints[0].position, scenario.start) ||
    !samePoint(geometryPoints[0].position, scenario.start)
  ) {
    fail(`${label} raw and geometry paths must start at scenario start`);
  }
  if (spatialCurves) {
    if (parameters.trajectoryPreserveAltitude !== 0) fail(`${label}.parameters.trajectoryPreserveAltitude must be 0 for XYZ smoothing`);
    validateAltitudeProfile(rawPoints, geometryPoints, label, 12);
  }
  const postprocess = smoothing(entry.smoothing, `${label}.smoothing`, allowShortcut, spatialCurves);
  if (
    postprocess.rawWaypointCount !== rawPoints.length ||
    postprocess.outputWaypointCount !== geometryPoints.length
  ) {
    fail(`${label}.smoothing waypoint counts disagree with raw/geometry paths`);
  }
  if (!postprocess.applied && !sameTimedPath(rawPoints, geometryPoints)) {
    fail(`${label} unapplied smoothing must preserve the raw path exactly`);
  }
  if (
    !sameNumber(rawPoints.at(-1).timeS, geometryPoints.at(-1).timeS) ||
    !samePoint(rawPoints.at(-1).position, geometryPoints.at(-1).position)
  ) {
    fail(`${label} geometry post-processing must preserve the endpoint and duration`);
  }
  const rawWaits = stationarySegments(rawPoints);
  const geometryWaits = waitIntervals(
    entry.geometryWaitIntervals,
    `${label}.geometryWaitIntervals`,
    scenario.bounds,
    geometryPoints,
  );
  if (
    rawWaits.length !== geometryWaits.length ||
    rawWaits.some(
      (wait, index) =>
        !sameNumber(wait.startTimeS, geometryWaits[index].startTimeS) ||
        !sameNumber(wait.endTimeS, geometryWaits[index].endTimeS) ||
        !samePoint(wait.position, geometryWaits[index].position),
    )
  ) {
    fail(`${label} geometry post-processing changed a planner wait interval`);
  }
  const geometryFrames = framesForPath(
    entry.geometryFrames,
    `${label}.geometryFrames`,
    scenario,
    geometryPoints,
  );

  const plannerMetrics = validateMetricsAgainstPath(
    entry.plannerMetrics,
    `${label}.plannerMetrics`,
    scenario,
    rawPoints,
  );
  const geometryMetrics = validateMetricsAgainstPath(
    entry.geometryMetrics,
    `${label}.geometryMetrics`,
    scenario,
    geometryPoints,
    geometryFrames,
  );
  const succeeded = entry.status === "success";
  if (
    plannerMetrics.success !== succeeded ||
    plannerMetrics.failureReason !== failureReason ||
    geometryMetrics.success !== plannerMetrics.success ||
    geometryMetrics.failureReason !== plannerMetrics.failureReason
  ) {
    fail(`${label} planner/geometry metric status fields disagree`);
  }
  if (succeeded && !postprocess.certified) {
    fail(`${label} successful geometry candidate must be collision-certified`);
  }

  const expectedWorkUnit =
    planner.id === "space-time-astar-4d"
      ? "expanded-spacetime-states"
      : planner.id === "dstar-lite-reset-3d" || planner.id === "dstar-lite-reuse-3d"
        ? "queue-pops"
        : planner.id === "repeated-astar-3d"
          ? "expanded-nodes"
          : null;
  if (expectedWorkUnit !== null && plannerMetrics.workUnit !== expectedWorkUnit) {
    fail(`${label}.plannerMetrics.workUnit disagrees with its planner`);
  }
  if (
    geometryMetrics.workUnit !== plannerMetrics.workUnit ||
    geometryMetrics.expandedStates !== plannerMetrics.expandedStates ||
    geometryMetrics.replans !== plannerMetrics.replans
  ) {
    fail(`${label}.geometryMetrics must retain planner work accounting`);
  }
  const visiblePlanningEvents = geometryFrames.filter(
    (item) => item.event?.kind === "replan" || item.event?.kind === "prediction-update",
  ).length;
  if (plannerMetrics.replans < visiblePlanningEvents) {
    fail(`${label}.plannerMetrics.replans is smaller than recorded planning events`);
  }

  const available =
    postprocess.execution.status === "qualified" &&
    postprocess.execution.qualified &&
    postprocess.execution.collisionCertified;
  const executionValues = [
    entry.executionTimedPath,
    entry.executionWaitIntervals,
    entry.executionMetrics,
    entry.executionFrames,
  ];
  if (executionValues.some((item) => item === null) !== !available) {
    fail(`${label} execution evidence availability disagrees with its qualification`);
  }
  if (!available && executionValues.some((item) => item !== null)) {
    fail(`${label} failed execution qualification must not expose candidate evidence`);
  }
  if (postprocess.execution.originalDurationS !== null &&
      !sameNumber(postprocess.execution.originalDurationS, geometryPoints.at(-1).timeS)) {
    fail(`${label}.smoothing.execution.originalDurationS disagrees with geometry path`);
  }
  if (!scheduled && postprocess.execution.qualification !== null &&
      postprocess.execution.qualification.diagnostics.segmentCount !== geometryPoints.length - 1) {
    fail(`${label}.smoothing.execution qualification waypoint count is inconsistent`);
  }
  if (available) {
    const executionPoints = timedPath(
      entry.executionTimedPath,
      `${label}.executionTimedPath`,
      scenario.bounds,
    );
    try { validateExecutionSequence(geometryPoints, executionPoints, scheduled); }
    catch (error) { fail(`${label}: ${error.message}`); }
    if (postprocess.execution.qualification.diagnostics.segmentCount !== executionPoints.length - 1)
      fail(`${label} qualification count disagrees with execution path`);
    if (
      !sameNumber(
        postprocess.execution.candidateDurationS,
        executionPoints.at(-1).timeS,
      ) ||
      (!scheduled && !sameNumber(stationaryDuration(executionPoints), stationaryDuration(geometryPoints)))
    ) {
      fail(`${label} execution duration metadata or preserved wait time is inconsistent`);
    }
    const executionWaits = waitIntervals(
      entry.executionWaitIntervals,
      `${label}.executionWaitIntervals`,
      scenario.bounds,
      executionPoints,
    );
    if (!sameNumber(
      executionWaits.reduce(
        (total, interval) => total + interval.endTimeS - interval.startTimeS,
        0,
      ),
      stationaryDuration(executionPoints),
    )) {
      fail(`${label}.executionWaitIntervals do not preserve total wait duration`);
    }
    const executionFrames = framesForPath(
      entry.executionFrames,
      `${label}.executionFrames`,
      scenario,
      executionPoints,
    );
    const executionMetrics = validateMetricsAgainstPath(
      entry.executionMetrics,
      `${label}.executionMetrics`,
      scenario,
      executionPoints,
      executionFrames,
    );
    if (
      executionMetrics.success !== plannerMetrics.success ||
      executionMetrics.failureReason !== plannerMetrics.failureReason ||
      executionMetrics.workUnit !== plannerMetrics.workUnit ||
      executionMetrics.expandedStates !== plannerMetrics.expandedStates ||
      executionMetrics.replans !== plannerMetrics.replans
    ) {
      fail(`${label}.executionMetrics must retain planner outcome and work accounting`);
    }
  }

  const runId = text(entry.runId, `${label}.runId`);
  if (!/^sha256:[0-9a-f]{64}$/.test(runId)) fail(`${label}.runId must be a SHA-256 digest`);
  return { runId, plannerId: planner.id };
}

function scenario(value, label, planners, declaredProtocol) {
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
    if (building.footprint !== undefined) {
      list(building.footprint, `${label}.buildings[${index}].footprint`).forEach((ring, ringIndex) => {
        const points = list(ring, `${label}.buildings[${index}].footprint[${ringIndex}]`);
        if (points.length < 4) fail(`${label}: footprint rings require at least four vertices`);
        points.forEach((point, pointIndex) => {
          const xy = vec2(point, `${label}.buildings[${index}].footprint[${ringIndex}][${pointIndex}]`);
          if (xy.some((coordinate, axis) => coordinate < bounds.min[axis] - TOLERANCE || coordinate > bounds.max[axis] + TOLERANCE)) {
            fail(`${label}: footprint vertices must lie inside the city bounds`);
          }
        });
      });
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
  const city = entry.city === undefined ? null : object(entry.city, `${label}.city`);
  if (city && (city.sourceKind !== "nyc-open-data" || city.collisionModel !== "conservative-aabb" || !/^(?:sha256:)?[0-9a-f]{64}$/.test(city.sourceSha256))) {
    fail(`${label}.city must declare its NYC source and conservative building collision geometry`);
  }
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
    const expectedEnvelope = (declaredProtocol.id === MANHATTAN_V4_PROTOCOL_ID) ? MANHATTAN_ENVELOPE : FROZEN_ENVELOPE;
    const rawSmoothing = object(runEntry.smoothing, `${label}.runs[${index}].smoothing`);
    const rawExecution = object(rawSmoothing.execution, `${label}.runs[${index}].smoothing.execution`);
    if (!matchesEnvelope(object(rawExecution.envelope, `${label}.runs[${index}].smoothing.execution.envelope`), expectedEnvelope)) {
      fail(`${label}: run execution envelope disagrees with the declared protocol`);
    }
    if ((declaredProtocol.id === MANHATTAN_V4_PROTOCOL_ID)) {
      const parameters = object(runEntry.parameters, `${label}.runs[${index}].parameters`);
      if (parameters.trajectoryShortcut !== 1) fail(`${label}: parameters.trajectoryShortcut must be 1 for current Manhattan v4`);
      if (declaredProtocol.trajectoryCurveDegree === 5 && parameters.trajectoryCurveDegree !== 5) {
        fail(`${label}: parameters.trajectoryCurveDegree must be 5 for local B-spline curves`);
      }
      if (plannerId === "space-time-astar-4d") {
        if (parameters.spaceTimeConnectivity !== 26) fail(`${label}: parameters.spaceTimeConnectivity must be 26 for current Manhattan v4`);
      } else if (parameters.spaceTimeConnectivity !== undefined) {
        fail(`${label}: spaceTimeConnectivity is only valid for the 4D planner`);
      }
    }
    return run(
      runEntry, `${label}.runs[${index}]`, parsedScenario, planner,
      (declaredProtocol.id === MANHATTAN_V4_PROTOCOL_ID),

      declaredProtocol.trajectoryDynamicScheduling === "certified-move-block-departures",
      declaredProtocol.id === MANHATTAN_V4_PROTOCOL_ID,
    );
  });
  if (runs.length !== planners.size || [...planners.keys()].some((id) => !seen.has(id))) {
    fail(`${label} must contain exactly one run for every declared planner`);
  }
  const dynamicHazards = temporaryNoFlyZones.length + movingSpheres.length;
  return {
    id: parsedScenario.id,
    fingerprint,
    cohort,
    city,
    bounds,
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
  if (root.schemaVersion !== 3) fail("schemaVersion must be 3");
  if (root.verificationStatus !== "PREDICTIVE_DEMO_NON_CONFIRMATORY") {
    fail("verificationStatus must be PREDICTIVE_DEMO_NON_CONFIRMATORY");
  }
  const generatedAt = text(root.generatedAt, "generatedAt");
  if (!Number.isFinite(Date.parse(generatedAt))) fail("generatedAt must be an ISO timestamp");
  const sourceCommit = text(root.sourceCommit, "sourceCommit");
  const protocol = parseProtocol(root.protocol);
  if ((protocol.id === MANHATTAN_V4_PROTOCOL_ID)) {
    if (!/^local-snapshot:sha256:[0-9a-f]{64}$/.test(sourceCommit)) {
      fail("Manhattan sourceCommit must explicitly identify a local SHA-256 snapshot");
    }
    const provenance = object(root.sourceProvenance, "sourceProvenance");
    if (provenance.kind !== "local-snapshot" || provenance.sha256 !== sourceCommit.slice("local-snapshot:".length)) {
      fail("sourceProvenance must agree with the local snapshot digest");
    }
    const files = list(provenance.files, "sourceProvenance.files");
    if (!files.length) fail("sourceProvenance.files cannot be empty");
    for (const file of files) {
      const entry = object(file, "sourceProvenance.files entry");
      text(entry.path, "sourceProvenance file path");
      if (!/^sha256:[0-9a-f]{64}$/.test(text(entry.sha256, "sourceProvenance file sha256"))) fail("source file digest is invalid");
    }
  } else if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(sourceCommit)) {
    fail("sourceCommit must be a full lowercase Git object ID");
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
    scenario(entry, `scenarios[${index}]`, planners, protocol),
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
  const declaredIds = (protocol.id === MANHATTAN_V4_PROTOCOL_ID) ? MANHATTAN_IDS : expectedScenarioIds;
  if (scenarios.length !== declaredIds.length || declaredIds.some((id) => !scenarios.some((entry) => entry.id === id))) {
    fail((protocol.id === MANHATTAN_V4_PROTOCOL_ID)
      ? "the Manhattan protocol requires its eight declared missions"
      : "the public v0.7 protocol requires the ten declared scenarios");
  }
  if ((protocol.id === MANHATTAN_V4_PROTOCOL_ID) && scenarios.some((entry) => !entry.city || entry.buildingCount < 1000 || entry.city.buildingCount !== entry.buildingCount)) {
    fail("Manhattan missions require NYC provenance and the complete dense city geometry");
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
  if (!complexDemo && !(protocol.id === MANHATTAN_V4_PROTOCOL_ID)) {
    fail("at least one demo scenario must have 14 buildings, a static NFZ, and two dynamic hazards");
  }
  const runIds = scenarios.flatMap((entry) => entry.runs.map((record) => record.runId));
  const expectedRunCount = (protocol.id === MANHATTAN_V4_PROTOCOL_ID) ? 32 : 40;
  if (runIds.length !== expectedRunCount) fail(`the ${protocol.id} protocol requires exactly ${expectedRunCount} planner runs`);
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
    protocolId: protocol.id,
    scenarioCount: scenarios.length,
    runCount: runIds.length,
    complexDemoId: complexDemo?.id ?? null,
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

async function verifyArtifact(reference, protocolId) {
  const artifactPath = resolve(dirname(inputPath), reference.path);
  const [contents, metadata] = await Promise.all([readFile(artifactPath), stat(artifactPath)]);
  const digest = `sha256:${createHash("sha256").update(contents).digest("hex")}`;
  if (metadata.size !== reference.bytes) {
    fail(`${reference.path} size is ${metadata.size}, expected ${reference.bytes}`);
  }
  if (digest !== reference.sha256) {
    fail(`${reference.path} digest is ${digest}, expected ${reference.sha256}`);
  }
  if (reference.path === "predictive-scenario-manifest.json") {
    let manifest;
    try {
      manifest = object(JSON.parse(contents.toString("utf8")), "scenarioManifest");
    } catch (error) {
      fail(
        `predictive-scenario-manifest.json is invalid JSON (${error instanceof Error ? error.message : String(error)})`,
      );
    }
    if (manifest.schemaVersion !== 3) {
      fail("predictive-scenario-manifest.json schemaVersion must be 3");
    }
    if (manifest.protocolId !== protocolId) {
      fail("predictive-scenario-manifest.json protocolId must agree with the bundle");
    }
    const datasetId = protocolId === MANHATTAN_V4_PROTOCOL_ID
      ? "manhattan-island-missions-v4" : "predictive-execution-envelope-v0.7";
    if (manifest.datasetId !== datasetId) {
      fail("predictive-scenario-manifest.json datasetId must agree with the declared protocol");
    }
  }
}

let value;
try {
  value = await readStudyData(inputPath);
} catch (error) {
  fail(`invalid JSON (${error instanceof Error ? error.message : String(error)})`);
}
const result = validateBundle(value);
await Promise.all(result.artifacts.map((reference) => verifyArtifact(reference, result.protocolId)));
console.log(
  `Validated ${result.scenarioCount} predictive scenarios, ${result.runCount} runs, complex demo ${result.complexDemoId}, and ${result.artifacts.length} referenced artifacts.`,
);
