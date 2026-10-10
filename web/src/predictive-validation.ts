import { auditSharedWorld } from "./shared-world";
import { ValidationCache } from "./validation-cache";
import { isRuntimeData } from "../shared/runtime-data.mjs";
import type {
  ArtifactReference,
  Bounds3,
  MovingSphereDefinition,
  MovingSphereKeyframe,
  MovingSphereState,
  PredictiveBundleV3,
  PredictiveExecutionEnvelope,
  PredictiveExecutionEvidence,
  PredictiveExecutionQualification,
  PredictiveExecutionStatus,
  PredictiveEvent,
  PredictiveFrame,
  PredictiveDiscreteKinematicDiagnostics,
  PredictiveKinematicDiagnostics,
  PredictiveMinimumSeparationWitness,
  PredictivePlanner,
  PredictiveProtocol,
  PredictiveRun,
  PredictiveRunMetrics,
  PredictiveScenario,
  PredictiveSmoothing,
  StaticNoFlyZone,
  TemporaryNoFlyZone,
  TimedWaypoint,
  Vec3,
  WaitInterval,
} from "./predictive-schema";
import { auditMissionTaskVisits, parseBuildingFootprint, parseCityMetadata, parseCityMission } from "./city-schema";
import { validateExecutionSequence } from "../shared/execution-sequence.mjs";

const TOLERANCE = 1e-6;
const FROZEN_EXECUTION_ENVELOPE = {
  maxSpeedMps: 8,
  maxAbsClimbRateMps: 3,
  maxDiscreteAccelerationProxyMps2: 4,
  reversalThresholdDeg: 150,
  allowReversals: false,
  maxExecutionTimeS: 90,
} as const;
const MANHATTAN_V4_PROTOCOL_ID = "manhattan-space-time-v4";
const MANHATTAN_SPATIAL_POSTPROCESSOR = "spatial-local-quintic-bspline-bounded-altitude-envelope-v6";
const MANHATTAN_EXECUTION_ENVELOPE = {
  ...FROZEN_EXECUTION_ENVELOPE,
  maxSpeedMps: 15,
  maxExecutionTimeS: 900,
} as const;

function matchesEnvelope(
  observed: PredictiveExecutionEnvelope,
  expected: typeof FROZEN_EXECUTION_ENVELOPE | typeof MANHATTAN_EXECUTION_ENVELOPE,
): boolean {
  return Object.entries(expected).every(([key, value]) => {
    const actual = observed[key as keyof typeof expected];
    return typeof value === "number"
      ? typeof actual === "number" && sameNumber(actual, value)
      : actual === value;
  });
}

function fail(message: string): never {
  throw new Error(`predictive-data.json: ${message}`);
}

function record(value: unknown, label: string): Record<string, unknown> {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    fail(`${label} must be an object`);
  }
  return value as Record<string, unknown>;
}

function array(value: unknown, label: string): unknown[] {
  if (!Array.isArray(value)) fail(`${label} must be an array`);
  return value;
}

function text(value: unknown, label: string): string {
  if (typeof value !== "string" || value.trim() === "") fail(`${label} must be a string`);
  return value;
}

function nullableText(value: unknown, label: string): string | null {
  return value === null ? null : text(value, label);
}

function finite(value: unknown, label: string): number {
  if (typeof value !== "number" || !Number.isFinite(value)) fail(`${label} must be finite`);
  return value;
}

function nonNegative(value: unknown, label: string): number {
  const parsed = finite(value, label);
  if (parsed < 0) fail(`${label} cannot be negative`);
  return parsed;
}

function positive(value: unknown, label: string): number {
  const parsed = finite(value, label);
  if (parsed <= 0) fail(`${label} must be positive`);
  return parsed;
}

function nonNegativeInteger(value: unknown, label: string): number {
  const parsed = nonNegative(value, label);
  if (!Number.isInteger(parsed)) fail(`${label} must be an integer`);
  return parsed;
}

function positiveInteger(value: unknown, label: string): number {
  const parsed = positive(value, label);
  if (!Number.isInteger(parsed)) fail(`${label} must be an integer`);
  return parsed;
}

function boolean(value: unknown, label: string): boolean {
  if (typeof value !== "boolean") fail(`${label} must be boolean`);
  return value;
}

const validationCache = new ValidationCache();
function vec3(value: unknown, label: string): Vec3 {
  return validationCache.memo(value, "vec3", () => {
  const coordinates = array(value, label);
  if (coordinates.length !== 3) fail(`${label} must contain three coordinates`);
  return coordinates.map((coordinate, index) =>
    finite(coordinate, `${label}[${index}]`),
  ) as Vec3;
  });
}

function vec2(value: unknown, label: string): [number, number] {
  const coordinates = array(value, label);
  if (coordinates.length !== 2) fail(`${label} must contain two coordinates`);
  return coordinates.map((coordinate, index) =>
    finite(coordinate, `${label}[${index}]`),
  ) as [number, number];
}

function sameNumber(left: number, right: number): boolean {
  return Math.abs(left - right) <= TOLERANCE * Math.max(1, Math.abs(left), Math.abs(right));
}

function samePoint(left: Vec3, right: Vec3): boolean {
  return left.every((coordinate, index) => sameNumber(coordinate, right[index]!));
}

function sameStationaryPosition(left: Vec3, right: Vec3): boolean {
  // Match Python geometry.almost_equal: motion must not depend on the ENU origin.
  return Math.hypot(...left.map((coordinate, index) => coordinate - right[index]!)) <= 1e-9;
}

function pointInBounds(point: Vec3, sceneBounds: Bounds3): boolean {
  return point.every(
    (coordinate, index) =>
      coordinate >= sceneBounds.min[index]! - TOLERANCE &&
      coordinate <= sceneBounds.max[index]! + TOLERANCE,
  );
}

function polylineLength(path: Vec3[]): number {
  let total = 0;
  for (let index = 1; index < path.length; index += 1) {
    total += Math.hypot(
      path[index]![0] - path[index - 1]![0],
      path[index]![1] - path[index - 1]![1],
      path[index]![2] - path[index - 1]![2],
    );
  }
  return total;
}

function parseBounds(value: unknown, label: string): Bounds3 {
  const item = record(value, label);
  const parsed = { min: vec3(item.min, `${label}.min`), max: vec3(item.max, `${label}.max`) };
  if (parsed.min.some((coordinate, index) => coordinate >= parsed.max[index]!)) {
    fail(`${label}.min must be strictly below max`);
  }
  return parsed;
}

function uniqueIdObjects(value: unknown, label: string): Array<Record<string, unknown>> {
  const items = array(value, label).map((item, index) => record(item, `${label}[${index}]`));
  const ids = new Set<string>();
  for (const [index, item] of items.entries()) {
    const id = text(item.id, `${label}[${index}].id`);
    if (ids.has(id)) fail(`${label} contains duplicate ID ${id}`);
    ids.add(id);
  }
  return items;
}

function executionEnvelope(value: unknown, label: string): PredictiveExecutionEnvelope {
  const item = record(value, label);
  if (item.model !== "discrete-segment-average-envelope-v1") {
    fail(`${label}.model must be discrete-segment-average-envelope-v1`);
  }
  if (item.continuousDynamicsCertified !== false) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  const parsed: PredictiveExecutionEnvelope = {
    model: "discrete-segment-average-envelope-v1",
    maxSpeedMps: positive(item.maxSpeedMps, `${label}.maxSpeedMps`),
    maxAbsClimbRateMps: positive(
      item.maxAbsClimbRateMps,
      `${label}.maxAbsClimbRateMps`,
    ),
    maxDiscreteAccelerationProxyMps2: positive(
      item.maxDiscreteAccelerationProxyMps2,
      `${label}.maxDiscreteAccelerationProxyMps2`,
    ),
    reversalThresholdDeg: positive(
      item.reversalThresholdDeg,
      `${label}.reversalThresholdDeg`,
    ),
    allowReversals: boolean(item.allowReversals, `${label}.allowReversals`),
    maxExecutionTimeS: positive(item.maxExecutionTimeS, `${label}.maxExecutionTimeS`),
    continuousDynamicsCertified: false,
  };
  if (parsed.reversalThresholdDeg > 180 + TOLERANCE) {
    fail(`${label}.reversalThresholdDeg cannot exceed 180 degrees`);
  }
  if (!matchesEnvelope(parsed, FROZEN_EXECUTION_ENVELOPE) && !matchesEnvelope(parsed, MANHATTAN_EXECUTION_ENVELOPE)) {
    fail(`${label} disagrees with the frozen v0.7 execution envelope or Manhattan execution envelope`);
  }
  return parsed;
}

function protocol(value: unknown): PredictiveProtocol {
  const item = record(value, "protocol");
  if (item.id !== "predictive-space-time-v4" && item.id !== MANHATTAN_V4_PROTOCOL_ID) {
    fail("protocol.id must be predictive-space-time-v4 or manhattan-space-time-v4");
  }
  if (item.continuousDynamicsCertified !== false) {
    fail("protocol.continuousDynamicsCertified must remain false");
  }
  const domains = record(item.metricDomains, "protocol.metricDomains");
  const metricDomains = {
    plannerMetrics: text(domains.plannerMetrics, "protocol.metricDomains.plannerMetrics"),
    geometryMetrics: text(domains.geometryMetrics, "protocol.metricDomains.geometryMetrics"),
    executionMetrics: text(domains.executionMetrics, "protocol.metricDomains.executionMetrics"),
  };
  const expectedDomains = {
    plannerMetrics: "raw planner or simulator output only",
    geometryMetrics: "common collision-certified geometric post-processing",
    executionMetrics: "optional discrete-envelope-qualified retimed candidate",
  };
  for (const key of Object.keys(expectedDomains) as Array<keyof typeof expectedDomains>) {
    if (metricDomains[key] !== expectedDomains[key]) {
      fail(`protocol.metricDomains.${key} does not declare the v0.7 evidence domain`);
    }
  }
  const parsed: PredictiveProtocol = {
    id: text(item.id, "protocol.id"),
    timeStepS: positive(item.timeStepS, "protocol.timeStepS"),
    cruiseSpeedMps: positive(item.cruiseSpeedMps, "protocol.cruiseSpeedMps"),
    maxTimeS: positive(item.maxTimeS, "protocol.maxTimeS"),
    resolutionM: positive(item.resolutionM, "protocol.resolutionM"),
    timeResolutionS: positive(item.timeResolutionS, "protocol.timeResolutionS"),
    planningHorizonS: positive(item.planningHorizonS, "protocol.planningHorizonS"),
    predictionHorizonS: positive(item.predictionHorizonS, "protocol.predictionHorizonS"),
    reactiveMaxWorkPerReplan: positiveInteger(
      item.reactiveMaxWorkPerReplan,
      "protocol.reactiveMaxWorkPerReplan",
    ),
    predictiveMaxExpandedStatesPerMission: positiveInteger(
      item.predictiveMaxExpandedStatesPerMission,
      "protocol.predictiveMaxExpandedStatesPerMission",
    ),
    trajectoryPostprocessor: text(
      item.trajectoryPostprocessor,
      "protocol.trajectoryPostprocessor",
    ),
    executionEnvelope: executionEnvelope(item.executionEnvelope, "protocol.executionEnvelope"),
    continuousDynamicsCertified: false,
    metricDomains,
  };
  if ((parsed.id === MANHATTAN_V4_PROTOCOL_ID)) {
    if (item.spaceTimeConnectivity !== 26) {
      fail("protocol.spaceTimeConnectivity must be 26 for current Manhattan v4");
    }
    if (item.trajectoryShortcut !== true) {
      fail("protocol.trajectoryShortcut must be true for current Manhattan v4");
    }
    if (parsed.trajectoryPostprocessor !== MANHATTAN_SPATIAL_POSTPROCESSOR) {
      fail("protocol.trajectoryPostprocessor disagrees with the declared Manhattan postprocessor");
    }
    parsed.spaceTimeConnectivity = 26;
    parsed.trajectoryShortcut = true;
  }
  if (parsed.id === MANHATTAN_V4_PROTOCOL_ID) {
    if (item.trajectoryPreserveAltitude !== false) fail("protocol.trajectoryPreserveAltitude must be false for Manhattan v4");
    if (item.trajectoryCurveDegree !== 5) fail("protocol.trajectoryCurveDegree must be 5");
    if (item.trajectoryCurveDimensions !== 3) fail("protocol.trajectoryCurveDimensions must be 3");
    if (item.trajectoryAltitudeDeviationLimitM !== 12) fail("protocol.trajectoryAltitudeDeviationLimitM must be 12");
    if (item.trajectoryDynamicScheduling !== "certified-move-block-departures") fail("Unsupported dynamic execution scheduling");
    parsed.trajectoryPreserveAltitude = false;
    parsed.trajectoryCurveDegree = 5;
    parsed.trajectoryCurveDimensions = 3;
    parsed.trajectoryAltitudeDeviationLimitM = 12;
    parsed.trajectoryDynamicScheduling = item.trajectoryDynamicScheduling;
  }
  if (parsed.timeResolutionS > parsed.planningHorizonS) {
    fail("protocol.timeResolutionS cannot exceed planningHorizonS");
  }
  if (parsed.predictionHorizonS > parsed.planningHorizonS) {
    fail("protocol.predictionHorizonS cannot exceed planningHorizonS");
  }
  if (parsed.planningHorizonS > parsed.maxTimeS) {
    fail("protocol.planningHorizonS cannot exceed maxTimeS");
  }
  if (!sameNumber(parsed.cruiseSpeedMps, parsed.executionEnvelope.maxSpeedMps)) {
    fail("protocol.cruiseSpeedMps must agree with executionEnvelope.maxSpeedMps");
  }
  if (!sameNumber(parsed.maxTimeS, parsed.executionEnvelope.maxExecutionTimeS)) {
    fail("protocol.maxTimeS must agree with executionEnvelope.maxExecutionTimeS");
  }
  const expectedEnvelope = (parsed.id === MANHATTAN_V4_PROTOCOL_ID)
    ? MANHATTAN_EXECUTION_ENVELOPE : FROZEN_EXECUTION_ENVELOPE;
  if (!matchesEnvelope(parsed.executionEnvelope, expectedEnvelope)) {
    fail("protocol.executionEnvelope disagrees with the declared protocol");
  }
  return parsed;
}

function artifact(value: unknown, label: string, expectedPath: string): ArtifactReference {
  const item = record(value, label);
  if (item.path !== expectedPath) fail(`${label}.path must be ${expectedPath}`);
  const sha256 = text(item.sha256, `${label}.sha256`);
  if (!/^sha256:[0-9a-f]{64}$/.test(sha256)) fail(`${label}.sha256 is invalid`);
  return {
    path: expectedPath,
    sha256: sha256 as `sha256:${string}`,
    bytes: positiveInteger(item.bytes, `${label}.bytes`),
  };
}

function staticZone(value: unknown, label: string, sceneBounds: Bounds3): StaticNoFlyZone {
  const item = record(value, label);
  const parsed: StaticNoFlyZone = {
    id: text(item.id, `${label}.id`),
    center: vec2(item.center, `${label}.center`),
    radiusM: positive(item.radiusM, `${label}.radiusM`),
    zMinM: finite(item.zMinM, `${label}.zMinM`),
    zMaxM: finite(item.zMaxM, `${label}.zMaxM`),
  };
  if (parsed.zMinM >= parsed.zMaxM) fail(`${label} must have positive height`);
  if (
    parsed.center[0] - parsed.radiusM < sceneBounds.min[0] - TOLERANCE ||
    parsed.center[0] + parsed.radiusM > sceneBounds.max[0] + TOLERANCE ||
    parsed.center[1] - parsed.radiusM < sceneBounds.min[1] - TOLERANCE ||
    parsed.center[1] + parsed.radiusM > sceneBounds.max[1] + TOLERANCE ||
    parsed.zMinM < sceneBounds.min[2] - TOLERANCE ||
    parsed.zMaxM > sceneBounds.max[2] + TOLERANCE
  ) {
    fail(`${label} must be contained by scenario bounds`);
  }
  return parsed;
}

function temporaryZone(
  value: unknown,
  label: string,
  sceneBounds: Bounds3,
): TemporaryNoFlyZone {
  const base = staticZone(value, label, sceneBounds);
  const item = value as Record<string, unknown>;
  const activeFromS = nonNegative(item.activeFromS, `${label}.activeFromS`);
  const activeUntilS = positive(item.activeUntilS, `${label}.activeUntilS`);
  if (activeFromS >= activeUntilS) fail(`${label} has an invalid half-open active interval`);
  return { ...base, activeFromS, activeUntilS };
}

function movingSphere(
  value: unknown,
  label: string,
  sceneBounds: Bounds3,
): MovingSphereDefinition {
  const item = record(value, label);
  const keyframes: MovingSphereKeyframe[] = array(item.keyframes, `${label}.keyframes`).map(
    (entry, index) => {
      const keyframe = record(entry, `${label}.keyframes[${index}]`);
      const position = vec3(keyframe.position, `${label}.keyframes[${index}].position`);
      if (!pointInBounds(position, sceneBounds)) {
        fail(`${label}.keyframes[${index}].position is outside scenario bounds`);
      }
      return {
        timeS: nonNegative(keyframe.timeS, `${label}.keyframes[${index}].timeS`),
        position,
      };
    },
  );
  if (keyframes.length < 2) fail(`${label} requires at least two keyframes`);
  for (let index = 1; index < keyframes.length; index += 1) {
    if (keyframes[index]!.timeS <= keyframes[index - 1]!.timeS) {
      fail(`${label}.keyframes must be strictly increasing in time`);
    }
  }
  return {
    id: text(item.id, `${label}.id`),
    radiusM: positive(item.radiusM, `${label}.radiusM`),
    keyframes,
  };
}

function movingPosition(definition: MovingSphereDefinition, timeS: number): Vec3 {
  const first = definition.keyframes[0]!;
  const last = definition.keyframes.at(-1)!;
  if (timeS <= first.timeS) return first.position;
  if (timeS >= last.timeS) return last.position;
  for (let index = 1; index < definition.keyframes.length; index += 1) {
    const right = definition.keyframes[index]!;
    const left = definition.keyframes[index - 1]!;
    if (timeS <= right.timeS) {
      const fraction = (timeS - left.timeS) / (right.timeS - left.timeS);
      return left.position.map(
        (coordinate, axis) => coordinate + (right.position[axis]! - coordinate) * fraction,
      ) as Vec3;
    }
  }
  return last.position;
}

function timedPath(value: unknown, label: string, sceneBounds: Bounds3): TimedWaypoint[] {
  const parsed = array(value, label).map((entry, index) => {
    const item = record(entry, `${label}[${index}]`);
    const position = vec3(item.position, `${label}[${index}].position`);
    if (!pointInBounds(position, sceneBounds)) {
      fail(`${label}[${index}].position is outside scenario bounds`);
    }
    return {
      timeS: nonNegative(item.timeS, `${label}[${index}].timeS`),
      position,
    };
  });
  if (parsed.length === 0 || parsed[0]!.timeS !== 0) {
    fail(`${label} must be non-empty and start at time 0`);
  }
  for (let index = 1; index < parsed.length; index += 1) {
    if (parsed[index]!.timeS <= parsed[index - 1]!.timeS) {
      fail(`${label} must be strictly increasing in time`);
    }
  }
  return parsed;
}

function sameTimedPath(left: TimedWaypoint[], right: TimedWaypoint[]): boolean {
  return (
    left.length === right.length &&
    left.every(
      (waypoint, index) =>
        sameNumber(waypoint.timeS, right[index]!.timeS) &&
        samePoint(waypoint.position, right[index]!.position),
    )
  );
}

function timedPosition(waypoints: TimedWaypoint[], timeS: number): Vec3 {
  const first = waypoints[0]!;
  const last = waypoints.at(-1)!;
  if (timeS <= first.timeS) return first.position;
  if (timeS >= last.timeS) return last.position;
  for (let index = 1; index < waypoints.length; index += 1) {
    const right = waypoints[index]!;
    const left = waypoints[index - 1]!;
    if (timeS <= right.timeS) {
      const fraction = (timeS - left.timeS) / (right.timeS - left.timeS);
      return left.position.map(
        (coordinate, axis) => coordinate + (right.position[axis]! - coordinate) * fraction,
      ) as Vec3;
    }
  }
  return last.position;
}

function validateAltitudeProfile(raw: TimedWaypoint[], geometry: TimedWaypoint[], label: string, maxDeviationM: number): void {
  if (Math.abs(raw.at(-1)!.timeS - geometry.at(-1)!.timeS) > TOLERANCE) {
    fail(`${label} altitude preservation requires the same absolute-time domain`);
  }
  // A difference of two linear profiles is linear between their union of knots. Its maximum
  // absolute value therefore occurs at one of these knots, including removed raw height knots.
  // This is an absolute metre tolerance, not sameNumber's coordinate-relative tolerance.
  const times = new Set([...raw, ...geometry].map((waypoint) => waypoint.timeS));
  for (const timeS of times) {
    if (Math.abs(timedPosition(raw, timeS)[2] - timedPosition(geometry, timeS)[2]) > maxDeviationM + TOLERANCE) {
      fail(`${label} geometry exceeds its declared altitude deviation at a profile knot`);
    }
  }
}

function waitIntervals(
  value: unknown,
  label: string,
  sceneBounds: Bounds3,
  waypoints: TimedWaypoint[],
): WaitInterval[] {
  const parsed = array(value, label).map((entry, index) => {
    const item = record(entry, `${label}[${index}]`);
    const position = vec3(item.position, `${label}[${index}].position`);
    if (!pointInBounds(position, sceneBounds)) {
      fail(`${label}[${index}].position is outside scenario bounds`);
    }
    const interval: WaitInterval = {
      startTimeS: nonNegative(item.startTimeS, `${label}[${index}].startTimeS`),
      endTimeS: positive(item.endTimeS, `${label}[${index}].endTimeS`),
      position,
      reason: text(item.reason, `${label}[${index}].reason`),
    };
    if (interval.startTimeS >= interval.endTimeS) {
      fail(`${label}[${index}] must have positive duration`);
    }
    if (interval.endTimeS > waypoints.at(-1)!.timeS + TOLERANCE) {
      fail(`${label}[${index}] extends beyond timedPath`);
    }
    if (
      !sameStationaryPosition(timedPosition(waypoints, interval.startTimeS), interval.position) ||
      !sameStationaryPosition(timedPosition(waypoints, interval.endTimeS), interval.position) ||
      waypoints.some(
        (waypoint) =>
          waypoint.timeS > interval.startTimeS &&
          waypoint.timeS < interval.endTimeS &&
          !sameStationaryPosition(waypoint.position, interval.position),
      )
    ) {
      fail(`${label}[${index}] is not stationary in timedPath`);
    }
    return interval;
  });
  for (let index = 1; index < parsed.length; index += 1) {
    if (parsed[index]!.startTimeS < parsed[index - 1]!.endTimeS - TOLERANCE) {
      fail(`${label} must be sorted and non-overlapping`);
    }
  }
  return parsed;
}

function predictiveEvent(value: unknown, label: string): PredictiveEvent | null {
  if (value === null) return null;
  const item = record(value, label);
  const kinds = new Set<PredictiveEvent["kind"]>([
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
  if (!kinds.has(item.kind as PredictiveEvent["kind"])) fail(`${label}.kind is unsupported`);
  return {
    kind: item.kind as PredictiveEvent["kind"],
    label: text(item.label, `${label}.label`),
    subjectId: item.subjectId === null ? null : text(item.subjectId, `${label}.subjectId`),
  };
}

function movingState(
  value: unknown,
  label: string,
  definitions: Map<string, MovingSphereDefinition>,
  sceneBounds: Bounds3,
  timeS: number,
): MovingSphereState {
  const item = record(value, label);
  const id = text(item.id, `${label}.id`);
  const definition = definitions.get(id);
  if (!definition) fail(`${label}.id does not reference a declared moving sphere`);
  const position = vec3(item.position, `${label}.position`);
  if (!pointInBounds(position, sceneBounds)) fail(`${label}.position is outside scenario bounds`);
  if (!samePoint(position, movingPosition(definition, timeS))) {
    fail(`${label}.position disagrees with the declared keyframes`);
  }
  const radiusM = positive(item.radiusM, `${label}.radiusM`);
  if (!sameNumber(radiusM, definition.radiusM)) fail(`${label}.radiusM disagrees with its definition`);
  return { id, position, radiusM };
}

function frame(
  value: unknown,
  label: string,
  scenario: Pick<
    PredictiveScenario,
    "bounds" | "start" | "goal" | "temporaryNoFlyZones" | "movingSpheres"
  >,
): PredictiveFrame {
  const item = record(value, label);
  if ("path" in item || "executedPath" in item) {
    fail(`${label} must not duplicate O(N) path geometry`);
  }
  const timeS = nonNegative(item.timeS, `${label}.timeS`);
  const vehicle = vec3(item.vehicle, `${label}.vehicle`);
  if (!pointInBounds(vehicle, scenario.bounds)) fail(`${label}.vehicle is outside scenario bounds`);

  const temporaryIds = new Set(scenario.temporaryNoFlyZones.map((zone) => zone.id));
  const activeTemporaryZoneIds = array(
    item.activeTemporaryZoneIds,
    `${label}.activeTemporaryZoneIds`,
  ).map((id, index) => {
    const parsed = text(id, `${label}.activeTemporaryZoneIds[${index}]`);
    if (!temporaryIds.has(parsed)) fail(`${label} activates unknown temporary zone ${parsed}`);
    return parsed;
  });
  if (new Set(activeTemporaryZoneIds).size !== activeTemporaryZoneIds.length) {
    fail(`${label}.activeTemporaryZoneIds contains duplicates`);
  }
  const expectedActive = new Set(
    scenario.temporaryNoFlyZones
      .filter((zone) => zone.activeFromS <= timeS && timeS < zone.activeUntilS)
      .map((zone) => zone.id),
  );
  if (
    activeTemporaryZoneIds.length !== expectedActive.size ||
    activeTemporaryZoneIds.some((id) => !expectedActive.has(id))
  ) {
    fail(`${label}.activeTemporaryZoneIds disagrees with the half-open schedules`);
  }

  const definitions = new Map(scenario.movingSpheres.map((sphere) => [sphere.id, sphere]));
  const movingSpheres = array(item.movingSpheres, `${label}.movingSpheres`).map((state, index) =>
    movingState(state, `${label}.movingSpheres[${index}]`, definitions, scenario.bounds, timeS),
  );
  if (
    movingSpheres.length !== definitions.size ||
    new Set(movingSpheres.map((state) => state.id)).size !== movingSpheres.length
  ) {
    fail(`${label}.movingSpheres must report every declared sphere exactly once`);
  }

  const event = predictiveEvent(item.event, `${label}.event`);
  if (event?.kind === "goal-reached" && !samePoint(vehicle, scenario.goal)) {
    fail(`${label} goal-reached event must be located at the goal`);
  }
  return {
    timeS,
    vehicle,
    activeTemporaryZoneIds,
    movingSpheres,
    event,
  };
}

function minimumSeparationWitness(
  value: unknown,
  label: string,
): PredictiveMinimumSeparationWitness | null {
  if (value === null) return null;
  const item = record(value, label);
  const obstacleKind = item.obstacleKind;
  if (obstacleKind !== "moving-sphere" && obstacleKind !== "temporary-cylinder") {
    fail(`${label}.obstacleKind is unsupported`);
  }
  return {
    separationM: finite(item.separationM, `${label}.separationM`),
    timeS: nonNegative(item.timeS, `${label}.timeS`),
    vehiclePosition: vec3(item.vehiclePosition, `${label}.vehiclePosition`),
    obstacleId: text(item.obstacleId, `${label}.obstacleId`),
    obstacleKind,
    obstaclePosition: vec3(item.obstaclePosition, `${label}.obstaclePosition`),
    declaredSafetyMarginM: nonNegative(
      item.declaredSafetyMarginM,
      `${label}.declaredSafetyMarginM`,
    ),
    method: text(item.method, `${label}.method`),
    exact: boolean(item.exact, `${label}.exact`),
  };
}

function metrics(value: unknown, label: string): PredictiveRunMetrics {
  const item = record(value, label);
  return {
    success: boolean(item.success, `${label}.success`),
    failureReason: nullableText(item.failureReason, `${label}.failureReason`),
    arrivalTimeS:
      item.arrivalTimeS === null ? null : nonNegative(item.arrivalTimeS, `${label}.arrivalTimeS`),
    travelTimeS:
      item.travelTimeS === null ? null : nonNegative(item.travelTimeS, `${label}.travelTimeS`),
    waitTimeS: nonNegative(item.waitTimeS, `${label}.waitTimeS`),
    executedPathLengthM: nonNegative(
      item.executedPathLengthM,
      `${label}.executedPathLengthM`,
    ),
    directDistanceM: positive(item.directDistanceM, `${label}.directDistanceM`),
    pathExcessPct:
      item.pathExcessPct === null ? null : finite(item.pathExcessPct, `${label}.pathExcessPct`),
    replans: nonNegativeInteger(item.replans, `${label}.replans`),
    expandedStates: nonNegativeInteger(item.expandedStates, `${label}.expandedStates`),
    workUnit:
      item.workUnit === "expanded-nodes" ||
      item.workUnit === "queue-pops" ||
      item.workUnit === "expanded-spacetime-states"
        ? item.workUnit
        : fail(`${label}.workUnit is unsupported`),
    minimumSeparationM:
      item.minimumSeparationM === null
        ? null
        : finite(item.minimumSeparationM, `${label}.minimumSeparationM`),
    minimumSeparationWitness: minimumSeparationWitness(
      item.minimumSeparationWitness,
      `${label}.minimumSeparationWitness`,
    ),
    safetyViolations: nonNegativeInteger(item.safetyViolations, `${label}.safetyViolations`),
  };
}

function turnAngle(value: unknown, label: string): number | null {
  if (value === null) return null;
  const parsed = nonNegative(value, label);
  if (parsed > 180 + TOLERANCE) fail(`${label} cannot exceed 180 degrees`);
  return parsed;
}

function discreteKinematicDiagnostics(
  value: unknown,
  label: string,
): PredictiveDiscreteKinematicDiagnostics {
  const item = record(value, label);
  if (item.status !== "discrete-diagnostic-only") {
    fail(`${label}.status must be discrete-diagnostic-only`);
  }
  if (item.continuousDynamicsCertified !== false) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  const parsed: PredictiveDiscreteKinematicDiagnostics = {
    status: "discrete-diagnostic-only",
    continuousDynamicsCertified: false,
    segmentCount: nonNegativeInteger(item.segmentCount, `${label}.segmentCount`),
    movementSegmentCount: nonNegativeInteger(
      item.movementSegmentCount,
      `${label}.movementSegmentCount`,
    ),
    reversalCount: nonNegativeInteger(item.reversalCount, `${label}.reversalCount`),
    reversalThresholdDeg: positive(
      item.reversalThresholdDeg,
      `${label}.reversalThresholdDeg`,
    ),
    maxSpeedMps: nonNegative(item.maxSpeedMps, `${label}.maxSpeedMps`),
    maxDiscreteVelocityChangeMps: nonNegative(
      item.maxDiscreteVelocityChangeMps,
      `${label}.maxDiscreteVelocityChangeMps`,
    ),
    maxDiscreteAccelerationProxyMps2: nonNegative(
      item.maxDiscreteAccelerationProxyMps2,
      `${label}.maxDiscreteAccelerationProxyMps2`,
    ),
    maxAbsClimbRateMps: nonNegative(
      item.maxAbsClimbRateMps,
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

function kinematicDiagnostics(value: unknown, label: string): PredictiveKinematicDiagnostics {
  const item = record(value, label);
  if (item.status !== "discrete-diagnostic-only") {
    fail(`${label}.status must be discrete-diagnostic-only`);
  }
  if (item.continuousDynamicsCertified !== false) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  return {
    status: "discrete-diagnostic-only",
    continuousDynamicsCertified: false,
    raw: discreteKinematicDiagnostics(item.raw, `${label}.raw`),
    output: discreteKinematicDiagnostics(item.output, `${label}.output`),
  };
}

function executionQualification(
  value: unknown,
  label: string,
  envelope: PredictiveExecutionEnvelope,
): PredictiveExecutionQualification {
  const item = record(value, label);
  const qualified = boolean(item.qualified, `${label}.qualified`);
  const status = item.status;
  if (status !== "qualified" && status !== "not-qualified") {
    fail(`${label}.status is unsupported`);
  }
  if ((status === "qualified") !== qualified) {
    fail(`${label}.status must agree with qualified`);
  }
  if (item.continuousDynamicsCertified !== false) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  const diagnostics = discreteKinematicDiagnostics(item.diagnostics, `${label}.diagnostics`);
  const boundaryAwareMaxDiscreteAccelerationProxyMps2 = nonNegative(
    item.boundaryAwareMaxDiscreteAccelerationProxyMps2,
    `${label}.boundaryAwareMaxDiscreteAccelerationProxyMps2`,
  );
  const allowedViolations = new Set([
    "speed-limit-exceeded",
    "climb-rate-limit-exceeded",
    "acceleration-proxy-limit-exceeded",
    "reversal-not-allowed",
    "execution-time-limit-exceeded",
  ]);
  const violations = array(item.violations, `${label}.violations`).map((entry, index) => {
    const parsed = text(entry, `${label}.violations[${index}]`);
    if (!allowedViolations.has(parsed)) fail(`${label}.violations[${index}] is unsupported`);
    return parsed;
  });
  if (new Set(violations).size !== violations.length) {
    fail(`${label}.violations contains duplicates`);
  }
  if (qualified !== (violations.length === 0)) {
    fail(`${label}.qualified must be true exactly when violations is empty`);
  }
  if (!sameNumber(diagnostics.reversalThresholdDeg, envelope.reversalThresholdDeg)) {
    fail(`${label}.diagnostics.reversalThresholdDeg disagrees with the execution envelope`);
  }
  if (
    qualified &&
    (diagnostics.maxSpeedMps > envelope.maxSpeedMps + TOLERANCE ||
      diagnostics.maxAbsClimbRateMps > envelope.maxAbsClimbRateMps + TOLERANCE ||
      boundaryAwareMaxDiscreteAccelerationProxyMps2 >
        envelope.maxDiscreteAccelerationProxyMps2 + TOLERANCE ||
      (!envelope.allowReversals && diagnostics.reversalCount > 0))
  ) {
    fail(`${label} claims qualification outside the declared execution envelope`);
  }
  return {
    status,
    qualified,
    continuousDynamicsCertified: false,
    diagnostics,
    boundaryAwareMaxDiscreteAccelerationProxyMps2,
    violations,
  };
}

function nullableDuration(value: unknown, label: string): number | null {
  return value === null ? null : nonNegative(value, label);
}

function executionEvidence(value: unknown, label: string): PredictiveExecutionEvidence {
  const item = record(value, label);
  const statuses = new Set<PredictiveExecutionStatus>([
    "not-evaluated",
    "qualified",
    "reversal-not-allowed",
    "execution-time-limit-exceeded",
    "time-parameterization-did-not-converge",
    "dynamic-collision-after-retiming",
  ]);
  if (!statuses.has(item.status as PredictiveExecutionStatus)) {
    fail(`${label}.status is unsupported`);
  }
  const status = item.status as PredictiveExecutionStatus;
  const qualified = boolean(item.qualified, `${label}.qualified`);
  const collisionCertified = boolean(
    item.collisionCertified,
    `${label}.collisionCertified`,
  );
  if (item.collisionCertificationScope !== "dense-piecewise-linear-space-time-path") {
    fail(`${label}.collisionCertificationScope is unsupported`);
  }
  if (item.continuousDynamicsCertified !== false) {
    fail(`${label}.continuousDynamicsCertified must remain false`);
  }
  const envelope = executionEnvelope(item.envelope, `${label}.envelope`);
  const qualification =
    item.qualification === null
      ? null
      : executionQualification(item.qualification, `${label}.qualification`, envelope);
  const timingIterations = nonNegativeInteger(item.timingIterations, `${label}.timingIterations`);
  const originalDurationS = nullableDuration(item.originalDurationS, `${label}.originalDurationS`);
  const candidateDurationS = nullableDuration(
    item.candidateDurationS,
    `${label}.candidateDurationS`,
  );
  const addedDurationS = nullableDuration(item.addedDurationS, `${label}.addedDurationS`);
  const durationsPresent =
    originalDurationS !== null && candidateDurationS !== null && addedDurationS !== null;

  if (status === "not-evaluated") {
    if (
      qualified ||
      collisionCertified ||
      qualification !== null ||
      timingIterations !== 0 ||
      originalDurationS !== null ||
      candidateDurationS !== null ||
      addedDurationS !== null
    ) {
      fail(`${label} not-evaluated evidence must not expose a candidate assessment`);
    }
  } else {
    if (qualification === null || !durationsPresent) {
      fail(`${label} evaluated evidence requires qualification and timing durations`);
    }
    if (
      candidateDurationS! + TOLERANCE < originalDurationS! ||
      !sameNumber(addedDurationS!, candidateDurationS! - originalDurationS!)
    ) {
      fail(`${label} time parameterization must not shorten the geometry candidate`);
    }
  }
  if (qualified !== (status === "qualified")) {
    fail(`${label}.qualified must be true exactly for status qualified`);
  }
  if (collisionCertified !== (status === "qualified")) {
    fail(`${label}.collisionCertified must be true exactly for status qualified`);
  }
  if (status === "qualified") {
    if (qualification === null || !qualification.qualified) {
      fail(`${label} qualified status requires a qualified execution-envelope verdict`);
    }
    if (candidateDurationS! > envelope.maxExecutionTimeS + TOLERANCE) {
      fail(`${label}.candidateDurationS exceeds the execution-time envelope`);
    }
  } else if (status === "dynamic-collision-after-retiming") {
    if (qualification === null || !qualification.qualified) {
      fail(`${label} collision-after-retiming requires a qualified kinematic candidate`);
    }
  } else if (status !== "not-evaluated" && qualification?.qualified) {
    fail(`${label} non-collision failure cannot carry a qualified verdict`);
  }
  if (
    status === "reversal-not-allowed" &&
    !qualification?.violations.includes("reversal-not-allowed")
  ) {
    fail(`${label} reversal status requires the corresponding qualification violation`);
  }
  if (
    status === "execution-time-limit-exceeded" &&
    !qualification?.violations.includes("execution-time-limit-exceeded")
  ) {
    fail(`${label} time-limit status requires the corresponding qualification violation`);
  }

  return {
    status,
    qualified,
    collisionCertified,
    collisionCertificationScope: "dense-piecewise-linear-space-time-path",
    continuousDynamicsCertified: false,
    envelope,
    qualification,
    timingIterations,
    originalDurationS,
    candidateDurationS,
    addedDurationS,
  };
}

function smoothing(
  value: unknown, label: string, allowShortcut: boolean, spatialCurves: boolean,
): PredictiveSmoothing {
  const item = record(value, label);
  const parsed: PredictiveSmoothing = {
    method: text(item.method, `${label}.method`),
    applied: boolean(item.applied, `${label}.applied`),
    certified: boolean(item.certified, `${label}.certified`),
    collisionCertified: boolean(item.collisionCertified, `${label}.collisionCertified`),
    collisionCertificationScope:
      item.collisionCertificationScope === "dense-piecewise-linear-space-time-path"
        ? item.collisionCertificationScope
        : fail(`${label}.collisionCertificationScope is unsupported`),
    rawWaypointCount: positiveInteger(item.rawWaypointCount, `${label}.rawWaypointCount`),
    outputWaypointCount: positiveInteger(
      item.outputWaypointCount,
      `${label}.outputWaypointCount`,
    ),
    roundedCornerCount: nonNegativeInteger(
      item.roundedCornerCount,
      `${label}.roundedCornerCount`,
    ),
    requestedTurnRadiusM: positive(
      item.requestedTurnRadiusM,
      `${label}.requestedTurnRadiusM`,
    ),
    appliedTurnRadiusM:
      item.appliedTurnRadiusM === null
        ? null
        // This legacy field is a trim/tan(theta/2) scale, not a quintic's
        // physical curvature radius. At a 180-degree reversal it tends to zero
        // and the export's eleven-decimal canonicalization can round it to 0.
        : spatialCurves && item.method === "spacetime-shortcut-plus-local-quintic-bspline"
          ? nonNegative(item.appliedTurnRadiusM, `${label}.appliedTurnRadiusM`)
        : positive(item.appliedTurnRadiusM, `${label}.appliedTurnRadiusM`),
    sampleSpacingM: positive(item.sampleSpacingM, `${label}.sampleSpacingM`),
    maxTurnAngleBeforeDeg: turnAngle(
      item.maxTurnAngleBeforeDeg,
      `${label}.maxTurnAngleBeforeDeg`,
    ),
    maxTurnAngleAfterDeg: turnAngle(
      item.maxTurnAngleAfterDeg,
      `${label}.maxTurnAngleAfterDeg`,
    ),
    kinematicDiagnostics: kinematicDiagnostics(
      item.kinematicDiagnostics,
      `${label}.kinematicDiagnostics`,
    ),
    execution: executionEvidence(item.execution, `${label}.execution`),
  };
  if (spatialCurves) {
    const axes = array(item.optimizationAxes, `${label}.optimizationAxes`);
    if (axes.length !== 3 || axes[0] !== "x" || axes[1] !== "y" || axes[2] !== "z")
      fail(`${label}.optimizationAxes must be exactly ['x', 'y', 'z']`);
    if (item.altitudePolicy !== "bounded-spatial-spline-v1") fail(`${label}.altitudePolicy must declare bounded XYZ smoothing`);
    if (item.altitudeDeviationLimitM !== 12) fail(`${label}.altitudeDeviationLimitM must be 12`);
    parsed.optimizationAxes = ["x", "y", "z"];
    parsed.altitudePolicy = "bounded-spatial-spline-v1";
    parsed.altitudeDeviationLimitM = 12;
  }
  if (parsed.collisionCertified !== parsed.certified) {
    fail(`${label}.collisionCertified must agree with certified`);
  }
  if (parsed.applied && !parsed.certified) {
    fail(`${label} applied output must be certified`);
  }
  const shortcutOnly = parsed.method === "spacetime-shortcut" ||
    parsed.method === "spacetime-shortcut-fillet-fallback";
  const shortcutCurve = parsed.method === "spacetime-shortcut-plus-local-quintic-bspline";
  if (parsed.method === "spacetime-shortcut-plus-local-quintic-bspline" && !spatialCurves) {
    fail(`${label} local B-spline curves require the XYZ protocol`);
  }
  if ((shortcutOnly || shortcutCurve) && !allowShortcut) {
    fail(`${label} shortcut methods require the current Manhattan v4 protocol`);
  }
  if (allowShortcut && !shortcutOnly && !shortcutCurve &&
      parsed.method !== "not-run-uncertified-raw-path") {
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
    fail(`${label}.appliedTurnRadiusM cannot exceed the requested radius`);
  }
  if (!parsed.applied && parsed.roundedCornerCount !== 0) {
    fail(`${label}.roundedCornerCount must be zero when smoothing is not applied`);
  }
  if (
    !(allowShortcut && (shortcutOnly || shortcutCurve)) &&
    parsed.maxTurnAngleBeforeDeg !== null &&
    parsed.maxTurnAngleAfterDeg !== null &&
    parsed.maxTurnAngleAfterDeg > parsed.maxTurnAngleBeforeDeg + 1e-5
  ) {
    fail(`${label} cannot increase the maximum turn angle`);
  }
  if (
    parsed.kinematicDiagnostics.raw.segmentCount !== parsed.rawWaypointCount - 1 ||
    parsed.kinematicDiagnostics.output.segmentCount !== parsed.outputWaypointCount - 1
  ) {
    fail(`${label}.kinematicDiagnostics segment counts disagree with waypoint counts`);
  }
  return parsed;
}

function parsedFrames(
  value: unknown,
  label: string,
  scenario: PredictiveScenario,
  path: TimedWaypoint[],
): PredictiveFrame[] {
  const parsed = array(value, label).map((entry, index) =>
    frame(entry, `${label}[${index}]`, scenario),
  );
  if (parsed.length === 0 || parsed[0]!.timeS !== 0) {
    fail(`${label} must be non-empty and start at time 0`);
  }
  for (let index = 0; index < parsed.length; index += 1) {
    const current = parsed[index]!;
    if (index > 0 && current.timeS <= parsed[index - 1]!.timeS) {
      fail(`${label} must be strictly increasing in time`);
    }
    if (
      current.timeS > path.at(-1)!.timeS + TOLERANCE ||
      !samePoint(current.vehicle, timedPosition(path, current.timeS))
    ) {
      fail(`${label}[${index}].vehicle disagrees with its evidence path`);
    }
  }
  const finalWaypoint = path.at(-1)!;
  const finalFrame = parsed.at(-1)!;
  if (
    !sameNumber(finalFrame.timeS, finalWaypoint.timeS) ||
    !samePoint(finalFrame.vehicle, finalWaypoint.position)
  ) {
    fail(`${label} and its evidence path must end together`);
  }
  return parsed;
}

function totalWaitTime(intervals: WaitInterval[]): number {
  return intervals.reduce(
    (total, interval) => total + interval.endTimeS - interval.startTimeS,
    0,
  );
}

export function stationaryDuration(path: TimedWaypoint[]): number {
  let total = 0;
  for (let index = 1; index < path.length; index += 1) {
    if (sameStationaryPosition(path[index - 1]!.position, path[index]!.position)) {
      total += path[index]!.timeS - path[index - 1]!.timeS;
    }
  }
  return total;
}

function validateMinimumSeparation(
  parsedMetrics: PredictiveRunMetrics,
  label: string,
  path: TimedWaypoint[],
  scenario: PredictiveScenario,
): void {
  const witness = parsedMetrics.minimumSeparationWitness;
  if ((parsedMetrics.minimumSeparationM === null) !== (witness === null)) {
    fail(`${label} minimum separation and witness must be present together`);
  }
  if (witness === null) return;
  if (
    !sameNumber(parsedMetrics.minimumSeparationM!, witness.separationM) ||
    !sameNumber(witness.declaredSafetyMarginM, scenario.constraints.safetyMarginM)
  ) {
    fail(`${label} minimum-separation witness is inconsistent`);
  }
  if (
    witness.timeS > path.at(-1)!.timeS + TOLERANCE ||
    !pointInBounds(witness.vehiclePosition, scenario.bounds) ||
    !pointInBounds(witness.obstaclePosition, scenario.bounds) ||
    !samePoint(witness.vehiclePosition, timedPosition(path, witness.timeS))
  ) {
    fail(`${label} minimum-separation witness is outside its evidence trajectory`);
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
    const definition = scenario.movingSpheres.find(
      (sphere) => sphere.id === witness.obstacleId,
    );
    const center = definition && movingPosition(definition, witness.timeS);
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
    parsedMetrics.safetyViolations === 0 &&
    witness.separationM + TOLERANCE < scenario.constraints.safetyMarginM
  ) {
    fail(`${label} safe run violates its declared dynamic safety margin`);
  }
}

function plannerWorkUnit(plannerId: string): PredictiveRunMetrics["workUnit"] | null {
  if (plannerId === "space-time-astar-4d") return "expanded-spacetime-states";
  if (plannerId === "dstar-lite-reset-3d" || plannerId === "dstar-lite-reuse-3d") {
    return "queue-pops";
  }
  if (plannerId === "repeated-astar-3d") return "expanded-nodes";
  return null;
}

function validateMetricsForPath(
  parsedMetrics: PredictiveRunMetrics,
  label: string,
  path: TimedWaypoint[],
  waitTimeS: number,
  frames: PredictiveFrame[] | null,
  scenario: PredictiveScenario,
  planner: PredictivePlanner,
  succeeded: boolean,
  failureReason: string | null,
): void {
  if (
    parsedMetrics.success !== succeeded ||
    parsedMetrics.failureReason !== failureReason
  ) {
    fail(`${label} status fields disagree with the raw planner outcome`);
  }
  const expectedWorkUnit = plannerWorkUnit(planner.id);
  if (expectedWorkUnit !== null && parsedMetrics.workUnit !== expectedWorkUnit) {
    fail(`${label}.workUnit disagrees with its planner`);
  }

  const finalWaypoint = path.at(-1)!;
  if (succeeded) {
    if (
      parsedMetrics.arrivalTimeS === null ||
      parsedMetrics.travelTimeS === null ||
      parsedMetrics.pathExcessPct === null ||
      !samePoint(finalWaypoint.position, scenario.goal)
    ) {
      fail(`${label} successful outcome is incomplete`);
    }
    if (
      !sameNumber(parsedMetrics.arrivalTimeS, finalWaypoint.timeS) ||
      !sameNumber(parsedMetrics.travelTimeS, parsedMetrics.arrivalTimeS - parsedMetrics.waitTimeS)
    ) {
      fail(`${label} arrival, travel, and wait times are inconsistent`);
    }
  } else if (
    parsedMetrics.arrivalTimeS !== null ||
    parsedMetrics.travelTimeS !== null ||
    parsedMetrics.pathExcessPct !== null
  ) {
    fail(`${label} failed outcome cannot define arrival, travel, or path excess`);
  }

  const directDistance = Math.hypot(
    scenario.goal[0] - scenario.start[0],
    scenario.goal[1] - scenario.start[1],
    scenario.goal[2] - scenario.start[2],
  );
  const executedLength = polylineLength(path.map((waypoint) => waypoint.position));
  if (
    !sameNumber(parsedMetrics.directDistanceM, directDistance) ||
    !sameNumber(parsedMetrics.executedPathLengthM, executedLength)
  ) {
    fail(`${label} path lengths disagree with its evidence path geometry`);
  }
  if (
    parsedMetrics.pathExcessPct !== null &&
    !sameNumber(
      parsedMetrics.pathExcessPct,
      (parsedMetrics.executedPathLengthM / parsedMetrics.directDistanceM - 1) * 100,
    )
  ) {
    fail(`${label}.pathExcessPct is inconsistent`);
  }
  if (!sameNumber(parsedMetrics.waitTimeS, waitTimeS)) {
    fail(`${label}.waitTimeS disagrees with its evidence path waits`);
  }
  validateMinimumSeparation(parsedMetrics, label, path, scenario);

  if (frames !== null) {
    const finalFrame = frames.at(-1)!;
    const expectedFinalEvent = succeeded ? "goal-reached" : "no-path";
    if (finalFrame.event?.kind !== expectedFinalEvent) {
      fail(`${label} final frame does not report the metric outcome`);
    }
    const visiblePlanningEvents = frames.filter(
      (entry) => entry.event?.kind === "replan" || entry.event?.kind === "prediction-update",
    ).length;
    if (parsedMetrics.replans < visiblePlanningEvents) {
      fail(`${label}.replans is smaller than the recorded planning events`);
    }
  }
}

function validatePlanningAccounting(
  candidate: PredictiveRunMetrics,
  candidateLabel: string,
  plannerMetrics: PredictiveRunMetrics,
): void {
  if (
    candidate.replans !== plannerMetrics.replans ||
    candidate.expandedStates !== plannerMetrics.expandedStates ||
    candidate.workUnit !== plannerMetrics.workUnit
  ) {
    fail(`${candidateLabel} must preserve planner work accounting`);
  }
}

function validateExecutionRetiming(
  geometryPath: TimedWaypoint[],
  executionPath: TimedWaypoint[],
  geometryWaits: WaitInterval[],
  executionWaits: WaitInterval[],
  label: string,
  scheduled = false,
): void {
  if (scheduled) {
    try { validateExecutionSequence(geometryPath, executionPath, true); }
    catch (error) { fail(`${label}: ${error instanceof Error ? error.message : error}`); }
    return;
  }
  if (
    geometryPath.length !== executionPath.length ||
    geometryPath.some(
      (waypoint, index) => !samePoint(waypoint.position, executionPath[index]!.position),
    )
  ) {
    fail(`${label} must preserve the geometry candidate waypoint sequence exactly`);
  }
  for (let index = 1; index < geometryPath.length; index += 1) {
    const geometryDuration = geometryPath[index]!.timeS - geometryPath[index - 1]!.timeS;
    const executionDuration = executionPath[index]!.timeS - executionPath[index - 1]!.timeS;
    if (executionDuration + TOLERANCE < geometryDuration) {
      fail(`${label} must not shorten any geometry-candidate segment`);
    }
    if (
      sameStationaryPosition(geometryPath[index - 1]!.position, geometryPath[index]!.position) &&
      !sameNumber(executionDuration, geometryDuration)
    ) {
      fail(`${label} must preserve each wait-segment duration`);
    }
  }
  if (geometryWaits.length !== executionWaits.length) {
    fail(`${label} must preserve every declared wait interval`);
  }
  for (let index = 0; index < geometryWaits.length; index += 1) {
    const geometryWait = geometryWaits[index]!;
    const executionWait = executionWaits[index]!;
    if (
      !samePoint(geometryWait.position, executionWait.position) ||
      geometryWait.reason !== executionWait.reason ||
      !sameNumber(
        geometryWait.endTimeS - geometryWait.startTimeS,
        executionWait.endTimeS - executionWait.startTimeS,
      )
    ) {
      fail(`${label} must preserve wait positions, reasons, and durations`);
    }
  }
  if (!sameNumber(totalWaitTime(geometryWaits), totalWaitTime(executionWaits))) {
    fail(`${label} total wait duration disagrees with the geometry candidate`);
  }
}

function run(
  value: unknown,
  label: string,
  scenario: PredictiveScenario,
  planner: PredictivePlanner,
  allowShortcut: boolean,
  scheduled: boolean,
  spatialCurves: boolean,
): PredictiveRun {
  const item = record(value, label);
  const statuses = new Set<PredictiveRun["status"]>([
    "success",
    "no-path",
    "timeout",
    "invalid",
  ]);
  if (!statuses.has(item.status as PredictiveRun["status"])) fail(`${label}.status is unsupported`);
  const status = item.status as PredictiveRun["status"];
  const failureReason = nullableText(item.failureReason, `${label}.failureReason`);
  if ((status === "success") !== (failureReason === null)) {
    fail(`${label}.failureReason is inconsistent with status`);
  }
  const succeeded = status === "success";
  const predictive = boolean(item.predictive, `${label}.predictive`);
  if (predictive !== planner.predictive) fail(`${label}.predictive disagrees with its planner`);

  const parametersItem = record(item.parameters, `${label}.parameters`);
  const parameters: Record<string, number> = {};
  for (const [key, parameter] of Object.entries(parametersItem)) {
    parameters[key] = finite(parameter, `${label}.parameters.${key}`);
  }
  if (Object.keys(parameters).length === 0) fail(`${label}.parameters cannot be empty`);

  const rawTimedPath = timedPath(item.rawTimedPath, `${label}.rawTimedPath`, scenario.bounds);
  const geometryTimedPath = timedPath(
    item.geometryTimedPath,
    `${label}.geometryTimedPath`,
    scenario.bounds,
  );
  if (spatialCurves) {
    if (parameters.trajectoryPreserveAltitude !== 0) fail(`${label}.parameters.trajectoryPreserveAltitude must be 0 for XYZ smoothing`);
    validateAltitudeProfile(rawTimedPath, geometryTimedPath, label, 12);
  }
  const executionTimedPath =
    item.executionTimedPath === null
      ? null
      : timedPath(item.executionTimedPath, `${label}.executionTimedPath`, scenario.bounds);
  const paths = [rawTimedPath, geometryTimedPath, ...(executionTimedPath ? [executionTimedPath] : [])];
  if (paths.some((path) => !samePoint(path[0]!.position, scenario.start))) {
    fail(`${label} paths must start at the scenario start`);
  }

  const parsedSmoothing = smoothing(item.smoothing, `${label}.smoothing`, allowShortcut, spatialCurves);
  if (
    parsedSmoothing.rawWaypointCount !== rawTimedPath.length ||
    parsedSmoothing.outputWaypointCount !== geometryTimedPath.length
  ) {
    fail(`${label}.smoothing waypoint counts disagree with the exported evidence paths`);
  }
  if (!parsedSmoothing.applied && !sameTimedPath(rawTimedPath, geometryTimedPath)) {
    fail(`${label} unapplied smoothing must preserve the raw timed path exactly`);
  }
  if (succeeded && !parsedSmoothing.certified) {
    fail(`${label} successful geometry evidence must be collision certified`);
  }

  const geometryWaitIntervals = waitIntervals(
    item.geometryWaitIntervals,
    `${label}.geometryWaitIntervals`,
    scenario.bounds,
    geometryTimedPath,
  );
  const executionWaitIntervals =
    item.executionWaitIntervals === null
      ? null
      : waitIntervals(
          item.executionWaitIntervals,
          `${label}.executionWaitIntervals`,
          scenario.bounds,
          executionTimedPath ?? geometryTimedPath,
        );
  const geometryFrames = parsedFrames(
    item.geometryFrames,
    `${label}.geometryFrames`,
    scenario,
    geometryTimedPath,
  );
  const executionFrames =
    item.executionFrames === null
      ? null
      : parsedFrames(
          item.executionFrames,
          `${label}.executionFrames`,
          scenario,
          executionTimedPath ?? geometryTimedPath,
        );

  const plannerMetrics = metrics(item.plannerMetrics, `${label}.plannerMetrics`);
  const geometryMetrics = metrics(item.geometryMetrics, `${label}.geometryMetrics`);
  const executionMetrics =
    item.executionMetrics === null
      ? null
      : metrics(item.executionMetrics, `${label}.executionMetrics`);

  validateMetricsForPath(
    plannerMetrics,
    `${label}.plannerMetrics`,
    rawTimedPath,
    stationaryDuration(rawTimedPath),
    null,
    scenario,
    planner,
    succeeded,
    failureReason,
  );
  validateMetricsForPath(
    geometryMetrics,
    `${label}.geometryMetrics`,
    geometryTimedPath,
    totalWaitTime(geometryWaitIntervals),
    geometryFrames,
    scenario,
    planner,
    succeeded,
    failureReason,
  );
  validatePlanningAccounting(geometryMetrics, `${label}.geometryMetrics`, plannerMetrics);

  const hasQualifiedExecutionCandidate =
    parsedSmoothing.execution.status === "qualified" &&
    parsedSmoothing.execution.qualified &&
    parsedSmoothing.execution.qualification?.qualified === true &&
    parsedSmoothing.execution.collisionCertified;
  const executionFieldsPresent =
    executionTimedPath !== null &&
    executionWaitIntervals !== null &&
    executionMetrics !== null &&
    executionFrames !== null;
  const executionFieldsAbsent =
    executionTimedPath === null &&
    executionWaitIntervals === null &&
    executionMetrics === null &&
    executionFrames === null;
  if ((!executionFieldsPresent && !executionFieldsAbsent) || executionFieldsPresent !== hasQualifiedExecutionCandidate) {
    fail(`${label} execution evidence fields must exist exactly for a qualified, collision-certified candidate`);
  }
  if (executionFieldsPresent) {
    validateExecutionRetiming(
      geometryTimedPath,
      executionTimedPath!,
      geometryWaitIntervals,
      executionWaitIntervals!,
      `${label}.executionTimedPath`,
      scheduled,
    );
    if (
      !sameNumber(
        parsedSmoothing.execution.originalDurationS!,
        geometryTimedPath.at(-1)!.timeS,
      ) ||
      !sameNumber(
        parsedSmoothing.execution.candidateDurationS!,
        executionTimedPath!.at(-1)!.timeS,
      )
    ) {
      fail(`${label}.smoothing.execution durations disagree with the exported paths`);
    }
    const qualification = parsedSmoothing.execution.qualification!;
    const movementSegments = executionTimedPath!.slice(1).filter(
      (waypoint, index) => !sameStationaryPosition(waypoint.position, executionTimedPath![index]!.position),
    ).length;
    if (
      qualification.diagnostics.segmentCount !== executionTimedPath!.length - 1 ||
      qualification.diagnostics.movementSegmentCount !== movementSegments
    ) {
      fail(`${label}.smoothing.execution qualification counts disagree with executionTimedPath`);
    }
    validateMetricsForPath(
      executionMetrics!,
      `${label}.executionMetrics`,
      executionTimedPath!,
      totalWaitTime(executionWaitIntervals!),
      executionFrames!,
      scenario,
      planner,
      succeeded,
      failureReason,
    );
    validatePlanningAccounting(executionMetrics!, `${label}.executionMetrics`, plannerMetrics);
  } else if (
    parsedSmoothing.execution.originalDurationS !== null &&
    !sameNumber(
      parsedSmoothing.execution.originalDurationS,
      geometryTimedPath.at(-1)!.timeS,
    )
  ) {
    fail(`${label}.smoothing.execution.originalDurationS disagrees with geometryTimedPath`);
  }

  const runId = text(item.runId, `${label}.runId`);
  if (!/^sha256:[0-9a-f]{64}$/.test(runId)) fail(`${label}.runId must be a SHA-256 digest`);
  return {
    runId: runId as `sha256:${string}`,
    plannerId: planner.id,
    predictive,
    status,
    failureReason,
    parameters,
    rawTimedPath,
    geometryTimedPath,
    executionTimedPath,
    smoothing: parsedSmoothing,
    geometryWaitIntervals,
    executionWaitIntervals,
    plannerMetrics,
    geometryMetrics,
    executionMetrics,
    geometryFrames,
    executionFrames,
  };
}

function scenario(
  value: unknown,
  label: string,
  planners: Map<string, PredictivePlanner>,
  allowShortcut: boolean,
  scheduled: boolean,
  spatialCurves: boolean,
): PredictiveScenario {
  const item = record(value, label);
  const sceneBounds = parseBounds(item.bounds, `${label}.bounds`);
  const start = vec3(item.start, `${label}.start`);
  const goal = vec3(item.goal, `${label}.goal`);
  if (!pointInBounds(start, sceneBounds) || !pointInBounds(goal, sceneBounds) || samePoint(start, goal)) {
    fail(`${label} endpoints are invalid`);
  }
  const constraintsItem = record(item.constraints, `${label}.constraints`);
  const constraints = {
    vehicleRadiusM: nonNegative(
      constraintsItem.vehicleRadiusM,
      `${label}.constraints.vehicleRadiusM`,
    ),
    safetyMarginM: nonNegative(
      constraintsItem.safetyMarginM,
      `${label}.constraints.safetyMarginM`,
    ),
  };
  if (constraints.vehicleRadiusM + constraints.safetyMarginM <= 0) {
    fail(`${label}.constraints must define positive combined clearance`);
  }

  const buildings = validationCache.memo(item.buildings, `buildings/${JSON.stringify(sceneBounds)}`, () => uniqueIdObjects(item.buildings, `${label}.buildings`).map((entry, index) => {
    const min = vec3(entry.min, `${label}.buildings[${index}].min`);
    const max = vec3(entry.max, `${label}.buildings[${index}].max`);
    if (
      min.some((coordinate, axis) => coordinate >= max[axis]!) ||
      !pointInBounds(min, sceneBounds) ||
      !pointInBounds(max, sceneBounds)
    ) {
      fail(`${label}.buildings[${index}] is invalid`);
    }
    const footprint = parseBuildingFootprint(entry.footprint, sceneBounds, `${label}.buildings[${index}].footprint`);
    return { id: entry.id as string, min, max, ...(footprint ? { footprint } : {}) };
  }));
  const staticNoFlyZones = uniqueIdObjects(
    item.staticNoFlyZones,
    `${label}.staticNoFlyZones`,
  ).map((entry, index) => staticZone(entry, `${label}.staticNoFlyZones[${index}]`, sceneBounds));
  const temporaryNoFlyZones = uniqueIdObjects(
    item.temporaryNoFlyZones,
    `${label}.temporaryNoFlyZones`,
  ).map((entry, index) =>
    temporaryZone(entry, `${label}.temporaryNoFlyZones[${index}]`, sceneBounds),
  );
  const movingSpheres = uniqueIdObjects(item.movingSpheres, `${label}.movingSpheres`).map(
    (entry, index) => movingSphere(entry, `${label}.movingSpheres[${index}]`, sceneBounds),
  );
  const environmentItem = record(item.environment, `${label}.environment`);
  const environment = {
    district: text(environmentItem.district, `${label}.environment.district`),
    streetPattern: text(
      environmentItem.streetPattern,
      `${label}.environment.streetPattern`,
    ),
    buildingCount: nonNegativeInteger(
      environmentItem.buildingCount,
      `${label}.environment.buildingCount`,
    ),
    hazardCount: nonNegativeInteger(
      environmentItem.hazardCount,
      `${label}.environment.hazardCount`,
    ),
  };
  const expectedHazards =
    staticNoFlyZones.length + temporaryNoFlyZones.length + movingSpheres.length;
  if (
    environment.buildingCount !== buildings.length ||
    environment.hazardCount !== expectedHazards
  ) {
    fail(`${label}.environment counts disagree with declared geometry`);
  }
  const obstacleIds = [
    ...buildings,
    ...staticNoFlyZones,
    ...temporaryNoFlyZones,
    ...movingSpheres,
  ].map((entry) => entry.id);
  if (new Set(obstacleIds).size !== obstacleIds.length) {
    fail(`${label} obstacle IDs must be unique across geometry types`);
  }

  const fingerprint = text(item.fingerprint, `${label}.fingerprint`);
  if (!/^sha256:[0-9a-f]{64}$/.test(fingerprint)) {
    fail(`${label}.fingerprint must be a SHA-256 digest`);
  }
  const parsed: PredictiveScenario = {
    id: text(item.id, `${label}.id`),
    label: text(item.label, `${label}.label`),
    description: text(item.description, `${label}.description`),
    fingerprint: fingerprint as `sha256:${string}`,
    bounds: sceneBounds,
    start,
    goal,
    constraints,
    environment,
    buildings,
    staticNoFlyZones,
    temporaryNoFlyZones,
    movingSpheres,
    runs: [],
  };
  const city = parseCityMetadata(item.city, `${label}.city`);
  if (city) parsed.city = city;
  if (item.mission !== undefined) {
    parsed.mission = parseCityMission(item.mission, sceneBounds, `${label}.mission`);
  }
  if (item.cohort !== undefined) parsed.cohort = text(item.cohort, `${label}.cohort`);
  const rawRuns = array(item.runs, `${label}.runs`);
  const seenPlannerIds = new Set<string>();
  parsed.runs = rawRuns.map((entry, index) => {
    const rawRun = record(entry, `${label}.runs[${index}]`);
    const plannerId = text(rawRun.plannerId, `${label}.runs[${index}].plannerId`);
    const planner = planners.get(plannerId);
    if (!planner) fail(`${label}.runs[${index}].plannerId is not declared`);
    if (seenPlannerIds.has(plannerId)) fail(`${label} contains duplicate run for ${plannerId}`);
    seenPlannerIds.add(plannerId);
    return run(rawRun, `${label}.runs[${index}]`, parsed, planner, allowShortcut, scheduled, spatialCurves);
  });
  if (
    parsed.runs.length !== planners.size ||
    [...planners.keys()].some((plannerId) => !seenPlannerIds.has(plannerId))
  ) {
    fail(`${label} must contain exactly one run for every declared planner`);
  }
  for (const run of parsed.runs) {
    if (run.status !== "success") continue;
    for (const path of [run.rawTimedPath, run.geometryTimedPath, run.executionTimedPath]) {
      if (path) auditMissionTaskVisits(path.map(point => point.position), parsed.mission, path.map(point => point.timeS));
    }
  }
  return parsed;
}

export function validatePredictiveBundle(value: unknown): PredictiveBundleV3 {
  return validationCache.run(() => parsePredictiveBundle(value), isRuntimeData(value));
}

function parsePredictiveBundle(value: unknown): PredictiveBundleV3 {
  const item = record(value, "root");
  if (item.schemaVersion !== 3) fail("schemaVersion must be 3");
  if (item.verificationStatus !== "PREDICTIVE_DEMO_NON_CONFIRMATORY") {
    fail("verificationStatus must be PREDICTIVE_DEMO_NON_CONFIRMATORY");
  }
  const generatedAt = text(item.generatedAt, "generatedAt");
  if (!Number.isFinite(Date.parse(generatedAt))) fail("generatedAt must be an ISO timestamp");
  const parsedProtocol = protocol(item.protocol);
  const sourceCommit = text(item.sourceCommit, "sourceCommit");
  if ((parsedProtocol.id === MANHATTAN_V4_PROTOCOL_ID)) {
    if (!/^local-snapshot:sha256:[0-9a-f]{64}$/.test(sourceCommit)) {
      fail("Manhattan sourceCommit must explicitly identify a local SHA-256 snapshot");
    }
    const provenance = record(item.sourceProvenance, "sourceProvenance");
    if (provenance.kind !== "local-snapshot" || provenance.sha256 !== sourceCommit.slice("local-snapshot:".length)) {
      fail("sourceProvenance must agree with the local snapshot digest");
    }
    const files = array(provenance.files, "sourceProvenance.files");
    if (!files.length) fail("sourceProvenance.files cannot be empty");
    files.forEach((file, index) => {
      const entry = record(file, `sourceProvenance.files[${index}]`);
      text(entry.path, `sourceProvenance.files[${index}].path`);
      if (!/^sha256:[0-9a-f]{64}$/.test(text(entry.sha256, `sourceProvenance.files[${index}].sha256`))) {
        fail("sourceProvenance file hashes must be SHA-256 digests");
      }
    });
  } else if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(sourceCommit)) {
    fail("sourceCommit must be a full lowercase Git object ID");
  }

  const planners = uniqueIdObjects(item.planners, "planners").map((planner, index) => ({
    id: planner.id as string,
    label: text(planner.label, `planners[${index}].label`),
    predictive: boolean(planner.predictive, `planners[${index}].predictive`),
  }));
  if (planners.length < 2) fail("at least two planners are required for comparison");
  if (!planners.some((planner) => planner.predictive) || planners.every((planner) => planner.predictive)) {
    fail("planners must include predictive and non-predictive baselines");
  }
  const plannerMap = new Map(planners.map((planner) => [planner.id, planner]));

  const scenarios = array(item.scenarios, "scenarios").map((entry, index) =>
    scenario(
      entry, `scenarios[${index}]`, plannerMap,
      (parsedProtocol.id === MANHATTAN_V4_PROTOCOL_ID),

      parsedProtocol.trajectoryDynamicScheduling === "certified-move-block-departures",
      parsedProtocol.id === MANHATTAN_V4_PROTOCOL_ID,
    ),
  );
  if (scenarios.length === 0) fail("at least one scenario is required");
  if (new Set(scenarios.map((entry) => entry.id)).size !== scenarios.length) {
    fail("scenario IDs must be unique");
  }
  if (new Set(scenarios.map((entry) => entry.fingerprint)).size !== scenarios.length) {
    fail("scenario fingerprints must be unique");
  }
  const runIds = scenarios.flatMap((entry) => entry.runs.map((runRecord) => runRecord.runId));
  if (new Set(runIds).size !== runIds.length) fail("runId values must be unique across the bundle");
  for (const entry of scenarios) {
    for (const runRecord of entry.runs) {
      const expectedEnvelope = (parsedProtocol.id === MANHATTAN_V4_PROTOCOL_ID)
        ? MANHATTAN_EXECUTION_ENVELOPE : FROZEN_EXECUTION_ENVELOPE;
      if (!matchesEnvelope(runRecord.smoothing.execution.envelope, expectedEnvelope)) {
        fail(`${entry.id}: execution envelope disagrees with the declared protocol`);
      }
      if (parsedProtocol.trajectoryCurveDegree === 5 && runRecord.parameters.trajectoryCurveDegree !== 5) {
        fail(`${entry.id}/${runRecord.plannerId}.parameters.trajectoryCurveDegree must be 5 for local B-spline curves`);
      }
      if ((parsedProtocol.id === MANHATTAN_V4_PROTOCOL_ID)) {
        if (runRecord.parameters.trajectoryShortcut !== 1) {
          fail(`${entry.id}/${runRecord.plannerId}: parameters.trajectoryShortcut must be 1 for current Manhattan v4`);
        }
        if (runRecord.plannerId === "space-time-astar-4d") {
          if (runRecord.parameters.spaceTimeConnectivity !== 26) {
            fail(`${entry.id}/${runRecord.plannerId}: parameters.spaceTimeConnectivity must be 26 for current Manhattan v4`);
          }
        } else if (runRecord.parameters.spaceTimeConnectivity !== undefined) {
          fail(`${entry.id}/${runRecord.plannerId}: spaceTimeConnectivity is only valid for the 4D planner`);
        }
      }
    }
  }
  if ((parsedProtocol.id === MANHATTAN_V4_PROTOCOL_ID)) {
    const expectedIds = ["manhattan-westside-delivery", "manhattan-medical-transfer", "manhattan-midtown-rooftops", "manhattan-riverfront-logistics", "manhattan-westside-backhaul", "manhattan-medical-return", "manhattan-eastside-backhaul", "manhattan-riverfront-return"];
    const expectedPlanners = ["repeated-astar-3d", "dstar-lite-reset-3d", "dstar-lite-reuse-3d", "space-time-astar-4d"];
    if (scenarios.length !== 8 || expectedIds.some((id) => !scenarios.some((entry) => entry.id === id)) || runIds.length !== 32) {
      fail("the Manhattan protocol requires its eight declared missions and 32 planner runs");
    }
    if (planners.length !== 4 || expectedPlanners.some((id) => !plannerMap.has(id))) {
      fail("the Manhattan protocol requires its four declared planner conditions");
    }
    if (scenarios.some((entry) => !entry.city || entry.environment.buildingCount < 1000 ||
        entry.city.buildingCount !== entry.environment.buildingCount || entry.buildings.some((building) => !building.footprint))) {
      fail("Manhattan missions require NYC provenance and the complete dense city geometry");
    }
  }

  auditSharedWorld(scenarios);
  const downloadsItem = record(item.downloads, "downloads");
  if (
    Object.keys(downloadsItem).length !== 2 ||
    !("recordsCsv" in downloadsItem) ||
    !("scenarioManifest" in downloadsItem)
  ) {
    fail("downloads must contain exactly recordsCsv and scenarioManifest");
  }
  const recordsCsv = artifact(
    downloadsItem.recordsCsv,
    "downloads.recordsCsv",
    "predictive-records.csv",
  );
  const scenarioManifest = artifact(
    downloadsItem.scenarioManifest,
    "downloads.scenarioManifest",
    "predictive-scenario-manifest.json",
  );

  return {
    schemaVersion: 3,
    generatedAt,
    sourceCommit,
    verificationStatus: "PREDICTIVE_DEMO_NON_CONFIRMATORY",
    protocol: parsedProtocol,
    planners,
    scenarios,
    downloads: {
      recordsCsv: { ...recordsCsv, path: "predictive-records.csv" },
      scenarioManifest: { ...scenarioManifest, path: "predictive-scenario-manifest.json" },
    },
  };
}


export interface PredictiveComparisonRow {
  plannerId: string;
  plannerLabel: string;
  predictive: boolean;
  status: PredictiveRun["status"];
  arrivalTimeS: number | null;
  waitTimeS: number;
  executedPathLengthM: number;
  minimumSeparationM: number | null;
  executionStatus: PredictiveRun["smoothing"]["execution"]["status"];
  executionQualified: boolean;
  smoothingApplied: boolean;
  smoothingCertified: boolean;
  maxTurnAngleAfterDeg: number | null;
  safetyViolations: number;
  expandedStates: number;
  workUnit: PredictiveRunMetrics["workUnit"];
}

export function buildPredictiveComparisonRows(
  bundle: PredictiveBundleV3,
  scenario: PredictiveScenario,
): PredictiveComparisonRow[] {
  const runs = new Map(scenario.runs.map((runRecord) => [runRecord.plannerId, runRecord]));
  return bundle.planners.map((planner) => {
    const runRecord = runs.get(planner.id);
    if (!runRecord) fail(`${scenario.id} has no run for ${planner.id}`);
    return {
      plannerId: planner.id,
      plannerLabel: planner.label,
      predictive: planner.predictive,
      status: runRecord.status,
      arrivalTimeS: runRecord.plannerMetrics.arrivalTimeS,
      waitTimeS: runRecord.plannerMetrics.waitTimeS,
      executedPathLengthM: runRecord.plannerMetrics.executedPathLengthM,
      minimumSeparationM: runRecord.plannerMetrics.minimumSeparationM,
      executionStatus: runRecord.smoothing.execution.status,
      executionQualified: runRecord.smoothing.execution.qualified,
      smoothingApplied: runRecord.smoothing.applied,
      smoothingCertified: runRecord.smoothing.certified,
      maxTurnAngleAfterDeg: runRecord.smoothing.maxTurnAngleAfterDeg,
      safetyViolations: runRecord.plannerMetrics.safetyViolations,
      expandedStates: runRecord.plannerMetrics.expandedStates,
      workUnit: runRecord.plannerMetrics.workUnit,
    };
  });
}
