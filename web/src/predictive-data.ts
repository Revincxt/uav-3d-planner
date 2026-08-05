import type {
  ArtifactReference,
  Bounds3,
  MovingSphereDefinition,
  MovingSphereKeyframe,
  MovingSphereState,
  PredictiveBundleV1,
  PredictiveEvent,
  PredictiveFrame,
  PredictivePlanner,
  PredictiveProtocol,
  PredictiveRun,
  PredictiveRunMetrics,
  PredictiveScenario,
  StaticNoFlyZone,
  TemporaryNoFlyZone,
  TimedWaypoint,
  Vec3,
  WaitInterval,
} from "./predictive-schema";

const TOLERANCE = 1e-6;

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

function vec3(value: unknown, label: string): Vec3 {
  const coordinates = array(value, label);
  if (coordinates.length !== 3) fail(`${label} must contain three coordinates`);
  return coordinates.map((coordinate, index) =>
    finite(coordinate, `${label}[${index}]`),
  ) as Vec3;
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

function protocol(value: unknown): PredictiveProtocol {
  const item = record(value, "protocol");
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
  };
  if (parsed.timeResolutionS > parsed.planningHorizonS) {
    fail("protocol.timeResolutionS cannot exceed planningHorizonS");
  }
  if (parsed.predictionHorizonS > parsed.planningHorizonS) {
    fail("protocol.predictionHorizonS cannot exceed planningHorizonS");
  }
  if (parsed.planningHorizonS > parsed.maxTimeS) {
    fail("protocol.planningHorizonS cannot exceed maxTimeS");
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

function path(value: unknown, label: string, sceneBounds: Bounds3): Vec3[] {
  return array(value, label).map((point, index) => {
    const parsed = vec3(point, `${label}[${index}]`);
    if (!pointInBounds(parsed, sceneBounds)) fail(`${label}[${index}] is outside scenario bounds`);
    return parsed;
  });
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
      !samePoint(timedPosition(waypoints, interval.startTimeS), interval.position) ||
      !samePoint(timedPosition(waypoints, interval.endTimeS), interval.position) ||
      waypoints.some(
        (waypoint) =>
          waypoint.timeS > interval.startTimeS &&
          waypoint.timeS < interval.endTimeS &&
          !samePoint(waypoint.position, interval.position),
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
  const timeS = nonNegative(item.timeS, `${label}.timeS`);
  const vehicle = vec3(item.vehicle, `${label}.vehicle`);
  if (!pointInBounds(vehicle, scenario.bounds)) fail(`${label}.vehicle is outside scenario bounds`);
  const plannedPath = path(item.path, `${label}.path`, scenario.bounds);
  const executedPath = path(item.executedPath, `${label}.executedPath`, scenario.bounds);
  if (executedPath.length === 0 || !samePoint(executedPath[0]!, scenario.start)) {
    fail(`${label}.executedPath must start at the scenario start`);
  }
  if (!samePoint(executedPath.at(-1)!, vehicle)) {
    fail(`${label}.executedPath must end at the vehicle position`);
  }
  if (
    plannedPath.length > 0 &&
    (!samePoint(plannedPath[0]!, vehicle) || !samePoint(plannedPath.at(-1)!, scenario.goal))
  ) {
    fail(`${label}.path endpoints must be the vehicle and goal`);
  }

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
    path: plannedPath,
    executedPath,
    activeTemporaryZoneIds,
    movingSpheres,
    event,
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
        : nonNegative(item.minimumSeparationM, `${label}.minimumSeparationM`),
    safetyViolations: nonNegativeInteger(item.safetyViolations, `${label}.safetyViolations`),
  };
}

function run(
  value: unknown,
  label: string,
  scenario: PredictiveScenario,
  planner: PredictivePlanner,
): PredictiveRun {
  const item = record(value, label);
  const statuses = new Set<PredictiveRun["status"]>([
    "success",
    "no-path",
    "timeout",
    "invalid",
  ]);
  if (!statuses.has(item.status as PredictiveRun["status"])) fail(`${label}.status is unsupported`);
  const failureReason = nullableText(item.failureReason, `${label}.failureReason`);
  if ((item.status === "success") !== (failureReason === null)) {
    fail(`${label}.failureReason is inconsistent with status`);
  }
  const predictive = boolean(item.predictive, `${label}.predictive`);
  if (predictive !== planner.predictive) fail(`${label}.predictive disagrees with its planner`);

  const parametersItem = record(item.parameters, `${label}.parameters`);
  const parameters: Record<string, number> = {};
  for (const [key, parameter] of Object.entries(parametersItem)) {
    parameters[key] = finite(parameter, `${label}.parameters.${key}`);
  }
  if (Object.keys(parameters).length === 0) fail(`${label}.parameters cannot be empty`);

  const parsedTimedPath = timedPath(item.timedPath, `${label}.timedPath`, scenario.bounds);
  if (!samePoint(parsedTimedPath[0]!.position, scenario.start)) {
    fail(`${label}.timedPath must start at the scenario start`);
  }
  const parsedWaits = waitIntervals(
    item.waitIntervals,
    `${label}.waitIntervals`,
    scenario.bounds,
    parsedTimedPath,
  );
  const parsedFrames = array(item.frames, `${label}.frames`).map((entry, index) =>
    frame(entry, `${label}.frames[${index}]`, scenario),
  );
  if (parsedFrames.length === 0 || parsedFrames[0]!.timeS !== 0) {
    fail(`${label}.frames must be non-empty and start at time 0`);
  }
  for (let index = 0; index < parsedFrames.length; index += 1) {
    const current = parsedFrames[index]!;
    if (index > 0) {
      const previous = parsedFrames[index - 1]!;
      if (current.timeS <= previous.timeS) {
        fail(`${label}.frames must be strictly increasing in time`);
      }
      if (
        previous.executedPath.length > current.executedPath.length ||
        previous.executedPath.some((point, pointIndex) =>
          !samePoint(point, current.executedPath[pointIndex]!),
        )
      ) {
        fail(`${label}.executedPath must grow monotonically across frames`);
      }
    }
    if (!samePoint(current.vehicle, timedPosition(parsedTimedPath, current.timeS))) {
      fail(`${label}.frames[${index}].vehicle disagrees with timedPath`);
    }
  }

  const parsedMetrics = metrics(item.metrics, `${label}.metrics`);
  const expectedWorkUnit = planner.id === "space-time-astar-4d"
    ? "expanded-spacetime-states"
    : planner.id === "dstar-lite-reset-3d" || planner.id === "dstar-lite-reuse-3d"
      ? "queue-pops"
      : planner.id === "repeated-astar-3d"
        ? "expanded-nodes"
        : null;
  if (expectedWorkUnit !== null && parsedMetrics.workUnit !== expectedWorkUnit) {
    fail(`${label}.metrics.workUnit disagrees with its planner`);
  }
  const runId = text(item.runId, `${label}.runId`);
  if (!/^sha256:[0-9a-f]{64}$/.test(runId)) fail(`${label}.runId must be a SHA-256 digest`);
  const succeeded = item.status === "success";
  if (
    parsedMetrics.success !== succeeded ||
    parsedMetrics.failureReason !== failureReason
  ) {
    fail(`${label}.metrics status fields disagree with the run`);
  }

  const finalWaypoint = parsedTimedPath.at(-1)!;
  const lastFrame = parsedFrames.at(-1)!;
  if (!sameNumber(lastFrame.timeS, finalWaypoint.timeS)) {
    fail(`${label}.frames and timedPath must end at the same time`);
  }
  if (succeeded) {
    if (
      parsedMetrics.arrivalTimeS === null ||
      parsedMetrics.travelTimeS === null ||
      parsedMetrics.pathExcessPct === null ||
      !samePoint(finalWaypoint.position, scenario.goal) ||
      !samePoint(lastFrame.vehicle, scenario.goal) ||
      lastFrame.event?.kind !== "goal-reached"
    ) {
      fail(`${label} successful outcome is incomplete`);
    }
    if (
      !sameNumber(parsedMetrics.arrivalTimeS, finalWaypoint.timeS) ||
      !sameNumber(parsedMetrics.travelTimeS, parsedMetrics.arrivalTimeS - parsedMetrics.waitTimeS)
    ) {
      fail(`${label}.metrics arrival, travel, and wait times are inconsistent`);
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
  const executedLength = polylineLength(parsedTimedPath.map((waypoint) => waypoint.position));
  if (
    !sameNumber(parsedMetrics.directDistanceM, directDistance) ||
    !sameNumber(parsedMetrics.executedPathLengthM, executedLength)
  ) {
    fail(`${label}.metrics path lengths disagree with timedPath geometry`);
  }
  if (
    parsedMetrics.pathExcessPct !== null &&
    !sameNumber(
      parsedMetrics.pathExcessPct,
      (parsedMetrics.executedPathLengthM / parsedMetrics.directDistanceM - 1) * 100,
    )
  ) {
    fail(`${label}.metrics.pathExcessPct is inconsistent`);
  }
  const waitTime = parsedWaits.reduce(
    (total, interval) => total + interval.endTimeS - interval.startTimeS,
    0,
  );
  if (!sameNumber(parsedMetrics.waitTimeS, waitTime)) {
    fail(`${label}.metrics.waitTimeS disagrees with waitIntervals`);
  }
  const visiblePlanningEvents = parsedFrames.filter(
    (entry) => entry.event?.kind === "replan" || entry.event?.kind === "prediction-update",
  ).length;
  if (parsedMetrics.replans < visiblePlanningEvents) {
    fail(`${label}.metrics.replans is smaller than the recorded planning events`);
  }

  return {
    runId: runId as `sha256:${string}`,
    plannerId: planner.id,
    predictive,
    status: item.status as PredictiveRun["status"],
    failureReason,
    parameters,
    timedPath: parsedTimedPath,
    waitIntervals: parsedWaits,
    metrics: parsedMetrics,
    frames: parsedFrames,
  };
}

function scenario(
  value: unknown,
  label: string,
  planners: Map<string, PredictivePlanner>,
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

  const buildings = uniqueIdObjects(item.buildings, `${label}.buildings`).map((entry, index) => {
    const min = vec3(entry.min, `${label}.buildings[${index}].min`);
    const max = vec3(entry.max, `${label}.buildings[${index}].max`);
    if (
      min.some((coordinate, axis) => coordinate >= max[axis]!) ||
      !pointInBounds(min, sceneBounds) ||
      !pointInBounds(max, sceneBounds)
    ) {
      fail(`${label}.buildings[${index}] is invalid`);
    }
    return { id: entry.id as string, min, max };
  });
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
    buildings,
    staticNoFlyZones,
    temporaryNoFlyZones,
    movingSpheres,
    runs: [],
  };
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
    return run(rawRun, `${label}.runs[${index}]`, parsed, planner);
  });
  if (
    parsed.runs.length !== planners.size ||
    [...planners.keys()].some((plannerId) => !seenPlannerIds.has(plannerId))
  ) {
    fail(`${label} must contain exactly one run for every declared planner`);
  }
  return parsed;
}

export function validatePredictiveBundle(value: unknown): PredictiveBundleV1 {
  const item = record(value, "root");
  if (item.schemaVersion !== 1) fail("schemaVersion must be 1");
  if (item.verificationStatus !== "PREDICTIVE_DEMO_NON_CONFIRMATORY") {
    fail("verificationStatus must be PREDICTIVE_DEMO_NON_CONFIRMATORY");
  }
  const generatedAt = text(item.generatedAt, "generatedAt");
  if (!Number.isFinite(Date.parse(generatedAt))) fail("generatedAt must be an ISO timestamp");
  const sourceCommit = text(item.sourceCommit, "sourceCommit");
  if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(sourceCommit)) {
    fail("sourceCommit must be a full lowercase Git object ID");
  }
  const parsedProtocol = protocol(item.protocol);

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
    scenario(entry, `scenarios[${index}]`, plannerMap),
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
    schemaVersion: 1,
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

export async function loadPredictiveBundle(
  fetcher: typeof fetch = fetch,
): Promise<PredictiveBundleV1> {
  const response = await fetcher(`${import.meta.env.BASE_URL}predictive-data.json`);
  if (!response.ok) throw new Error(`Could not load predictive data (${response.status})`);
  return validatePredictiveBundle(await response.json());
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
  safetyViolations: number;
  expandedStates: number;
  workUnit: PredictiveRunMetrics["workUnit"];
}

export function buildPredictiveComparisonRows(
  bundle: PredictiveBundleV1,
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
      arrivalTimeS: runRecord.metrics.arrivalTimeS,
      waitTimeS: runRecord.metrics.waitTimeS,
      executedPathLengthM: runRecord.metrics.executedPathLengthM,
      minimumSeparationM: runRecord.metrics.minimumSeparationM,
      safetyViolations: runRecord.metrics.safetyViolations,
      expandedStates: runRecord.metrics.expandedStates,
      workUnit: runRecord.metrics.workUnit,
    };
  });
}
