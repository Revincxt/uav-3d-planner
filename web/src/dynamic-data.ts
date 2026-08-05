import type {
  ArtifactReference,
  Bounds3,
  DynamicBundleV1,
  DynamicEvent,
  DynamicFrame,
  DynamicPlannerId,
  DynamicProtocol,
  DynamicRun,
  DynamicRunMetrics,
  DynamicScenario,
  MovingSphereDefinition,
  MovingSphereKeyframe,
  MovingSphereState,
  StaticNoFlyZone,
  TemporaryNoFlyZone,
  Vec3,
} from "./dynamic-schema";

const TOLERANCE = 1e-6;
const PLANNERS = new Map<DynamicPlannerId, string>([
  ["repeated-astar-3d", "Repeated 3D A*"],
  ["repeated-lazy-theta-star", "Repeated Lazy Theta*"],
  ["dstar-lite-3d", "3D D* Lite"],
]);

function fail(message: string): never {
  throw new Error(`dynamic-data.json: ${message}`);
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
  if (value === null) return null;
  return text(value, label);
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

function pointInBounds(point: Vec3, bounds: Bounds3): boolean {
  return point.every(
    (coordinate, index) =>
      coordinate >= bounds.min[index]! - TOLERANCE &&
      coordinate <= bounds.max[index]! + TOLERANCE,
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

function bounds(value: unknown, label: string): Bounds3 {
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

function protocol(value: unknown): DynamicProtocol {
  const item = record(value, "protocol");
  const parsed = {
    id: text(item.id, "protocol.id"),
    timeStepS: positive(item.timeStepS, "protocol.timeStepS"),
    replanIntervalS: positive(item.replanIntervalS, "protocol.replanIntervalS"),
    cruiseSpeedMps: positive(item.cruiseSpeedMps, "protocol.cruiseSpeedMps"),
    maxTimeS: positive(item.maxTimeS, "protocol.maxTimeS"),
    resolutionM: positive(item.resolutionM, "protocol.resolutionM"),
    maxExpansions: positiveInteger(item.maxExpansions, "protocol.maxExpansions"),
  };
  if (
    parsed.timeStepS !== 1 ||
    parsed.replanIntervalS !== 4 ||
    parsed.cruiseSpeedMps !== 8 ||
    parsed.maxTimeS !== 180 ||
    parsed.resolutionM !== 4 ||
    parsed.maxExpansions !== 120_000
  ) {
    fail("protocol does not match the declared deterministic replay configuration");
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

function dynamicEvent(value: unknown, label: string): DynamicEvent | null {
  if (value === null) return null;
  const item = record(value, label);
  const kinds = new Set<DynamicEvent["kind"]>([
    "none",
    "temporary-zone-activated",
    "temporary-zone-deactivated",
    "replan",
    "wait",
    "goal-reached",
    "no-path",
  ]);
  if (!kinds.has(item.kind as DynamicEvent["kind"])) fail(`${label}.kind is unsupported`);
  const subjectId = item.subjectId === null ? null : text(item.subjectId, `${label}.subjectId`);
  return {
    kind: item.kind as DynamicEvent["kind"],
    label: text(item.label, `${label}.label`),
    subjectId,
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
    DynamicScenario,
    "bounds" | "start" | "goal" | "temporaryNoFlyZones" | "movingSpheres"
  >,
): DynamicFrame {
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

  const event = dynamicEvent(item.event, `${label}.event`);
  const replanned = boolean(item.replanned, `${label}.replanned`);
  const replanReason = nullableText(item.replanReason, `${label}.replanReason`);
  const plannerSuccess =
    item.plannerSuccess === null
      ? null
      : boolean(item.plannerSuccess, `${label}.plannerSuccess`);
  const planningTimeMs =
    item.planningTimeMs === null
      ? null
      : nonNegative(item.planningTimeMs, `${label}.planningTimeMs`);
  const workUsed = nonNegativeInteger(item.workUsed, `${label}.workUsed`);
  const changedEdges = nonNegativeInteger(item.changedEdges, `${label}.changedEdges`);
  if (replanned && (replanReason === null || plannerSuccess === null)) {
    fail(`${label} replanning frames require reason and plannerSuccess`);
  }
  if (!replanned && (replanReason !== null || plannerSuccess !== null || workUsed !== 0 || changedEdges !== 0)) {
    fail(`${label} non-replanning frame contains planner outcomes`);
  }
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
    replanned,
    replanReason,
    plannerSuccess,
    planningTimeMs,
    workUsed,
    changedEdges,
  };
}

function metrics(value: unknown, label: string): DynamicRunMetrics {
  const item = record(value, label);
  const completionTimeS =
    item.completionTimeS === null
      ? null
      : nonNegative(item.completionTimeS, `${label}.completionTimeS`);
  const pathExcessPct =
    item.pathExcessPct === null ? null : finite(item.pathExcessPct, `${label}.pathExcessPct`);
  const parsed: DynamicRunMetrics = {
    success: boolean(item.success, `${label}.success`),
    failureReason: nullableText(item.failureReason, `${label}.failureReason`),
    completionTimeS,
    executedPathLengthM: nonNegative(item.executedPathLengthM, `${label}.executedPathLengthM`),
    directDistanceM: positive(item.directDistanceM, `${label}.directDistanceM`),
    pathExcessPct,
    replans: nonNegativeInteger(item.replans, `${label}.replans`),
    failedReplans: nonNegativeInteger(item.failedReplans, `${label}.failedReplans`),
    holds: nonNegativeInteger(item.holds, `${label}.holds`),
    safetyGateActivations: nonNegativeInteger(
      item.safetyGateActivations,
      `${label}.safetyGateActivations`,
    ),
    collisionCount: nonNegativeInteger(item.collisionCount, `${label}.collisionCount`),
    totalPlanningWork: nonNegativeInteger(item.totalPlanningWork, `${label}.totalPlanningWork`),
    workUnit:
      item.workUnit === "expanded-nodes" || item.workUnit === "queue-pops"
        ? item.workUnit
        : fail(`${label}.workUnit is unsupported`),
    totalChangedEdges: nonNegativeInteger(item.totalChangedEdges, `${label}.totalChangedEdges`),
  };
  if (item.deadlineMisses !== undefined) {
    parsed.deadlineMisses = nonNegativeInteger(item.deadlineMisses, `${label}.deadlineMisses`);
  }
  if (item.minimumClearanceM !== undefined) {
    parsed.minimumClearanceM =
      item.minimumClearanceM === null
        ? null
        : nonNegative(item.minimumClearanceM, `${label}.minimumClearanceM`);
  }
  if (parsed.failedReplans > parsed.replans || parsed.safetyGateActivations > parsed.replans) {
    fail(`${label} replanning counts are inconsistent`);
  }
  return parsed;
}

function run(value: unknown, label: string, scenario: DynamicScenario): DynamicRun {
  const item = record(value, label);
  const statuses = new Set<DynamicRun["status"]>([
    "success",
    "no-path",
    "timeout",
    "invalid",
  ]);
  if (!statuses.has(item.status as DynamicRun["status"])) fail(`${label}.status is unsupported`);
  const failureReason = nullableText(item.failureReason, `${label}.failureReason`);
  if ((item.status === "success") !== (failureReason === null)) {
    fail(`${label}.failureReason is inconsistent with status`);
  }
  const parametersItem = record(item.parameters, `${label}.parameters`);
  const parameters: Record<string, number> = {};
  for (const [key, parameter] of Object.entries(parametersItem)) {
    parameters[key] = finite(parameter, `${label}.parameters.${key}`);
  }
  if (Object.keys(parameters).length === 0) fail(`${label}.parameters cannot be empty`);

  const parsedFrames = array(item.frames, `${label}.frames`).map((entry, index) =>
    frame(entry, `${label}.frames[${index}]`, scenario),
  );
  if (parsedFrames.length === 0 || parsedFrames[0]!.timeS !== 0) {
    fail(`${label}.frames must be non-empty and start at time 0`);
  }
  for (let index = 1; index < parsedFrames.length; index += 1) {
    const current = parsedFrames[index]!;
    const previous = parsedFrames[index - 1]!;
    if (current.timeS <= previous.timeS) fail(`${label}.frames must be strictly increasing in time`);
    if (
      previous.executedPath.length > current.executedPath.length ||
      previous.executedPath.some((point, pointIndex) =>
        !samePoint(point, current.executedPath[pointIndex]!),
      )
    ) {
      fail(`${label}.executedPath must grow monotonically across frames`);
    }
  }

  const parsedMetrics = metrics(item.metrics, `${label}.metrics`);
  const plannerId = text(item.plannerId, `${label}.plannerId`) as DynamicPlannerId;
  const expectedWorkUnit = plannerId === "dstar-lite-3d" ? "queue-pops" : "expanded-nodes";
  if (parsedMetrics.workUnit !== expectedWorkUnit) {
    fail(`${label}.metrics.workUnit does not match the planner`);
  }
  const runId = text(item.runId, `${label}.runId`);
  if (!/^sha256:[0-9a-f]{64}$/.test(runId)) fail(`${label}.runId must be a SHA-256 digest`);
  const succeeded = item.status === "success";
  if (parsedMetrics.success !== succeeded || parsedMetrics.failureReason !== failureReason) {
    fail(`${label}.metrics status fields disagree with the run`);
  }
  const lastFrame = parsedFrames.at(-1)!;
  if (succeeded) {
    if (
      parsedMetrics.completionTimeS === null ||
      parsedMetrics.pathExcessPct === null ||
      lastFrame.event?.kind !== "goal-reached" ||
      !samePoint(lastFrame.vehicle, scenario.goal)
    ) {
      fail(`${label} successful outcome is incomplete`);
    }
    if (!sameNumber(parsedMetrics.completionTimeS, lastFrame.timeS)) {
      fail(`${label}.metrics.completionTimeS must match the final frame`);
    }
  } else if (parsedMetrics.completionTimeS !== null || parsedMetrics.pathExcessPct !== null) {
    fail(`${label} failed outcome cannot define completion time or path excess`);
  }
  const directDistance = Math.hypot(
    scenario.goal[0] - scenario.start[0],
    scenario.goal[1] - scenario.start[1],
    scenario.goal[2] - scenario.start[2],
  );
  const executedLength = polylineLength(lastFrame.executedPath);
  if (
    !sameNumber(parsedMetrics.directDistanceM, directDistance) ||
    !sameNumber(parsedMetrics.executedPathLengthM, executedLength)
  ) {
    fail(`${label}.metrics path lengths disagree with the recorded geometry`);
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
  if (
    parsedMetrics.holds !==
      parsedFrames.filter((entry) => entry.event?.kind === "wait").length ||
    parsedMetrics.totalPlanningWork !==
      parsedFrames.reduce((total, entry) => total + entry.workUsed, 0) ||
    parsedMetrics.totalChangedEdges !==
      parsedFrames.reduce((total, entry) => total + entry.changedEdges, 0) ||
    parsedMetrics.replans < parsedFrames.filter((entry) => entry.replanned).length
  ) {
    fail(`${label}.metrics do not aggregate the recorded frames`);
  }

  return {
    runId: runId as `sha256:${string}`,
    plannerId,
    status: item.status as DynamicRun["status"],
    failureReason,
    parameters,
    metrics: parsedMetrics,
    frames: parsedFrames,
  };
}

function scenario(value: unknown, label: string): DynamicScenario {
  const item = record(value, label);
  const sceneBounds = bounds(item.bounds, `${label}.bounds`);
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
  const allObstacleIds = [
    ...buildings,
    ...staticNoFlyZones,
    ...temporaryNoFlyZones,
    ...movingSpheres,
  ].map((entry) => entry.id);
  if (new Set(allObstacleIds).size !== allObstacleIds.length) {
    fail(`${label} obstacle IDs must be unique across geometry types`);
  }

  const parsed: DynamicScenario = {
    id: text(item.id, `${label}.id`),
    label: text(item.label, `${label}.label`),
    description: text(item.description, `${label}.description`),
    fingerprint: text(item.fingerprint, `${label}.fingerprint`),
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
  if (!/^sha256:[0-9a-f]{64}$/.test(parsed.fingerprint)) {
    fail(`${label}.fingerprint must be a SHA-256 digest`);
  }
  parsed.runs = array(item.runs, `${label}.runs`).map((entry, index) =>
    run(entry, `${label}.runs[${index}]`, parsed),
  );
  const runPlannerIds = new Set(parsed.runs.map((entry) => entry.plannerId));
  if (
    parsed.runs.length !== PLANNERS.size ||
    runPlannerIds.size !== PLANNERS.size ||
    [...PLANNERS.keys()].some((id) => !runPlannerIds.has(id))
  ) {
    fail(`${label} must contain exactly one run for each declared planner`);
  }
  return parsed;
}

export function validateDynamicBundle(value: unknown): DynamicBundleV1 {
  const item = record(value, "root");
  if (item.schemaVersion !== 1) fail("schemaVersion must be 1");
  if (item.verificationStatus !== "DYNAMIC_DEMO_NON_CONFIRMATORY") {
    fail("verificationStatus must be DYNAMIC_DEMO_NON_CONFIRMATORY");
  }
  const sourceCommit = text(item.sourceCommit, "sourceCommit");
  if (!/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/.test(sourceCommit)) {
    fail("sourceCommit must be a full lowercase Git object ID");
  }
  const generatedAt = text(item.generatedAt, "generatedAt");
  if (!Number.isFinite(Date.parse(generatedAt))) fail("generatedAt must be an ISO timestamp");
  const parsedProtocol = protocol(item.protocol);

  const planners = uniqueIdObjects(item.planners, "planners").map((planner, index) => ({
    id: planner.id as DynamicPlannerId,
    label: text(planner.label, `planners[${index}].label`),
  }));
  if (
    planners.length !== PLANNERS.size ||
    planners.some(
      (planner, index) =>
        planner.id !== [...PLANNERS.keys()][index] || planner.label !== PLANNERS.get(planner.id),
    )
  ) {
    fail("planners must contain the declared three-planner comparison in protocol order");
  }

  const scenarios = array(item.scenarios, "scenarios").map((entry, index) =>
    scenario(entry, `scenarios[${index}]`),
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
  const recordsCsv = artifact(downloadsItem.recordsCsv, "downloads.recordsCsv", "dynamic-records.csv");
  const scenarioManifest = artifact(
    downloadsItem.scenarioManifest,
    "downloads.scenarioManifest",
    "dynamic-scenario-manifest.json",
  );

  return {
    schemaVersion: 1,
    generatedAt,
    sourceCommit,
    verificationStatus: "DYNAMIC_DEMO_NON_CONFIRMATORY",
    protocol: parsedProtocol,
    planners,
    scenarios,
    downloads: {
      recordsCsv: { ...recordsCsv, path: "dynamic-records.csv" },
      scenarioManifest: { ...scenarioManifest, path: "dynamic-scenario-manifest.json" },
    },
  };
}

export async function loadDynamicBundle(
  fetcher: typeof fetch = fetch,
): Promise<DynamicBundleV1> {
  const response = await fetcher(`${import.meta.env.BASE_URL}dynamic-data.json`);
  if (!response.ok) throw new Error(`Could not load dynamic data (${response.status})`);
  return validateDynamicBundle(await response.json());
}

export interface DynamicComparisonRow {
  plannerId: DynamicPlannerId;
  plannerLabel: string;
  status: DynamicRun["status"];
  completionTimeS: number | null;
  executedPathLengthM: number;
  replans: number;
  holds: number;
  safetyGateActivations: number;
  totalPlanningWork: number;
  workUnit: DynamicRunMetrics["workUnit"];
}

export function buildDynamicComparisonRows(
  bundle: DynamicBundleV1,
  scenario: DynamicScenario,
): DynamicComparisonRow[] {
  return bundle.planners.map((planner) => {
    const selectedRun = scenario.runs.find((candidate) => candidate.plannerId === planner.id);
    if (!selectedRun) fail(`${scenario.id} has no run for ${planner.id}`);
    return {
      plannerId: planner.id,
      plannerLabel: planner.label,
      status: selectedRun.status,
      completionTimeS: selectedRun.metrics.completionTimeS,
      executedPathLengthM: selectedRun.metrics.executedPathLengthM,
      replans: selectedRun.metrics.replans,
      holds: selectedRun.metrics.holds,
      safetyGateActivations: selectedRun.metrics.safetyGateActivations,
      totalPlanningWork: selectedRun.metrics.totalPlanningWork,
      workUnit: selectedRun.metrics.workUnit,
    };
  });
}
