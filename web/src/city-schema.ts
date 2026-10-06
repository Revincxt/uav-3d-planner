/** Real public map data; hypothetical missions are intentionally separate. */
export interface CityMetadata {
  id: string;
  name: string;
  sourceKind: "nyc-open-data";
  sourceUrl: string;
  sourceSha256: string;
  collisionModel: "conservative-aabb";
  [key: string]: unknown;
}

export interface CityMission {
  origin: string;
  destination: string;
  purpose: string;
  taskPoints?: MissionTaskPoint[];
  challenge?: MissionChallenge;
  sharedWorld?: SharedWorld;
}

export interface SharedWorld {
  id: string;
  fingerprint: string;
  missionCount: number;
  clockOriginS: number;
  airframeSpanM: number;
  missionDeconfliction: "not-jointly-optimized";
}

export interface MissionChallenge {
  kind: "static" | "dynamic" | "predictive";
  title: string;
  focusPosition: [number, number, number];
  startTimeS: number;
  endTimeS: number;
}

export interface MissionTaskPoint {
  id: string;
  order: number;
  label: string;
  action: string;
  position: [number, number, number];
  serviceDurationS: number;
  visitMode?: "fly-through" | "service";
  buildingId: string;
}

/** A hard visit must be an actual trajectory knot; near misses do not count. */
export function auditMissionTaskVisits(positions: readonly (readonly number[])[], mission?: CityMission, times?: readonly number[]): void {
  let cursor = 0;
  const matches = (position: readonly number[], target: readonly number[]): boolean => Math.hypot(...position.map((value, axis) => value - target[axis]!)) <= 1e-5;
  for (const task of mission?.taskPoints ?? []) {
    while (cursor < positions.length && !matches(positions[cursor]!, task.position)) cursor += 1;
    if (cursor === positions.length) throw new Error(`Required task ${task.id} is omitted or out of order`);
    if (times) {
      let departure = cursor;
      while (departure + 1 < positions.length && matches(positions[departure + 1]!, task.position)) departure += 1;
      if (times[departure]! - times[cursor]! < task.serviceDurationS - 1e-5) throw new Error(`Required service duration missing at ${task.id}`);
      if (task.visitMode === "fly-through" && times[departure]! - times[cursor]! > 1e-5)
        throw new Error(`Fly-through task ${task.id} must not contain a dwell`);
      cursor = departure;
    }
    cursor += 1;
  }
}

export function parseCityMission(value: unknown, bounds: { min: readonly number[]; max: readonly number[] }, label = "mission"): CityMission | undefined {
  if (value === undefined) return undefined;
  if (typeof value !== "object" || value === null || Array.isArray(value)) throw new Error(`${label} must be an object`);
  const item = value as Record<string, unknown>;
  for (const key of ["origin", "destination", "purpose"]) {
    if (typeof item[key] !== "string" || !(item[key] as string).trim()) throw new Error(`${label}.${key} is required`);
  }
  const mission: CityMission = { origin: item.origin as string, destination: item.destination as string, purpose: item.purpose as string };
  if (item.sharedWorld !== undefined) {
    const world = item.sharedWorld as Record<string, unknown>;
    if (!world || typeof world !== "object" || Array.isArray(world) ||
      typeof world.id !== "string" || !world.id || typeof world.fingerprint !== "string" ||
      !/^sha256:[0-9a-f]{64}$/.test(world.fingerprint) ||
      !Number.isInteger(world.missionCount) || Number(world.missionCount) < 1 || world.clockOriginS !== 0 ||
      typeof world.airframeSpanM !== "number" || !Number.isFinite(world.airframeSpanM) || world.airframeSpanM <= 0 ||
      world.missionDeconfliction !== "not-jointly-optimized") throw new Error(`${label}.sharedWorld is invalid`);
    mission.sharedWorld = world as unknown as SharedWorld;
  }
  if (item.challenge !== undefined) {
    const challenge = item.challenge as Record<string, unknown>;
    if (!challenge || typeof challenge !== "object" || Array.isArray(challenge) ||
      !["static", "dynamic", "predictive"].includes(challenge.kind as string) ||
      typeof challenge.title !== "string" || !challenge.title.trim() ||
      !Array.isArray(challenge.focusPosition) || challenge.focusPosition.length !== 3 ||
      challenge.focusPosition.some((coordinate, axis) => typeof coordinate !== "number" ||
        !Number.isFinite(coordinate) || coordinate < bounds.min[axis]! || coordinate > bounds.max[axis]!) ||
      typeof challenge.startTimeS !== "number" || !Number.isFinite(challenge.startTimeS) || challenge.startTimeS < 0 ||
      typeof challenge.endTimeS !== "number" || !Number.isFinite(challenge.endTimeS) || challenge.endTimeS <= challenge.startTimeS) {
      throw new Error(`${label}.challenge is invalid`);
    }
    mission.challenge = { kind: challenge.kind as MissionChallenge["kind"], title: challenge.title,
      focusPosition: [...challenge.focusPosition] as [number, number, number],
      startTimeS: challenge.startTimeS, endTimeS: challenge.endTimeS };
  }
  if (item.taskPoints !== undefined) {
    if (!Array.isArray(item.taskPoints) || item.taskPoints.length < 6 || item.taskPoints.length > 8) throw new Error(`${label} requires 6–8 task points`);
    const ids = new Set<string>();
    mission.taskPoints = item.taskPoints.map((raw: unknown, index: number) => {
      if (typeof raw !== "object" || raw === null || Array.isArray(raw)) throw new Error(`${label}.taskPoints[${index}] must be an object`);
      const task = raw as Record<string, unknown>;
      for (const key of ["id", "label", "action", "buildingId"]) {
        if (typeof task[key] !== "string" || !(task[key] as string).trim()) throw new Error(`${label}.taskPoints[${index}].${key} is required`);
      }
      if (ids.has(task.id as string) || task.order !== index + 1) throw new Error(`${label} has duplicate or unordered task points`);
      ids.add(task.id as string);
      if (!Array.isArray(task.position) || task.position.length !== 3 || task.position.some((value, axis) =>
        typeof value !== "number" || !Number.isFinite(value) || value < bounds.min[axis]! || value > bounds.max[axis]!)) throw new Error(`${label} task point leaves the city bounds`);
      if (task.visitMode !== undefined && task.visitMode !== "fly-through" && task.visitMode !== "service") throw new Error(`${label} task visitMode is invalid`);
      if (typeof task.serviceDurationS !== "number" || !Number.isFinite(task.serviceDurationS) ||
        (task.visitMode === "fly-through" ? task.serviceDurationS !== 0 : task.serviceDurationS <= 0))
        throw new Error(`${label} task duration does not match its visit mode`);
      return { id: task.id as string, order: index + 1, label: task.label as string, action: task.action as string,
        buildingId: task.buildingId as string, position: [...task.position] as [number, number, number], serviceDurationS: task.serviceDurationS,
        ...(task.visitMode ? { visitMode: task.visitMode as MissionTaskPoint["visitMode"] } : {}) };
    });
  }
  return mission;
}

/** Preserve legacy city checks while explicitly recognizing the redesigned physical ROI. */
export function hasCompleteCityExtent(city: CityMetadata | undefined, bounds: { min: readonly number[]; max: readonly number[] }): boolean {
  const width = bounds.max[0]! - bounds.min[0]!, depth = bounds.max[1]! - bounds.min[1]!;
  const region = city?.planningRegion as { id?: unknown } | undefined;
  if (region?.id === "midtown-expanded-v3") {
    return width >= 3500 && width <= 3700 && depth >= 3700 && depth <= 3900;
  }
  return region?.id === "midtown-landscape-v2"
    ? width >= 3500 && depth >= 1800 && width / depth >= 1.7 && width / depth <= 2
    : width >= 2000 && depth >= 2000;
}

export function parseCityMetadata(value: unknown, label = "city"): CityMetadata | undefined {
  if (value === undefined) return undefined;
  if (typeof value !== "object" || value === null || Array.isArray(value)) throw new Error(`${label} must be an object`);
  const record = value as Record<string, unknown>;
  for (const key of ["id", "name", "sourceUrl", "sourceSha256"]) {
    if (typeof record[key] !== "string" || !(record[key] as string).trim()) throw new Error(`${label}.${key} is required`);
  }
  if (record.sourceKind !== "nyc-open-data" || record.collisionModel !== "conservative-aabb") {
    throw new Error(`${label} must declare its public data and conservative collision model`);
  }
  if (!/^(?:sha256:)?[0-9a-f]{64}$/.test(record.sourceSha256 as string)) throw new Error(`${label}.sourceSha256 is invalid`);
  const url = new URL(record.sourceUrl as string);
  if (url.protocol !== "https:" || !["services6.arcgis.com", "data.cityofnewyork.us", "nycmaps-nyc.hub.arcgis.com"].includes(url.hostname)) {
    throw new Error(`${label}.sourceUrl must identify the official NYC source`);
  }
  return { ...record } as CityMetadata;
}

export function parseBuildingFootprint(
  value: unknown,
  bounds: { min: readonly number[]; max: readonly number[] },
  label = "footprint",
): number[][][] | undefined {
  if (value === undefined) return undefined;
  if (!Array.isArray(value) || value.length === 0) throw new Error(`${label} requires an exterior ring`);
  return value.map((ring, ringIndex) => {
    if (!Array.isArray(ring) || ring.length < 3 || ring.length > 10000) throw new Error(`${label}[${ringIndex}] is not a polygon ring`);
    return ring.map((point, pointIndex) => {
      if (!Array.isArray(point) || point.length !== 2 || !point.every((coordinate) => typeof coordinate === "number" && Number.isFinite(coordinate))) {
        throw new Error(`${label}[${ringIndex}][${pointIndex}] must be a finite map coordinate`);
      }
      if (point.some((coordinate, axis) => coordinate < bounds.min[axis]! - 1e-5 || coordinate > bounds.max[axis]! + 1e-5)) {
        throw new Error(`${label}[${ringIndex}][${pointIndex}] leaves its collision envelope`);
      }
      return [...point] as number[];
    });
  });
}
