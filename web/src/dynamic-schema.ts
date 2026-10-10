import type { CityMetadata, CityMission } from "./city-schema";

export type Vec3 = [number, number, number];
export type DynamicPlannerId =
  | "repeated-astar-3d"
  | "repeated-lazy-theta-star"
  | "dstar-lite-3d";

export interface DynamicPlanner {
  id: DynamicPlannerId;
  label: string;
}

export interface DynamicProtocol {
  id: string;
  timeStepS: number;
  replanIntervalS: number;
  cruiseSpeedMps: number;
  maxTimeS: number;
  resolutionM: number;
  maxExpansions: number;
  pathShortcut?: 1;
  preserveAltitude?: 1;
  smoothTurns?: 1;
  turnScaleM?: number;
  curveSampleSpacingM?: number;
  horizontalEscape?: 1;
  verticalCostScale?: number;
  maxClimbRateMps?: number;
}

export interface ArtifactReference {
  path: string;
  sha256: `sha256:${string}`;
  bytes: number;
}

export interface Bounds3 {
  min: Vec3;
  max: Vec3;
}

export interface Building {
  id: string;
  min: Vec3;
  max: Vec3;
  footprint?: number[][][];
}

export interface StaticNoFlyZone {
  id: string;
  center: [number, number];
  radiusM: number;
  zMinM: number;
  zMaxM: number;
}

export interface TemporaryNoFlyZone extends StaticNoFlyZone {
  activeFromS: number;
  activeUntilS: number;
}

export interface MovingSphereKeyframe {
  timeS: number;
  position: Vec3;
}

export interface MovingSphereDefinition {
  id: string;
  radiusM: number;
  keyframes: MovingSphereKeyframe[];
}

export interface MovingSphereState {
  id: string;
  position: Vec3;
  radiusM: number;
}

export interface DynamicEvent {
  kind:
    | "none"
    | "temporary-zone-activated"
    | "temporary-zone-deactivated"
    | "replan"
    | "wait"
    | "goal-reached"
    | "no-path";
  label: string;
  subjectId: string | null;
}

export interface DynamicFrame {
  timeS: number;
  vehicle: Vec3;
  path: Vec3[];
  executedPath: Vec3[];
  activeTemporaryZoneIds: string[];
  movingSpheres: MovingSphereState[];
  event: DynamicEvent | null;
  replanned: boolean;
  replanReason: string | null;
  plannerSuccess: boolean | null;
  planningTimeMs: number | null;
  workUsed: number;
  changedEdges: number;
}

export interface DynamicRunMetrics {
  success: boolean;
  failureReason: string | null;
  completionTimeS: number | null;
  executedPathLengthM: number;
  directDistanceM: number;
  pathExcessPct: number | null;
  replans: number;
  failedReplans: number;
  holds: number;
  safetyGateActivations: number;
  collisionCount: number;
  totalPlanningWork: number;
  workUnit: "expanded-nodes" | "queue-pops";
  totalChangedEdges: number;
  deadlineMisses?: number;
  minimumClearanceM?: number | null;
}

export interface DynamicRun {
  runId: `sha256:${string}`;
  plannerId: DynamicPlannerId;
  status: "success" | "no-path" | "timeout" | "invalid";
  failureReason: string | null;
  parameters: Record<string, number>;
  metrics: DynamicRunMetrics;
  frames: DynamicFrame[];
  executionTimedPath?: { timeS: number; position: Vec3; action: "start" | "move" | "wait" }[];
}

export interface DynamicScenario {
  city?: CityMetadata;
  mission?: CityMission;
  id: string;
  label: string;
  description: string;
  fingerprint: string;
  bounds: Bounds3;
  start: Vec3;
  goal: Vec3;
  constraints: {
    vehicleRadiusM: number;
    safetyMarginM: number;
  };
  buildings: Building[];
  staticNoFlyZones: StaticNoFlyZone[];
  temporaryNoFlyZones: TemporaryNoFlyZone[];
  movingSpheres: MovingSphereDefinition[];
  runs: DynamicRun[];
}

export interface DynamicBundleV1 {
  schemaVersion: 1;
  generatedAt: string;
  sourceCommit: string;
  verificationStatus: "DYNAMIC_DEMO_NON_CONFIRMATORY";
  protocol: DynamicProtocol;
  planners: DynamicPlanner[];
  scenarios: DynamicScenario[];
  downloads: {
    recordsCsv: ArtifactReference & { path: "dynamic-records.csv" };
    scenarioManifest: ArtifactReference & { path: "dynamic-scenario-manifest.json" };
  };
}
