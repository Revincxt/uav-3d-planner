import type {
  ArtifactReference,
  Bounds3,
  Building,
  MovingSphereDefinition,
  MovingSphereKeyframe,
  MovingSphereState,
  StaticNoFlyZone,
  TemporaryNoFlyZone,
  Vec3,
} from "./dynamic-schema";

export type {
  ArtifactReference,
  Bounds3,
  Building,
  MovingSphereDefinition,
  MovingSphereKeyframe,
  MovingSphereState,
  StaticNoFlyZone,
  TemporaryNoFlyZone,
  Vec3,
} from "./dynamic-schema";

export type PredictivePlannerId = string;

export interface PredictivePlanner {
  id: PredictivePlannerId;
  label: string;
  predictive: boolean;
}

export interface PredictiveProtocol {
  id: string;
  timeStepS: number;
  cruiseSpeedMps: number;
  maxTimeS: number;
  resolutionM: number;
  timeResolutionS: number;
  planningHorizonS: number;
  predictionHorizonS: number;
  reactiveMaxWorkPerReplan: number;
  predictiveMaxExpandedStatesPerMission: number;
}

export interface TimedWaypoint {
  timeS: number;
  position: Vec3;
}

export interface WaitInterval {
  startTimeS: number;
  endTimeS: number;
  position: Vec3;
  reason: string;
}

export interface PredictiveEvent {
  kind:
    | "none"
    | "temporary-zone-activated"
    | "temporary-zone-deactivated"
    | "replan"
    | "wait"
    | "wait-start"
    | "wait-end"
    | "prediction-update"
    | "goal-reached"
    | "no-path";
  label: string;
  subjectId: string | null;
}

export interface PredictiveFrame {
  timeS: number;
  vehicle: Vec3;
  path: Vec3[];
  executedPath: Vec3[];
  activeTemporaryZoneIds: string[];
  movingSpheres: MovingSphereState[];
  event: PredictiveEvent | null;
}

export interface PredictiveRunMetrics {
  success: boolean;
  failureReason: string | null;
  arrivalTimeS: number | null;
  travelTimeS: number | null;
  waitTimeS: number;
  executedPathLengthM: number;
  directDistanceM: number;
  pathExcessPct: number | null;
  replans: number;
  expandedStates: number;
  workUnit: "expanded-nodes" | "queue-pops" | "expanded-spacetime-states";
  minimumSeparationM: number | null;
  safetyViolations: number;
}

export interface PredictiveRun {
  runId: `sha256:${string}`;
  plannerId: PredictivePlannerId;
  predictive: boolean;
  status: "success" | "no-path" | "timeout" | "invalid";
  failureReason: string | null;
  parameters: Record<string, number>;
  timedPath: TimedWaypoint[];
  waitIntervals: WaitInterval[];
  metrics: PredictiveRunMetrics;
  frames: PredictiveFrame[];
}

export interface PredictiveScenario {
  id: string;
  label: string;
  description: string;
  fingerprint: `sha256:${string}`;
  cohort?: string;
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
  runs: PredictiveRun[];
}

export interface PredictiveBundleV1 {
  schemaVersion: 1;
  generatedAt: string;
  sourceCommit: string;
  verificationStatus: "PREDICTIVE_DEMO_NON_CONFIRMATORY";
  protocol: PredictiveProtocol;
  planners: PredictivePlanner[];
  scenarios: PredictiveScenario[];
  downloads: {
    recordsCsv: ArtifactReference & { path: "predictive-records.csv" };
    scenarioManifest: ArtifactReference & { path: "predictive-scenario-manifest.json" };
  };
}
