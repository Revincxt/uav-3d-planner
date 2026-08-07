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
  trajectoryPostprocessor: string;
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
  activeTemporaryZoneIds: string[];
  movingSpheres: MovingSphereState[];
  event: PredictiveEvent | null;
}

export interface PredictiveSmoothing {
  method: string;
  applied: boolean;
  certified: boolean;
  collisionCertified: boolean;
  collisionCertificationScope: "dense-piecewise-linear-space-time-path";
  rawWaypointCount: number;
  outputWaypointCount: number;
  roundedCornerCount: number;
  requestedTurnRadiusM: number;
  appliedTurnRadiusM: number | null;
  sampleSpacingM: number;
  maxTurnAngleBeforeDeg: number | null;
  maxTurnAngleAfterDeg: number | null;
  kinematicDiagnostics: PredictiveKinematicDiagnostics;
}

export interface PredictiveDiscreteKinematicDiagnostics {
  status: "discrete-diagnostic-only";
  continuousDynamicsCertified: false;
  segmentCount: number;
  movementSegmentCount: number;
  reversalCount: number;
  reversalThresholdDeg: number;
  maxSpeedMps: number;
  maxDiscreteVelocityChangeMps: number;
  maxDiscreteAccelerationProxyMps2: number;
  maxAbsClimbRateMps: number;
}

export interface PredictiveKinematicDiagnostics {
  status: "discrete-diagnostic-only";
  continuousDynamicsCertified: false;
  raw: PredictiveDiscreteKinematicDiagnostics;
  output: PredictiveDiscreteKinematicDiagnostics;
}

export interface PredictiveMinimumSeparationWitness {
  separationM: number;
  timeS: number;
  vehiclePosition: Vec3;
  obstacleId: string;
  obstacleKind: "moving-sphere" | "temporary-cylinder";
  obstaclePosition: Vec3;
  declaredSafetyMarginM: number;
  method: string;
  exact: boolean;
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
  minimumSeparationWitness: PredictiveMinimumSeparationWitness | null;
  safetyViolations: number;
}

export interface PredictiveRun {
  runId: `sha256:${string}`;
  plannerId: PredictivePlannerId;
  predictive: boolean;
  status: "success" | "no-path" | "timeout" | "invalid";
  failureReason: string | null;
  parameters: Record<string, number>;
  rawTimedPath: TimedWaypoint[];
  timedPath: TimedWaypoint[];
  smoothing: PredictiveSmoothing;
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
  environment: {
    district: string;
    streetPattern: string;
    buildingCount: number;
    hazardCount: number;
  };
  buildings: Building[];
  staticNoFlyZones: StaticNoFlyZone[];
  temporaryNoFlyZones: TemporaryNoFlyZone[];
  movingSpheres: MovingSphereDefinition[];
  runs: PredictiveRun[];
}

export interface PredictiveBundleV2 {
  schemaVersion: 2;
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

export type PredictivePathMode = "raw" | "certified";
