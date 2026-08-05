export type Vec3 = [number, number, number];
export type PlannerId = "astar-3d" | "lazy-theta-star" | "rrt-star";

export interface Building {
  id: string;
  min: Vec3;
  max: Vec3;
}

export interface NoFlyZone {
  id: string;
  kind: "cylinder";
  center: [number, number];
  radiusM: number;
  zMinM: number;
  zMaxM: number;
}

export interface PlannerResult {
  plannerId: PlannerId;
  runId: string;
  plannerSeed: number | null;
  status: "success" | "no-path" | "invalid";
  failureReason: string | null;
  budget: { maxIterations: number };
  searchEffort: { kind: "expanded-nodes" | "samples"; value: number };
  paths: { raw: Vec3[]; smoothed: Vec3[] } | null;
  smoothing: {
    outcome: "bspline" | "shortcut" | "shortcut-fallback" | "not-run";
    collisionFree: boolean;
  };
  metrics: {
    planningTimeMs: number;
    rawLengthM: number | null;
    smoothedLengthM: number | null;
    minClearanceM: number | null;
  };
}

export interface DemoScenario {
  id: string;
  label: string;
  description: string;
  scenarioSeed: number | null;
  fingerprint: string;
  bounds: { min: Vec3; max: Vec3 };
  start: Vec3;
  goal: Vec3;
  constraints: {
    vehicleRadiusM: number;
    safetyMarginM: number;
    maxAltitudeM: number;
  };
  buildings: Building[];
  noFlyZones: NoFlyZone[];
  results: PlannerResult[];
}

export interface DemoBundle {
  schemaVersion: 1;
  generatedAt: string;
  verificationStatus: "DEMO_NON_CONFIRMATORY";
  coordinateSystem: { frame: "ENU"; distanceUnit: "m"; timeUnit: "ms" };
  defaultScenarioId: string;
  planners: Array<{ id: PlannerId; label: string }>;
  scenarios: DemoScenario[];
}

