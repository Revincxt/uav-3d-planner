import { describe, expect, it } from "vitest";
import { validatePredictiveBundle } from "../src/predictive-data";
import losFixture from "./fixtures/predictive-los-only.json";
import { readStudyData } from '../scripts/study-reader.mjs';

const source = await readStudyData(new URL("../public/predictive-data.json", import.meta.url));
// Each test owns its mutable declarations, without parsing/duplicating the full
// enlarged city and long replay histories dozens of times. Validation still
// inspects every native field and sample; only fixture construction is cheaper.
const cityBundle = () => ({ ...source, protocol: structuredClone(source.protocol),
  scenarios: source.scenarios.map((scenario: typeof source.scenarios[number]) => ({ ...scenario,
    mission: structuredClone(scenario.mission),
    environment: { ...scenario.environment },
    runs: scenario.runs.map((run: typeof scenario.runs[number]) => ({ ...run,
      parameters: { ...run.parameters }, smoothing: structuredClone(run.smoothing),
    })),
  })),
});

function losOnlyBundle() {
  const bundle = cityBundle();
  for (const scenario of bundle.scenarios) delete scenario.mission.sharedWorld;
  bundle.scenarios[0] = {
    ...bundle.scenarios[0],
    start: losFixture.start,
    goal: losFixture.goal,
    fingerprint: losFixture.fingerprint,
    // This isolated LOS schema fixture is not the multistop mission it was copied from.
    mission: { ...bundle.scenarios[0].mission, taskPoints: undefined },
    // Keep this schema-only fixture's own stationary traffic state; the production
    // mission's new corridor must not silently replace its recorded hazard context.
    movingSpheres: losFixture.runTemplate.geometryFrames[0]!.movingSpheres.map(sphere => ({
      id: sphere.id, label: "LOS fixture traffic", radiusM: sphere.radiusM,
      keyframes: [{ timeS: 0, position: sphere.position }, { timeS: 900, position: sphere.position }],
    })),
    runs: losFixture.planners.map((planner) => {
      const run = {
        ...structuredClone(losFixture.runTemplate),
        plannerId: planner.plannerId,
        runId: planner.runId,
        predictive: planner.predictive,
        parameters: { ...planner.parameters, trajectoryPreserveAltitude: 0, trajectoryCurveDegree: 5 },
      };
      Object.assign(run.smoothing, {
        optimizationAxes: ["x", "y", "z"], altitudePolicy: "bounded-spatial-spline-v1", altitudeDeviationLimitM: 12,
      });
      for (const metrics of [run.plannerMetrics, run.geometryMetrics, run.executionMetrics]) {
        metrics.workUnit = planner.workUnit;
      }
      run.geometryFrames[0]!.event = planner.initialGeometryEvent;
      run.executionFrames[0]!.event = planner.initialExecutionEvent;
      return run;
    }),
  };
  bundle.scenarios[0].environment.hazardCount = bundle.scenarios[0].staticNoFlyZones.length +
    bundle.scenarios[0].temporaryNoFlyZones.length + bundle.scenarios[0].movingSpheres.length;
  return bundle;
}

// Full-cohort parsing and certification are integrity checks, not 5-second speed tests.
// Keep the extended deadline local to these data-heavy integration checks.
describe("computed Manhattan predictive protocol", { timeout: 30000 }, () => {
  it("retains actual polygons, city provenance, missions and qualified evidence", () => {
    const bundle = validatePredictiveBundle(cityBundle());
    expect(bundle.protocol.id).toBe("manhattan-space-time-v4");
    expect(bundle.protocol.spaceTimeConnectivity).toBe(26);
    expect(bundle.protocol.trajectoryShortcut).toBe(true);
    expect(bundle.protocol.trajectoryPreserveAltitude).toBe(false);
    expect(bundle.protocol.trajectoryCurveDegree).toBe(5);
    expect(bundle.protocol.trajectoryCurveDimensions).toBe(3);
    expect(bundle.protocol.trajectoryAltitudeDeviationLimitM).toBe(12);
    expect(bundle.scenarios).toHaveLength(8);
    expect(bundle.scenarios.flatMap((scenario) => scenario.runs)).toHaveLength(32);
    for (const scenario of bundle.scenarios) {
      expect(scenario.city?.sourceKind).toBe("nyc-open-data");
      expect(scenario.buildings.length).toBeGreaterThan(4000);
      expect(scenario.buildings.every((building) => building.footprint?.length)).toBe(true);
      expect(scenario.mission?.origin).toBeTruthy();
      expect(scenario.movingSpheres).toHaveLength(12);
      for (const run of scenario.runs) {
        expect(run.status).toBe("success");
        expect(run.smoothing.execution.qualified).toBe(true);
        expect(run.plannerMetrics.safetyViolations).toBe(0);
        expect(run.executionMetrics?.safetyViolations).toBe(0);
        expect(run.parameters.trajectoryShortcut).toBe(1);
        expect(run.parameters.trajectoryPreserveAltitude).toBe(0);
        expect(run.parameters.trajectoryCurveDegree).toBe(5);
        expect(["spacetime-shortcut-plus-local-quintic-bspline", "spacetime-shortcut-fillet-fallback", "spacetime-shortcut"])
          .toContain(run.smoothing.method);
        if (run.smoothing.method === "spacetime-shortcut-plus-local-quintic-bspline")
          expect(run.smoothing.roundedCornerCount).toBeGreaterThan(0);
        expect(run.smoothing.optimizationAxes).toEqual(["x", "y", "z"]);
        expect(run.smoothing.altitudePolicy).toBe("bounded-spatial-spline-v1");
        expect(run.parameters.spaceTimeConnectivity).toBe(
          run.plannerId === "space-time-astar-4d" ? 26 : undefined,
        );
      }
    }
  });

  it("rejects a snapshot incorrectly represented as a Git commit", () => {
    const bundle = cityBundle();
    bundle.sourceCommit = "a".repeat(40);
    expect(() => validatePredictiveBundle(bundle)).toThrow(/local SHA-256 snapshot/);
  });

  it("rejects a mixture of old and new execution envelopes", () => {
    const bundle = cityBundle();
    bundle.protocol.executionEnvelope.maxSpeedMps = 8;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/execution envelope/);
  });

  it("does not allow metropolitan evidence to masquerade as the frozen study", () => {
    const bundle = cityBundle();
    bundle.protocol.id = "predictive-space-time-v4";
    expect(() => validatePredictiveBundle(bundle)).toThrow(/declared protocol/);
  });

  it("requires the complete eight-mission, four-planner city cohort", () => {
    const bundle = cityBundle();
    bundle.scenarios.pop();
    expect(() => validatePredictiveBundle(bundle)).toThrow(/eight declared missions/);
  });

  it("rejects missing public geometry provenance", () => {
    const bundle = cityBundle();
    delete bundle.scenarios[0].city;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/NYC provenance/);
  });

  it.each([1, 2, 3])("rejects retired Manhattan space-time protocol v%s", (version) => {
    const bundle = cityBundle();
    bundle.protocol.id = `manhattan-space-time-v${version}`;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol/);
  });

  it.each([
    ["spaceTimeConnectivity", undefined],
    ["spaceTimeConnectivity", 6],
    ["trajectoryShortcut", undefined],
    ["trajectoryShortcut", 1],
    ["trajectoryShortcut", false],
    ["trajectoryPostprocessor", "certified-fillet-plus-discrete-execution-envelope-v2"],
  ])("rejects an incompatible v4 protocol declaration for %s=%s", (field, value) => {
    const bundle = cityBundle();
    bundle.protocol[field as string] = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\./);
  });

  it.each([undefined, true, 0])("rejects v4 protocol.trajectoryPreserveAltitude=%s", (value) => {
    const bundle = cityBundle();
    bundle.protocol.trajectoryPreserveAltitude = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\.trajectoryPreserveAltitude/);
  });

  it("rejects a v4 protocol with the old free-3D postprocessor", () => {
    const bundle = cityBundle();
    bundle.protocol.trajectoryPostprocessor = "spacetime-shortcut-fillet-plus-sampling-stable-discrete-envelope-v3";
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\.trajectoryPostprocessor/);
  });

  it.each([undefined, 3, true])("requires the declared local curve degree: %s", (value) => {
    const bundle = cityBundle();
    bundle.protocol.trajectoryCurveDegree = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\.trajectoryCurveDegree/);
  });

  it.each([undefined, 3, true])("requires the per-run local curve degree: %s", (value) => {
    const bundle = cityBundle();
    bundle.scenarios[0].runs[0].parameters.trajectoryCurveDegree = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/parameters\.trajectoryCurveDegree/);
  });

  it.each([undefined, 2, true])("requires three-dimensional protocol curves: %s", (value) => {
    const bundle = cityBundle();
    bundle.protocol.trajectoryCurveDimensions = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\.trajectoryCurveDimensions/);
  });

  it.each([undefined, 6, 24])("requires the declared total height budget: %s", (value) => {
    const bundle = cityBundle();
    bundle.protocol.trajectoryAltitudeDeviationLimitM = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\.trajectoryAltitudeDeviationLimitM/);
  });

  it.each([undefined, 1, true])("requires numeric per-run XYZ declaration: %s", (value) => {
    const bundle = cityBundle();
    bundle.scenarios[0].runs[0].parameters.trajectoryPreserveAltitude = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/parameters\.trajectoryPreserveAltitude/);
  });

  it.each([
    ["optimizationAxes", undefined],
    ["optimizationAxes", ["x", "y"]],
    ["optimizationAxes", ["y", "x"]],
    ["altitudePolicy", undefined],
    ["altitudePolicy", "free-3d"],
    ["altitudeDeviationLimitM", undefined],
    ["altitudeDeviationLimitM", 24],
  ])("rejects missing or incompatible v4 smoothing %s", (field, value) => {
    const bundle = cityBundle();
    bundle.scenarios[0].runs[0].smoothing[field as string] = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(new RegExp(`smoothing\\.${field}`));
  });

  it.each([12.0000015, 12.00002, 13])("checks an interior raw altitude knot against the XYZ budget: %s", (offset) => {
    const bundle = losOnlyBundle();
    // Equal endpoints cannot prove the height budget at a removed raw midpoint.
    // Check the union of all knots with an absolute metre tolerance.
    bundle.scenarios[0].runs[0].rawTimedPath[1].position[2] += offset;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/declared altitude deviation/);
  });

  it("accepts a canonicalized zero trim scale for a qualified spatial reversal spline", () => {
    const bundle = cityBundle();
    const smoothing = bundle.scenarios[0].runs[0].smoothing;
    expect(smoothing.method).toBe("spacetime-shortcut-plus-local-quintic-bspline");
    smoothing.appliedTurnRadiusM = 0;
    expect(validatePredictiveBundle(bundle).scenarios[0]!.runs[0]!.smoothing.appliedTurnRadiusM).toBe(0);
    smoothing.appliedTurnRadiusM = -0.00001;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/appliedTurnRadiusM/);
  });

  it("also checks altitude knots introduced only by geometry sampling", () => {
    const bundle = losOnlyBundle();
    const path = bundle.scenarios[0].runs[0].geometryTimedPath;
    const left = path[0], right = path[1];
    path.splice(1, 0, {
      timeS: (left.timeS + right.timeS) / 2,
      position: [(left.position[0] + right.position[0]) / 2, left.position[1], left.position[2] + 13],
    });
    expect(() => validatePredictiveBundle(bundle)).toThrow(/declared altitude deviation/);
  });

  it("requires shortcut declarations on every computed run", () => {
    const bundle = cityBundle();
    delete bundle.scenarios[0].runs[0].parameters.trajectoryShortcut;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/parameters\.trajectoryShortcut/);
  });

  it("requires 26-connected motion on the 4D run only", () => {
    const missing = cityBundle();
    const predictive = missing.scenarios[0].runs.find((run: { predictive: boolean }) => run.predictive);
    predictive.parameters.spaceTimeConnectivity = 6;
    expect(() => validatePredictiveBundle(missing)).toThrow(/parameters\.spaceTimeConnectivity/);
    const misplaced = cityBundle();
    misplaced.scenarios[0].runs[0].parameters.spaceTimeConnectivity = 26;
    expect(() => validatePredictiveBundle(misplaced)).toThrow(/only valid for the 4D planner/);
  });

  it("loads a runtime-computed two-point LOS-only path with no fictitious fillet radius", () => {
    const bundle = validatePredictiveBundle(losOnlyBundle());
    for (const run of bundle.scenarios[0]!.runs) {
      expect(run.rawTimedPath).toHaveLength(3);
      expect(run.geometryTimedPath).toHaveLength(2);
      expect(run.executionTimedPath).toHaveLength(2);
      expect(run.smoothing.method).toBe("spacetime-shortcut");
      expect(run.smoothing.applied).toBe(true);
      expect(run.smoothing.appliedTurnRadiusM).toBeNull();
      expect(run.smoothing.roundedCornerCount).toBe(0);
    }
  });

  it("does not allow an applied LOS-only candidate to omit collision certification", () => {
    const bundle = losOnlyBundle();
    bundle.scenarios[0].runs[0].smoothing.certified = false;
    bundle.scenarios[0].runs[0].smoothing.collisionCertified = false;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/applied output must be certified/);
  });

  it.each([
    ["spacetime-shortcut", 1, null],
    ["spacetime-shortcut", 0, 24],
    ["spacetime-shortcut-plus-local-quintic-bspline", 0, null],
  ])("rejects inconsistent shortcut method/corners/radius %s %s %s", (method, corners, radius) => {
    const bundle = losOnlyBundle();
    const smoothing = bundle.scenarios[0].runs[0].smoothing;
    smoothing.method = method;
    smoothing.roundedCornerCount = corners;
    smoothing.appliedTurnRadiusM = radius;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/shortcut method|shortcut-plus-curve method/);
  });

  it("rejects a retired circular-fillet method on the current XYZ protocol", () => {
    const bundle = cityBundle();
    bundle.scenarios[0].runs[0].smoothing.method = "spacetime-shortcut-plus-sampled-circular-fillet";
    expect(() => validatePredictiveBundle(bundle)).toThrow(/method is unsupported/);
  });

  it("allows differently sampled turn-angle diagnostics for the current spatial curves", () => {
    const bundle = cityBundle();
    // Segment angles depend on sampling density, rather than just curvature.
    const smoothing = bundle.scenarios[0].runs[0].smoothing;
    smoothing.maxTurnAngleBeforeDeg = 1.5;
    smoothing.maxTurnAngleAfterDeg = 4.7368;
    expect(validatePredictiveBundle(bundle).scenarios[0]!.runs[0]!.smoothing.maxTurnAngleAfterDeg)
      .toBe(4.7368);
  });

  it.each([181, Number.NaN])("still rejects an invalid v4 turn angle %s", (angle) => {
    const bundle = cityBundle();
    bundle.scenarios[0].runs[0].smoothing.maxTurnAngleAfterDeg = angle;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/maxTurnAngleAfterDeg/);
  });
});
