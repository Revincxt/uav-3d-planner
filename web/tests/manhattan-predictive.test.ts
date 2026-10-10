import { describe, expect, it } from "vitest";
import { validatePredictiveBundle } from "../src/predictive-data";
import losFixture from "./fixtures/predictive-los-only.json";

const filesystemModule = "node:fs";
const { readFileSync }: { readFileSync: (path: URL, encoding: "utf8") => string } = await import(filesystemModule);
const source = readFileSync(new URL("../public/predictive-data.json", import.meta.url), "utf8");
const cityBundle = () => JSON.parse(source);

function omitSchedulingOnlyEvidence(bundle: ReturnType<typeof cityBundle>) {
  delete bundle.protocol.trajectoryDynamicScheduling;
  // A modern stopped reversal cannot be relabelled as legacy duration-only
  // retiming. The schema fixture omits that optional flight instead; it does
  // not fabricate a legacy timing certificate or weaken the old parser gate.
  for (const scenario of bundle.scenarios) for (const run of scenario.runs) {
    if (run.executionTimedPath?.length !== run.geometryTimedPath.length) {
      run.executionTimedPath = run.executionWaitIntervals = run.executionMetrics = run.executionFrames = null;
      Object.assign(run.smoothing.execution, { status: "not-evaluated", qualified: false,
        collisionCertified: false, qualification: null, timingIterations: 0,
        originalDurationS: null, candidateDurationS: null, addedDurationS: null });
    }
  }
}

function legacyV2Bundle() {
  const bundle = cityBundle();
  omitSchedulingOnlyEvidence(bundle);
  bundle.protocol.id = "manhattan-space-time-v2";
  bundle.protocol.trajectoryPostprocessor = "spacetime-shortcut-fillet-plus-sampling-stable-discrete-envelope-v3";
  delete bundle.protocol.trajectoryPreserveAltitude;
  delete bundle.protocol.trajectoryCurveDegree;
  for (const scenario of bundle.scenarios) {
    for (const run of scenario.runs) {
      delete run.parameters.trajectoryPreserveAltitude;
      delete run.parameters.trajectoryCurveDegree;
      delete run.smoothing.optimizationAxes;
      delete run.smoothing.altitudePolicy;
      if (run.smoothing.method === "spacetime-shortcut-plus-local-quintic-bspline") {
        // Schema-only legacy fixture, not evidence produced by the old postprocessor.
        run.smoothing.method = "spacetime-shortcut-plus-sampled-circular-fillet";
      }
    }
  }
  return bundle;
}

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
        parameters: { ...planner.parameters, trajectoryPreserveAltitude: 1, trajectoryCurveDegree: 5 },
      };
      Object.assign(run.smoothing, {
        optimizationAxes: ["x", "y"], altitudePolicy: "preserve-raw-z-time-profile",
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
    expect(bundle.protocol.id).toBe("manhattan-space-time-v3");
    expect(bundle.protocol.spaceTimeConnectivity).toBe(26);
    expect(bundle.protocol.trajectoryShortcut).toBe(true);
    expect(bundle.protocol.trajectoryPreserveAltitude).toBe(true);
    expect(bundle.protocol.trajectoryCurveDegree).toBe(5);
    expect(bundle.scenarios).toHaveLength(8);
    expect(bundle.scenarios.flatMap((scenario) => scenario.runs)).toHaveLength(32);
    for (const scenario of bundle.scenarios) {
      expect(scenario.city?.sourceKind).toBe("nyc-open-data");
      expect(scenario.buildings.length).toBeGreaterThan(4000);
      expect(scenario.buildings.every((building) => building.footprint?.length)).toBe(true);
      expect(scenario.mission?.origin).toBeTruthy();
      expect(scenario.movingSpheres).toHaveLength(7);
      for (const run of scenario.runs) {
        expect(run.status).toBe("success");
        expect(run.smoothing.execution.qualified).toBe(true);
        expect(run.plannerMetrics.safetyViolations).toBe(0);
        expect(run.executionMetrics?.safetyViolations).toBe(0);
        expect(run.parameters.trajectoryShortcut).toBe(1);
        expect(run.parameters.trajectoryPreserveAltitude).toBe(1);
        expect(run.parameters.trajectoryCurveDegree).toBe(5);
        expect(["spacetime-shortcut-plus-local-quintic-bspline", "spacetime-shortcut-fillet-fallback", "spacetime-shortcut"])
          .toContain(run.smoothing.method);
        if (run.smoothing.method === "spacetime-shortcut-plus-local-quintic-bspline")
          expect(run.smoothing.roundedCornerCount).toBeGreaterThan(0);
        expect(run.smoothing.optimizationAxes).toEqual(["x", "y"]);
        expect(run.smoothing.altitudePolicy).toBe("preserve-raw-z-time-profile");
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

  it("accepts the legacy Manhattan v1 protocol shape without enhancement declarations", () => {
    const bundle = cityBundle();
    omitSchedulingOnlyEvidence(bundle);
    bundle.protocol.id = "manhattan-space-time-v1";
    bundle.protocol.trajectoryPostprocessor = "certified-fillet-plus-discrete-execution-envelope-v2";
    delete bundle.protocol.spaceTimeConnectivity;
    delete bundle.protocol.trajectoryShortcut;
    delete bundle.protocol.trajectoryPreserveAltitude;
    for (const scenario of bundle.scenarios) {
      for (const run of scenario.runs) {
        delete run.parameters.trajectoryShortcut;
        delete run.parameters.spaceTimeConnectivity;
        delete run.parameters.trajectoryPreserveAltitude;
        delete run.smoothing.optimizationAxes;
        delete run.smoothing.altitudePolicy;
        run.smoothing.method = "sampled-circular-fillet";
        // Schema-only fixture: v3's differently sampled angle diagnostics are not v1 evidence.
        // Omit these optional fields; the dedicated test below still enforces the v1 angle gate.
        run.smoothing.maxTurnAngleBeforeDeg = null;
        run.smoothing.maxTurnAngleAfterDeg = null;
        // Schema-only legacy fixture: v1 "applied" always declares a fillet radius.
        // Newly computed v3 runs may instead use an applied, radius-free LOS shortcut.
        if (run.smoothing.applied && run.smoothing.appliedTurnRadiusM === null) {
          run.smoothing.appliedTurnRadiusM = run.smoothing.requestedTurnRadiusM;
        }
      }
    }
    expect(validatePredictiveBundle(bundle).protocol.id).toBe("manhattan-space-time-v1");
  });

  it.each([
    ["spaceTimeConnectivity", undefined],
    ["spaceTimeConnectivity", 6],
    ["trajectoryShortcut", undefined],
    ["trajectoryShortcut", 1],
    ["trajectoryShortcut", false],
    ["trajectoryPostprocessor", "certified-fillet-plus-discrete-execution-envelope-v2"],
  ])("rejects an incompatible v2 protocol declaration for %s=%s", (field, value) => {
    const bundle = legacyV2Bundle();
    bundle.protocol[field as string] = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\./);
  });

  it("retains compatibility with the legacy Manhattan v2 free-3D postprocessor", () => {
    expect(validatePredictiveBundle(legacyV2Bundle()).protocol.id).toBe("manhattan-space-time-v2");
  });

  it.each([undefined, false, 1])("rejects v3 protocol.trajectoryPreserveAltitude=%s", (value) => {
    const bundle = cityBundle();
    bundle.protocol.trajectoryPreserveAltitude = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/protocol\.trajectoryPreserveAltitude/);
  });

  it("rejects a v3 protocol with the old free-3D postprocessor", () => {
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

  it.each([undefined, 0, true])("requires numeric per-run altitude preservation: %s", (value) => {
    const bundle = cityBundle();
    bundle.scenarios[0].runs[0].parameters.trajectoryPreserveAltitude = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/parameters\.trajectoryPreserveAltitude/);
  });

  it.each([
    ["optimizationAxes", undefined],
    ["optimizationAxes", ["x", "y", "z"]],
    ["optimizationAxes", ["y", "x"]],
    ["altitudePolicy", undefined],
    ["altitudePolicy", "free-3d"],
  ])("rejects missing or incompatible v3 smoothing %s", (field, value) => {
    const bundle = cityBundle();
    bundle.scenarios[0].runs[0].smoothing[field as string] = value;
    expect(() => validatePredictiveBundle(bundle)).toThrow(new RegExp(`smoothing\\.${field}`));
  });

  it.each([1.5e-6, 2e-5, 0.25])("checks an interior raw altitude knot with absolute metre tolerance: %s", (offset) => {
    const bundle = losOnlyBundle();
    // The geometry has only two points: equal endpoints cannot prove preservation of the
    // removed midpoint. Even 1.5 micrometres is below the old relative comparator at 90 m.
    bundle.scenarios[0].runs[0].rawTimedPath[1].position[2] += offset;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/preserve raw z\(t\)/);
  });

  it("also checks altitude knots introduced only by geometry sampling", () => {
    const bundle = losOnlyBundle();
    const path = bundle.scenarios[0].runs[0].geometryTimedPath;
    const left = path[0], right = path[1];
    path.splice(1, 0, {
      timeS: (left.timeS + right.timeS) / 2,
      position: [(left.position[0] + right.position[0]) / 2, left.position[1], left.position[2] + 0.25],
    });
    expect(() => validatePredictiveBundle(bundle)).toThrow(/preserve raw z\(t\)/);
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
    ["spacetime-shortcut-plus-sampled-circular-fillet", 0, null],
  ])("rejects inconsistent shortcut method/corners/radius %s %s %s", (method, corners, radius) => {
    const bundle = losOnlyBundle();
    const smoothing = bundle.scenarios[0].runs[0].smoothing;
    smoothing.method = method;
    smoothing.roundedCornerCount = corners;
    smoothing.appliedTurnRadiusM = radius;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/shortcut method|shortcut-plus-fillet method/);
  });

  it("allows differently sampled turn-angle diagnostics in v2 without weakening the v1 gate", () => {
    const bundle = legacyV2Bundle();
    // A 120-segment arc can report 1.5 degrees per raw segment, while its safe shortened fillet
    // samples report 4.7368 degrees. These fields describe different sampling, not curvature limits.
    const smoothing = bundle.scenarios[0].runs[0].smoothing;
    smoothing.maxTurnAngleBeforeDeg = 1.5;
    smoothing.maxTurnAngleAfterDeg = 4.7368;
    expect(validatePredictiveBundle(bundle).scenarios[0]!.runs[0]!.smoothing.maxTurnAngleAfterDeg)
      .toBe(4.7368);

    bundle.protocol.id = "manhattan-space-time-v1";
    bundle.protocol.trajectoryPostprocessor = "certified-fillet-plus-discrete-execution-envelope-v2";
    delete bundle.protocol.spaceTimeConnectivity;
    delete bundle.protocol.trajectoryShortcut;
    for (const scenario of bundle.scenarios) {
      for (const run of scenario.runs) {
        delete run.parameters.trajectoryShortcut;
        delete run.parameters.spaceTimeConnectivity;
        run.smoothing.method = "sampled-circular-fillet";
      }
    }
    // Schema-only v1 fillet fixture: modern LOS-only fallbacks have no radius.
    // Give this deliberate angle violation a valid fillet declaration so that
    // it reaches the legacy angle gate, independently of current demo routes.
    Object.assign(smoothing, { applied: true, appliedTurnRadiusM: 24, roundedCornerCount: 1 });
    expect(() => validatePredictiveBundle(bundle)).toThrow(/cannot increase the maximum turn angle/);
  });

  it.each([181, Number.NaN])("still rejects an invalid v2 turn angle %s", (angle) => {
    const bundle = legacyV2Bundle();
    bundle.scenarios[0].runs[0].smoothing.maxTurnAngleAfterDeg = angle;
    expect(() => validatePredictiveBundle(bundle)).toThrow(/maxTurnAngleAfterDeg/);
  });
});
