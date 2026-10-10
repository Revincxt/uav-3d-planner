import { describe, expect, it } from "vitest";
import { validateDynamicBundle } from "../src/dynamic-data";
import { readStudyData } from '../scripts/study-reader.mjs';

const source = await readStudyData(new URL("../public/dynamic-data.json", import.meta.url));

// Copy only the mutable schema declarations. Hundreds of MB of read-only city
// polygons and replay samples must not be cloned for each negative test.
const copyDeclarations = () => ({ ...source, protocol: { ...source.protocol },
  scenarios: source.scenarios.map((scenario: typeof source.scenarios[number]) => ({ ...scenario,
    runs: scenario.runs.map((run: typeof scenario.runs[number]) => ({ ...run, parameters: { ...run.parameters } })),
  })),
});

// These integration tests validate the complete native cohort.
// Runner contention must not turn data-integrity checks into 5-second speed tests.
describe("recorded Manhattan reactive missions", { timeout: 30000 }, () => {
  it("retains the complete real city, footprints, mission identity and actual replay", () => {
    const bundle = validateDynamicBundle(source);
    expect(bundle.protocol.id).toBe("manhattan-reactive-demo-v5");
    expect(bundle.protocol.pathShortcut).toBe(1);
    expect(bundle.protocol.preserveAltitude).toBe(1);
    expect(bundle.protocol.smoothTurns).toBe(1);
    expect(bundle.protocol.curveDimensions).toBe(3);
    expect(bundle.protocol.turnScaleM).toBe(60);
    expect(bundle.protocol.curveSampleSpacingM).toBe(2);
    expect(bundle.protocol.verticalCostScale).toBe(5);
    expect(bundle.protocol.maxClimbRateMps).toBe(3);
    expect(bundle.sourceCommit).toMatch(/^local-snapshot:sha256:[0-9a-f]{64}$/);
    expect(bundle.scenarios).toHaveLength(8);
    for (const scenario of bundle.scenarios) {
      expect(scenario.city?.sourceKind).toBe("nyc-open-data");
      expect(scenario.buildings.length).toBe(scenario.city?.buildingCount);
      expect(scenario.buildings.length).toBeGreaterThan(1000);
      expect(scenario.buildings.every((building) => building.footprint && building.footprint.length > 0)).toBe(true);
      expect(scenario.mission?.origin).toBeTruthy();
      expect(scenario.movingSpheres).toHaveLength(12);
      expect(scenario.runs).toHaveLength(3);
      for (const run of scenario.runs) {
        expect(run.status).toBe("success");
        expect(run.metrics.collisionCount).toBe(0);
        expect(run.frames.at(-1)?.vehicle).toEqual(scenario.goal);
        expect(run.parameters.pathShortcut).toBe(1);
        expect(run.parameters.preserveAltitude).toBe(1);
        expect(run.parameters.smoothTurns).toBe(1);
        expect(run.parameters.curveDimensions).toBe(3);
        expect(run.parameters.turnScaleM).toBe(bundle.protocol.turnScaleM);
        expect(run.parameters.curveSampleSpacingM).toBe(bundle.protocol.curveSampleSpacingM);
        expect(run.parameters.verticalCostScale).toBe(5);
        expect(run.executionTimedPath?.at(-1)?.timeS).toBeCloseTo(run.frames.at(-1)!.timeS, 8);
      }
    }
  });

  it("rejects undeclared city-grid changes instead of accepting arbitrary protocols", () => {
    expect(() => validateDynamicBundle({ ...source, protocol: { ...source.protocol, resolutionM: 51 } }))
      .toThrow(/protocol/);
  });

  it("does not let a local export claim to be a Git commit", () => {
    expect(() => validateDynamicBundle({ ...source, sourceCommit: "a".repeat(40) }))
      .toThrow(/local source snapshot/);
  });

  it("rejects a selectively truncated city even when remaining buildings are individually valid", () => {
    const scenario = { ...source.scenarios[0], buildings: source.scenarios[0].buildings.slice(1) };
    expect(() => validateDynamicBundle({ ...source, scenarios: [scenario, ...source.scenarios.slice(1)] }))
      .toThrow(/complete physical district/);
  });

  it("rejects a source polygon that leaves its planning collision envelope", () => {
    const original = source.scenarios[0].buildings[0];
    const footprint = original.footprint.map((ring: number[][]) => ring.map((point) => [...point]));
    footprint[0][0][0] = original.max[0] + 1;
    const building = { ...original, footprint };
    const scenario = { ...source.scenarios[0], buildings: [building, ...source.scenarios[0].buildings.slice(1)] };
    expect(() => validateDynamicBundle({ ...source, scenarios: [scenario, ...source.scenarios.slice(1)] }))
      .toThrow(/collision envelope/);
  });

  it.each([1, 2, 3, 4])("rejects retired Manhattan replay protocol v%s", (version) => {
    const bundle = copyDeclarations();
    bundle.protocol.id = `manhattan-reactive-demo-v${version}`;
    expect(() => validateDynamicBundle(bundle)).toThrow(/protocol/);
  });

  it.each([undefined, true, 0, 2])("rejects v5 protocol.pathShortcut=%s", (value) => {
    expect(() => validateDynamicBundle({ ...source, protocol: {
      ...source.protocol, pathShortcut: value,
    } }))
      .toThrow(/protocol\.pathShortcut/);
  });

  it.each([undefined, true, 0, 2])("rejects v5 protocol.preserveAltitude=%s", (value) => {
    expect(() => validateDynamicBundle({ ...source, protocol: { ...source.protocol, preserveAltitude: value } }))
      .toThrow(/protocol\.preserveAltitude/);
  });

  it.each([undefined, 0, true])("requires numeric preserveAltitude on every v5 replay run: %s", (value) => {
    const bundle = copyDeclarations();
    bundle.scenarios[0].runs[0].parameters.preserveAltitude = value;
    expect(() => validateDynamicBundle(bundle)).toThrow(/parameters\.preserveAltitude/);
  });

  it("requires a shortcut declaration on every replay run", () => {
    const bundle = copyDeclarations();
    delete bundle.scenarios[0].runs[0].parameters.pathShortcut;
    expect(() => validateDynamicBundle(bundle)).toThrow(/parameters\.pathShortcut/);
  });

  it.each([undefined, 0, true])("requires numeric protocol.smoothTurns for declared curves: %s", (value) => {
    expect(() => validateDynamicBundle({ ...source, protocol: { ...source.protocol, smoothTurns: value } }))
      .toThrow(/protocol\.smoothTurns/);
  });

  it.each([undefined, 2, true])("requires declared XYZ curve dimensions: %s", (value) => {
    expect(() => validateDynamicBundle({ ...source, protocol: { ...source.protocol, curveDimensions: value } }))
      .toThrow(/protocol\.curveDimensions/);
  });

  it.each([
    ["smoothTurns", 0],
    ["curveDimensions", 2],
    ["turnScaleM", 30],
    ["curveSampleSpacingM", 10],
  ])("requires matching per-run curve configuration: %s", (field, value) => {
    const bundle = copyDeclarations();
    bundle.scenarios[0].runs[0].parameters[field] = value;
    expect(() => validateDynamicBundle(bundle)).toThrow(/curve.*protocol/);
  });
});
