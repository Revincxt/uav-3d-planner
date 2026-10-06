import { describe, expect, it } from "vitest";
import { validateDynamicBundle } from "../src/dynamic-data";

// Runtime-only Node helper; the browser project does not require Node type declarations.
const filesystemModule = "node:fs";
const { readFileSync } = await import(filesystemModule);
const source = JSON.parse(readFileSync(new URL("../public/dynamic-data.json", import.meta.url), "utf8"));

// These integration tests clone and validate the complete 126 MB native cohort.
// Runner contention must not turn data-integrity checks into 5-second speed tests.
describe("recorded Manhattan reactive missions", { timeout: 30000 }, () => {
  it("retains the complete real city, footprints, mission identity and actual replay", () => {
    const bundle = validateDynamicBundle(source);
    expect(bundle.protocol.id).toBe("manhattan-reactive-demo-v3");
    expect(bundle.protocol.pathShortcut).toBe(1);
    expect(bundle.protocol.preserveAltitude).toBe(1);
    expect(bundle.protocol.smoothTurns).toBe(1);
    expect(bundle.protocol.turnScaleM).toBe(60);
    expect(bundle.protocol.curveSampleSpacingM).toBe(2);
    expect(bundle.sourceCommit).toMatch(/^local-snapshot:sha256:[0-9a-f]{64}$/);
    expect(bundle.scenarios).toHaveLength(8);
    for (const scenario of bundle.scenarios) {
      expect(scenario.city?.sourceKind).toBe("nyc-open-data");
      expect(scenario.buildings.length).toBe(scenario.city?.buildingCount);
      expect(scenario.buildings.length).toBeGreaterThan(1000);
      expect(scenario.buildings.every((building) => building.footprint && building.footprint.length > 0)).toBe(true);
      expect(scenario.mission?.origin).toBeTruthy();
      expect(scenario.movingSpheres).toHaveLength(7);
      expect(scenario.runs).toHaveLength(3);
      for (const run of scenario.runs) {
        expect(run.status).toBe("success");
        expect(run.metrics.collisionCount).toBe(0);
        expect(run.frames.at(-1)?.vehicle).toEqual(scenario.goal);
        expect(run.parameters.pathShortcut).toBe(1);
        expect(run.parameters.preserveAltitude).toBe(1);
        expect(run.parameters.smoothTurns).toBe(1);
        expect(run.parameters.turnScaleM).toBe(bundle.protocol.turnScaleM);
        expect(run.parameters.curveSampleSpacingM).toBe(bundle.protocol.curveSampleSpacingM);
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

  it("retains compatibility with the legacy Manhattan v1 replay shape", () => {
    const bundle = structuredClone(source);
    bundle.protocol.id = "manhattan-reactive-demo-v1";
    delete bundle.protocol.pathShortcut;
    delete bundle.protocol.preserveAltitude;
    for (const scenario of bundle.scenarios) {
      for (const run of scenario.runs) {
        delete run.parameters.pathShortcut;
        delete run.parameters.preserveAltitude;
      }
    }
    expect(validateDynamicBundle(bundle).protocol.id).toBe("manhattan-reactive-demo-v1");
  });

  it("retains compatibility with the legacy Manhattan v2 unconstrained-height replay", () => {
    const bundle = structuredClone(source);
    bundle.protocol.id = "manhattan-reactive-demo-v2";
    delete bundle.protocol.preserveAltitude;
    for (const scenario of bundle.scenarios) {
      for (const run of scenario.runs) delete run.parameters.preserveAltitude;
    }
    expect(validateDynamicBundle(bundle).protocol.id).toBe("manhattan-reactive-demo-v2");
  });

  it.each([undefined, true, 0, 2])("rejects v2 protocol.pathShortcut=%s", (value) => {
    expect(() => validateDynamicBundle({ ...source, protocol: {
      ...source.protocol, id: "manhattan-reactive-demo-v2", pathShortcut: value,
    } }))
      .toThrow(/protocol\.pathShortcut/);
  });

  it.each([undefined, true, 0, 2])("rejects v3 protocol.preserveAltitude=%s", (value) => {
    expect(() => validateDynamicBundle({ ...source, protocol: { ...source.protocol, preserveAltitude: value } }))
      .toThrow(/protocol\.preserveAltitude/);
  });

  it.each([undefined, 0, true])("requires numeric preserveAltitude on every v3 replay run: %s", (value) => {
    const bundle = structuredClone(source);
    bundle.scenarios[0].runs[0].parameters.preserveAltitude = value;
    expect(() => validateDynamicBundle(bundle)).toThrow(/parameters\.preserveAltitude/);
  });

  it("requires a shortcut declaration on every replay run", () => {
    const bundle = structuredClone(source);
    delete bundle.scenarios[0].runs[0].parameters.pathShortcut;
    expect(() => validateDynamicBundle(bundle)).toThrow(/parameters\.pathShortcut/);
  });

  it.each([undefined, 0, true])("requires numeric protocol.smoothTurns for declared curves: %s", (value) => {
    expect(() => validateDynamicBundle({ ...source, protocol: { ...source.protocol, smoothTurns: value } }))
      .toThrow(/protocol\.smoothTurns/);
  });

  it.each([
    ["smoothTurns", 0],
    ["turnScaleM", 30],
    ["curveSampleSpacingM", 10],
  ])("requires matching per-run curve configuration: %s", (field, value) => {
    const bundle = structuredClone(source);
    bundle.scenarios[0].runs[0].parameters[field] = value;
    expect(() => validateDynamicBundle(bundle)).toThrow(/curve.*protocol/);
  });
});
