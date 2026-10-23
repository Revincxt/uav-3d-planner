import { describe, expect, it } from "vitest";
import { packRuntimeData, splitRuntimeCity, unpackRuntimeData } from "../shared/runtime-data.mjs";
import { validateBundle } from "../src/static-validation";
import { validateDynamicBundle } from "../src/dynamic-validation";
import { validatePredictiveBundle } from "../src/predictive-validation";
import { readStudyData } from '../scripts/study-reader.mjs';

const cryptoModule = "node:crypto";
const { createHash } = await import(cryptoModule);
// Same JSON value digest without materializing a >512 MiB root string.
const digest = (value: Record<string, unknown>) => {
  const hash = createHash("sha256").update('{');
  Object.entries(value).forEach(([key, item], index) => {
    if (index) hash.update(',');
    hash.update(`${JSON.stringify(key)}:`);
    if (key === 'scenarios' && Array.isArray(item)) {
      hash.update('['); item.forEach((scenario, i) => { if (i) hash.update(','); hash.update(JSON.stringify(scenario)); }); hash.update(']');
    } else hash.update(JSON.stringify(item));
  });
  return hash.update('}').digest('hex');
};

describe("lossless runtime delivery", () => {
  it.each([
    ["demo-data", validateBundle], ["dynamic-data", validateDynamicBundle], ["predictive-data", validatePredictiveBundle],
  ] as const)("preserves every native field and validation in %s", async (name, validate) => {
    const original = await readStudyData(new URL(`../public/${name}.json`, import.meta.url));
    const expected = digest(original);
    const packed = packRuntimeData(original);
    const split = splitRuntimeCity(packed);
    const decoded = unpackRuntimeData(JSON.parse(JSON.stringify({ runtimeVersion: 2, citySha256: 'a'.repeat(64),
      pool: split.pool, bundle: split.bundle })), JSON.parse(JSON.stringify(split.city))) as typeof original;
    expect(digest(decoded)).toBe(expected);
    expect(digest(original)).toBe(expected);
    expect(decoded.scenarios[0].buildings).toBe(decoded.scenarios[1].buildings);
    const parsed = validate(decoded);
    expect(parsed.scenarios).toHaveLength(8);
    expect(parsed.scenarios[0]!.buildings).toBe(parsed.scenarios[1]!.buildings);
    if (name === "dynamic-data") {
      const frames = validateDynamicBundle(decoded).scenarios[0]!.runs[0]!.frames;
      expect(frames[0]!.executedPath[0]).toBe(frames.at(-1)!.executedPath[0]);
    }
  }, 60000);
  it("preserves non-prefix histories instead of assuming execution is cumulative", () => {
    const original = { scenarios: [{ runs: [{ frames: [
      { executedPath: [[0, 1, 2], [1, 2, 3]] }, { executedPath: [[0, 1, 2], [4, 5, 6]] },
    ] }] }] };
    expect(unpackRuntimeData(packRuntimeData(original))).toEqual(original);
  });
  it("shares repeated history prefixes within a decode, without a persistent slice cache", () => {
    const input = { runtimeVersion: 1, pool: [[[0, 1, 2], [3, 4, 5]]], bundle: {
      a: { $runtimeRef: 0, prefix: 1 }, b: { $runtimeRef: 0, prefix: 1 },
      c: { $runtimeRef: 0, prefix: 2 },
    } };
    const first = unpackRuntimeData(input) as { a: number[][]; b: number[][]; c: number[][] };
    expect(first.a).toBe(first.b); expect(first.a).not.toBe(first.c);
    expect(first).toEqual({ a: [[0, 1, 2]], b: [[0, 1, 2]], c: [[0, 1, 2], [3, 4, 5]] });
    const second = unpackRuntimeData(input) as typeof first;
    expect(second.a).not.toBe(first.a);
    expect(input.pool[0]).toHaveLength(2);
  });
  it("rejects corrupt references and impossible prefixes", () => {
    for (const ref of [{ $runtimeRef: -1 }, { $runtimeRef: 1 }, { $runtimeRef: 0.5 },
      { $runtimeRef: 0, prefix: -1 }, { $runtimeRef: 0, prefix: 2 }, { $runtimeRef: 0, prefix: 0.5 },
      { $runtimeRef: 0, forged: true }]) {
      expect(() => unpackRuntimeData({ runtimeVersion: 1, pool: [[1]], bundle: { ref } })).toThrow("Invalid runtime");
    }
    expect(() => unpackRuntimeData({ runtimeVersion: 2, pool: [], bundle: {} })).toThrow("Unsupported");
    expect(() => unpackRuntimeData({ runtimeVersion: 1, pool: [1], bundle: { ref: { $runtimeRef: 0, prefix: 0 } } })).toThrow("prefix");
  });
  it('requires matching cities for every task without modifying the original packed data', () => {
    const city = { bounds: {}, buildings: [], city: { id: 'first' } };
    const packed = packRuntimeData({ scenarios: [{ ...city }, { ...city, city: { id: 'different' } }] });
    const before = JSON.stringify(packed);
    expect(() => splitRuntimeCity(packed)).toThrow('different cities');
    expect(JSON.stringify(packed)).toBe(before);
    expect(() => splitRuntimeCity(packRuntimeData({ scenarios: [] }))).toThrow('Missing shared runtime city');
  });
  it('rejects unknown, missing and augmented shared-city references', () => {
    for (const ref of [{ $runtimeCity: 'missing' }, { $runtimeCity: 'buildings' }, { $runtimeCity: 'city', extra: true }]) {
      expect(() => unpackRuntimeData({ runtimeVersion: 2, citySha256: 'a'.repeat(64), pool: [ref],
        bundle: { ref: { $runtimeRef: 0 } } }, { city: {} })).toThrow('Invalid runtime city reference');
    }
  });
});

