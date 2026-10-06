import { describe, expect, it, afterEach, vi } from "vitest";
import { packRuntimeData, unpackRuntimeData } from "../shared/runtime-data.mjs";
import { fetchRuntimeData } from "../src/runtime-fetch";
import { validateBundle } from "../src/static-validation";
import { validateDynamicBundle } from "../src/dynamic-validation";
import { validatePredictiveBundle } from "../src/predictive-validation";

const filesystem = "node:fs/promises", compression = "node:zlib", cryptoModule = "node:crypto";
const { readFile } = await import(filesystem), { gzipSync } = await import(compression), { createHash } = await import(cryptoModule);
const digest = (value: unknown) => createHash("sha256").update(JSON.stringify(value)).digest("hex");
afterEach(() => vi.unstubAllGlobals());

describe("lossless runtime delivery", () => {
  it.each([
    ["demo-data", validateBundle], ["dynamic-data", validateDynamicBundle], ["predictive-data", validatePredictiveBundle],
  ] as const)("preserves every native field and validation in %s", async (name, validate) => {
    const original = JSON.parse(await readFile(new URL(`../public/${name}.json`, import.meta.url), "utf8"));
    const expected = digest(original);
    const packed = packRuntimeData(original);
    const decoded = unpackRuntimeData(JSON.parse(JSON.stringify(packed))) as typeof original;
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
  }, 30000);
  it("preserves non-prefix histories instead of assuming execution is cumulative", () => {
    const original = { scenarios: [{ runs: [{ frames: [
      { executedPath: [[0, 1, 2], [1, 2, 3]] }, { executedPath: [[0, 1, 2], [4, 5, 6]] },
    ] }] }] };
    expect(unpackRuntimeData(packRuntimeData(original))).toEqual(original);
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
});

describe("compressed fetch and compatibility", () => {
  const original = { scenarios: [{ buildings: [], start: [1, 2, 3] }] };
  it("decompresses a real gzip response before lossless decoding", async () => {
    const bytes = gzipSync(JSON.stringify(packRuntimeData(original)));
    const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(bytes));
    expect(await fetchRuntimeData("static", "/app/", fetcher)).toEqual(original);
    expect(fetcher).toHaveBeenCalledExactlyOnceWith("/app/demo-data.runtime.json.gz");
  });
  it("uses ordinary compact JSON when streaming decompression is unavailable", async () => {
    vi.stubGlobal("DecompressionStream", undefined);
    const fetcher = vi.fn<typeof fetch>().mockResolvedValue(Response.json(packRuntimeData(original)));
    expect(await fetchRuntimeData("predictive", "./", fetcher)).toEqual(original);
    expect(fetcher).toHaveBeenCalledWith("./predictive-data.runtime.json");
  });
  it("accepts .gz that a static host already decompressed via Content-Encoding", async () => {
    const fetcher = vi.fn<typeof fetch>().mockResolvedValue(Response.json(packRuntimeData(original), { headers: { "Content-Encoding": "gzip" } }));
    expect(await fetchRuntimeData("static", "./", fetcher)).toEqual(original);
  });
  it("recognizes gzip when its magic bytes arrive in separate stream chunks", async () => {
    const bytes: Uint8Array = gzipSync(JSON.stringify(packRuntimeData(original)));
    const body = new ReadableStream({ start(controller) {
      controller.enqueue(bytes.slice(0, 1)); controller.enqueue(bytes.slice(1, 2)); controller.enqueue(bytes.slice(2)); controller.close();
    } });
    const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(body));
    expect(await fetchRuntimeData("static", "./", fetcher)).toEqual(original);
  });
  it("keeps the original static-hosted dataset as a 404-only fallback", async () => {
    const fetcher = vi.fn<typeof fetch>().mockResolvedValueOnce(new Response(null, { status: 404 })).mockResolvedValueOnce(Response.json(original));
    expect(await fetchRuntimeData("dynamic", "./", fetcher)).toEqual(original);
    expect(fetcher.mock.calls.map(call => call[0])).toEqual(["./dynamic-data.runtime.json.gz", "./dynamic-data.json"]);
  });
  it.each([403, 500])("does not hide HTTP %s behind the old dataset", async status => {
    const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(null, { status }));
    await expect(fetchRuntimeData("static", "./", fetcher)).rejects.toThrow(String(status));
    expect(fetcher).toHaveBeenCalledTimes(1);
  });
  it("does not accept corrupted compressed bytes or references", async () => {
    const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response("bad gzip"));
    await expect(fetchRuntimeData("static", "./", fetcher)).rejects.toThrow();
    expect(fetcher).toHaveBeenCalledTimes(1);
    vi.stubGlobal("DecompressionStream", undefined);
    fetcher.mockClear().mockResolvedValue(Response.json({ runtimeVersion: 1, pool: [], bundle: { scenarios: { $runtimeRef: 7 } } }));
    await expect(fetchRuntimeData("static", "./", fetcher)).rejects.toThrow("reference");
    expect(fetcher).toHaveBeenCalledTimes(1);
  });
});
