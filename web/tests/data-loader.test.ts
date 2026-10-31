import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
class FakeWorker {
  static all: FakeWorker[] = [];
  onmessage?: (event: { data: unknown }) => void;
  onerror?: (event: { message: string }) => void;
  onmessageerror?: () => void;
  postMessage = vi.fn(); terminate = vi.fn();
  constructor() { FakeWorker.all.push(this); }
}
beforeEach(() => {
  vi.resetModules(); FakeWorker.all = [];
  vi.stubEnv("BASE_URL", "./");
  vi.stubGlobal("document", { baseURI: "https://example.test/planner/index.html" });
  vi.stubGlobal("Worker", FakeWorker);
});
afterEach(() => { vi.unstubAllGlobals(); vi.unstubAllEnvs(); });
describe("background dataset lifecycle", () => {
  it.each(["static", "dynamic", "predictive"] as const)("uses the shared worker and memoized protocol through the public %s loader", async kind => {
    const load = kind === "static" ? (await import("../src/data")).loadDemoBundle
      : kind === "dynamic" ? (await import("../src/dynamic-data")).loadDynamicBundle
      : (await import("../src/predictive-data")).loadPredictiveBundle;
    const result = load();
    expect(FakeWorker.all).toHaveLength(1);
    expect(FakeWorker.all[0]!.postMessage).toHaveBeenCalledExactlyOnceWith({ kind, base: "https://example.test/planner/" });
    const value = { scenarios: [] };
    FakeWorker.all[0]!.onmessage!({ data: { value } });
    await expect(result).resolves.toBe(value);
    await expect(load()).resolves.toBe(value);
    expect(FakeWorker.all).toHaveLength(1);
  });
  it("shares one in-flight load, keeps relative deployment base and terminates on delivery", async () => {
    const { loadInBackground } = await import("../src/data-loader");
    const first = loadInBackground("static"), second = loadInBackground("static");
    expect(first).toBe(second); expect(FakeWorker.all).toHaveLength(1);
    expect(FakeWorker.all[0]!.postMessage).toHaveBeenCalledWith({ kind: "static", base: "https://example.test/planner/" });
    FakeWorker.all[0]!.onmessage!({ data: { value: { scenarios: [] } } });
    await expect(first).resolves.toEqual({ scenarios: [] });
    expect(FakeWorker.all[0]!.terminate).toHaveBeenCalledTimes(1);
    expect(loadInBackground("static")).toBe(first);
  });
  it("rejects validation errors without falling back or retaining a failed cache entry", async () => {
    const { loadDataset } = await import("../src/data-loader"), fallback = vi.fn();
    const failed = loadDataset("static", fallback);
    FakeWorker.all[0]!.onmessage!({ data: { error: "Invalid safety envelope" } });
    await expect(failed).rejects.toThrow("Invalid safety envelope");
    expect(fallback).not.toHaveBeenCalled(); expect(FakeWorker.all[0]!.terminate).toHaveBeenCalled();
    const retry = loadDataset("static", fallback);
    expect(FakeWorker.all).toHaveLength(2);
    FakeWorker.all[1]!.onmessage!({ data: { value: { scenarios: [] } } }); await expect(retry).resolves.toEqual({ scenarios: [] });
  });
  it("can fall back to the same validator when workers are absent or forbidden", async () => {
    const { loadDataset } = await import("../src/data-loader"), fallback = vi.fn().mockResolvedValue({ scenarios: [] });
    vi.stubGlobal("Worker", undefined);
    await expect(loadDataset("static", fallback)).resolves.toEqual({ scenarios: [] });
    vi.stubGlobal("Worker", class { constructor() { throw new Error("Blocked by host CSP"); } });
    await expect(loadDataset("static", fallback)).resolves.toEqual({ scenarios: [] });
    expect(fallback).toHaveBeenCalledTimes(2);
  });
  it("releases a worker whose script cannot load and retries with validated foreground loading", async () => {
    const { loadDataset } = await import("../src/data-loader"), fallback = vi.fn().mockResolvedValue({ scenarios: [] });
    const result = loadDataset("static", fallback);
    FakeWorker.all[0]!.onerror!({ message: "Worker asset unavailable" });
    await expect(result).resolves.toEqual({ scenarios: [] });
    expect(fallback).toHaveBeenCalledTimes(1); expect(FakeWorker.all[0]!.terminate).toHaveBeenCalledTimes(1);
  });
  it("fails closed on an incomplete worker response", async () => {
    const { loadDataset } = await import("../src/data-loader"), fallback = vi.fn();
    const result = loadDataset("static", fallback);
    FakeWorker.all[0]!.onmessage!({ data: {} });
    await expect(result).rejects.toThrow("Incomplete"); expect(fallback).not.toHaveBeenCalled();
  });
});
