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
    const failed = loadDataset("results", fallback);
    FakeWorker.all[0]!.onmessage!({ data: { error: "Invalid safety envelope" } });
    await expect(failed).rejects.toThrow("Invalid safety envelope");
    expect(fallback).not.toHaveBeenCalled(); expect(FakeWorker.all[0]!.terminate).toHaveBeenCalled();
    const retry = loadDataset("results", fallback);
    expect(FakeWorker.all).toHaveLength(2);
    FakeWorker.all[1]!.onmessage!({ data: { value: [] } }); await expect(retry).resolves.toEqual([]);
  });
  it("can fall back to the same validator when workers are absent or forbidden", async () => {
    const { loadDataset } = await import("../src/data-loader"), fallback = vi.fn().mockResolvedValue([]);
    vi.stubGlobal("Worker", undefined);
    await expect(loadDataset("results", fallback)).resolves.toEqual([]);
    vi.stubGlobal("Worker", class { constructor() { throw new Error("Blocked by host CSP"); } });
    await expect(loadDataset("results", fallback)).resolves.toEqual([]);
    expect(fallback).toHaveBeenCalledTimes(2);
  });
  it("releases a worker whose script cannot load and retries with validated foreground loading", async () => {
    const { loadDataset } = await import("../src/data-loader"), fallback = vi.fn().mockResolvedValue([]);
    const result = loadDataset("results", fallback);
    FakeWorker.all[0]!.onerror!({ message: "Worker asset unavailable" });
    await expect(result).resolves.toEqual([]);
    expect(fallback).toHaveBeenCalledTimes(1); expect(FakeWorker.all[0]!.terminate).toHaveBeenCalledTimes(1);
  });
  it("fails closed on an incomplete worker response", async () => {
    const { loadDataset } = await import("../src/data-loader"), fallback = vi.fn();
    const result = loadDataset("results", fallback);
    FakeWorker.all[0]!.onmessage!({ data: {} });
    await expect(result).rejects.toThrow("Incomplete"); expect(fallback).not.toHaveBeenCalled();
  });
});
