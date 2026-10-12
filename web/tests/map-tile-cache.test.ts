import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { decodeImageryTile } from "../src/map-imagery";
import { ImageryRequestError, loadImageryTile, TILE_CACHE } from "../src/map-tile-cache";

vi.mock("../src/map-imagery", () => ({ decodeImageryTile: vi.fn(async () => ({} as HTMLImageElement)) }));
const original = { ...TILE_CACHE };
const url = "https://basemap.nationalmap.gov/imagery-fixture.jpg";
const image = () => new Response(new Blob(["image"], { type: "image/jpeg" }));
const settle = async () => { for (let pass = 0; pass < 60; pass++) await Promise.resolve(); };
beforeEach(() => { vi.mocked(decodeImageryTile).mockReset().mockResolvedValue({} as HTMLImageElement); });
afterEach(async () => {
  await settle(); Object.assign(TILE_CACHE, original); vi.restoreAllMocks(); vi.unstubAllGlobals();
});

function cacheFixture() {
  const items = new Map<string, Response>();
  const cache = {
    match: vi.fn(async (key: string | Request) => items.get(typeof key === "string" ? key : key.url)?.clone()),
    delete: vi.fn(async (key: string | Request) => items.delete(typeof key === "string" ? key : key.url)),
    put: vi.fn(async (key: string, value: Response) => { items.set(key, value); }),
    keys: vi.fn(async () => [...items.keys()].map(key => new Request(key))),
  };
  const open = vi.fn(async () => cache); vi.stubGlobal("caches", { open });
  return { cache, items, open };
}

describe("bounded reusable public imagery", () => {
  it("does not require browser storage support to load a real image", async () => {
    vi.stubGlobal("caches", undefined);
    const fetcher = vi.fn(async () => image()); vi.stubGlobal("fetch", fetcher);
    const abort = new AbortController();
    await loadImageryTile(url, abort.signal, 256, 0);
    expect(fetcher).toHaveBeenCalledWith(url, { signal: abort.signal, credentials: "omit" });
    expect(decodeImageryTile).toHaveBeenCalledWith(expect.any(Blob), 256);
  });

  it("reuses validated disk-cached imagery across scene/layer replacements", async () => {
    const { cache, items, open } = cacheFixture(), fetcher = vi.fn(async () => image()); vi.stubGlobal("fetch", fetcher);
    await loadImageryTile(url, new AbortController().signal, 256); await settle();
    expect(items.has(url)).toBe(true); expect(open).toHaveBeenCalledWith(TILE_CACHE.name);
    await loadImageryTile(url, new AbortController().signal, 256);
    expect(fetcher).toHaveBeenCalledOnce(); expect(cache.put).toHaveBeenCalledOnce();
  });

  it("tolerates disabled storage and write quota failures", async () => {
    vi.stubGlobal("caches", { open: vi.fn(async () => { throw new DOMException("disabled", "SecurityError"); }) });
    vi.stubGlobal("fetch", vi.fn(async () => image()));
    await expect(loadImageryTile(url, new AbortController().signal, 512)).resolves.toBeTruthy();
    const { cache } = cacheFixture(); cache.put.mockRejectedValue(new DOMException("full", "QuotaExceededError"));
    await expect(loadImageryTile(url, new AbortController().signal, 512)).resolves.toBeTruthy(); await settle();
  });

  it("does not persist error pages or raster data that fail native decoding", async () => {
    const { items } = cacheFixture(); vi.stubGlobal("fetch", vi.fn(async () => new Response("error", { status: 503 })));
    await expect(loadImageryTile(url, new AbortController().signal, 512)).rejects.toBeInstanceOf(ImageryRequestError);
    vi.stubGlobal("fetch", vi.fn(async () => image()));
    vi.mocked(decodeImageryTile).mockRejectedValueOnce(new Error("wrong dimensions"));
    await expect(loadImageryTile(url, new AbortController().signal, 512)).rejects.toThrow("wrong dimensions");
    await settle(); expect(items.size).toBe(0);
  });

  it("refetches corrupt cached data instead of leaving the map permanently failed", async () => {
    const { items } = cacheFixture(); items.set(url, image());
    vi.mocked(decodeImageryTile).mockRejectedValueOnce(new Error("corrupt cached image"));
    const fetcher = vi.fn(async () => image()); vi.stubGlobal("fetch", fetcher);
    await expect(loadImageryTile(url, new AbortController().signal, 512)).resolves.toBeTruthy();
    expect(fetcher).toHaveBeenCalledOnce();
  });

  it("bounds the persistent cache by both byte size and entry count", async () => {
    const { items } = cacheFixture(); TILE_CACHE.maxEntries = 2; TILE_CACHE.maxBytes = 8;
    vi.stubGlobal("fetch", vi.fn(async () => image()));
    for (let index = 0; index < 4; index++) {
      await loadImageryTile(`${url}?tile=${index}`, new AbortController().signal, 512); await settle();
    }
    await vi.waitFor(() => expect(items.size).toBe(1));
    expect([...items.keys()]).toEqual([`${url}?tile=3`]);
  });

  it("shares four network slots and prioritizes cached map coverage over queued detail", async () => {
    vi.stubGlobal("caches", undefined);
    const pending: { url: string; complete: () => void }[] = [];
    vi.stubGlobal("fetch", vi.fn((url: string) => new Promise(resolve => pending.push({ url, complete: () => resolve(image()) }))));
    const aborts = Array.from({ length: 6 }, () => new AbortController());
    const loads = aborts.map((abort, index) => loadImageryTile(`${url}?tile=${index}`, abort.signal, 512, index === 5 ? 0 : 1).catch(error => error));
    try {
      await settle(); expect(pending).toHaveLength(4);
      pending[0]!.complete(); await vi.waitFor(() => expect(pending).toHaveLength(5));
      expect(pending[4]!.url).toBe(`${url}?tile=5`);
      pending[1]!.complete(); await vi.waitFor(() => expect(pending).toHaveLength(6));
      expect(pending[5]!.url).toBe(`${url}?tile=4`);
      for (const request of pending) request.complete(); await Promise.all(loads);
    } finally {
      for (const abort of aborts) abort.abort(); for (const request of pending) request.complete();
      await Promise.all(loads);
    }
  });

  it("cancels queued requests before transport and releases an aborted network slot", async () => {
    vi.stubGlobal("caches", undefined);
    const pending: (() => void)[] = [], fetcher = vi.fn((_url: string, options: RequestInit) => new Promise((resolve, reject) => {
      options.signal!.addEventListener("abort", () => reject(new DOMException("Cancelled", "AbortError")), { once: true });
      pending.push(() => resolve(image()));
    }));
    vi.stubGlobal("fetch", fetcher);
    const aborts = Array.from({ length: 6 }, () => new AbortController());
    const loads = aborts.map(abort => loadImageryTile(url, abort.signal, 512).catch(error => error));
    try {
      await settle(); expect(fetcher).toHaveBeenCalledTimes(4);
      aborts[4]!.abort(); aborts[0]!.abort(); await settle();
      expect(fetcher).toHaveBeenCalledTimes(5);
      for (const finish of pending) finish(); const result = await Promise.all(loads);
      expect(result[4]).toBeInstanceOf(DOMException); expect(result[0]).toBeInstanceOf(DOMException);
    } finally {
      for (const abort of aborts) abort.abort(); for (const finish of pending) finish(); await Promise.all(loads);
    }
  });

  it("starts the network deadline only after acquiring a transport slot", async () => {
    vi.stubGlobal("caches", undefined);
    const pending: (() => void)[] = [];
    vi.stubGlobal("fetch", vi.fn((_url: string, options: RequestInit) => new Promise((resolve, reject) => {
      options.signal!.addEventListener("abort", () => reject(new DOMException("Cancelled", "AbortError")), { once: true });
      pending.push(() => resolve(image()));
    })));
    const aborts = Array.from({ length: 5 }, () => new AbortController()), starts = aborts.map(() => vi.fn());
    const loads = aborts.map((abort, index) => loadImageryTile(url, abort.signal, 512, 1, starts[index]).catch(error => error));
    try {
      await settle(); expect(starts[4]).not.toHaveBeenCalled();
      aborts[0]!.abort(); await settle(); expect(starts[4]).toHaveBeenCalledOnce();
    } finally {
      for (const abort of aborts) abort.abort(); for (const finish of pending) finish(); await Promise.all(loads);
    }
  });
});
