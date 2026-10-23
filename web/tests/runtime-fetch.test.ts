import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { packRuntimeData, splitRuntimeCity } from "../shared/runtime-data.mjs";
import { fetchRuntimeData } from "../src/runtime-fetch";
import { RUNTIME_CACHE, cacheRuntimeResponse } from "../src/runtime-cache";
const cryptoModule = "node:crypto", zlibModule = "node:zlib";
const { createHash } = await import(cryptoModule), { gzipSync } = await import(zlibModule);
const BASE = "https://example.test/app/";
const CITY = { bounds: { min: [0, 0, 0], max: [100, 100, 100] }, buildings: [], city: { id: "city" } };
function asset(label: string, value: unknown) {
  const bytes = new TextEncoder().encode(JSON.stringify(value)), gzip = new Uint8Array(gzipSync(bytes));
  const sha256 = createHash('sha256').update(bytes).digest('hex');
  const path = `runtime/${label}.${sha256}.json`;
  return { descriptor: { path, gzipPath: `${path}.gz`, sha256, bytes: bytes.length, gzipBytes: gzip.length }, bytes, gzip };
}
function fixture(label = "first", city = CITY) {
  const original = { label, scenarios: [{ ...city, start: [1, 2, 3] }, { ...city, start: [4, 5, 6] }] };
  const split = splitRuntimeCity(packRuntimeData(original)), common = asset("city", split.city);
  const data = asset("demo-data", { runtimeVersion: 2, citySha256: common.descriptor.sha256, pool: split.pool, bundle: split.bundle });
  const manifest = { schema: "uav-runtime-v2", datasets: { static: { sourceSha256: 'a'.repeat(64), data: data.descriptor, city: common.descriptor } } };
  const resources = new Map([[data.descriptor.path, data.bytes], [data.descriptor.gzipPath, data.gzip],
    [common.descriptor.path, common.bytes], [common.descriptor.gzipPath, common.gzip]]);
  const fetcher = vi.fn<typeof fetch>(async input => {
    const url = String(input);
    if (url.includes('runtime-manifest.json')) return Response.json(manifest);
    const bytes = resources.get(new URL(url).pathname.slice('/app/'.length));
    return bytes ? new Response(bytes) : new Response(null, { status: 404 });
  });
  return { original, common, data, manifest, resources, fetcher };
}
class MemoryCache {
  entries = new Map<string, Response>();
  key(input: RequestInfo | URL): string { return input instanceof Request ? input.url : String(input); }
  async match(input: RequestInfo | URL) { return this.entries.get(this.key(input))?.clone(); }
  async put(input: RequestInfo | URL, response: Response) { this.entries.set(this.key(input), response.clone()); }
  async delete(input: RequestInfo | URL) { return this.entries.delete(this.key(input)); }
  async keys() { return [...this.entries.keys()].map(url => new Request(url)); }
}
let stores: Map<string, MemoryCache>;
beforeEach(() => {
  stores = new Map();
  vi.stubGlobal('caches', { open: vi.fn(async (name: string) => {
    if (!stores.has(name)) stores.set(name, new MemoryCache());
    return stores.get(name)!;
  }) });
});
afterEach(() => { vi.unstubAllGlobals(); });
const assetRequests = (fetcher: ReturnType<typeof fixture>['fetcher']) => fetcher.mock.calls.map(call => String(call[0])).filter(url => url.includes('/runtime/'));

describe('content-addressed, persistent runtime delivery', () => {
  it('loads and reconstructs all original fields from a shared city and real gzip', async () => {
    const f = fixture();
    expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    expect(assetRequests(f.fetcher)).toHaveLength(2);
    expect(f.fetcher).toHaveBeenCalledWith(BASE + 'runtime-manifest.json?study=static', { cache: 'no-cache' });
    expect([...stores.values()][0]!.entries.size).toBe(3);
  });
  it('revalidates only the small manifest on subsequent visits, even after module reload', async () => {
    const f = fixture(); await fetchRuntimeData('static', BASE, f.fetcher); f.fetcher.mockClear();
    vi.resetModules(); const reloaded = await import('../src/runtime-fetch');
    expect(await reloaded.fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    expect(f.fetcher).toHaveBeenCalledTimes(1);
  });
  it('downloads only a changed mission package and reuses an unchanged city', async () => {
    const first = fixture(), next = fixture('second');
    await fetchRuntimeData('static', BASE, first.fetcher);
    expect(await fetchRuntimeData('static', BASE, next.fetcher)).toEqual(next.original);
    expect(assetRequests(next.fetcher)).toEqual([BASE + next.data.descriptor.gzipPath]);
  });
  it('downloads a changed city as well rather than pairing a new route with an old city', async () => {
    const first = fixture(), next = fixture('new city', { ...CITY, city: { id: 'replacement' } });
    await fetchRuntimeData('static', BASE, first.fetcher);
    expect(await fetchRuntimeData('static', BASE, next.fetcher)).toEqual(next.original);
    expect(assetRequests(next.fetcher)).toHaveLength(2);
  });
  it('recovers corrupt cached bytes from the network with HTTP-cache bypass', async () => {
    const f = fixture(); await fetchRuntimeData('static', BASE, f.fetcher);
    const cache = [...stores.values()][0]!;
    await cache.put(BASE + f.data.descriptor.gzipPath, new Response('corrupt', { headers: { 'Content-Length': String(f.data.gzip.length) } }));
    f.fetcher.mockClear();
    expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    expect(f.fetcher).toHaveBeenCalledWith(BASE + f.data.descriptor.gzipPath, { cache: 'reload' });
    expect(assetRequests(f.fetcher)).toHaveLength(1);
  });
  it.each(['absent', 'denied', 'quota'])('works when persistent storage is %s', async mode => {
    if (mode === 'absent') vi.stubGlobal('caches', undefined);
    if (mode === 'denied') vi.stubGlobal('caches', { open: async () => { throw new Error('denied'); } });
    if (mode === 'quota') vi.stubGlobal('caches', { open: async () => ({ match: async () => undefined, delete: async () => false, put: async () => { throw new Error('quota'); } }) });
    const f = fixture(); expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
  });
  it('uses a verified known manifest and cached assets while offline', async () => {
    const f = fixture(); await fetchRuntimeData('static', BASE, f.fetcher);
    f.fetcher.mockRejectedValue(new TypeError('offline'));
    expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
  });
  it.each([403, 404, 500])('does not hide manifest HTTP %s behind a stale manifest', async status => {
    const f = fixture(); await fetchRuntimeData('static', BASE, f.fetcher);
    f.fetcher.mockResolvedValue(new Response(null, { status }));
    await expect(fetchRuntimeData('static', BASE, f.fetcher)).rejects.toThrow(String(status));
  });
  it('does not hide an invalid new manifest behind a cached version', async () => {
    const f = fixture(); await fetchRuntimeData('static', BASE, f.fetcher);
    f.manifest.schema = 'wrong'; await expect(fetchRuntimeData('static', BASE, f.fetcher)).rejects.toThrow('manifest');
  });
  it('rejects corrupt network content and never persists it', async () => {
    const f = fixture(); f.resources.set(f.data.descriptor.gzipPath, new TextEncoder().encode('x'.repeat(f.data.bytes.length)));
    await expect(fetchRuntimeData('static', BASE, f.fetcher)).rejects.toThrow('SHA-256');
    expect([...stores.values()][0]!.entries.has(BASE + f.data.descriptor.gzipPath)).toBe(false);
  });
  it('rejects unsafe asset paths before requesting them', async () => {
    const f = fixture(); f.data.descriptor.path = 'https://untrusted.example/data';
    await expect(fetchRuntimeData('static', BASE, f.fetcher)).rejects.toThrow('descriptor');
    expect(assetRequests(f.fetcher)).toHaveLength(0);
  });
  it('falls back to the same hashed JSON when gzip is absent and reuses that fallback', async () => {
    const f = fixture(); f.resources.delete(f.data.descriptor.gzipPath);
    expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    expect(assetRequests(f.fetcher)).toContain(BASE + f.data.descriptor.path);
    f.fetcher.mockClear(); await fetchRuntimeData('static', BASE, f.fetcher);
    expect(assetRequests(f.fetcher)).toHaveLength(0);
  });
  it('keeps compact JSON support without DecompressionStream', async () => {
    vi.stubGlobal('DecompressionStream', undefined);
    const f = fixture(); expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    expect(assetRequests(f.fetcher).every(url => url.endsWith('.json'))).toBe(true);
  });
  it('accepts gzip URLs transparently decompressed by a static host', async () => {
    const f = fixture(); f.resources.set(f.data.descriptor.gzipPath, f.data.bytes); f.resources.set(f.common.descriptor.gzipPath, f.common.bytes);
    expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    const response = await [...stores.values()][0]!.match(BASE + f.data.descriptor.gzipPath);
    expect([...new Uint8Array(await response!.arrayBuffer()).slice(0, 2)]).toEqual([0x1f, 0x8b]);
    f.fetcher.mockClear(); expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    expect(assetRequests(f.fetcher)).toHaveLength(0);
  });
  it('keeps raw verified cache bytes when optional recompression is unavailable', async () => {
    vi.stubGlobal('CompressionStream', undefined);
    const f = fixture(); f.resources.set(f.data.descriptor.gzipPath, f.data.bytes);
    expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    f.fetcher.mockClear(); expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
    expect(assetRequests(f.fetcher)).toHaveLength(0);
  });
  it('handles gzip signatures split across streamed network chunks', async () => {
    const f = fixture();
    f.fetcher.mockImplementation(async input => {
      if (String(input).includes('runtime-manifest.json')) return Response.json(f.manifest);
      const bytes = f.resources.get(new URL(String(input)).pathname.slice('/app/'.length))!;
      return new Response(new ReadableStream({ start(controller) {
        controller.enqueue(bytes.slice(0, 1)); controller.enqueue(bytes.slice(1, 2)); controller.enqueue(bytes.slice(2)); controller.close();
      } }));
    });
    expect(await fetchRuntimeData('static', BASE, f.fetcher)).toEqual(f.original);
  });
  it('bounds decompression and refuses to persist an oversized payload', async () => {
    const f = fixture(); f.data.descriptor.bytes = 1;
    await expect(fetchRuntimeData('static', BASE, f.fetcher)).rejects.toThrow('byte budget');
    expect([...stores.values()][0]!.entries.has(BASE + f.data.descriptor.gzipPath)).toBe(false);
  });
  it('rejects mismatched city references even when both individual asset hashes are valid', async () => {
    const f = fixture(), packed = JSON.parse(new TextDecoder().decode(f.data.bytes));
    const bad = asset('demo-data', { ...packed, citySha256: 'c'.repeat(64) });
    f.manifest.datasets.static.data = bad.descriptor; f.resources.set(bad.descriptor.gzipPath, bad.gzip);
    await expect(fetchRuntimeData('static', BASE, f.fetcher)).rejects.toThrow('city digest mismatch');
  });
  it('bounds cached entry count and never deletes another project cache', async () => {
    const other = new MemoryCache(); stores.set('another-project', other);
    await other.put(BASE + 'keep', new Response('keep'));
    for (let index = 0; index < RUNTIME_CACHE.maxEntries + 4; index++) {
      await cacheRuntimeResponse(BASE, BASE + index, new Uint8Array([index]));
    }
    expect(stores.get(RUNTIME_CACHE.prefix + '/app/')!.entries.size).toBe(RUNTIME_CACHE.maxEntries);
    expect(other.entries.size).toBe(1);
    expect(await stores.get(RUNTIME_CACHE.prefix + '/app/')!.match(BASE + '0')).toBeUndefined();
  });
  it('scopes caches to the application path', async () => {
    await cacheRuntimeResponse(BASE, BASE + 'asset', new Uint8Array([1]));
    await cacheRuntimeResponse('https://example.test/other/', 'https://example.test/other/asset', new Uint8Array([1]));
    expect(stores.size).toBe(2);
  });
  it('also bounds cached bytes, skips oversized entries and evicts the oldest first', async () => {
    const budget = RUNTIME_CACHE.maxBytes; RUNTIME_CACHE.maxBytes = 5;
    try {
      await cacheRuntimeResponse(BASE, BASE + 'old', new Uint8Array(3));
      await cacheRuntimeResponse(BASE, BASE + 'new', new Uint8Array(3));
      await cacheRuntimeResponse(BASE, BASE + 'oversized', new Uint8Array(6));
      const cache = stores.get(RUNTIME_CACHE.prefix + '/app/')!;
      expect([...cache.entries.keys()]).toEqual([BASE + 'new']);
    } finally { RUNTIME_CACHE.maxBytes = budget; }
  });
  it('serializes writes with a project-scoped cross-tab lock when supported', async () => {
    const request = vi.fn(async (_name: string, save: () => Promise<void>) => save());
    vi.stubGlobal('navigator', { locks: { request } });
    await Promise.all([cacheRuntimeResponse(BASE, BASE + 'one', new Uint8Array([1])),
      cacheRuntimeResponse(BASE, BASE + 'two', new Uint8Array([2]))]);
    expect(request.mock.calls.map(call => call[0])).toEqual([RUNTIME_CACHE.prefix + '/app/', RUNTIME_CACHE.prefix + '/app/']);
    expect([...stores.values()][0]!.entries.size).toBe(2);
  });
});
