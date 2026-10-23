import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { readStudyDataWithHash } from '../scripts/study-reader.mjs';
import { unpackRuntimeData } from '../shared/runtime-data.mjs';

vi.mock('../scripts/study-reader.mjs', () => ({ readStudyDataWithHash: vi.fn() }));

const filesystemModule = 'node:fs/promises', osModule = 'node:os', pathModule = 'node:path';
const pluginModule = '../build/runtime-data.mjs', compressionModule = 'node:zlib';
const { mkdtemp, mkdir, rm, writeFile, utimes } = await import(filesystemModule);
const { tmpdir } = await import(osModule), { join, basename } = await import(pathModule);
const { gunzipSync } = await import(compressionModule);
const { runtimeDataPlugin } = await import(pluginModule);
const names = ['demo-data', 'dynamic-data', 'predictive-data'];
const kinds = ['static', 'dynamic', 'predictive'];
const city = { bounds: { min: [0, 0, 0], max: [10, 10, 10] }, buildings: [], city: { id: 'test-city' } };
const native = (name: string) => ({ name, city: 'a'.repeat(64), scenarios: [{ ...city }] });

describe('runtime assets and Benchmark share a single source read', () => {
  let root: string;
  const reader = vi.mocked(readStudyDataWithHash);
  const summarize = vi.fn((name: string, value: { name: string; city: string }) => ({
    citySha256: `sha256:${value.city}`, analysis: { name, verified: value.name === name },
  }));
  beforeEach(async () => {
    reader.mockReset(); summarize.mockClear();
    root = await mkdtemp(join(tmpdir(), 'uav-runtime-plugin-'));
    await mkdir(join(root, 'public'));
    for (const name of names) await writeFile(join(root, 'public', `${name}.json`), '{}');
    reader.mockImplementation(async (path) => {
      const name = basename(String(path), '.json');
      return { value: native(name), sha256: 'b'.repeat(64) };
    });
  });
  afterEach(async () => { await rm(root, { recursive: true, force: true }); });

  async function setup() {
    const plugin = runtimeDataPlugin(summarize);
    await plugin.configResolved({ root });
    const use = vi.fn();
    plugin.configureServer({ middlewares: { use } });
    return { plugin, handler: use.mock.calls[0]![0] };
  }
  async function request(handler: Function, url: string) {
    const response = { setHeader: vi.fn(), end: vi.fn() }, next = vi.fn();
    await handler({ url }, response, next);
    expect(next).not.toHaveBeenCalled();
    expect(response.end).toHaveBeenCalledOnce();
    return response.end.mock.calls[0]![0];
  }

  it('does not reread native data to build the production Benchmark summary', async () => {
    const { plugin } = await setup(), emitFile = vi.fn();
    await plugin.generateBundle.call({ emitFile, info: vi.fn() });
    expect(reader).toHaveBeenCalledTimes(3);
    expect(summarize).toHaveBeenCalledTimes(3);
    expect(emitFile).toHaveBeenCalledTimes(10);
    const assets = emitFile.mock.calls.map(call => call[0]);
    const delivery = JSON.parse(assets.find(asset => asset.fileName === 'runtime-manifest.json')!.source);
    for (const [index, name] of names.entries()) {
      const entry = delivery.datasets[kinds[index]!];
      const compact = assets.find(asset => asset.fileName === entry.data.path)!;
      const compressed = assets.find(asset => asset.fileName === entry.data.gzipPath)!;
      expect(gunzipSync(compressed.source).toString()).toBe(compact.source);
      const common = JSON.parse(assets.find(asset => asset.fileName === entry.city.path)!.source);
      expect(unpackRuntimeData(JSON.parse(compact.source), common)).toEqual(native(name));
    }
    expect(new Set(Object.values(delivery.datasets).map((entry: any) => entry.city.path)).size).toBe(1);
    expect(assets.filter(asset => asset.fileName.startsWith('runtime/city.'))).toHaveLength(2);
    expect(assets.some(asset => /^\w+-data\.runtime\.json/.test(asset.fileName))).toBe(false);
    const summary = JSON.parse(assets.find(asset => asset.fileName === 'benchmark-summary.json')!.source);
    expect(summary.citySha256).toBe('a'.repeat(64));
    expect(summary.analyses).toEqual(names.map(name => ({ name, verified: true })));
    expect(summary.sources).toEqual(Object.fromEntries(names.map(name => [name, 'b'.repeat(64)])));
  });

  it('caches a dev summary and invalidates it when an export changes', async () => {
    const { handler } = await setup();
    const first = await request(handler, '/app/benchmark-summary.json?refresh=1');
    expect(await request(handler, '/app/benchmark-summary.json')).toBe(first);
    expect(reader).toHaveBeenCalledTimes(3);
    await utimes(join(root, 'public', 'demo-data.json'), 100, 100);
    await request(handler, '/app/benchmark-summary.json');
    expect(reader).toHaveBeenCalledTimes(6);
  });

  it('keeps addresses stable across builds and replaces only changed study assets', async () => {
    const { plugin } = await setup();
    const build = async () => {
      const emitFile = vi.fn();
      await plugin.generateBundle.call({ emitFile, info: vi.fn() });
      const assets = emitFile.mock.calls.map(call => call[0]);
      return { assets, manifest: JSON.parse(assets.find(asset => asset.fileName === 'runtime-manifest.json')!.source) };
    };
    const first = await build(), second = await build();
    expect(second.manifest).toEqual(first.manifest);
    expect(reader).toHaveBeenCalledTimes(3);
    reader.mockImplementation(async path => {
      const name = basename(String(path), '.json');
      const value = native(name);
      return { value: { ...value, revision: 'changed' }, sha256: 'c'.repeat(64) };
    });
    await utimes(join(root, 'public', 'demo-data.json'), 100, 100);
    const changed = await build();
    expect(changed.manifest.datasets.static.data.path).not.toBe(first.manifest.datasets.static.data.path);
    expect(changed.manifest.datasets.static.city).toEqual(first.manifest.datasets.static.city);
    for (const kind of ['dynamic', 'predictive']) expect(changed.manifest.datasets[kind]).toEqual(first.manifest.datasets[kind]);
    expect(changed.assets).toHaveLength(10);
    expect(changed.assets.some(asset => asset.fileName === first.manifest.datasets.static.data.path)).toBe(false);
    expect(reader).toHaveBeenCalledTimes(4);
  });

  it('rejects different shared cities before emitting a production manifest', async () => {
    const { plugin } = await setup(), emitFile = vi.fn();
    reader.mockResolvedValueOnce({ value: { ...native(names[0]!), scenarios: [{ ...city, buildings: [{ id: 'different' }] }] }, sha256: 'b'.repeat(64) });
    await expect(plugin.generateBundle.call({ emitFile, info: vi.fn() })).rejects.toThrow('Runtime studies use different cities');
    expect(emitFile).not.toHaveBeenCalled();
  });

  it('preserves every native field even when the summarizer normalizes its input', async () => {
    summarize.mockImplementationOnce((name, value) => {
      const verified = value.name === name;
      value.name = 'normalized schema view';
      return { citySha256: `sha256:${value.city}`, analysis: { name, verified } };
    });
    const { handler } = await setup();
    const delivery = JSON.parse(await request(handler, '/runtime-manifest.json?study=static'));
    const entry = delivery.datasets.static;
    const packed = await request(handler, '/' + entry.data.path);
    const common = await request(handler, '/' + entry.city.path);
    expect(unpackRuntimeData(JSON.parse(packed), JSON.parse(common))).toEqual(native('demo-data'));
    expect(reader).toHaveBeenCalledOnce();
  });

  it('reuses a dev runtime asset audit for a subsequent Benchmark request', async () => {
    const { handler } = await setup();
    await request(handler, '/runtime-manifest.json?study=dynamic');
    await request(handler, '/benchmark-summary.json');
    expect(reader).toHaveBeenCalledTimes(3);
    expect(summarize).toHaveBeenCalledTimes(3);
  });

  it('rejects mixed-city assets rather than publishing misleading comparisons', async () => {
    const { handler } = await setup();
    reader.mockResolvedValueOnce({ value: { name: names[0], city: 'b'.repeat(64) }, sha256: 'sha256:first' });
    const next = vi.fn(), response = { setHeader: vi.fn(), end: vi.fn() };
    await handler({ url: '/benchmark-summary.json' }, response, next);
    expect(next.mock.calls[0]![0]).toEqual(expect.objectContaining({ message: 'Benchmark datasets use different city extracts' }));
    expect(response.end).not.toHaveBeenCalled();
    // Failed summaries are not retained, so a repaired export can be retried.
    expect(JSON.parse(await request(handler, '/benchmark-summary.json')).analyses).toHaveLength(3);
  });
});
