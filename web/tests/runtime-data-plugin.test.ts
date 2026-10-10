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
      return { value: { name, city: 'a'.repeat(64), scenarios: [] }, sha256: `sha256:${name}` };
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
    expect(emitFile).toHaveBeenCalledTimes(7);
    const assets = emitFile.mock.calls.map(call => call[0]);
    for (const name of names) {
      const compact = assets.find(asset => asset.fileName === `${name}.runtime.json`)!;
      const compressed = assets.find(asset => asset.fileName === `${name}.runtime.json.gz`)!;
      expect(gunzipSync(compressed.source).toString()).toBe(compact.source);
    }
    const summary = JSON.parse(assets.find(asset => asset.fileName === 'benchmark-summary.json')!.source);
    expect(summary.citySha256).toBe('a'.repeat(64));
    expect(summary.analyses).toEqual(names.map(name => ({ name, verified: true })));
    expect(summary.sources).toEqual(Object.fromEntries(names.map(name => [name, `sha256:${name}`])));
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

  it('preserves every native field even when the summarizer normalizes its input', async () => {
    summarize.mockImplementationOnce((name, value) => {
      const verified = value.name === name;
      value.name = 'normalized schema view';
      return { citySha256: `sha256:${value.city}`, analysis: { name, verified } };
    });
    const { handler } = await setup();
    const packed = await request(handler, '/demo-data.runtime.json');
    expect(unpackRuntimeData(JSON.parse(packed))).toEqual({
      name: 'demo-data', city: 'a'.repeat(64), scenarios: [],
    });
    expect(reader).toHaveBeenCalledOnce();
  });

  it('reuses a dev runtime asset audit for a subsequent Benchmark request', async () => {
    const { handler } = await setup();
    await request(handler, '/dynamic-data.runtime.json.gz');
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
