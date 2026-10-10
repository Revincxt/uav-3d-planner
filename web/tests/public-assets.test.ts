import { afterEach, describe, expect, it, vi } from 'vitest';
import { publicAssetsPlugin, PUBLIC_SITE_ASSETS } from '../build/public-assets.mjs';

const fsModule = 'node:fs/promises', osModule = 'node:os', pathModule = 'node:path';
const { mkdtemp, writeFile, rm, readFile } = await import(fsModule);
const { tmpdir } = await import(osModule), { join } = await import(pathModule);
let root: string | undefined;
afterEach(async () => { if (root) await rm(root, { recursive: true, force: true }); root = undefined; });

describe('explicit public site assets', () => {
  it('publishes icons without copying native studies, CSV or audit manifests', async () => {
    root = await mkdtemp(join(tmpdir(), 'uav-public-assets-'));
    for (const name of [...PUBLIC_SITE_ASSETS, 'dynamic-data.json', 'dynamic-records.csv', 'predictive-scenario-manifest.json', 'forgotten-data.json']) {
      await writeFile(join(root, name), name);
    }
    const plugin = publicAssetsPlugin(), emitFile = vi.fn();
    plugin.configResolved({ publicDir: root! });
    await plugin.generateBundle.call({ emitFile });
    expect(emitFile.mock.calls.map(([asset]) => asset.fileName)).toEqual(['favicon.ico', 'favicon.svg']);
    for (const [asset] of emitFile.mock.calls) expect(String(asset.source)).toBe(asset.fileName);
    const config = await readFile(new URL('../vite.config.ts', import.meta.url), 'utf8');
    expect(config).toContain('copyPublicDir: false');
  });
  it('fails the build when a required icon is missing', async () => {
    root = await mkdtemp(join(tmpdir(), 'uav-public-assets-'));
    const plugin = publicAssetsPlugin(); plugin.configResolved({ publicDir: root! });
    await expect(plugin.generateBundle.call({ emitFile: vi.fn() })).rejects.toMatchObject({ code: 'ENOENT' });
  });
  it('keeps offline map geometry independently cacheable across UI changes', async () => {
    const config = await readFile(new URL('../vite.config.ts', import.meta.url), 'utf8');
    expect(config).toContain('id.endsWith("/src/map-background-data.ts")');
    expect(config).toContain('return "map-context"');
  });
});
