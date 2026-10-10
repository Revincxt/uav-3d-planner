import { readFile } from 'node:fs/promises';
import { resolve } from 'node:path';

// Native JSON, CSV and audit manifests are offline inputs, not site downloads.
// Copy only deliberate public assets; runtime-data emits browser datasets separately.
export const PUBLIC_SITE_ASSETS = ['favicon.ico', 'favicon.svg'];

export function publicAssetsPlugin() {
  let publicDir;
  return {
    name: 'public-site-assets',
    configResolved(config) { publicDir = config.publicDir; },
    async generateBundle() {
      for (const fileName of PUBLIC_SITE_ASSETS) {
        this.emitFile({ type: 'asset', fileName, source: await readFile(resolve(publicDir, fileName)) });
      }
    },
  };
}
