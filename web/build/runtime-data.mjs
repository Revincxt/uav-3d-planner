import { stat } from 'node:fs/promises';
import { resolve } from 'node:path';
import { gzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import { readStudyDataWithHash } from '../scripts/study-reader.mjs';
import { packRuntimeData, splitRuntimeCity } from '../shared/runtime-data.mjs';
import { restoreStudyData } from '../scripts/study-data.mjs';
const NAMES = ['demo-data', 'dynamic-data', 'predictive-data'];
const KINDS = ['static', 'dynamic', 'predictive'];

/** Stable content addresses survive UI-only deployments. Native records stay offline. */
export function runtimeDataPlugin(summarize) {
  let root;
  const cache = new Map(), resources = new Map();
  let summaryCache;
  function encode(label, value) {
    const json = JSON.stringify(value), sha256 = createHash('sha256').update(json).digest('hex');
    const path = `runtime/${label}.${sha256}.json`, gzipPath = `${path}.gz`;
    const gzip = resources.get(gzipPath)?.source ?? gzipSync(json, { level: 6 });
    const descriptor = { path, gzipPath, sha256, bytes: Buffer.byteLength(json), gzipBytes: gzip.length };
    if (!resources.has(path)) {
      resources.set(path, { source: json, type: 'application/json' });
      resources.set(gzipPath, { source: gzip, type: 'application/gzip' });
    }
    return descriptor;
  }
  async function asset(name) {
    const source = resolve(root, 'public', `${name}.json`), info = await stat(source), stamp = `${info.size}/${info.mtimeMs}`;
    if (cache.get(name)?.stamp === stamp) return cache.get(name).promise;
    const promise = readStudyDataWithHash(source).then(({ value, sha256 }) => {
      const split = splitRuntimeCity(packRuntimeData(value));
      const city = encode('city', split.city);
      const data = encode(name, { runtimeVersion: 2, citySha256: city.sha256, pool: split.pool, bundle: split.bundle });
      // Summarizers may normalize their input. Freeze the complete delivery first.
      const summary = summarize?.(name, value);
      return { descriptor: { sourceSha256: sha256, data, city }, summary, bytes: info.size };
    });
    cache.set(name, { stamp, promise });
    try { return await promise; } catch (error) { cache.delete(name); throw error; }
  }
  async function manifest(kind) {
    const datasets = {};
    for (const [index, name] of NAMES.entries()) {
      if (kind && kind !== KINDS[index]) continue;
      datasets[KINDS[index]] = (await asset(name)).descriptor;
    }
    if (new Set(Object.values(datasets).map(entry => entry.city.sha256)).size > 1) throw new Error('Runtime studies use different cities');
    return JSON.stringify({ schema: 'uav-runtime-v2', datasets });
  }
  async function summary() {
    const stamps = await Promise.all(NAMES.map(async name => { const info = await stat(resolve(root, 'public', `${name}.json`)); return `${info.size}/${info.mtimeMs}`; }));
    const stamp = stamps.join('|');
    if (summaryCache?.stamp === stamp) return summaryCache.promise;
    const promise = (async () => {
      const analyses = [], sources = {}; let citySha256;
      for (const [index, name] of NAMES.entries()) {
        const retained = cache.get(name);
        let result, sha256;
        if (retained?.stamp === stamps[index]) {
          const built = await retained.promise;
          result = built.summary; sha256 = built.descriptor.sourceSha256;
        } else {
          const source = await readStudyDataWithHash(resolve(root, 'public', `${name}.json`));
          result = summarize(name, source.value); sha256 = source.sha256;
        }
        const city = result.citySha256.replace(/^sha256:/, '');
        if (citySha256 && city !== citySha256) throw new Error('Benchmark datasets use different city extracts');
        citySha256 = city; analyses.push(result.analysis); sources[name] = sha256;
      }
      return JSON.stringify({ schema: 'uav-benchmark-v1', citySha256, sources, analyses });
    })();
    summaryCache = { stamp, promise };
    try { return await promise; } catch (error) { summaryCache = undefined; throw error; }
  }
  return {
    name: 'lossless-runtime-data',
    async configResolved(config) { root = config.root; await restoreStudyData({ publicDir: resolve(root, 'public') }); },
    configureServer(server) {
      server.middlewares.use(async (request, response, next) => {
        const url = new URL(request.url, 'http://localhost');
        try {
          if (summarize && url.pathname.endsWith('/benchmark-summary.json')) {
            response.setHeader('Content-Type', 'application/json'); response.setHeader('Cache-Control', 'no-cache'); response.end(await summary()); return;
          }
          if (url.pathname.endsWith('/runtime-manifest.json')) {
            const kind = url.searchParams.get('study');
            if (kind && !KINDS.includes(kind)) throw new Error('Unsupported runtime study');
            response.setHeader('Content-Type', 'application/json'); response.setHeader('Cache-Control', 'no-cache'); response.end(await manifest(kind)); return;
          }
          const match = url.pathname.match(/\/(runtime\/(city|demo-data|dynamic-data|predictive-data)\.[a-f\d]{64}\.json(?:\.gz)?)$/);
          if (!match) return next();
          if (match[2] !== 'city') await asset(match[2]);
          const resource = resources.get(match[1]);
          if (!resource) { response.statusCode = 404; response.end(); return; }
          response.setHeader('Content-Type', resource.type);
          response.setHeader('Cache-Control', 'public, max-age=31536000, immutable'); response.end(resource.source);
        } catch (error) { next(error); }
      });
    },
    async generateBundle() {
      const delivery = await manifest();
      const active = new Set(Object.values(JSON.parse(delivery).datasets)
        .flatMap(({ data, city }) => [data.path, data.gzipPath, city.path, city.gzipPath]));
      for (const fileName of active) this.emitFile({ type: 'asset', fileName, source: resources.get(fileName).source });
      this.emitFile({ type: 'asset', fileName: 'runtime-manifest.json', source: delivery });
      if (summarize) this.emitFile({ type: 'asset', fileName: 'benchmark-summary.json', source: await summary() });
      for (const name of NAMES) {
        const result = await asset(name);
        this.info(`${name}: ${result.bytes} → ${result.descriptor.data.gzipBytes} bytes + shared city (lossless delivery)`);
      }
    },
  };
}
