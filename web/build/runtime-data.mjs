import { stat } from 'node:fs/promises';
import { resolve } from 'node:path';
import { gzipSync } from 'node:zlib';
import { readStudyDataWithHash } from '../scripts/study-reader.mjs';
import { packRuntimeData } from '../shared/runtime-data.mjs';
import { restoreStudyData } from '../scripts/study-data.mjs';
const NAMES = ['demo-data', 'dynamic-data', 'predictive-data'];
export function runtimeDataPlugin(summarize) {
  let root; const cache = new Map();
  let summaryCache;
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
          const asset = await retained.promise;
          result = asset.summary; sha256 = asset.sha256;
        } else {
          // A dev-only Benchmark request need not build/compress all replay assets.
          const source = await readStudyDataWithHash(resolve(root, 'public', `${name}.json`));
          result = summarize(name, source.value); sha256 = source.sha256;
        }
        const city = result.citySha256.replace(/^sha256:/, '');
        if (citySha256 && city !== citySha256) throw new Error('Benchmark datasets use different city extracts');
        citySha256 = city; analyses.push(result.analysis);
        sources[name] = sha256;
      }
      return JSON.stringify({ schema: 'uav-benchmark-v1', citySha256, sources, analyses });
    })();
    summaryCache = { stamp, promise };
    try { return await promise; } catch (error) { summaryCache = undefined; throw error; }
  }
  async function asset(name) {
    const source = resolve(root, 'public', `${name}.json`), info = await stat(source), stamp = `${info.size}/${info.mtimeMs}`;
    if (cache.get(name)?.stamp === stamp) return cache.get(name).promise;
    const promise = readStudyDataWithHash(source).then(({ value, sha256 }) => {
      // Production needs both outputs. Audit/summarize while the native value is
      // already present, then release it instead of rereading ~770 MiB later.
      const packed = JSON.stringify(packRuntimeData(value));
      // Some validators normalize optional mission declarations in-place.
      // Capture the lossless record before that audit, not its narrowed schema.
      const summary = summarize?.(name, value);
      return { packed, gzip: gzipSync(packed, { level: 6 }), bytes: info.size, sha256, summary };
    });
    cache.set(name, { stamp, promise });
    try { return await promise; } catch (error) { cache.delete(name); throw error; }
  }
  return {
    name: 'lossless-runtime-data',
    async configResolved(config) { root = config.root; await restoreStudyData({ publicDir: resolve(root, 'public') }); },
    configureServer(server) {
      server.middlewares.use(async (request, response, next) => {
        if (summarize && request.url?.split('?')[0].endsWith('/benchmark-summary.json')) {
          try { response.setHeader('Content-Type', 'application/json'); response.setHeader('Cache-Control', 'no-cache'); response.end(await summary()); }
          catch (error) { next(error); }
          return;
        }
        const match = request.url?.split('?')[0].match(/\/(demo-data|dynamic-data|predictive-data)\.runtime\.json(\.gz)?$/);
        if (!match) return next();
        try {
          const result = await asset(match[1]);
          response.setHeader('Content-Type', match[2] ? 'application/gzip' : 'application/json');
          response.setHeader('Cache-Control', 'no-cache'); response.end(match[2] ? result.gzip : result.packed);
        } catch (error) { next(error); }
      });
    },
    async generateBundle() {
      for (const name of NAMES) {
        const result = await asset(name);
        this.emitFile({ type: 'asset', fileName: `${name}.runtime.json`, source: result.packed });
        this.emitFile({ type: 'asset', fileName: `${name}.runtime.json.gz`, source: result.gzip });
        this.info(`${name}: ${result.bytes} → ${result.gzip.length} bytes (lossless delivery)`);
      }
      if (summarize) this.emitFile({ type: 'asset', fileName: 'benchmark-summary.json', source: await summary() });
    },
  };
}
