/** Lossless delivery; native records and validation contracts stay unchanged. */
const WORLD = ['bounds', 'buildings', 'city', 'constraints', 'noFlyZones', 'staticNoFlyZones', 'temporaryNoFlyZones', 'movingSpheres'];
const TIMED = ['rawTimedPath', 'geometryTimedPath', 'executionTimedPath', 'geometryFrames', 'executionFrames'];
const samePoint = (a, b) => Array.isArray(a) && Array.isArray(b) && a.length === b.length && a.every((value, i) => value === b[i]);
const decodedBundles = new WeakSet();
export const isRuntimeData = value => Boolean(value && typeof value === 'object' && decodedBundles.has(value));
export function packRuntimeData(bundle) {
  const pool = [], indices = new Map();
  const intern = value => {
    const key = JSON.stringify(value); let index = indices.get(key);
    if (index === undefined) { index = pool.length; indices.set(key, index); pool.push(value); }
    return { $runtimeRef: index };
  };
  const scenarios = bundle.scenarios.map(scenario => {
    const packed = { ...scenario };
    for (const field of WORLD) if (scenario[field] !== undefined) packed[field] = intern(scenario[field]);
    if (scenario.runs) packed.runs = scenario.runs.map(run => {
      const result = { ...run };
      for (const field of TIMED) if (Array.isArray(run[field])) result[field] = intern(run[field]);
      if (run.frames) {
        const final = run.frames.at(-1)?.executedPath;
        const ref = Array.isArray(final) ? intern(final).$runtimeRef : undefined;
        result.frames = run.frames.map(frame => {
          const encoded = { ...frame };
          for (const field of ['path', 'movingSpheres']) if (Array.isArray(frame[field])) encoded[field] = intern(frame[field]);
          if (Array.isArray(frame.executedPath)) encoded.executedPath = ref !== undefined && frame.executedPath.length <= final.length && frame.executedPath.every((point, i) => samePoint(point, final[i]))
            ? { $runtimeRef: ref, prefix: frame.executedPath.length } : intern(frame.executedPath);
          return encoded;
        });
      }
      return result;
    });
    return packed;
  });
  return { runtimeVersion: 1, pool, bundle: { ...bundle, scenarios } };
}
export function unpackRuntimeData(value) {
  if (!value || value.runtimeVersion !== 1 || !Array.isArray(value.pool) || !value.bundle || typeof value.bundle !== 'object') throw new Error('Unsupported runtime delivery format');
  const decode = item => {
    if (!item || typeof item !== 'object') return item;
    if (Object.hasOwn(item, '$runtimeRef')) {
      const index = item.$runtimeRef;
      if (!Number.isInteger(index) || index < 0 || index >= value.pool.length || Object.keys(item).some(key => key !== '$runtimeRef' && key !== 'prefix')) throw new Error('Invalid runtime data reference');
      const entry = value.pool[index];
      if (!Object.hasOwn(item, 'prefix')) return entry;
      if (!Array.isArray(entry) || !Number.isInteger(item.prefix) || item.prefix < 0 || item.prefix > entry.length) throw new Error('Invalid runtime path prefix');
      return entry.slice(0, item.prefix);
    }
    if (Array.isArray(item)) return item.map(decode);
    return Object.fromEntries(Object.entries(item).map(([key, entry]) => [key, decode(entry)]));
  };
  const decoded = decode(value.bundle);
  decodedBundles.add(decoded);
  return decoded;
}
