import { decodeImageryTile } from "./map-imagery";
import { MAP_SURROUND } from "./map-surround-data";

export const TILE_CACHE = { name: "uav-public-ortho-v1", maxEntries: 96, maxBytes: 24 * 1024 * 1024 };
// One transport budget for preview and detail layers combined, not four per layer.
let active = 0;
const queue: { priority: number; signal: AbortSignal; run: () => void; cancel: () => void }[] = [];
function drain(): void {
  queue.sort((a, b) => a.priority - b.priority);
  while (active < MAP_SURROUND.maxConcurrentRequests && queue.length) {
    const next = queue.shift()!;
    next.signal.removeEventListener("abort", next.cancel);
    if (!next.signal.aborted) next.run();
  }
}
function transport<T>(signal: AbortSignal, priority: number, action: () => Promise<T>): Promise<T> {
  return new Promise((resolve, reject) => {
    const cancelled = () => reject(new DOMException("Map request cancelled", "AbortError"));
    if (signal.aborted) { cancelled(); return; }
    const task = { priority, signal, run: () => {
      active++;
      let released = false;
      const release = () => {
        if (released) return;
        released = true; active--; drain();
      };
      // Keep the slot until fetch/body cancellation settles, rather than opening
      // a second batch while the browser is still terminating the previous one.
      void action().then(resolve, reject).finally(release);
    }, cancel: () => {
      const index = queue.indexOf(task); if (index >= 0) queue.splice(index, 1);
      cancelled();
    } };
    queue.push(task); signal.addEventListener("abort", task.cancel, { once: true }); drain();
  });
}

export class ImageryRequestError extends Error {
  constructor(readonly status: number) { super(`Map request failed (${status})`); }
}
let writes = Promise.resolve();
let storage: CacheStorage | undefined, opening: Promise<Cache | undefined> | undefined;
const inventories = new WeakMap<Cache, Promise<Map<string, number>>>();
function openCache(): Promise<Cache | undefined> {
  if (typeof caches === "undefined") return Promise.resolve(undefined);
  if (storage !== caches || !opening) {
    storage = caches; opening = caches.open(TILE_CACHE.name).catch(() => undefined);
  }
  return opening;
}
function inventory(cache: Cache): Promise<Map<string, number>> {
  let result = inventories.get(cache);
  if (!result) {
    result = cache.keys().then(keys => Promise.all(keys.map(async key => {
      const size = Number((await cache.match(key))?.headers.get("Content-Length"));
      return [key.url, Number.isFinite(size) && size > 0 ? size : TILE_CACHE.maxBytes] as const;
    }))).then(entries => new Map(entries));
    inventories.set(cache, result);
  }
  return result;
}
function store(cache: Cache, url: string, blob: Blob): void {
  // Cache/storage failure must never prevent a valid tile from reaching the map.
  writes = writes.catch(() => {}).then(async () => {
    const entries = await inventory(cache);
    await cache.delete(url);
    await cache.put(url, new Response(blob, { headers: {
      "Content-Type": blob.type, "Content-Length": String(blob.size),
    } }));
    entries.delete(url); entries.set(url, blob.size);
    let bytes = 0;
    for (const size of entries.values()) bytes += size;
    for (const [key, size] of entries) {
      if (entries.size <= TILE_CACHE.maxEntries && bytes <= TILE_CACHE.maxBytes) break;
      await cache.delete(key); entries.delete(key); bytes -= size;
    }
  }).catch(() => {});
}

/** Only validated real raster imagery is persisted; storage is optional and bounded. */
export async function loadImageryTile(
  url: string, signal: AbortSignal, tileSize: number, priority = 1, onNetworkStart?: () => void,
): Promise<HTMLImageElement> {
  let cache: Cache | undefined;
  try {
    cache = await openCache();
    if (cache) {
      const saved = await cache.match(url);
      if (saved) {
        const image = await decodeImageryTile(await saved.blob(), tileSize);
        signal.throwIfAborted(); return image;
      }
    }
  } catch { /* Disabled/quota-limited/corrupt storage falls back to normal CORS. */ }
  signal.throwIfAborted();
  const blob = await transport(signal, priority, async () => {
    onNetworkStart?.();
    const response = await fetch(url, { signal, credentials: "omit" });
    if (!response.ok) {
      await response.body?.cancel();
      throw new ImageryRequestError(response.status);
    }
    return response.blob();
  });
  signal.throwIfAborted();
  const image = await decodeImageryTile(blob, tileSize);
  signal.throwIfAborted();
  if (cache) store(cache, url, blob);
  return image;
}
