/** Project-scoped, best-effort FIFO cache. Never touches imagery or other sites' caches. */
export const RUNTIME_CACHE = { prefix: "uav-runtime-v2:", maxEntries: 24, maxBytes: 96 * 1024 * 1024 };
let writes: Promise<unknown> = Promise.resolve();
function name(base: string): string { return RUNTIME_CACHE.prefix + new URL(base).pathname; }
async function open(base: string): Promise<Cache | undefined> {
  try { return typeof caches === "undefined" ? undefined : await caches.open(name(base)); }
  catch { return undefined; }
}
export async function cachedRuntimeResponse(base: string, url: string): Promise<Response | undefined> {
  try { return await (await open(base))?.match(url); } catch { return undefined; }
}
export async function discardRuntimeResponse(base: string, url: string): Promise<void> {
  try { await (await open(base))?.delete(url); } catch { /* Storage is optional. */ }
}
/** Await writes before the dataset worker terminates. Locks also bound concurrent tabs. */
export async function cacheRuntimeResponse(base: string, url: string, bytes: Uint8Array<ArrayBuffer>, compress = false): Promise<void> {
  if (!compress && bytes.byteLength > RUNTIME_CACHE.maxBytes) return;
  const save = async (): Promise<void> => {
    const cache = await open(base); if (!cache) return;
    let stored = bytes;
    // Some hosts decode .gz automatically; keep those cache entries compact too.
    if (compress && typeof CompressionStream !== "undefined") {
      try {
        stored = new Uint8Array(await new Response(new Response(bytes).body!.pipeThrough(new CompressionStream("gzip"))).arrayBuffer());
      } catch { /* Raw bytes remain a valid fallback when compression is unavailable. */ }
    }
    if (stored.byteLength > RUNTIME_CACHE.maxBytes) return;
    await cache.delete(url);
    await cache.put(url, new Response(stored, { headers: {
      "Content-Type": "application/octet-stream", "Content-Length": String(stored.byteLength),
    } }));
    const keys = await cache.keys();
    const entries = await Promise.all(keys.map(async key => {
      const size = Number((await cache.match(key))?.headers.get("Content-Length"));
      return { key, size: Number.isFinite(size) && size > 0 ? size : RUNTIME_CACHE.maxBytes };
    }));
    let total = entries.reduce((sum, entry) => sum + entry.size, 0), count = entries.length;
    for (const entry of entries) {
      if (count <= RUNTIME_CACHE.maxEntries && total <= RUNTIME_CACHE.maxBytes) break;
      await cache.delete(entry.key); total -= entry.size; count--;
    }
  };
  const write = writes.catch(() => {}).then(async () => {
    if (typeof navigator !== "undefined" && navigator.locks) await navigator.locks.request(name(base), save);
    else await save();
  });
  writes = write.catch(() => {});
  await writes;
}
