import type { DemoBundle } from "./schema";
import type { DynamicBundleV1 } from "./dynamic-schema";
import type { PredictiveBundleV3 } from "./predictive-schema";
import type { Analysis } from "./results-data";
export interface LoadedData { static: DemoBundle; dynamic: DynamicBundleV1; predictive: PredictiveBundleV3; results: Analysis[] }
const pending = new Map<keyof LoadedData, Promise<LoadedData[keyof LoadedData]>>();
class WorkerUnavailable extends Error {}
/** Parsing and unchanged validation run off-thread; terminate after delivery. */
export function loadInBackground<K extends keyof LoadedData>(kind: K): Promise<LoadedData[K]> | null {
  if (typeof Worker === "undefined" || typeof document === "undefined") return null;
  const cached = pending.get(kind); if (cached) return cached as Promise<LoadedData[K]>;
  let worker: Worker;
  try { worker = new Worker(new URL("./data-worker.ts", import.meta.url), { type: "module", name: `uav-${kind}-data` }); }
  catch { return null; }
  const promise = new Promise<LoadedData[K]>((resolve, reject) => {
    worker.onmessage = (event: MessageEvent<{ value?: LoadedData[K]; error?: string }>) => {
      worker.terminate();
      if (event.data.error) reject(new Error(event.data.error));
      else if (event.data.value !== undefined) resolve(event.data.value);
      else reject(new Error("Incomplete dataset worker response"));
    };
    worker.onerror = event => { worker.terminate(); reject(new WorkerUnavailable(event.message || "Dataset worker could not start")); };
    worker.onmessageerror = () => { worker.terminate(); reject(new WorkerUnavailable("Dataset worker response could not be decoded")); };
    try { worker.postMessage({ kind, base: new URL(import.meta.env.BASE_URL, document.baseURI).href }); }
    catch (error) { worker.terminate(); reject(new WorkerUnavailable(String(error))); }
  });
  pending.set(kind, promise); void promise.catch(() => { pending.delete(kind); }); return promise;
}

/** Preserve operation in browsers/hosts that disallow workers, not invalid records. */
export async function loadDataset<K extends keyof LoadedData>(kind: K, fallback: () => Promise<LoadedData[K]>): Promise<LoadedData[K]> {
  const background = loadInBackground(kind);
  if (!background) return fallback();
  try { return await background; }
  catch (error) { if (error instanceof WorkerUnavailable) return fallback(); throw error; }
}
