import { unpackRuntimeData } from "../shared/runtime-data.mjs";
import { cachedRuntimeResponse, cacheRuntimeResponse, discardRuntimeResponse } from "./runtime-cache";

export type DatasetKind = "static" | "dynamic" | "predictive";
interface Asset { path: string; gzipPath: string; sha256: string; bytes: number; gzipBytes: number }
interface Dataset { sourceSha256: string; data: Asset; city: Asset }
interface Manifest { schema: "uav-runtime-v2"; datasets: Partial<Record<DatasetKind, Dataset>> }
const names = { static: "demo-data", dynamic: "dynamic-data", predictive: "predictive-data" };
const HASH = /^[a-f\d]{64}$/;
const MAX_BYTES = 128 * 1024 * 1024;

function descriptor(value: unknown, label: string): Asset {
  const asset = value as Asset;
  if (!asset || !HASH.test(asset.sha256) || asset.path !== `runtime/${label}.${asset.sha256}.json`
    || asset.gzipPath !== `${asset.path}.gz` || !Number.isInteger(asset.bytes) || asset.bytes < 1 || asset.bytes > MAX_BYTES
    || !Number.isInteger(asset.gzipBytes) || asset.gzipBytes < 1 || asset.gzipBytes > MAX_BYTES) throw new Error("Invalid runtime asset descriptor");
  return asset;
}
function manifest(value: unknown, kind: DatasetKind): Manifest {
  const data = value as Manifest;
  if (!data || data.schema !== "uav-runtime-v2" || !data.datasets || !data.datasets[kind]
    || Object.keys(data.datasets).some(key => !Object.hasOwn(names, key))) throw new Error("Invalid runtime manifest");
  for (const [key, entry] of Object.entries(data.datasets)) {
    if (!entry || !HASH.test(entry.sourceSha256)) throw new Error("Invalid runtime source digest");
    descriptor(entry.data, names[key as DatasetKind]); descriptor(entry.city, "city");
  }
  return data;
}
async function readBytes(response: Response, limit: number): Promise<Uint8Array<ArrayBuffer>> {
  if (Number(response.headers.get("Content-Length")) > limit) throw new Error("Runtime asset exceeds byte budget");
  if (!response.body) return new Uint8Array(await response.arrayBuffer());
  const reader = response.body.getReader(), chunks: Uint8Array<ArrayBuffer>[] = [];
  let size = 0;
  try {
    for (;;) {
      const { value, done } = await reader.read(); if (done) break;
      size += value.byteLength;
      if (size > limit) throw new Error("Runtime asset exceeds byte budget");
      chunks.push(value);
    }
  } catch (error) { await reader.cancel().catch(() => {}); throw error; }
  return new Uint8Array(await new Blob(chunks).arrayBuffer());
}
async function verifiedJSON(bytes: Uint8Array<ArrayBuffer>, asset: Asset): Promise<unknown> {
  // Support both raw gzip and hosts which transparently decode Content-Encoding.
  const decoded = bytes[0] === 0x1f && bytes[1] === 0x8b
    ? await readBytes(new Response(new Response(bytes).body!.pipeThrough(new DecompressionStream("gzip"))), asset.bytes)
    : bytes;
  if (decoded.byteLength !== asset.bytes) throw new Error("Runtime asset length mismatch");
  const digest = await crypto.subtle.digest("SHA-256", decoded);
  const hash = Array.from(new Uint8Array(digest), byte => byte.toString(16).padStart(2, "0")).join("");
  if (hash !== asset.sha256) throw new Error("Runtime asset SHA-256 mismatch");
  return JSON.parse(new TextDecoder().decode(decoded));
}
async function loadAsset(asset: Asset, base: string, fetcher: typeof fetch): Promise<unknown> {
  const paths = typeof DecompressionStream === "undefined" ? [asset.path] : [asset.gzipPath, asset.path];
  let damaged = false;
  for (const path of paths) {
    const url = new URL(path, base).href, saved = await cachedRuntimeResponse(base, url);
    if (!saved) continue;
    try { return await verifiedJSON(await readBytes(saved, Math.max(asset.bytes, asset.gzipBytes)), asset); }
    catch { damaged = true; await discardRuntimeResponse(base, url); }
  }
  for (const [index, path] of paths.entries()) {
    const url = new URL(path, base).href;
    const response = damaged ? await fetcher(url, { cache: "reload" }) : await fetcher(url);
    if (response.status === 404 && index + 1 < paths.length) continue;
    if (!response.ok) throw new Error(`Runtime data request failed (${response.status})`);
    const bytes = await readBytes(response, Math.max(asset.bytes, asset.gzipBytes));
    const value = await verifiedJSON(bytes, asset);
    await cacheRuntimeResponse(base, url, bytes, path === asset.gzipPath && !(bytes[0] === 0x1f && bytes[1] === 0x8b));
    return value;
  }
  throw new Error("Runtime asset unavailable");
}
async function loadManifest(kind: DatasetKind, base: string, fetcher: typeof fetch): Promise<Manifest> {
  // The query enables lazy dev builds; static Pages serves the same complete file.
  const url = new URL(`runtime-manifest.json?study=${kind}`, base).href;
  let response: Response;
  try { response = await fetcher(url, { cache: "no-cache" }); }
  catch (error) {
    const saved = await cachedRuntimeResponse(base, url);
    if (!saved) throw error;
    return manifest(JSON.parse(new TextDecoder().decode(await readBytes(saved, 64 * 1024))), kind);
  }
  // Network failures can use a known manifest; HTTP/schema failures cannot hide behind old records.
  if (!response.ok) throw new Error(`Runtime manifest request failed (${response.status})`);
  const bytes = await readBytes(response, 64 * 1024);
  const parsed = manifest(JSON.parse(new TextDecoder().decode(bytes)), kind);
  await cacheRuntimeResponse(base, url, bytes);
  return parsed;
}
/** Fetch only a small fresh manifest; reuse verified, content-addressed compressed assets. */
export async function fetchRuntimeData(kind: DatasetKind, base: string, fetcher: typeof fetch = fetch): Promise<unknown> {
  const origin = typeof document !== "undefined" ? document.baseURI : typeof location !== "undefined" ? location.href : "http://localhost/";
  const directory = new URL(base, origin).href;
  const entry = (await loadManifest(kind, directory, fetcher)).datasets[kind]!;
  const [value, city] = await Promise.all([loadAsset(entry.data, directory, fetcher), loadAsset(entry.city, directory, fetcher)]);
  const data = value as { runtimeVersion?: number; citySha256?: string };
  if (!data || data.runtimeVersion !== 2 || data.citySha256 !== entry.city.sha256) throw new Error("Runtime city digest mismatch");
  return unpackRuntimeData(value, city);
}
