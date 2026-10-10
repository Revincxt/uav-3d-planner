import { unpackRuntimeData } from "../shared/runtime-data.mjs";
export type DatasetKind = "static" | "dynamic" | "predictive";
const names: Record<DatasetKind, string> = { static: "demo-data", dynamic: "dynamic-data", predictive: "predictive-data" };
async function compressedJSON(response: Response): Promise<unknown> {
  if (!response.body) return response.json();
  // Hosts can serve raw gzip or transparently decode it via Content-Encoding.
  // Peek/replay two bytes, retaining streaming and avoiding a second decompression.
  const reader = response.body.getReader(), chunks: Uint8Array<ArrayBuffer>[] = [], magic: number[] = [];
  let ended = false;
  while (magic.length < 2 && !ended) {
    const { value, done } = await reader.read(); ended = done;
    if (value) { chunks.push(value); for (const byte of value) { magic.push(byte); if (magic.length === 2) break; } }
  }
  const stream = new ReadableStream<Uint8Array<ArrayBuffer>>({
    start(controller) { for (const chunk of chunks) controller.enqueue(chunk); if (ended) controller.close(); },
    async pull(controller) { const { value, done } = await reader.read(); if (done) controller.close(); else controller.enqueue(value); },
    cancel(reason) { return reader.cancel(reason); },
  });
  return new Response(magic[0] === 0x1f && magic[1] === 0x8b
    ? stream.pipeThrough(new DecompressionStream("gzip")) : stream).json();
}
/** Only absent gzip falls back to compact JSON; native audit records stay offline. */
export async function fetchRuntimeData(kind: DatasetKind, base: string, fetcher: typeof fetch = fetch): Promise<unknown> {
  const name = names[kind], compressed = typeof DecompressionStream !== "undefined";
  let response = await fetcher(`${base}${name}.runtime.json${compressed ? ".gz" : ""}`);
  if (compressed && response.status === 404) response = await fetcher(`${base}${name}.runtime.json`);
  if (!response.ok) throw new Error(`Could not load ${kind} data (${response.status})`);
  return unpackRuntimeData(await (compressed ? compressedJSON(response) : response.json()));
}
