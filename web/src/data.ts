import type { DemoBundle } from "./schema";
import { validateBundle } from "./static-validation";
import { loadDataset } from "./data-loader";
import { fetchRuntimeData } from "./runtime-fetch";
export { validateBundle } from "./static-validation";
export async function loadDemoBundle(): Promise<DemoBundle> {
  return loadDataset("static", async () => validateBundle(await fetchRuntimeData("static", import.meta.env.BASE_URL)));
}
