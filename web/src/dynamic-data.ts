import type { DynamicBundleV1 } from "./dynamic-schema";
import { validateDynamicBundle } from "./dynamic-validation";
import { loadDataset } from "./data-loader";
import { fetchRuntimeData } from "./runtime-fetch";
export async function loadDynamicBundle(): Promise<DynamicBundleV1> {
  return loadDataset("dynamic", async () => validateDynamicBundle(await fetchRuntimeData("dynamic", import.meta.env.BASE_URL)));
}
