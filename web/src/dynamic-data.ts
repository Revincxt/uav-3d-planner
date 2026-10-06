import type { DynamicBundleV1 } from "./dynamic-schema";
import { validateDynamicBundle } from "./dynamic-validation";
import { loadDataset } from "./data-loader";
import { fetchRuntimeData } from "./runtime-fetch";
export { validateDynamicBundle, buildDynamicComparisonRows, type DynamicComparisonRow } from "./dynamic-validation";
export async function loadDynamicBundle(fetcher: typeof fetch = fetch): Promise<DynamicBundleV1> {
  if (fetcher !== fetch) {
    const response = await fetcher(`${import.meta.env.BASE_URL}dynamic-data.json`);
    if (!response.ok) throw new Error(`Could not load dynamic data (${response.status})`);
    return validateDynamicBundle(await response.json());
  }
  return loadDataset("dynamic", async () => validateDynamicBundle(await fetchRuntimeData("dynamic", import.meta.env.BASE_URL)));
}
