import type { PredictiveBundleV3 } from "./predictive-schema";
import { validatePredictiveBundle } from "./predictive-validation";
import { loadDataset } from "./data-loader";
import { fetchRuntimeData } from "./runtime-fetch";
export { validatePredictiveBundle, buildPredictiveComparisonRows, stationaryDuration, type PredictiveComparisonRow } from "./predictive-validation";
export async function loadPredictiveBundle(fetcher: typeof fetch = fetch): Promise<PredictiveBundleV3> {
  if (fetcher !== fetch) {
    const response = await fetcher(`${import.meta.env.BASE_URL}predictive-data.json`);
    if (!response.ok) throw new Error(`Could not load predictive data (${response.status})`);
    return validatePredictiveBundle(await response.json());
  }
  return loadDataset("predictive", async () => validatePredictiveBundle(await fetchRuntimeData("predictive", import.meta.env.BASE_URL)));
}
