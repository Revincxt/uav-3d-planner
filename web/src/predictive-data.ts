import type { PredictiveBundleV3 } from "./predictive-schema";
import { validatePredictiveBundle } from "./predictive-validation";
import { loadDataset } from "./data-loader";
import { fetchRuntimeData } from "./runtime-fetch";
export async function loadPredictiveBundle(): Promise<PredictiveBundleV3> {
  return loadDataset("predictive", async () => validatePredictiveBundle(await fetchRuntimeData("predictive", import.meta.env.BASE_URL)));
}
