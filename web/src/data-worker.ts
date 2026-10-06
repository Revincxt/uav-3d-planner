import { fetchRuntimeData, type DatasetKind } from "./runtime-fetch";
import { validateBundle } from "./static-validation";
import { validateDynamicBundle } from "./dynamic-validation";
import { validatePredictiveBundle } from "./predictive-validation";
import { analysesFromBundles } from "./results-data";
const validators = { static: validateBundle, dynamic: validateDynamicBundle, predictive: validatePredictiveBundle };
self.onmessage = async (event: MessageEvent<{ kind: DatasetKind | "results"; base: string }>) => {
  try {
    const { kind, base } = event.data;
    if (kind === "results") {
      // Sequential native audits bound worker memory; only tiny summaries reach the UI.
      const stat = validateBundle(await fetchRuntimeData("static", base));
      const dyn = validateDynamicBundle(await fetchRuntimeData("dynamic", base));
      const pred = validatePredictiveBundle(await fetchRuntimeData("predictive", base));
      self.postMessage({ value: analysesFromBundles(stat, dyn, pred) });
    } else {
      self.postMessage({ value: validators[kind](await fetchRuntimeData(kind, base)) });
    }
  } catch (error) { self.postMessage({ error: error instanceof Error ? error.message : "Dataset validation failed" }); }
};
