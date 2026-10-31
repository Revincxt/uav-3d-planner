import { fetchRuntimeData, type DatasetKind } from "./runtime-fetch";
import { validateBundle } from "./static-validation";
import { validateDynamicBundle } from "./dynamic-validation";
import { validatePredictiveBundle } from "./predictive-validation";
const validators = { static: validateBundle, dynamic: validateDynamicBundle, predictive: validatePredictiveBundle };
self.onmessage = async (event: MessageEvent<{ kind: DatasetKind; base: string }>) => {
  try {
    const { kind, base } = event.data;
    self.postMessage({ value: validators[kind](await fetchRuntimeData(kind, base)) });
  } catch (error) { self.postMessage({ error: error instanceof Error ? error.message : "Dataset validation failed" }); }
};
