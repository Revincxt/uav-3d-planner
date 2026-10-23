type LoadingStage = "data" | "scene" | "ready" | "error";
function stage(value: LoadingStage): void {
  window.dispatchEvent(new CustomEvent("uav:page-loading", { detail: { stage: value } }));
}
/** Yield through a paint, not just a microtask, before synchronous scene construction. */
const paint = (): Promise<void> => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(() => resolve())));

export async function prepareScene(): Promise<void> {
  stage("scene");
  await paint();
}

/** Keep the HTML shell until data, controls and the first scheduled render are ready. */
export async function runScene(start: () => Promise<void>): Promise<void> {
  try {
    stage("data");
    await paint();
    await start();
    await paint();
    stage("ready");
  } catch (error) {
    stage("error");
    throw error;
  }
}
