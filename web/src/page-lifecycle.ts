interface PageLifecycle { pause(): void; restore(): void; dispose(): void }
/** A persisted pagehide belongs to BFCache, not destruction of the renderer. */
export function mountPageLifecycle(actions: PageLifecycle, host: Window = window): () => void {
  const hide = (event: PageTransitionEvent): void => { actions.pause(); if (!event.persisted) actions.dispose(); };
  const show = (event: PageTransitionEvent): void => { if (event.persisted) actions.restore(); };
  host.addEventListener("pagehide", hide); host.addEventListener("pageshow", show);
  return () => { host.removeEventListener("pagehide", hide); host.removeEventListener("pageshow", show); };
}
