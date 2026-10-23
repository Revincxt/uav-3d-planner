import { readFile } from 'node:fs/promises';

/** Inline the tiny loading shell so it paints before any app/Three.js download. */
export function pageLoadingPlugin() {
  return {
    name: 'immediate-page-loading',
    async transformIndexHtml() {
      const [styles, bootstrap] = await Promise.all(['css', 'js'].map(extension =>
        readFile(new URL(`../shared/page-loading.${extension}`, import.meta.url), 'utf8')));
      return [
        { tag: 'style', children: styles, injectTo: 'head' },
        { tag: 'div', attrs: { id: 'page-loading', role: 'status', 'aria-live': 'polite', 'aria-atomic': 'true' },
          children: `<div class="page-loading-card">
            <div class="page-loading-heading"><span class="page-loading-spinner" aria-hidden="true"></span><span id="page-loading-label">Flight simulator</span></div>
            <strong id="page-loading-title">Opening scene</strong>
            <p id="page-loading-detail">Loading scene modules…</p>
            <div class="page-loading-track" aria-hidden="true"><i></i></div>
            <button id="page-loading-retry" type="button" hidden>Retry</button>
            <noscript>Enable JavaScript to open the simulator.</noscript>
          </div>`, injectTo: 'body-prepend' },
        { tag: 'script', children: bootstrap, injectTo: 'body-prepend' },
      ];
    },
  };
}
