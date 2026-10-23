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
          children: `<div class="page-loading-field" aria-hidden="true"></div>
          <div class="page-loading-content">
            <div class="page-loading-emblem" aria-hidden="true">
              <svg class="page-loading-flight" viewBox="0 0 200 200" fill="none" focusable="false">
                <g class="page-loading-reference" stroke="currentColor">
                  <circle cx="100" cy="100" r="79" stroke-dasharray="1 10.28" />
                  <path d="M100 9v10m0 162v10M9 100h10m162 0h10M42 42l6 6m104 104 6 6M42 158l6-6m104-104 6-6" />
                  <path d="M31 80a72 72 0 0 1 49-49m40 0a72 72 0 0 1 49 49m0 40a72 72 0 0 1-49 49m-40 0a72 72 0 0 1-49-49" />
                </g>
                <g class="page-loading-course" stroke="currentColor" stroke-linecap="round">
                  <path d="M100 13a87 87 0 0 1 75.35 43.5" stroke-width="1.5" />
                  <path d="M100 187a87 87 0 0 1-75.35-43.5" stroke-width="1.5" opacity=".35" />
                  <circle cx="175.35" cy="56.5" r="2.5" fill="currentColor" stroke="none" />
                </g>
                <g class="page-loading-drone" transform="translate(68 68)">
                  <path d="m21 21 22 22m0-22L21 43" stroke="currentColor" stroke-width="3" stroke-linecap="round" />
                  <g stroke="currentColor" stroke-width="1.5"><circle cx="19" cy="19" r="10" /><circle cx="45" cy="19" r="10" /><circle cx="19" cy="45" r="10" /><circle cx="45" cy="45" r="10" /></g>
                  <path d="M11 19h16m10 0h16M11 45h16m10 0h16" stroke="currentColor" stroke-linecap="round" opacity=".5" />
                  <path class="page-loading-heading" d="m32 19 9 21-9-4-9 4Z" fill="currentColor" />
                </g>
                <path class="page-loading-scan" d="M61 72h78" stroke="currentColor" stroke-linecap="round" />
              </svg>
              <svg class="page-loading-warning" viewBox="0 0 48 48" fill="none" focusable="false"><path d="M24 7 43 39H5L24 7Z" stroke="currentColor" stroke-width="1.5" stroke-linejoin="round" /><path d="M24 19v9m0 5v1" stroke="currentColor" stroke-width="2" stroke-linecap="round" /></svg>
            </div>
            <div class="page-loading-copy">
              <strong id="page-loading-title">Loading…</strong>
              <p id="page-loading-detail" hidden></p>
              <button id="page-loading-retry" type="button" hidden>Retry</button>
              <noscript>Enable JavaScript to open the simulator.</noscript>
            </div>
          </div>`, injectTo: 'body-prepend' },
        { tag: 'script', children: bootstrap, injectTo: 'body-prepend' },
      ];
    },
  };
}
