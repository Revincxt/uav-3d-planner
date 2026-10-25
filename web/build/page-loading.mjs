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
          children: `<div class="page-loading-field" aria-hidden="true">
            <svg class="page-loading-ground" viewBox="0 0 1440 460" preserveAspectRatio="none" fill="none" focusable="false">
              <defs>
                <linearGradient id="loading-grid-clarity" x1="0" y1="0" x2="0" y2="1"><stop stop-color="white" stop-opacity="0" /><stop offset=".18" stop-color="white" stop-opacity="0" /><stop offset=".38" stop-color="white" stop-opacity=".3" /><stop offset=".7" stop-color="white" stop-opacity=".85" /><stop offset="1" stop-color="white" /></linearGradient>
                <linearGradient id="loading-grid-distance" x1="0" y1="0" x2="0" y2="1"><stop stop-color="white" stop-opacity="0" /><stop offset=".16" stop-color="white" stop-opacity=".22" /><stop offset=".3" stop-color="white" stop-opacity=".5" /><stop offset=".56" stop-color="white" stop-opacity="0" /><stop offset="1" stop-color="white" stop-opacity="0" /></linearGradient>
                <mask id="loading-grid-near" maskUnits="userSpaceOnUse" x="0" y="0" width="1440" height="460"><path d="M0 0h1440v460H0Z" fill="url(#loading-grid-clarity)" /></mask>
                <mask id="loading-grid-far" maskUnits="userSpaceOnUse" x="0" y="0" width="1440" height="460"><path d="M0 0h1440v460H0Z" fill="url(#loading-grid-distance)" /></mask>
                <filter id="loading-grid-haze" filterUnits="userSpaceOnUse" x="0" y="0" width="1440" height="300"><feGaussianBlur stdDeviation="2.8" /></filter>
                <g id="loading-grid-lines">
                  ${[false, true].map(major => `<path stroke="${major ? '#a8bdca' : '#afc6d4'}" stroke-opacity="${major ? '.42' : '.2'}" stroke-width="${major ? '1.5' : '1'}" vector-effect="non-scaling-stroke" d="${Array.from({ length: 49 }, (_, index) => ({ x: -1440 + index * 90, major: index % 4 === 0 })).filter(line => line.major === major).map(({ x }) => `M720 0 ${x} 460`).join(' ')} ${[6, 12, 20, 31, 45, 62, 84, 111, 144, 184, 232, 290, 360, 444, 460].filter((_, index) => (index % 4 === 0) === major).map(y => `M0 ${y}h1440`).join(' ')}" />`).join('')}
                </g>
              </defs>
              <use href="#loading-grid-lines" mask="url(#loading-grid-far)" filter="url(#loading-grid-haze)" opacity=".7" />
              <use href="#loading-grid-lines" mask="url(#loading-grid-near)" />
            </svg>
          </div>
          <div class="page-loading-content">
            <div class="page-loading-emblem" aria-hidden="true">
              <svg class="page-loading-flight" viewBox="0 0 200 200" fill="none" focusable="false">
                <defs>
                  <linearGradient id="loading-airframe" x1="0" y1="0" x2="1" y2="1"><stop stop-color="#b8cbd6" /><stop offset=".5" stop-color="#98b1c0" /><stop offset="1" stop-color="#6e8a9e" /></linearGradient>
                  <linearGradient id="loading-motor" x1="0" y1="0" x2="1" y2="0"><stop stop-color="#3a5264" /><stop offset=".45" stop-color="#6d8799" /><stop offset="1" stop-color="#476377" /></linearGradient>
                  <radialGradient id="loading-ground-shadow"><stop stop-color="#203646" stop-opacity=".4" /><stop offset="1" stop-color="#203646" stop-opacity="0" /></radialGradient>
                </defs>
                <g stroke-width=".8">
                  <ellipse cx="100" cy="174" rx="61" ry="19" fill="#344b5b" fill-opacity=".4" stroke="#7c98ac" stroke-opacity=".4" />
                  <ellipse cx="100" cy="170" rx="61" ry="19" fill="#405868" fill-opacity=".5" stroke="#9bb3c2" stroke-opacity=".6" />
                  <ellipse cx="100" cy="170" rx="43" ry="13" stroke="#9fc0d3" stroke-opacity=".5" stroke-dasharray="6 4" />
                  <ellipse cx="100" cy="170" rx="42" ry="13" fill="url(#loading-ground-shadow)" stroke="none" />
                  <path d="M100 145v10m0 30v10M32 170h15m106 0h15" stroke="#acc3d1" stroke-opacity=".6" />
                </g>
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
                <g class="page-loading-drone">
                  <!-- Horizontal surfaces share an 18° camera elevation and 22° aircraft heading. -->
                  <path d="m94.5 103.8-4.6 25.3m26.8-28.1 4.7 24.2m-39-1.8 13.1 10m18.4-14 13.1 10" stroke="#3b5364" stroke-width="4" stroke-linecap="round" />
                  <path d="m94.5 103.8-4.6 25.3m26.8-28.1 4.7 24.2" stroke="#819cac" stroke-width="1.2" stroke-linecap="round" />
                  <path transform="matrix(.927 -.116 .375 .286 100 102)" d="m-46-46 92 92m0-92-92 92" stroke="#3c5669" stroke-width="9" stroke-linecap="round" />
                  <path transform="matrix(.927 -.116 .375 .286 100 97.5)" d="m-46-46 92 92m0-92-92 92" stroke="#849eaf" stroke-width="7" stroke-linecap="round" />
                  ${[[46, -46], [-46, -46], [46, 46], [-46, 46]].map(([x, y]) => `<g transform="translate(${(100 + .927 * x + .375 * y).toFixed(2)} ${(100 - .116 * x + .286 * y).toFixed(2)})">
                    <path d="M-5.5-4v7a5.5 1.8 0 0 0 11 0v-7" fill="url(#loading-motor)" />
                    <ellipse cy="-4" rx="5.5" ry="1.8" fill="#91acbd" />
                    <g transform="matrix(.927 -.116 .375 .286 0 -5)">
                      <circle r="20" fill="#a3bece" fill-opacity=".04" stroke="#a1bdce" stroke-opacity=".22" stroke-width=".7" vector-effect="non-scaling-stroke" />
                      <g class="page-loading-propeller"><path d="M-2 1c-5 3-13 3-18 1l1-3c7-2 14-1 21 0 5-3 13-3 18-1l-1 3c-7 2-14 1-21 0Z" fill="#a6bdcb" /></g>
                      <circle r="3" fill="#526f84" stroke="#a2bdce" stroke-width=".8" />
                    </g>
                  </g>`).join('')}
                  <path d="m76.4 82.9 20.2 15.4 1.1 8.8-18.7-14.4Z" fill="#718d9f" />
                  <path d="m96.6 98.3 27.8-3.4-2.6 9.1-24.1 3.1Z" fill="#4c6679" />
                  <g transform="matrix(.927 -.116 .375 .286 100 88.6)">
                    <rect x="-15" y="-26" width="30" height="54" rx="6" fill="url(#loading-airframe)" stroke="#b7cad6" stroke-opacity=".7" stroke-width=".9" />
                    <rect x="-7" y="-15" width="14" height="26" rx="3" fill="#698697" stroke="#8fa9b9" stroke-width="1" />
                    <path d="M-4-8h8m-8 5h8m-8 5h8" stroke="#435f72" stroke-width="1.6" />
                    <path d="M-8 21H8" stroke="#92b9c9" stroke-width="2.2" stroke-linecap="round" />
                  </g>
                  <path d="m108 105 1 5" stroke="#8fa9ba" stroke-width="3" />
                  <path d="m103 110 12-2 5 4v7l-12 3-5-4Z" fill="#3d576a" stroke="#6b879b" stroke-width=".7" />
                  <ellipse cx="111" cy="115" rx="4.5" ry="4.8" fill="#293f50" stroke="#8da8ba" stroke-width=".9" />
                  <ellipse cx="111" cy="115" rx="2.4" ry="2.7" fill="#527a95" />
                  <circle cx="110.2" cy="114.2" r=".8" fill="#9abccc" />
                </g>
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
