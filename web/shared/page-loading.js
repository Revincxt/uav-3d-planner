/* Runs directly from HTML, independently of the app's module graph. */
(() => {
  const root = document.documentElement;
  const overlay = document.getElementById('page-loading');
  const title = document.getElementById('page-loading-title');
  const detail = document.getElementById('page-loading-detail');
  const retry = document.getElementById('page-loading-retry');
  let ready = false, timer;
  const busy = () => {
    const main = document.querySelector('main');
    if (main) { main.inert = root.dataset.pageLoading !== 'ready'; main.setAttribute('aria-busy', String(main.inert)); }
  };
  function stage(value) {
    clearTimeout(timer);
    root.dataset.pageLoading = value;
    overlay.setAttribute('aria-hidden', String(value === 'ready'));
    overlay.setAttribute('role', value === 'error' ? 'alert' : 'status');
    retry.hidden = value !== 'error';
    busy();
    if (value === 'ready') { ready = true; return; }
    if (value === 'error') {
      title.textContent = 'Unable to load scene'; detail.textContent = 'Check your connection and retry.'; detail.hidden = false; return;
    }
    title.textContent = 'Loading…';
    detail.textContent = ''; detail.hidden = true;
    timer = setTimeout(() => {
      // A cancelled/stalled navigation must not trap a working scene behind a veil.
      if (value === 'navigation' && ready) { stage('ready'); return; }
      detail.textContent = 'Taking longer than usual. You can retry.'; detail.hidden = false; retry.hidden = false;
    }, 12000);
  }
  stage('boot');
  retry.addEventListener('click', () => location.reload());
  document.addEventListener('DOMContentLoaded', busy, { once: true });
  window.addEventListener('uav:page-loading', event => {
    if (['data', 'scene', 'ready', 'error'].includes(event.detail?.stage)) stage(event.detail.stage);
  });
  window.addEventListener('pageshow', event => {
    if (event.persisted && ready) stage('ready');
  });
  window.addEventListener('error', event => {
    const target = event.target;
    if (!ready && (target?.tagName === 'SCRIPT' || (target?.tagName === 'LINK' && target.rel === 'stylesheet') || event.message)) stage('error');
  }, true);
  document.addEventListener('click', event => {
    if (event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey || event.defaultPrevented) return;
    const link = event.target?.closest?.('.study-nav a');
    if (!link || link.hasAttribute('download') || (link.target && link.target !== '_self')) return;
    const destination = new URL(link.href, location.href);
    if (destination.origin !== location.origin || destination.href === location.href) return;
    // Bubble after Benchmark's link handler, but paint before navigation starts.
    stage('navigation');
  });
})();
