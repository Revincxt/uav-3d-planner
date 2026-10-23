import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { prepareScene, runScene } from "../src/page-loading";

const filesystemModule = "node:fs/promises", pluginModule = "../build/page-loading.mjs";
const { readFile } = await import(filesystemModule);
const bootstrap = await readFile(new URL("../shared/page-loading.js", import.meta.url), "utf8");
const { pageLoadingPlugin } = await import(pluginModule);

function fixture(path = "dynamic.html") {
  const node = () => ({ textContent: "", hidden: false, attributes: new Map<string, string>(),
    events: new Map<string, () => void>(),
    setAttribute(name: string, value: string) { this.attributes.set(name, value); },
    addEventListener(name: string, listener: () => void) { this.events.set(name, listener); },
  });
  const ids = ['page-loading', 'page-loading-title', 'page-loading-detail', 'page-loading-retry'];
  const elements = Object.fromEntries(ids.map(id => [id, node()]));
  const root = { dataset: {} as Record<string, string> }, main = { ...node(), inert: false };
  const events = new Map<string, (event: any) => void>();
  const document = { documentElement: root, getElementById: (id: string) => elements[id],
    querySelector: () => main, addEventListener: (name: string, listener: (event: any) => void) => events.set(name, listener) };
  const host = new EventTarget();
  const location = { href: `https://example.test/app/${path}`, origin: 'https://example.test', pathname: `/app/${path}`, reload: vi.fn() };
  new Function('document', 'window', 'location', 'setTimeout', 'clearTimeout', bootstrap)(document, host, location, setTimeout, clearTimeout);
  vi.stubGlobal('window', host);
  vi.stubGlobal('requestAnimationFrame', (callback: FrameRequestCallback) => setTimeout(() => callback(0), 16));
  const stage = (stage: string) => host.dispatchEvent(new CustomEvent('uav:page-loading', { detail: { stage } }));
  const transition = (persisted: boolean) => {
    const event = new Event('pageshow'); Object.defineProperty(event, 'persisted', { value: persisted }); host.dispatchEvent(event);
  };
  const click = (options: Record<string, unknown> = {}, linkOptions: Record<string, unknown> = {}) => {
    const link = { href: 'https://example.test/app/predictive.html', target: '', textContent: 'Predictive', hasAttribute: () => false, ...linkOptions };
    const event = { button: 0, defaultPrevented: false, target: { closest: () => link }, ...options };
    events.get('click')!(event); return event;
  };
  return { elements, root, main, events, host, location, stage, transition, click };
}

beforeEach(() => vi.useFakeTimers());
afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });

describe('immediate HTML loading shell', () => {
  it('inlines its styles, status and bootstrap without another network request', async () => {
    const tags = await pageLoadingPlugin().transformIndexHtml();
    expect(tags.map((tag: { tag: string }) => tag.tag)).toEqual(['style', 'div', 'script']);
    expect(tags[0].injectTo).toBe('head');
    expect(tags[0].children).toContain('prefers-reduced-motion');
    expect(tags[1].attrs).toMatchObject({ id: 'page-loading', role: 'status', 'aria-live': 'polite' });
    expect(tags[2].children).toBe(bootstrap);
    expect(tags[2].injectTo).toBe('body-prepend');
    expect(tags.every((tag: { attrs?: Record<string, string> }) => !tag.attrs?.src)).toBe(true);
  });
  it('uses a frameless local flight mark without stage bars or decorative asset requests', async () => {
    const tags = await pageLoadingPlugin().transformIndexHtml(), markup = tags[1].children;
    expect(markup).toContain('class="page-loading-drone"');
    expect(markup).toContain('class="page-loading-warning"');
    expect(markup).toContain('class="page-loading-content"');
    expect(markup).toContain('class="page-loading-copy"');
    expect(markup).not.toMatch(/page-loading-card|page-loading-steps|data-step=|page-loading-label/);
    expect(markup).toContain('<strong id="page-loading-title">Loading…</strong>');
    expect(markup).toContain('<p id="page-loading-detail" hidden></p>');
    const content = tags[0].children.match(/\.page-loading-content\s*\{([^}]+)\}/)![1]!;
    for (const rule of ['border: 0', 'border-radius: 0', 'background: none', 'box-shadow: none']) expect(content).toContain(rule);
    for (const rule of ['display: flex', 'align-items: center', 'width: max-content']) expect(content).toContain(rule);
    expect(tags[0].children).not.toMatch(/page-loading-card|page-loading-step|page-loading-stage/);
    expect(markup).not.toMatch(/<img|<canvas|<iframe|\bsrc=/);
    expect(tags[0].children).not.toMatch(/@import|url\(/);
  });
  it('matches the simulator HUD palette with local, reduced-motion-aware flight instrumentation', async () => {
    const tags = await pageLoadingPlugin().transformIndexHtml();
    expect(tags[0].children).toContain('color-scheme: dark');
    expect(tags[0].children).toContain('--loading-text: #e5eef4');
    expect(tags[0].children).toContain('--loading-muted: #cfdae3');
    expect(tags[1].children).toContain('class="page-loading-reference"');
    expect(tags[1].children).toContain('class="page-loading-scan"');
    expect(tags[1].children).toContain('class="page-loading-field" aria-hidden="true"');
    expect(tags[0].children).toMatch(/prefers-reduced-motion: reduce[\s\S]+\.page-loading-course, #page-loading \.page-loading-scan \{ animation: none;/);
    expect(tags[0].children).toContain('html[data-page-loading="navigation"] #page-loading .page-loading-field { display: none; }');
  });
  it.each(['', 'dynamic.html', 'predictive.html', 'results.html'])(
    'shows only a fixed loading message immediately for %s', (path) => {
      const f = fixture(path);
      expect(f.root.dataset.pageLoading).toBe('boot');
      expect(f.elements['page-loading-title']!.textContent).toBe('Loading…');
      expect(f.elements['page-loading-detail']!.hidden).toBe(true);
      expect(f.main.inert).toBe(true);
    });
  it('keeps the shell until initialization and a following render opportunity complete', async () => {
    const f = fixture(), start = vi.fn(async () => { await prepareScene(); });
    const result = runScene(start);
    expect(f.root.dataset.pageLoading).toBe('data');
    expect(start).not.toHaveBeenCalled();
    await vi.advanceTimersByTimeAsync(32);
    expect(f.root.dataset.pageLoading).toBe('scene');
    expect(f.elements['page-loading-title']!.textContent).toBe('Loading…');
    expect(f.elements['page-loading-detail']!.hidden).toBe(true);
    await vi.advanceTimersByTimeAsync(48);
    expect(f.main.inert).toBe(true);
    await vi.advanceTimersByTimeAsync(16); await result;
    expect(f.root.dataset.pageLoading).toBe('ready');
    expect(f.main.inert).toBe(false);
    expect(f.main.attributes.get('aria-busy')).toBe('false');
    expect(f.elements['page-loading']!.attributes.get('aria-hidden')).toBe('true');
    await vi.advanceTimersByTimeAsync(20000);
    expect(f.elements['page-loading-retry']!.hidden).toBe(true);
  });
  it('reports initialization errors and offers a real reload without hiding the error', async () => {
    const f = fixture(), error = new Error('invalid dataset');
    const result = expect(runScene(async () => { throw error; })).rejects.toBe(error);
    await vi.advanceTimersByTimeAsync(32); await result;
    expect(f.root.dataset.pageLoading).toBe('error');
    expect(f.elements['page-loading']!.attributes.get('role')).toBe('alert');
    expect(f.elements['page-loading-retry']!.hidden).toBe(false);
    expect(f.elements['page-loading-detail']!.hidden).toBe(false);
    f.elements['page-loading-retry']!.events.get('click')!();
    expect(f.location.reload).toHaveBeenCalledOnce();
  });
  it('also handles module failures before the app can run its own error handler', () => {
    const f = fixture(), event = new Event('error');
    Object.defineProperty(event, 'target', { value: { tagName: 'SCRIPT' } }); f.host.dispatchEvent(event);
    expect(f.root.dataset.pageLoading).toBe('error');
  });
  it('does not turn a favicon or speculative module-preload failure into a fatal scene error', () => {
    const f = fixture();
    for (const rel of ['icon', 'modulepreload']) {
      const event = new Event('error'); Object.defineProperty(event, 'target', { value: { tagName: 'LINK', rel } }); f.host.dispatchEvent(event);
    }
    expect(f.root.dataset.pageLoading).toBe('boot');
  });
  it('offers retry on a slow connection without claiming a failure or fake percentage', async () => {
    const f = fixture(); await vi.advanceTimersByTimeAsync(12000);
    expect(f.root.dataset.pageLoading).toBe('boot');
    expect(f.elements['page-loading-detail']!.textContent).toContain('Taking longer');
    expect(f.elements['page-loading-retry']!.hidden).toBe(false);
  });
});

describe('navigation and BFCache', () => {
  it('uses the same single loading message over the existing scene while navigating', async () => {
    const f = fixture(); f.stage('ready'); f.click();
    await vi.advanceTimersByTimeAsync(0);
    expect(f.root.dataset.pageLoading).toBe('navigation');
    expect(f.elements['page-loading-title']!.textContent).toBe('Loading…');
    expect(f.elements['page-loading-detail']!.hidden).toBe(true);
  });
  it('leaves the in-place Benchmark dialog and cancelled clicks alone', async () => {
    const f = fixture(); f.stage('ready');
    f.click({ defaultPrevented: true });
    await vi.advanceTimersByTimeAsync(0);
    expect(f.root.dataset.pageLoading).toBe('ready');
  });
  it.each([{ ctrlKey: true }, { metaKey: true }, { shiftKey: true }, { altKey: true }, { button: 1 }, { defaultPrevented: true }])(
    'does not intercept modified or prevented clicks: %j', async options => {
      const f = fixture(); f.stage('ready'); f.click(options); await vi.advanceTimersByTimeAsync(0);
      expect(f.root.dataset.pageLoading).toBe('ready');
    });
  it.each([{ target: '_blank' }, { hasAttribute: () => true }, { href: 'https://other.test/' }, { href: 'https://example.test/app/dynamic.html' }])(
    'ignores external, new-tab, download and same-page links: %j', async link => {
      const f = fixture(); f.stage('ready'); f.click({}, link); await vi.advanceTimersByTimeAsync(0);
      expect(f.root.dataset.pageLoading).toBe('ready');
    });
  it('restores usable controls when returning from BFCache', async () => {
    const f = fixture(); f.stage('ready'); f.click(); f.transition(true);
    await vi.advanceTimersByTimeAsync(0);
    expect(f.root.dataset.pageLoading).toBe('ready');
    f.click(); await vi.advanceTimersByTimeAsync(0); expect(f.main.inert).toBe(true);
    f.transition(true); expect(f.main.inert).toBe(false);
    expect(f.root.dataset.pageLoading).toBe('ready');
  });
  it('releases a working scene if its navigation stalls or is cancelled by the browser', async () => {
    const f = fixture(); f.stage('ready'); f.click(); await vi.advanceTimersByTimeAsync(12000);
    expect(f.root.dataset.pageLoading).toBe('ready'); expect(f.main.inert).toBe(false);
  });
});
