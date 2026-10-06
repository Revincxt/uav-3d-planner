import { describe, expect, it } from "vitest";

async function styles(file = "simulator-ui.css"): Promise<string> {
  const filesystemModule = "node:fs";
  const { readFileSync } = await import(filesystemModule);
  return readFileSync(new URL(`../src/${file}`, import.meta.url), "utf8");
}

function rgbaToken(css: string, name: string): number[] {
  const value = css.match(new RegExp(`--${name}:\\s*rgba\\(([^)]+)\\)`))?.[1];
  expect(value, `missing shared ${name} surface`).toBeDefined();
  return value!.split(",").map(Number);
}

function luminance(rgb: number[]): number {
  return rgb.map(value => value / 255)
    .map(value => value <= .04045 ? value / 12.92 : ((value + .055) / 1.055) ** 2.4)
    .reduce((sum, value, index) => sum + value * [.2126, .7152, .0722][index]!, 0);
}

describe("shared translucent simulator surfaces", () => {
  it("keeps every shared surface partially transparent", async () => {
    const css = await styles();
    for (const name of ["glass-surface", "glass-panel", "glass-card", "glass-popup"]) {
      const rgba = rgbaToken(css, name);
      expect(rgba).toHaveLength(4);
      expect(rgba[3]).toBeGreaterThan(0);
      expect(rgba[3]).toBeLessThan(.9);
    }
    expect(css).toContain("--hud-surface: var(--glass-surface)");
  });

  it("shares the glass theme across Details, Results, HUD and chart cards", async () => {
    const css = await styles();
    expect(css).toMatch(/\.workspace \.inspector-card\s*\{[^}]*background: var\(--glass-panel\)/);
    expect(css).toMatch(/\.workspace-results \.stage-card\s*\{[^}]*background: var\(--glass-panel\)/);
    expect(css).toMatch(/\.workspace-results \.app-header\s*\{[^}]*background: var\(--glass-surface\)/);
    expect(css).toMatch(/\.flight-hud\s*\{[^}]*background: var\(--hud-surface\)/);
    const charts = await styles("results.css");
    expect(charts).toMatch(/\.comparison-chart\s*\{[^}]*background: var\(--glass-card\)/);
    expect(charts).toMatch(/::after\s*\{[^}]*background: var\(--glass-popup\)/);
  });

  it("does not fade text or icons by changing a whole panel's opacity", async () => {
    const css = await styles();
    const opacityRules = [...css.matchAll(/([^{}]+)\{[^{}]*\bopacity:\s*[^;}]+/g)];
    expect(opacityRules).toHaveLength(1);
    expect(opacityRules[0]![1]).toContain("button:disabled");
    expect(opacityRules[0]![1]).toContain("select:disabled");
    expect(css).toMatch(/#play-pause\[aria-pressed="true"\]\s*\{[^}]*background: #[0-9a-f]{8};/);
  });

  it("keeps normal, muted and unit labels readable even above a white map", async () => {
    const css = await styles(), surface = rgbaToken(css, "glass-surface");
    const background = surface.slice(0, 3).map(value => value * surface[3]! + 255 * (1 - surface[3]!));
    for (const name of ["ink", "muted", "faint"]) {
      const hex = css.match(new RegExp(`--${name}:\\s*#([0-9a-f]{6})\\b`))![1]!;
      const foreground = hex.match(/../g)!.map(value => parseInt(value, 16));
      const contrast = (luminance(foreground) + .05) / (luminance(background) + .05);
      expect(contrast, `${name} contrast over the brightest possible map`).toBeGreaterThanOrEqual(4.5);
    }
  });

  it("reuses panel blur instead of adding a blur layer to every nested card", async () => {
    const css = await styles(), charts = await styles("results.css");
    for (const rule of [
      charts.match(/\.comparison-chart\s*\{([^}]+)\}/)![1],
      css.match(/\.workspace \.inspector-card :is\(\.metric-card[^}]+\{([^}]+)\}/)![1],
    ]) expect(rule).not.toContain("backdrop-filter");
  });
});
