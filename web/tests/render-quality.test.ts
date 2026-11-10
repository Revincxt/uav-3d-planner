import { describe, expect, it, vi } from "vitest";
import { RenderQuality } from "../src/render-quality";

function harness(dpr = 2) {
  const renderer = { setPixelRatio: vi.fn(), setSize: vi.fn(), render: vi.fn() }, invalidate = vi.fn();
  const quality = new RenderQuality(renderer, invalidate);
  quality.resize(1440, 900, dpr);
  let now = 0;
  const frames = (count: number, interval: number, cost = 4, active = true, visible = true) => {
    for (let i = 0; i < count; i++) quality.sample(now += interval, cost, active, visible);
  };
  return { quality, renderer, invalidate, frames, ratio: () => renderer.setPixelRatio.mock.lastCall![0] as number };
}

describe("adaptive map drawing buffer", () => {
  it("preserves native sharpness on normal and high refresh displays, capped at 2×", () => {
    for (const dpr of [1, 2, 3]) {
      const test = harness(dpr);
      test.frames(600, 1000 / 60); test.frames(600, 1000 / 144);
      expect(test.ratio()).toBe(Math.min(2, dpr));
      expect(test.renderer.setSize).toHaveBeenCalledWith(1440, 900, false);
      expect(test.invalidate).not.toHaveBeenCalled();
    }
  });

  it("ignores brief stalls, idle gaps and hidden tabs", () => {
    const test = harness();
    test.frames(20, 40); test.frames(100, 16.7);
    test.frames(100, 500, 4, false); test.frames(100, 40, 4, true, false);
    expect(test.ratio()).toBe(2);
    expect(test.invalidate).not.toHaveBeenCalled();
  });

  it("reduces sustained overload without changing imagery resolution or CSS sizing", () => {
    const test = harness();
    test.frames(81, 40);
    expect(test.ratio()).toBe(1.3);
    expect(test.quality.imageryPixelRatio).toBe(2);
    expect(test.renderer.setSize).toHaveBeenCalledTimes(1);
    expect(test.invalidate).toHaveBeenCalledOnce();
    test.frames(15, 40);
    expect(test.invalidate).toHaveBeenCalledOnce();
  });

  it("bounds low-end density and restores full sharpness when animation stops", () => {
    const test = harness(1);
    test.frames(1000, 50);
    expect(test.ratio()).toBe(.75);
    test.frames(1, 16.7, 1, false);
    expect(test.ratio()).toBe(1);
    const slow = harness(); slow.frames(40, 100);
    expect(slow.ratio()).toBeLessThan(2);
  });

  it("requires sustained headroom before raising density again", () => {
    const test = harness();
    test.frames(81, 40);
    const reduced = test.ratio();
    test.frames(400, 16.7);
    expect(test.ratio()).toBe(reduced);
    test.frames(180, 16.7);
    expect(test.ratio()).toBeGreaterThan(reduced);
    expect(test.ratio()).toBeLessThan(2);
  });

  it("recalibrates after moving between monitors", () => {
    const test = harness();
    test.frames(81, 40);
    test.quality.resize(1280, 720, 1);
    expect(test.ratio()).toBe(1);
    expect(test.quality.imageryPixelRatio).toBe(1);
  });
});
