import { describe, expect, it } from "vitest";

const filesystemModule = "node:fs";
const { readFileSync } = await import(filesystemModule);
const source = (path: string): string => readFileSync(new URL(`../${path}`, import.meta.url), "utf8");

describe("project flight mark", () => {
  it("keeps the vector self-contained, accessible and legible without fonts", () => {
    const svg = source("public/favicon.svg");
    expect(svg).toContain('viewBox="0 0 64 64"');
    expect(svg).toContain('aria-labelledby="title"');
    expect(svg).toContain("UAV 3D Planner");
    expect(svg.match(/<circle\b/g)).toHaveLength(4);
    expect(svg).toContain('id="flight-direction"');
    expect(svg).toContain('id="flight-path"');
    expect(svg).not.toMatch(/<(?:script|foreignObject|text|image)\b|\son\w+=|\shref=/i);
  });

  it.each(["index.html", "dynamic.html", "predictive.html", "results.html"])(
    "uses the same versioned SVG and ICO assets in %s", file => {
      const links = [...source(file).matchAll(/<link\s+rel="icon"[^>]+>/g)].map(match => match[0]);
      expect(links).toHaveLength(2);
      expect(links[0]).toContain('type="image/x-icon"');
      expect(links[0]).toContain('sizes="16x16 32x32 48x48"');
      expect(links[0]).toContain('href="/favicon.ico?v=flight-mark-1"');
      expect(links[1]).toContain('type="image/svg+xml"');
      expect(links[1]).toContain('sizes="any"');
      expect(links[1]).toContain('href="/favicon.svg?v=flight-mark-1"');
    },
  );

  it("provides independently rasterized 16, 32 and 48 pixel ICO images", () => {
    const ico = readFileSync(new URL("../public/favicon.ico", import.meta.url));
    expect(ico.readUInt16LE(0)).toBe(0);
    expect(ico.readUInt16LE(2)).toBe(1);
    expect(ico.readUInt16LE(4)).toBe(3);
    [16, 32, 48].forEach((size, index) => {
      const entry = 6 + index * 16, bytes = ico.readUInt32LE(entry + 8), offset = ico.readUInt32LE(entry + 12);
      expect(ico[entry]).toBe(size);
      expect(ico[entry + 1]).toBe(size);
      expect(offset).toBeGreaterThanOrEqual(54);
      expect(offset + bytes).toBeLessThanOrEqual(ico.length);
      expect(ico.subarray(offset, offset + 8).toString("hex")).toBe("89504e470d0a1a0a");
      expect(ico.readUInt32BE(offset + 16)).toBe(size);
      expect(ico.readUInt32BE(offset + 20)).toBe(size);
    });
  });
});
