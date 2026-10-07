import { afterEach, describe, expect, it, vi } from "vitest";
import { mountInspector } from "../src/workspace";

afterEach(() => vi.unstubAllGlobals());

async function html(file: string): Promise<string> {
  const filesystemModule = "node:fs";
  const { readFileSync } = await import(filesystemModule);
  return readFileSync(new URL(`../${file}`, import.meta.url), "utf8");
}

describe("shared navigation and flight controls", () => {
  it("anchors the flight status at the viewport top center without moving the flight HUD", async () => {
    const styles = await html("src/simulator-ui.css");
    const status = styles.match(/\.workspace:not\(\.workspace-results\) \.playback-state\s*\{([^}]+)\}/)![1]!;
    expect(status).toContain("top: var(--hud-gap)");
    expect(status).toContain("left: 50%");
    expect(status).toContain("transform: translateX(-50%)");
    expect(styles).not.toMatch(/\.playback-state\s*\{[^}]*top: (170|244|139)px/);
    const hud = styles.match(/\.flight-hud\s*\{([^}]+)\}/)![1]!;
    expect(hud).toContain("bottom: var(--hud-gap)");
  });

  it("keeps algorithm and follow on the left while camera controls align to the right", async () => {
    const styles = await html("src/simulator-ui.css");
    const toolbar = styles.match(/\.workspace:not\(\.workspace-results\) \.stage-toolbar\s*\{([^}]+)\}/)![1]!;
    expect(toolbar).toContain("left: var(--hud-gap)");
    expect(toolbar).toContain("right: var(--hud-gap)");
    expect(toolbar).toContain("width: auto");
    expect(styles).toMatch(/\.stage-toolbar > \.camera-controls\s*\{\s*margin-left: auto;/);
    expect(styles).toMatch(/\.workspace-static \.stage-toolbar \.planner-control\s*\{[^}]*width: 146px;/);
  });

  it.each(["index.html", "dynamic.html", "predictive.html", "results.html"])(
    "keeps four named links and one active page in %s", async (file) => {
      const navigation = (await html(file)).match(/<nav class="study-nav"[^>]*>([\s\S]*?)<\/nav>/)![1]!;
      const links = [...navigation.matchAll(/<a href="([^"]+)"([^>]*)>([^<]+)<\/a>/g)];
      expect(links.map(link => [link[1], link[3]])).toEqual([
        ["./", "Static"], ["./dynamic.html", "Dynamic"],
        ["./predictive.html", "Predictive"], ["./results.html", "Benchmark"],
      ]);
      expect(links.filter(link => link[2]!.includes('aria-current="page"'))).toHaveLength(1);
    },
  );

  it.each(["index.html", "dynamic.html", "predictive.html"])(
    "uses the same algorithm, follow and camera group order in %s", async (file) => {
      const source = await html(file);
      expect(source).toContain('role="group" aria-label="Flight controls"');
      expect(source).toContain('class="planner-control"');
      expect(source).toContain('class="view-controls camera-controls"');
      expect(source).not.toContain('class="stage-controls"');
      expect(source.indexOf('class="planner-control"')).toBeLessThan(source.indexOf('class="follow-controls"'));
      expect(source.indexOf('class="follow-controls"')).toBeLessThan(source.indexOf('class="view-controls camera-controls"'));
    },
  );
});

function inspectorFixture(id = "") {
  function button() {
    const attributes = new Map<string, string>();
    const events = new Map<string, () => void>();
    return {
      type: "", className: "", textContent: "", title: "", attributes, events,
      setAttribute: (name: string, value: string) => attributes.set(name, value),
      addEventListener: (name: string, listener: () => void) => events.set(name, listener),
      focus: vi.fn(),
    };
  }
  const toggle = button(), close = button(), classes = new Set<string>();
  const panel = {
    id, append: vi.fn(),
    classList: {
      contains: (name: string) => classes.has(name),
      remove: (name: string) => classes.delete(name),
      toggle: (name: string) => {
        if (classes.has(name)) { classes.delete(name); return false; }
        classes.add(name); return true;
      },
    },
  };
  const toolbar = { append: vi.fn() };
  const events = new Map<string, (event: { key: string }) => void>();
  vi.stubGlobal("document", {
    querySelector: (selector: string) => selector === ".inspector-card" ? panel : selector === ".stage-toolbar" ? toolbar : null,
    createElement: vi.fn().mockReturnValueOnce(toggle).mockReturnValueOnce(close),
    addEventListener: (name: string, listener: (event: { key: string }) => void) => events.set(name, listener),
  });
  mountInspector();
  return { toggle, close, panel, toolbar, events };
}

describe("Inspector control", () => {
  it("lives in the flight toolbar and identifies its controlled panel", () => {
    const f = inspectorFixture();
    expect(f.toolbar.append).toHaveBeenCalledExactlyOnceWith(f.toggle);
    expect(f.toggle.textContent).toBe("Inspector");
    expect(f.toggle.attributes.get("aria-controls")).toBe("flight-inspector");
    expect(f.toggle.attributes.get("aria-expanded")).toBe("false");
    expect(f.panel.append).toHaveBeenCalledExactlyOnceWith(f.close);
  });

  it("keeps its label and changes only the open state and tooltip", () => {
    const f = inspectorFixture("existing-panel");
    expect(f.toggle.attributes.get("aria-controls")).toBe("existing-panel");
    f.toggle.events.get("click")!();
    expect(f.panel.classList.contains("inspector-open")).toBe(true);
    expect(f.toggle.attributes.get("aria-expanded")).toBe("true");
    expect(f.toggle.title).toBe("Close inspector");
    expect(f.toggle.textContent).toBe("Inspector");
    f.toggle.events.get("click")!();
    expect(f.panel.classList.contains("inspector-open")).toBe(false);
    expect(f.toggle.attributes.get("aria-expanded")).toBe("false");
    expect(f.toggle.title).toBe("Open inspector");
  });

  it("returns focus to the control when the panel close button is used", () => {
    const f = inspectorFixture();
    f.toggle.events.get("click")!();
    f.close.events.get("click")!();
    expect(f.toggle.attributes.get("aria-expanded")).toBe("false");
    expect(f.panel.classList.contains("inspector-open")).toBe(false);
    expect(f.toggle.focus).toHaveBeenCalledOnce();
  });

  it("closes on Escape without stealing focus when already closed", () => {
    const f = inspectorFixture();
    f.events.get("keydown")!({ key: "Escape" });
    expect(f.toggle.focus).not.toHaveBeenCalled();
    f.toggle.events.get("click")!();
    f.events.get("keydown")!({ key: "ArrowRight" });
    expect(f.panel.classList.contains("inspector-open")).toBe(true);
    f.events.get("keydown")!({ key: "Escape" });
    expect(f.panel.classList.contains("inspector-open")).toBe(false);
    expect(f.toggle.attributes.get("aria-expanded")).toBe("false");
    expect(f.toggle.focus).toHaveBeenCalledOnce();
  });
});
