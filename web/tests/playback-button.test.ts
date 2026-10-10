import { describe, expect, it, vi } from "vitest";
import { showPlaybackButton, showPlaybackState } from "../src/playback-state";

describe("icon-only playback control", () => {
  it("retains its SVG and toggles accessible action labels, without visible text", () => {
    const button = { textContent: "", innerHTML: '<svg aria-hidden="true"></svg>', title: "", setAttribute: vi.fn() };
    showPlaybackButton(button as unknown as HTMLButtonElement, true);
    expect(button.setAttribute).toHaveBeenCalledWith("aria-pressed", "true");
    expect(button.setAttribute).toHaveBeenCalledWith("aria-label", "Pause");
    expect(button.title).toBe("Pause");
    showPlaybackButton(button as unknown as HTMLButtonElement, false);
    expect(button.setAttribute).toHaveBeenCalledWith("aria-pressed", "false");
    expect(button.setAttribute).toHaveBeenCalledWith("aria-label", "Play");
    expect(button.title).toBe("Play");
    expect(button.innerHTML).toBe('<svg aria-hidden="true"></svg>');
    expect(button.textContent).toBe("");
  });
  it("styles an actual starting forecast hold as waiting, not ready", () => {
    const host = { textContent: "", dataset: {} as Record<string, string>, title: "",
      ownerDocument: { documentElement: { dataset: {} } } };
    showPlaybackState(host as unknown as HTMLElement, "predictive", "Ready", false,
      { kind: "waiting", reason: "forecast-aware waiting action", remainingS: 3.1 });
    expect(host.textContent).toBe("Space-time · Await slot · 4 s");
    expect(host.dataset.action).toBe("waiting");
    showPlaybackState(host as unknown as HTMLElement, "predictive", "Flying", false, { kind: "climbing" });
    expect(host.textContent).toBe("Space-time · Climbing");
    expect(host.dataset.action).toBe("flying");
  });
});
