import { describe, expect, it, vi } from "vitest";
import { showPlaybackButton } from "../src/playback-state";

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
});
