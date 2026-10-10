import { describe, expect, it, vi } from "vitest";
import { mountPageLifecycle } from "../src/page-lifecycle";
describe("map page lifecycle", () => {
  it("retains the renderer across repeated BFCache restores, then disposes on actual unload", () => {
    const host = new EventTarget(), pause = vi.fn(), restore = vi.fn(), dispose = vi.fn();
    const remove = mountPageLifecycle({ pause, restore, dispose }, host as unknown as Window);
    const send = (type: string, persisted: boolean) => { const event = new Event(type); Object.defineProperty(event, "persisted", { value: persisted }); host.dispatchEvent(event); };
    for (let i = 0; i < 3; i++) { send("pagehide", true); send("pageshow", true); }
    expect(pause).toHaveBeenCalledTimes(3); expect(restore).toHaveBeenCalledTimes(3); expect(dispose).not.toHaveBeenCalled();
    send("pageshow", false); expect(restore).toHaveBeenCalledTimes(3);
    send("pagehide", false); expect(dispose).toHaveBeenCalledTimes(1);
    remove(); send("pagehide", false); expect(dispose).toHaveBeenCalledTimes(1);
  });
});
