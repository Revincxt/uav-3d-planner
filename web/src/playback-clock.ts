// A normal flight retains the previous 4× presentation pace; rates are relative to it.
export const BASE_PLAYBACK_SPEED = 4;
export const DEFAULT_PLAYBACK_RATE = 1;
export const PLAYBACK_RATES = [0.5, 1, 2] as const;

/** One wall-clock mapping for every vehicle, hazard, trajectory and checkpoint in a page. */
export class PlaybackClock {
  private traceStartS = 0;
  private wallStartMs = 0;
  private playbackRate = DEFAULT_PLAYBACK_RATE;
  get rate(): number { return this.playbackRate; }
  start(timeS: number, nowMs = performance.now()): void {
    if (!Number.isFinite(timeS) || timeS < 0 || !Number.isFinite(nowMs)) throw new Error("Invalid playback clock");
    this.traceStartS = timeS; this.wallStartMs = nowMs;
  }
  sample(nowMs: number): number {
    if (!Number.isFinite(nowMs)) throw new Error("Invalid playback clock");
    return this.traceStartS + Math.max(0, nowMs - this.wallStartMs) / 1000 * BASE_PLAYBACK_SPEED * this.playbackRate;
  }
  setRate(rate: number, timeS: number, nowMs = performance.now()): boolean {
    if (!(PLAYBACK_RATES as readonly number[]).includes(rate)) return false;
    this.start(timeS, nowMs); this.playbackRate = rate; return true;
  }
}

export function mountPlaybackSpeedControls(clock: PlaybackClock, currentTime: () => number, onChange?: (rate: number) => void): void {
  const buttons = [...document.querySelectorAll<HTMLButtonElement>("[data-speed]")];
  const update = (): void => buttons.forEach(button => button.setAttribute("aria-pressed", String(Number(button.dataset.speed) === clock.rate)));
  update();
  for (const button of buttons) button.addEventListener("click", () => {
    if (!clock.setRate(Number(button.dataset.speed), currentTime())) return;
    update(); onChange?.(clock.rate);
  });
}
