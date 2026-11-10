import * as THREE from "three";

export function createMapRenderer(): THREE.WebGLRenderer {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.shadowMap.autoUpdate = false;
  return renderer;
}

/** Adapt only the drawing buffer. CSS instruments, route widths and imagery stay sharp. */
export class RenderQuality {
  private nativeRatio = 1;
  private ratio = 1;
  private previous?: number;
  private intervals: number[] = [];
  private costs: number[] = [];
  private windowStart?: number;
  private slow = 0;
  private fast = 0;
  private cooldown = 0;
  playing = false;
  interacting = false;

  constructor(private readonly renderer: Pick<THREE.WebGLRenderer, "setPixelRatio" | "setSize" | "render">,
    private readonly invalidate: () => void) {}

  get imageryPixelRatio(): number { return this.nativeRatio; }

  resize(width: number, height: number, deviceRatio = window.devicePixelRatio): void {
    const native = Math.max(.5, Math.min(2, Number.isFinite(deviceRatio) ? deviceRatio : 1));
    if (native !== this.nativeRatio || this.previous === undefined) {
      this.nativeRatio = native; this.ratio = native;
      this.renderer.setPixelRatio(native);
    }
    this.renderer.setSize(width, height, false);
    this.reset();
  }

  render(scene: THREE.Scene, camera: THREE.Camera, startedAt: number, settling: boolean): void {
    this.renderer.render(scene, camera);
    this.sample(startedAt, performance.now() - startedAt,
      this.playing || this.interacting || settling, document.visibilityState === "visible");
  }

  /** Sustained frame pressure lowers density; recovery takes longer to avoid oscillation. */
  sample(now: number, costMs: number, active: boolean, visible = true): void {
    const interval = this.previous === undefined ? 0 : now - this.previous;
    this.previous = now;
    if (!active || !visible || interval <= 0 || interval > 1000) {
      this.reset(); this.previous = now;
      if (!active) this.setRatio(this.nativeRatio);
      return;
    }
    if (now < this.cooldown) return;
    this.windowStart ??= now;
    this.intervals.push(interval); this.costs.push(costMs);
    if (this.intervals.length < 8 || this.intervals.length < 40 && now - this.windowStart < 1200) return;
    this.intervals.sort((a, b) => a - b); this.costs.sort((a, b) => a - b);
    const middle = this.intervals.length >>> 1;
    const median = this.intervals[middle]!, p90 = this.intervals[Math.floor(this.intervals.length * .9)]!, cost = this.costs[middle]!;
    this.intervals.length = this.costs.length = 0;
    this.windowStart = undefined;
    if (median > 24) {
      this.fast = 0;
      if (++this.slow < 2) return;
      this.slow = 0;
      this.setRatio(Math.max(Math.min(.75, this.nativeRatio), this.ratio * Math.max(.65, Math.min(.85, Math.sqrt(16.7 / median)))));
      this.cooldown = now + 1000;
    } else if (p90 < 18 && cost < 8) {
      this.slow = 0;
      if (++this.fast < 12) return;
      this.fast = 0;
      this.setRatio(Math.min(this.nativeRatio, this.ratio * 1.15));
      this.cooldown = now + 2000;
    } else this.slow = this.fast = 0;
  }

  private setRatio(value: number): void {
    const ratio = Math.min(this.nativeRatio, Math.round(value * 20) / 20);
    if (ratio === this.ratio) return;
    this.ratio = ratio; this.renderer.setPixelRatio(ratio); this.invalidate();
  }

  private reset(): void {
    this.previous = undefined;
    this.windowStart = undefined;
    this.intervals.length = this.costs.length = 0;
    this.slow = this.fast = 0;
  }
}
