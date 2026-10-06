import { flightHeading, routeColorCSS, timedPosition, waypointIndex, type OverviewRoute } from "./route-overview";

export interface FlightReadout {
  altitudeM: number;
  speedMps: number;
  headingDeg: number;
}

/** Recorded simulation units, independent of wall-clock replay speed or camera orientation. */
export function flightReadout(route: OverviewRoute, timeS: number): FlightReadout | null {
  const path = route.timedPath;
  if (!path?.length || !Number.isFinite(timeS)) return null;
  const index = waypointIndex(path, timeS), left = path[index]!, right = path[index + 1];
  const speedMps = right && timeS >= left.timeS && right.timeS > left.timeS
    ? Math.hypot(...right.position.map((value, axis) => value - left.position[axis]!)) / (right.timeS - left.timeS)
    : 0;
  const heading = flightHeading(path, timeS);
  const headingDeg = ((Math.atan2(heading.x, -heading.z) * 180 / Math.PI) + 360) % 360;
  return {
    altitudeM: timedPosition(path, timeS)[2], speedMps, headingDeg,
  };
}

/** One retained instrument with its existing playback controls; no extra animation loop. */
export class FlightHud {
  private readonly root = document.createElement("section");
  private readonly name: HTMLElement;
  private readonly fields: HTMLElement[];
  private lastUpdateMs = -Infinity;
  private routeId = "";
  constructor(container: HTMLElement, playback?: HTMLElement) {
    this.root.className = "flight-hud";
    this.root.setAttribute("aria-label", "Recorded flight telemetry");
    this.root.innerHTML = `<div class="hud-telemetry"><div class="hud-heading"><span class="hud-aircraft">UAV 01</span><span class="hud-frame" title="Local east, north, up coordinates">ENU</span></div>
      <div class="hud-flight"><div class="hud-compass" aria-hidden="true"><svg class="hud-dial" viewBox="0 0 80 80" fill="none"><circle cx="40" cy="40" r="38.5" /><circle class="hud-dial-inner" cx="40" cy="40" r="28.5" />${Array.from({ length: 24 }, (_, index) => `<path d="M40 4.5v${index % 3 ? 3 : 6}" transform="rotate(${index * 15} 40 40)" class="${index % 3 ? "hud-tick" : "hud-tick-major"}" />`).join("")}</svg><span>N</span><span>E</span><span>S</span><span>W</span><i class="hud-bearing"><svg viewBox="0 0 20 36"><path class="hud-needle" d="M10 1 16 23 10 20 4 23Z" /><path class="hud-needle-tail" d="m10 35-3-11h6Z" /></svg></i><i class="hud-compass-center"></i></div><div class="hud-instruments">
      <div class="hud-reading"><span>ALT <small>m</small></span><output data-readout="altitude">—</output></div>
      <div class="hud-reading"><span>SPD <small>m/s</small></span><output data-readout="speed" title="Trajectory speed in simulation units, not wall-clock playback speed">—</output></div>
      <div class="hud-reading"><span>HDG <small>°</small></span><output data-readout="heading">—</output></div></div></div>
      </div>`;
    this.name = this.root.querySelector<HTMLElement>(".hud-aircraft")!;
    this.fields = ["altitude", "speed", "heading"].map(key => this.root.querySelector<HTMLElement>(`[data-readout="${key}"]`)!);
    if (playback) this.root.append(playback);
    container.append(this.root);
  }
  update(route: OverviewRoute, timeS: number, index: number, force = false): void {
    const now = performance.now(), changed = this.routeId !== route.id;
    if (!force && !changed && now - this.lastUpdateMs < 100) return;
    this.lastUpdateMs = now;
    const readout = flightReadout(route, timeS);
    this.root.hidden = readout === null;
    if (!readout) return;
    if (changed) {
      this.routeId = route.id; this.root.dataset.routeId = route.id;
      this.name.textContent = `UAV ${String(index + 1).padStart(2, "0")}`;
      this.root.title = route.label;
      this.root.style.setProperty("--hud-route", routeColorCSS(index));
    }
    const values = [readout.altitudeM.toFixed(1), readout.speedMps.toFixed(1), String(Math.round(readout.headingDeg) % 360).padStart(3, "0")];
    this.fields.forEach((field, index) => { if (field.textContent !== values[index]) field.textContent = values[index]!; });
    this.root.style.setProperty("--heading", `${readout.headingDeg}deg`);
  }
  dispose(): void { this.root.remove(); }
}
