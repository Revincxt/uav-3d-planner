import { flightHeading, routeColorCSS, timedPosition, waypointIndex, type OverviewRoute } from "./route-overview";
import { PLAN_LABELS, planMeaning } from "./trajectory-semantics";

export interface FlightReadout {
  altitudeM: number;
  speedMps: number;
  headingDeg: number;
}

type AltitudeTrend = "climb" | "descent" | null;

const DIAL_LIMITS = { speed: 40, altitude: 800 } as const;
const instrumentDial = (name: keyof typeof DIAL_LIMITS, label: string, unit: string): string => `<div class="hud-reading hud-analog hud-${name}" role="group" aria-label="${name === "speed" ? "Speed in metres per second" : "Altitude in metres"}">
  <svg class="hud-gauge-svg" viewBox="0 0 100 100" aria-hidden="true">
    <circle class="hud-gauge-bezel" cx="50" cy="50" r="45" /><circle class="hud-gauge-face" cx="50" cy="50" r="42" />
    ${Array.from({ length: 21 }, (_, index) => {
      const angle = -135 + index * 13.5, radians = angle * Math.PI / 180;
      const tick = `<path class="${index % 5 ? "hud-gauge-tick" : "hud-gauge-major"}" d="M50 12v${index % 5 ? 4 : 7}" transform="rotate(${angle} 50 50)" />`;
      return index % 5 ? tick : `${tick}<text x="${(50 + 29 * Math.sin(radians)).toFixed(2)}" y="${(52.8 - 29 * Math.cos(radians)).toFixed(2)}">${DIAL_LIMITS[name] * index / 20}</text>`;
    }).join("")}
    <g class="hud-gauge-needle"><path class="hud-gauge-counterweight" d="M48.5 49h3v13h-3Z" /><path class="hud-gauge-hand" d="M48.7 54 50 18 51.3 54Z" /></g>
    <circle class="hud-gauge-hub" cx="50" cy="50" r="3.8" /><circle class="hud-gauge-pin" cx="50" cy="50" r="1.7" />
  </svg><output data-readout="${name}">—</output><span>${label} <small>${unit}</small></span></div>`;

/** Recent recorded height change, with a dead band for small curve oscillations.
 * Check the current segment as well, so real level flight, waits and goals clear immediately. */
export function altitudeTrend(route: OverviewRoute, timeS: number, previous: AltitudeTrend = null): AltitudeTrend {
  const path = route.timedPath;
  if (!path?.length || !Number.isFinite(timeS)) return null;
  const index = waypointIndex(path, timeS), left = path[index]!, right = path[index + 1];
  if (!right || timeS < left.timeS || right.timeS <= left.timeS) return null;
  const velocity = (right.position[2] - left.position[2]) / (right.timeS - left.timeS);
  if (Math.abs(velocity) < .08) return null;
  const fromS = Math.max(path[0]!.timeS, timeS - .8);
  if (timeS <= fromS) return null;
  const change = timedPosition(path, timeS)[2] - timedPosition(path, fromS)[2];
  if (Math.abs(change) < .15 || change * velocity <= 0) return null;
  const direction = change > 0 ? "climb" : "descent";
  return Math.abs(change) / (timeS - fromS) >= (previous === direction ? .15 : .35) ? direction : null;
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
  private readonly key: HTMLElement;
  private readonly historyKey: HTMLElement;
  private readonly altitudeHint: HTMLElement;
  private readonly timeline?: HTMLInputElement;
  private trend: AltitudeTrend = null;
  private lastUpdateMs = -Infinity;
  private routeId = "";
  constructor(container: HTMLElement, playback?: HTMLElement) {
    this.root.className = "flight-hud";
    this.root.setAttribute("aria-label", "Recorded flight telemetry");
    this.root.innerHTML = `<div class="hud-telemetry"><div class="hud-flight hud-instruments">
      <svg class="hud-halo" viewBox="0 0 400 184" fill="none" aria-hidden="true"><defs><filter id="hud-contour-soften" x="-15%" y="-20%" width="130%" height="140%"><feGaussianBlur in="SourceAlpha" stdDeviation="4" result="soft-edge" /><feComposite in="soft-edge" in2="SourceAlpha" operator="out" result="outer-edge" /><feFlood class="hud-edge-tint" result="edge-color" /><feComposite in="edge-color" in2="outer-edge" operator="in" /></filter></defs><use href="#hud-case-outline" fill="white" filter="url(#hud-contour-soften)" /></svg>
      <div class="hud-compass"><svg class="hud-dial" viewBox="0 0 400 184" fill="none" aria-hidden="true">
        <defs>
          <path id="hud-case-outline" d="M18 182a182 182 0 0 1 364 0v2H18Z" />
          <linearGradient id="hud-bezel-metal" x1="0" y1="0" x2="0" y2="1"><stop stop-color="#8096a3" stop-opacity=".22" /><stop offset=".08" stop-color="#617c8c" stop-opacity=".22" /><stop offset=".45" stop-color="#4a6575" stop-opacity=".22" /><stop offset=".85" stop-color="#3d5768" stop-opacity=".22" /><stop offset="1" stop-color="#5c7787" stop-opacity=".22" /></linearGradient>
        </defs>
        <use class="hud-case" href="#hud-case-outline" />
        <path class="hud-bezel" d="M26 180a174 174 0 0 1 348 0" /><path class="hud-dial-inner" d="M40 180a160 160 0 0 1 320 0" />
        <g class="hud-fasteners"><circle cx="75" cy="55" r="3.5" /><circle cx="325" cy="55" r="3.5" /><path d="m73 53 4 4m246-4 4 4" /></g>
        <g class="hud-compass-card">${Array.from({ length: 72 }, (_, index) => `<path d="M200 13v${index % 6 ? index % 2 ? 4 : 7 : 11}" transform="rotate(${index * 5} 200 180)" class="${index % 6 ? "hud-tick" : "hud-tick-major"}" />`).join("")}${Array.from({ length: 12 }, (_, index) => {
          const x = (200 + 148 * Math.sin(index * Math.PI / 6)).toFixed(2), y = (180 - 148 * Math.cos(index * Math.PI / 6)).toFixed(2);
          return `<text x="${x}" y="${y}" dy="3.5" transform="rotate(${index * 30} ${x} ${y})" class="${index % 3 ? "" : "hud-cardinal"}">${["N", "03", "06", "E", "12", "15", "S", "21", "24", "W", "30", "33"][index]}</text>`;
        }).join("")}</g>
        <path class="hud-pointer-mount" d="M189 3h22l-2 11h-18Z" /><path class="hud-heading-index" d="m197 4 3 13 3-13Z" /><path class="hud-airframe" d="M177 105h14l9-8 9 8h14m-23-8v16" />
      </svg></div>
      <small class="hud-altitude-trend" hidden></small>
      <div class="hud-readouts" role="group" aria-label="Flight instruments">${instrumentDial("speed", "SPD", "m/s")}
      <div class="hud-reading hud-heading-reading"><span>HDG <small>°</small></span><output data-readout="heading">—</output></div>
      ${instrumentDial("altitude", "ALT", "m")}</div>
      </div><div class="hud-heading" role="group" aria-label="Aircraft and route legend"><span class="hud-aircraft">UAV 01</span><span class="hud-route-key"><span class="hud-key-flown">Flown</span><span class="hud-key-plan">Fixed route</span></span></div></div>`;
    this.name = this.root.querySelector<HTMLElement>(".hud-aircraft")!;
    this.key = this.root.querySelector<HTMLElement>(".hud-key-plan")!;
    this.historyKey = this.root.querySelector<HTMLElement>(".hud-key-flown")!;
    this.altitudeHint = this.root.querySelector<HTMLElement>(".hud-altitude-trend")!;
    this.fields = ["altitude", "speed", "heading"].map(key => this.root.querySelector<HTMLElement>(`[data-readout="${key}"]`)!);
    if (playback) {
      this.timeline = playback.querySelector<HTMLInputElement>("#timeline") ?? undefined;
      this.root.append(playback);
    }
    container.append(this.root);
  }
  update(route: OverviewRoute, timeS: number, index: number, force = false): void {
    // Keep the track fill in sync with the existing frame-rate timeline, while
    // text below retains its 10 Hz budget. The original input/listeners survive.
    if (this.timeline) {
      const min = Number(this.timeline.min), max = Number(this.timeline.max);
      const progress = max > min ? 100 * (Number(this.timeline.value) - min) / (max - min) : 0;
      this.timeline.style.setProperty("--timeline-progress", `${Math.max(0, Math.min(100, progress))}%`);
    }
    const now = performance.now(), changed = this.routeId !== route.id;
    if (!force && !changed && now - this.lastUpdateMs < 100) return;
    this.lastUpdateMs = now;
    const readout = flightReadout(route, timeS);
    this.root.hidden = readout === null;
    if (!readout) return;
    const trend = altitudeTrend(route, timeS, force || changed ? null : this.trend);
    if (trend !== this.trend) {
      this.trend = trend;
      this.altitudeHint.hidden = trend === null;
      this.altitudeHint.textContent = trend === "climb" ? "Climb" : trend === "descent" ? "Descent" : "";
      if (trend) this.altitudeHint.dataset.direction = trend;
      else delete this.altitudeHint.dataset.direction;
    }
    const meaning = planMeaning(route);
    this.historyKey.textContent = timeS <= 0 ? "Result" : "Flown";
    this.root.dataset.planMeaning = meaning;
    if (this.key.textContent !== PLAN_LABELS[meaning]) this.key.textContent = PLAN_LABELS[meaning];
    this.key.title = meaning === "recorded"
      ? "Future recorded result for comparison, not knowledge available to the reactive planner."
      : meaning === "local" ? "Latest recorded local plan; replaced when the UAV replans."
      : meaning === "scheduled" ? "Preplanned space-time route checked against known traffic schedules."
      : "Fixed route around static obstacles; playback does not replan.";
    if (changed) {
      this.routeId = route.id; this.root.dataset.routeId = route.id;
      this.name.textContent = `UAV ${String(index + 1).padStart(2, "0")}`;
      this.root.title = route.label;
      this.root.style.setProperty("--hud-route", routeColorCSS(index));
    }
    const values = [readout.altitudeM.toFixed(1), readout.speedMps.toFixed(1), String(Math.round(readout.headingDeg) % 360).padStart(3, "0")];
    this.fields.forEach((field, index) => { if (field.textContent !== values[index]) field.textContent = values[index]!; });
    this.root.style.setProperty("--speed-angle", `${Math.max(0, Math.min(1, readout.speedMps / DIAL_LIMITS.speed)) * 270 - 135}deg`);
    this.root.style.setProperty("--altitude-angle", `${Math.max(0, Math.min(1, readout.altitudeM / DIAL_LIMITS.altitude)) * 270 - 135}deg`);
    this.root.style.setProperty("--heading", `${readout.headingDeg}deg`);
  }
  dispose(): void { this.root.remove(); }
}
