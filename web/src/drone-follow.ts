import * as THREE from "three";
import type { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { MIN_MAP_ELEVATION_RAD } from "./map-navigation";
import type { CityBox } from "./city-scene";
import { displayHeading, timedPosition, type RouteWaypoint } from "./route-overview";
import { BASE_PLAYBACK_SPEED, PLAYBACK_RATES } from "./playback-clock";

const CHASE_DISTANCE_M = 8 / Math.cos(MIN_MAP_ELEVATION_RAD);
const CAMERA_GRID_M = 64;
const YAW_RESPONSE = 8, MAX_YAW_RATE = Math.PI, MAX_YAW_ACCELERATION = 3 * Math.PI;
const MAX_TRACE_RATE = BASE_PLAYBACK_SPEED * Math.max(...PLAYBACK_RATES);
const PROBE_HORIZONS = [.12, .24] as const;

/** Blue evening sky with a westward sunset and thin clouds, baked once per viewer. */
function createFollowSky(): THREE.DataTexture {
  const width = 512, height = 256, pixels = new Uint8Array(width * height * 4);
  const stops = [
    [-90, 0x9daeb9], [-8, 0xb1c2ce], [0, 0xc8d5dc], [6, 0xb0c7d9],
    [16, 0x88aac8], [36, 0x5e87ad], [90, 0x355f8a],
  ].map(([elevation, hex]) => ({ elevation: elevation!, color: new THREE.Color(hex!) }));
  const glow = new THREE.Color(0xffd09e), cloudShade = new THREE.Color(0x91a3b4), cloudLight = new THREE.Color(0xf5e2c9);
  const base = new THREE.Color(), color = new THREE.Color();
  const columns = Array.from({ length: width }, (_, column) => {
    const longitude = column / width * Math.PI * 2 - Math.PI;
    const sunward = ((1 - Math.cos(longitude)) / 2) ** 6;
    return { sunward, wave4: longitude * 4, wave9: longitude * 9, wave17: longitude * 17,
      warp: Math.sin(longitude * 2), cloud: cloudShade.clone().lerp(cloudLight, .2 + .55 * sunward) };
  });
  for (let row = 0; row < height; row++) {
    const elevation = row / (height - 1) * 180 - 90;
    const upper = stops.findIndex(stop => stop.elevation >= elevation);
    const left = stops[Math.max(0, upper - 1)]!, right = stops[upper]!;
    const fraction = right.elevation === left.elevation ? 0 : (elevation - left.elevation) / (right.elevation - left.elevation);
    base.copy(left.color).lerp(right.color, THREE.MathUtils.smoothstep(fraction, 0, 1));
    const haze = Math.exp(-(((elevation - 1) / 7.5) ** 2));
    const cloudBand = Math.exp(-(((elevation - 13) / 7) ** 2)) + .35 * Math.exp(-(((elevation - 27) / 5) ** 2));
    for (let column = 0; column < width; column++) {
      const { sunward, wave4, wave9, wave17, warp, cloud } = columns[column]!;
      // The warmer glow stays in the geographic west as the aircraft turns.
      color.copy(base).lerp(glow, sunward * haze * .65);
      if (cloudBand > .005) {
        const wisps = .5 + .23 * Math.sin(wave4 + elevation * .75 + warp)
          + .14 * Math.sin(wave9 - elevation * 1.3) + .08 * Math.sin(wave17 + elevation * 2.1);
        color.lerp(cloud, THREE.MathUtils.smoothstep(wisps, .52, .82) * cloudBand * (.22 + .18 * sunward));
      }
      color.convertLinearToSRGB();
      const offset = (row * width + column) * 4;
      pixels[offset] = Math.round(color.r * 255);
      pixels[offset + 1] = Math.round(color.g * 255);
      pixels[offset + 2] = Math.round(color.b * 255);
      pixels[offset + 3] = 255;
    }
  }
  const sky = new THREE.DataTexture(pixels, width, height);
  sky.colorSpace = THREE.SRGBColorSpace;
  sky.mapping = THREE.EquirectangularReflectionMapping;
  sky.wrapS = THREE.RepeatWrapping;
  sky.minFilter = sky.magFilter = THREE.LinearFilter;
  sky.needsUpdate = true;
  return sky;
}

/** Small, fixed sky points stay crisp without enlarging the panorama texture. */
function createDaylightStars(): THREE.Points<THREE.BufferGeometry, THREE.PointsMaterial> {
  const positions = [];
  for (let i = 0; i < 36; i++) {
    const longitude = i * Math.PI * (3 - Math.sqrt(5));
    const elevation = THREE.MathUtils.degToRad(i % 3 === 0 ? 24 + i * 17 % 37 : 9 + i * 11 % 14);
    positions.push(7000 * Math.cos(elevation) * Math.cos(longitude),
      7000 * Math.sin(elevation), 7000 * Math.cos(elevation) * Math.sin(longitude));
  }
  const pixels = new Uint8Array(16 * 16 * 4);
  for (let y = 0; y < 16; y++) for (let x = 0; x < 16; x++) {
    const offset = (y * 16 + x) * 4, radiusSq = ((x + .5) / 8 - 1) ** 2 + ((y + .5) / 8 - 1) ** 2;
    pixels[offset] = pixels[offset + 1] = pixels[offset + 2] = 255;
    pixels[offset + 3] = Math.round(255 * Math.max(0, 1 - radiusSq) ** 2);
  }
  const sprite = new THREE.DataTexture(pixels, 16, 16);
  sprite.minFilter = sprite.magFilter = THREE.LinearFilter; sprite.needsUpdate = true;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const stars = new THREE.Points(geometry, new THREE.PointsMaterial({ map: sprite,
    color: 0xf3f6ff, size: 2.2, sizeAttenuation: false, transparent: true, opacity: .68,
    depthTest: true, depthWrite: false, toneMapped: false }));
  stars.name = "follow-daylight-stars"; stars.visible = false; stars.renderOrder = -1;
  return stars;
}

/** Close, centered chase camera; the overview and its orbit state remain untouched. */
export class DroneFollow {
  readonly camera = new THREE.PerspectiveCamera(70, 1, 0.15, 8000);
  readonly sky = createFollowSky();
  readonly stars = createDaylightStars();
  routeId: string | null = null;
  private previousControlsEnabled = true;
  private clock = 0;
  private traceTime?: number;
  private yaw = 0;
  private yawVelocity = 0;
  private headingYaw = 0;
  private boomDistance = CHASE_DISTANCE_M;
  private buildingSource?: readonly CityBox[];
  private readonly blockers = new Map<string, THREE.Box3[]>();
  private readonly rear = new THREE.Vector3();
  private readonly ahead = new THREE.Vector3();
  private readonly aheadRear = new THREE.Vector3();
  private readonly sample: [number, number, number] = [0, 0, 0];
  private readonly ray = new THREE.Ray();
  private readonly hit = new THREE.Vector3();
  private readonly seen = new Set<THREE.Box3>();
  onChange?: (enabled: boolean) => void;
  constructor(private readonly controls: OrbitControls) {
    this.stars.layers.set(1); this.camera.layers.enable(1);
  }

  setBuildings(buildings: readonly CityBox[]): void {
    if (buildings === this.buildingSource) return;
    this.buildingSource = buildings; this.blockers.clear();
    for (const building of buildings) {
      const box = new THREE.Box3(new THREE.Vector3(building.min[0], building.min[2], -building.max[1]!),
        new THREE.Vector3(building.max[0], building.max[2], -building.min[1]!));
      for (let x = Math.floor(box.min.x / CAMERA_GRID_M); x <= Math.floor(box.max.x / CAMERA_GRID_M); x++)
        for (let z = Math.floor(box.min.z / CAMERA_GRID_M); z <= Math.floor(box.max.z / CAMERA_GRID_M); z++) {
          const key = `${x},${z}`, cell = this.blockers.get(key) ?? [];
          cell.push(box); this.blockers.set(key, cell);
        }
    }
  }

  start(id: string, position: THREE.Vector3, heading: THREE.Vector3, width: number, height: number): void {
    if (this.routeId === null) this.previousControlsEnabled = this.controls.enabled;
    this.routeId = id; this.clock = performance.now(); this.traceTime = undefined;
    this.controls.enabled = false;
    this.stars.visible = true;
    this.resize(width, height);
    this.place(position, heading, true, 0, 0, 0);
    this.onChange?.(true);
  }

  resize(width: number, height: number): boolean {
    if (!this.routeId) return false;
    this.camera.aspect = Math.max(1, width) / Math.max(1, height);
    this.camera.updateProjectionMatrix(); return true;
  }

  update(position: THREE.Vector3, heading: THREE.Vector3, timeS: number, now = performance.now(), path?: readonly RouteWaypoint[]): boolean {
    if (!this.routeId) return false;
    const elapsed = Math.max(0, (now - this.clock) / 1000);
    const advance = this.traceTime === undefined ? 0 : timeS - this.traceTime;
    // Normal 2× playback advances eight trace seconds per wall second. A slow
    // render frame must not be mistaken for a timeline seek and snap the camera.
    const seeking = this.traceTime === undefined || advance < 0 || advance > Math.max(2, elapsed * MAX_TRACE_RATE + .1);
    const rate = !seeking && elapsed > 0 ? Math.min(MAX_TRACE_RATE, Math.max(0, advance / elapsed)) : 0;
    this.clock = now; this.traceTime = timeS;
    return this.place(position, heading, seeking, elapsed, timeS, rate, path);
  }

  private place(position: THREE.Vector3, heading: THREE.Vector3, snap: boolean, elapsed: number, timeS: number, rate: number, path?: readonly RouteWaypoint[]): boolean {
    const dt = Math.min(0.1, elapsed);
    const desiredYaw = heading.x * heading.x + heading.z * heading.z > 1e-10
      ? Math.atan2(-heading.x, -heading.z) : this.yaw;
    const difference = (to: number, from: number) => Math.atan2(Math.sin(to - from), Math.cos(to - from));
    const error = () => difference(desiredYaw, this.yaw);
    if (snap) { this.yaw += error(); this.yawVelocity = 0; }
    else {
      // Match the aircraft's turn rate, then brake before the upcoming heading.
      // Preview only the current stopping time, avoiding a delayed counter-swing.
      const turn = difference(desiredYaw, this.headingYaw);
      const headingRate = elapsed > 0 ? THREE.MathUtils.clamp(turn / elapsed, -MAX_YAW_RATE, MAX_YAW_RATE) : 0;
      const previewS = Math.abs(this.yawVelocity) / MAX_YAW_ACCELERATION;
      const future = path?.length && rate > 0 ? displayHeading(path, timeS + previewS * rate) : null;
      const futureTurn = future ? difference(Math.atan2(-future.x, -future.z), desiredYaw) : headingRate * previewS;
      const steps = Math.max(1, Math.ceil(dt * 120)), stepS = dt / steps;
      for (let i = 0; i < steps; i++) {
        const target = this.headingYaw + turn * (i + .5) / steps;
        const error = difference(target, this.yaw);
        const correction = Math.sign(error) * Math.min(YAW_RESPONSE * Math.abs(error), Math.sqrt(2 * MAX_YAW_ACCELERATION * Math.abs(error)));
        let targetVelocity = THREE.MathUtils.clamp(headingRate + correction, -MAX_YAW_RATE, MAX_YAW_RATE);
        const remaining = difference(desiredYaw, this.yaw) + futureTurn;
        // Reserve this substep's travel before applying the stopping-distance bound.
        const brakingSpeed = Math.sqrt(2 * MAX_YAW_ACCELERATION * Math.max(0, Math.abs(remaining) - Math.abs(this.yawVelocity) * stepS));
        targetVelocity = targetVelocity * remaining <= 0 ? 0 : Math.sign(targetVelocity) * Math.min(Math.abs(targetVelocity), brakingSpeed);
        const previousVelocity = this.yawVelocity;
        this.yawVelocity += THREE.MathUtils.clamp(targetVelocity - this.yawVelocity, -MAX_YAW_ACCELERATION * stepS, MAX_YAW_ACCELERATION * stepS);
        this.yaw += (previousVelocity + this.yawVelocity) * .5 * stepS;
      }
      if (Math.abs(error()) < 1e-5 && Math.abs(this.yawVelocity) < 1e-4) { this.yaw += error(); this.yawVelocity = 0; }
    }
    this.headingYaw = desiredYaw;
    const turning = !snap && (Math.abs(error()) > 1e-5 || Math.abs(this.yawVelocity) > 1e-4);
    // Orbit only the short camera boom, not the recorded aircraft position.
    const rear = this.rear.set(Math.sin(this.yaw) * Math.cos(MIN_MAP_ELEVATION_RAD),
      Math.sin(MIN_MAP_ELEVATION_RAD), Math.cos(this.yaw) * Math.cos(MIN_MAP_ELEVATION_RAD));
    const safeDistance = this.clearBoomDistance(position, rear);
    let desiredDistance = safeDistance;
    if (!snap && path?.length && rate > 0) {
      for (const horizon of PROBE_HORIZONS) {
        const point = timedPosition(path, timeS + horizon * rate, this.sample);
        this.ahead.set(point[0], point[2], -point[1]);
        const yaw = this.yaw + this.yawVelocity * horizon;
        this.aheadRear.set(Math.sin(yaw) * Math.cos(MIN_MAP_ELEVATION_RAD), Math.sin(MIN_MAP_ELEVATION_RAD), Math.cos(yaw) * Math.cos(MIN_MAP_ELEVATION_RAD));
        desiredDistance = Math.min(desiredDistance, this.clearBoomDistance(this.ahead, this.aheadRear));
      }
    }
    const previousDistance = this.boomDistance;
    const response = desiredDistance < previousDistance ? 12 : 8;
    // Anticipate walls along the recorded flight and ease inward before reaching
    // them. The current collision bound always wins, including on seeks.
    this.boomDistance = snap || Math.abs(desiredDistance - previousDistance) < 1e-5 ? desiredDistance
      : Math.min(safeDistance, previousDistance + (desiredDistance - previousDistance) * -Math.expm1(-dt * response));
    const movingBoom = Math.abs(this.boomDistance - previousDistance) > 1e-5;
    this.camera.position.copy(position).addScaledVector(rear, this.boomDistance);
    this.camera.rotation.set(-MIN_MAP_ELEVATION_RAD, this.yaw, 0, "YXZ");
    this.camera.updateMatrixWorld(true);
    this.stars.position.copy(this.camera.position);
    return turning || movingBoom;
  }

  private clearBoomDistance(position: THREE.Vector3, rear: THREE.Vector3): number {
    const ray = this.ray.set(position, rear), hit = this.hit, seen = this.seen;
    seen.clear();
    let distance = CHASE_DISTANCE_M;
    for (let x = Math.floor((position.x - CHASE_DISTANCE_M) / CAMERA_GRID_M); x <= Math.floor((position.x + CHASE_DISTANCE_M) / CAMERA_GRID_M); x++)
      for (let z = Math.floor((position.z - CHASE_DISTANCE_M) / CAMERA_GRID_M); z <= Math.floor((position.z + CHASE_DISTANCE_M) / CAMERA_GRID_M); z++) {
        for (const box of this.blockers.get(`${x},${z}`) ?? []) {
          if (seen.has(box)) continue;
          seen.add(box);
          if (ray.intersectBox(box, hit)) distance = Math.min(distance, Math.max(0.25, hit.distanceTo(position) - 0.5));
        }
      }
    return distance;
  }

  stop(): void {
    if (this.routeId === null) return;
    this.routeId = null; this.traceTime = undefined;
    this.stars.visible = false;
    this.controls.enabled = this.previousControlsEnabled;
    this.onChange?.(false);
  }

  dispose(): void {
    this.stop(); this.sky.dispose();
    this.stars.removeFromParent(); this.stars.geometry.dispose();
    this.stars.material.map!.dispose(); this.stars.material.dispose();
  }
}

interface FollowViewer {
  setFollowRoute(id: string | null): void;
  onFollowChange?: (enabled: boolean) => void;
}

export function mountFollowControls(viewer: FollowViewer | null, select: HTMLSelectElement, onChange?: (enabled: boolean) => void): void {
  const toggle = document.querySelector<HTMLButtonElement>("#follow-toggle");
  const target = document.querySelector<HTMLSelectElement>("#follow-target");
  if (!toggle || !target) return;
  toggle.disabled = target.disabled = !viewer;
  for (const [index, source] of [...select.options].entries()) {
    const option = document.createElement("option"); option.value = source.value;
    option.textContent = `UAV ${index + 1}`; option.title = source.text; target.append(option);
  }
  target.value = select.value;
  let enabled = false;
  let previousView: [HTMLElement, string | null][] = [];
  if (viewer) viewer.onFollowChange = active => {
    if (active && !enabled) {
      previousView = [...document.querySelectorAll<HTMLElement>("[data-view][aria-pressed]")].map(button => [button, button.getAttribute("aria-pressed")]);
      previousView.forEach(([button]) => button.setAttribute("aria-pressed", "false"));
    } else if (!active && enabled) {
      previousView.forEach(([button, pressed]) => button.setAttribute("aria-pressed", pressed ?? "false"));
    }
    enabled = active; toggle.setAttribute("aria-pressed", String(active));
    toggle.textContent = active ? "Exit follow" : "Follow";
    onChange?.(active);
  };
  toggle.addEventListener("click", () => viewer?.setFollowRoute(enabled ? null : target.value));
  target.addEventListener("change", () => {
    select.value = target.value; select.dispatchEvent(new Event("change", { bubbles: true }));
  });
  select.addEventListener("change", () => {
    target.value = select.value;
    if (enabled) viewer?.setFollowRoute(select.value);
  });
  document.addEventListener("keydown", event => {
    if (event.key === "Escape" && enabled) viewer?.setFollowRoute(null);
  });
}
