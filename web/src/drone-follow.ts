import * as THREE from "three";
import type { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { MIN_MAP_ELEVATION_RAD } from "./map-navigation";
import type { CityBox } from "./city-scene";

const CHASE_DISTANCE_M = 12 / Math.cos(MIN_MAP_ELEVATION_RAD);
const CAMERA_GRID_M = 64;

/** Close, centered chase camera; the overview and its orbit state remain untouched. */
export class DroneFollow {
  readonly camera = new THREE.PerspectiveCamera(70, 1, 0.15, 8000);
  readonly sky = new THREE.Color(0xe9f0f6);
  routeId: string | null = null;
  private previousControlsEnabled = true;
  private clock = 0;
  private traceTime?: number;
  private yaw = 0;
  private boomDistance = CHASE_DISTANCE_M;
  private buildingSource?: readonly CityBox[];
  private readonly blockers = new Map<string, THREE.Box3[]>();
  private readonly rear = new THREE.Vector3();
  private readonly ray = new THREE.Ray();
  private readonly hit = new THREE.Vector3();
  private readonly seen = new Set<THREE.Box3>();
  onChange?: (enabled: boolean) => void;
  constructor(private readonly controls: OrbitControls) {}

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
    this.resize(width, height);
    this.place(position, heading, true, 0);
    this.onChange?.(true);
  }

  resize(width: number, height: number): boolean {
    if (!this.routeId) return false;
    this.camera.aspect = Math.max(1, width) / Math.max(1, height);
    this.camera.updateProjectionMatrix(); return true;
  }

  update(position: THREE.Vector3, heading: THREE.Vector3, timeS: number, now = performance.now()): boolean {
    if (!this.routeId) return false;
    const seeking = this.traceTime === undefined || timeS < this.traceTime || Math.abs(timeS - this.traceTime) > 2;
    const dt = Math.max(0, Math.min(0.1, (now - this.clock) / 1000));
    this.clock = now; this.traceTime = timeS;
    return this.place(position, heading, seeking, dt);
  }

  private place(position: THREE.Vector3, heading: THREE.Vector3, snap: boolean, dt: number): boolean {
    const desiredYaw = heading.x * heading.x + heading.z * heading.z > 1e-10
      ? Math.atan2(-heading.x, -heading.z) : this.yaw;
    const turn = Math.atan2(Math.sin(desiredYaw - this.yaw), Math.cos(desiredYaw - this.yaw));
    const turning = !snap && Math.abs(turn) > 1e-5;
    this.yaw += turning ? turn * (1 - Math.exp(-dt * 8)) : turn;
    // Orbit only the short camera boom, not the recorded aircraft position.
    const rear = this.rear.set(Math.sin(this.yaw) * Math.cos(MIN_MAP_ELEVATION_RAD),
      Math.sin(MIN_MAP_ELEVATION_RAD), Math.cos(this.yaw) * Math.cos(MIN_MAP_ELEVATION_RAD));
    const safeDistance = this.clearBoomDistance(position, rear);
    const extending = !snap && safeDistance - this.boomDistance > 1e-5;
    // Retract immediately to stay clear of walls; ease back out once the obstruction clears.
    this.boomDistance = extending ? this.boomDistance + (safeDistance - this.boomDistance) * (1 - Math.exp(-dt * 8)) : safeDistance;
    this.camera.position.copy(position).addScaledVector(rear, this.boomDistance);
    this.camera.rotation.set(-MIN_MAP_ELEVATION_RAD, this.yaw, 0, "YXZ");
    this.camera.updateMatrixWorld(true);
    return turning || extending;
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
    this.controls.enabled = this.previousControlsEnabled;
    this.onChange?.(false);
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
