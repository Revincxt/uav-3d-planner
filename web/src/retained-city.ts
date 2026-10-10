import * as THREE from "three";
import { addCityBuildings, addCityContext, type CityScenario, type CityBox } from "./city-scene";
import { BuildingOcclusion } from "./building-occlusion";
import type { DisplayCamera } from "./camera-scale";
import { disposeRenderObject } from "./render-resources";

/** Owns the expensive, mission-independent city and its online tile cache. */
export class RetainedCity {
  readonly group = new THREE.Group();
  readonly buildings = new THREE.Group();
  private key?: string;
  private bounds = new THREE.Box3();
  private sourceBuildings: readonly CityBox[] = [];
  private occlusion?: BuildingOcclusion;
  private readonly raycaster = new THREE.Raycaster();
  private readonly screenPoint = new THREE.Vector3();
  constructor() { this.group.name = "retained-planning-city"; }

  mount(host: THREE.Group, scenario: CityScenario, onReady?: (texture?: THREE.Texture) => void): THREE.Box3 {
    const key = scenario.city
      ? JSON.stringify([scenario.city.id, scenario.city.sourceSha256, scenario.city.planningRegion, scenario.bounds])
      : scenario.id;
    if (key !== this.key) {
      disposeRenderObject(this.group);
      this.group.clear();
      this.buildings.clear();
      this.bounds = addCityContext(this.group, scenario, onReady);
      addCityBuildings(this.buildings, scenario.buildings ?? []);
      this.sourceBuildings = scenario.buildings ?? []; this.occlusion = undefined;
      this.group.add(this.buildings);
      this.key = key;
    }
    host.add(this.group);
    return this.bounds.clone();
  }

  blocksSight(camera: DisplayCamera, point: THREE.Vector3): boolean {
    if (!this.buildings.visible || !this.group.visible) return false;
    this.occlusion ??= new BuildingOcclusion(this.sourceBuildings);
    this.screenPoint.copy(point).project(camera);
    this.raycaster.setFromCamera(new THREE.Vector2(this.screenPoint.x, this.screenPoint.y), camera);
    return this.occlusion.blocks(this.raycaster.ray, point.distanceTo(this.raycaster.ray.origin));
  }

  dispose(): void { disposeRenderObject(this.group); this.group.clear(); this.group.removeFromParent(); this.key = undefined; this.occlusion = undefined; this.sourceBuildings = []; }
}
