import * as THREE from "three";
import { addCityBuildings, addCityContext, type CityScenario } from "./city-scene";
import { disposeRenderObject } from "./render-resources";

/** Owns the expensive, mission-independent city and its online tile cache. */
export class RetainedCity {
  readonly group = new THREE.Group();
  readonly buildings = new THREE.Group();
  private key?: string;
  private bounds = new THREE.Box3();
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
      this.group.add(this.buildings);
      this.key = key;
    }
    host.add(this.group);
    return this.bounds.clone();
  }

  dispose(): void { disposeRenderObject(this.group); this.group.clear(); this.group.removeFromParent(); this.key = undefined; }
}
