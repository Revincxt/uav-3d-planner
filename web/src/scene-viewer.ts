import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

import { enuToThree } from "./coordinates";
import type { DemoScenario, PlannerId, Vec3 } from "./schema";

type PathMode = "raw" | "smoothed";
type ViewPreset = "isometric" | "top" | "reset";

const PATH_STYLES: Record<
  PlannerId,
  { color: number; dashed: boolean; dashScale: number }
> = {
  "astar-3d": { color: 0x2d64a8, dashed: false, dashScale: 1 },
  "lazy-theta-star": { color: 0x278064, dashed: true, dashScale: 1.2 },
  "rrt-star": { color: 0xb6543c, dashed: true, dashScale: 0.45 },
};

function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    if ("geometry" in child && child.geometry instanceof THREE.BufferGeometry) {
      child.geometry.dispose();
    }
    if ("material" in child) {
      const material = child.material as THREE.Material | THREE.Material[];
      (Array.isArray(material) ? material : [material]).forEach((item) => item.dispose());
    }
  });
}

export class SceneViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera();
  private readonly controls: OrbitControls;
  private readonly content = new THREE.Group();
  private readonly lineMaterials: LineMaterial[] = [];
  private readonly resizeObserver: ResizeObserver;
  private scenario: DemoScenario | null = null;
  private mode: PathMode = "smoothed";
  private visiblePlanners = new Set<PlannerId>();

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.container.append(this.renderer.domElement);
    this.scene.background = new THREE.Color(0xf5f4ef);
    this.scene.add(this.content);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xc4c0b3, 2.1));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(-30, 80, 35);
    this.scene.add(sun);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
    this.controls.addEventListener("change", () => this.render());
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
  }

  setScenario(scenario: DemoScenario, visible: Set<PlannerId>, mode: PathMode): void {
    this.clearContent();
    this.scenario = scenario;
    this.visiblePlanners = new Set(visible);
    this.mode = mode;
    this.addGround(scenario);
    scenario.buildings.forEach((building) => {
      const size = building.max.map((value, index) => value - building.min[index]!) as Vec3;
      const center = building.min.map((value, index) => value + size[index]! / 2) as Vec3;
      const geometry = new THREE.BoxGeometry(size[0], size[2], size[1]);
      const material = new THREE.MeshStandardMaterial({ color: 0x9c9b96, roughness: 0.95 });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.fromArray(enuToThree(center));
      this.content.add(mesh);
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geometry),
        new THREE.LineBasicMaterial({ color: 0x686864, transparent: true, opacity: 0.55 }),
      );
      edges.position.copy(mesh.position);
      this.content.add(edges);
    });
    scenario.noFlyZones.forEach((zone) => this.addNoFlyZone(zone));
    this.addEndpoint(scenario.start, "start");
    this.addEndpoint(scenario.goal, "goal");
    this.addPaths();
    this.setView("isometric");
    this.render();
  }

  setPathMode(mode: PathMode): void {
    if (!this.scenario || this.mode === mode) return;
    this.setScenario(this.scenario, this.visiblePlanners, mode);
  }

  setPlannerVisibility(planners: Set<PlannerId>): void {
    if (!this.scenario) return;
    this.setScenario(this.scenario, planners, this.mode);
  }

  setView(preset: ViewPreset): void {
    if (!this.scenario) return;
    const min = this.scenario.bounds.min;
    const max = this.scenario.bounds.max;
    const center = new THREE.Vector3((min[0] + max[0]) / 2, 0, -(min[1] + max[1]) / 2);
    const span = Math.max(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
    if (preset === "top") {
      this.camera.position.set(center.x, span * 2, center.z + 0.001);
      this.camera.up.set(0, 0, -1);
    } else {
      this.camera.position.set(center.x + span * 1.15, span * 0.92, center.z + span * 1.15);
      this.camera.up.set(0, 1, 0);
    }
    this.controls.target.set(center.x, max[2] * 0.28, center.z);
    this.controls.update();
    this.resize();
  }

  dispose(): void {
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.clearContent();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private clearContent(): void {
    this.lineMaterials.length = 0;
    for (const child of [...this.content.children]) {
      this.content.remove(child);
      disposeObject(child);
    }
  }

  private addGround(scenario: DemoScenario): void {
    const width = scenario.bounds.max[0] - scenario.bounds.min[0];
    const depth = scenario.bounds.max[1] - scenario.bounds.min[1];
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(width, depth),
      new THREE.MeshStandardMaterial({ color: 0xe9e8e2, roughness: 1 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.set(
      scenario.bounds.min[0] + width / 2,
      scenario.bounds.min[2],
      -(scenario.bounds.min[1] + depth / 2),
    );
    this.content.add(ground);
    const grid = new THREE.GridHelper(Math.max(width, depth), 10, 0xb7b5ad, 0xd2d0c9);
    grid.position.copy(ground.position);
    grid.position.y += 0.02;
    this.content.add(grid);
  }

  private addNoFlyZone(zone: DemoScenario["noFlyZones"][number]): void {
    const height = zone.zMaxM - zone.zMinM;
    const geometry = new THREE.CylinderGeometry(zone.radiusM, zone.radiusM, height, 48, 1, true);
    const mesh = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color: 0xb84e46,
        transparent: true,
        opacity: 0.14,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    mesh.position.set(zone.center[0], zone.zMinM + height / 2, -zone.center[1]);
    this.content.add(mesh);
    const wireframe = new THREE.LineSegments(
      new THREE.WireframeGeometry(geometry),
      new THREE.LineBasicMaterial({ color: 0x9d3f39, transparent: true, opacity: 0.5 }),
    );
    wireframe.position.copy(mesh.position);
    this.content.add(wireframe);
  }

  private addEndpoint(point: Vec3, kind: "start" | "goal"): void {
    const geometry =
      kind === "start"
        ? new THREE.SphereGeometry(1.8, 20, 12)
        : new THREE.ConeGeometry(2.2, 4.5, 5);
    const material = new THREE.MeshStandardMaterial({
      color: kind === "start" ? 0x202224 : 0xf0eee7,
      emissive: kind === "start" ? 0x111111 : 0x222222,
    });
    const marker = new THREE.Mesh(geometry, material);
    marker.position.fromArray(enuToThree(point));
    this.content.add(marker);
  }

  private addPaths(): void {
    if (!this.scenario) return;
    for (const result of this.scenario.results) {
      if (!result.paths || !this.visiblePlanners.has(result.plannerId)) continue;
      const path = result.paths[this.mode];
      const positions = path.flatMap((point) => enuToThree(point));
      const style = PATH_STYLES[result.plannerId];
      const geometry = new LineGeometry();
      geometry.setPositions(positions);
      const material = new LineMaterial({
        color: style.color,
        linewidth: 3,
        worldUnits: false,
        dashed: style.dashed,
        dashSize: 7 * style.dashScale,
        gapSize: 4,
      });
      this.lineMaterials.push(material);
      const line = new Line2(geometry, material);
      line.computeLineDistances();
      this.content.add(line);
    }
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    const aspect = width / height;
    const span = this.scenario
      ? Math.max(
          this.scenario.bounds.max[0] - this.scenario.bounds.min[0],
          this.scenario.bounds.max[1] - this.scenario.bounds.min[1],
        )
      : 100;
    const vertical = span * 0.72;
    this.camera.left = (-vertical * aspect) / 2;
    this.camera.right = (vertical * aspect) / 2;
    this.camera.top = vertical / 2;
    this.camera.bottom = -vertical / 2;
    this.camera.near = 0.1;
    this.camera.far = span * 10;
    this.camera.updateProjectionMatrix();
    this.lineMaterials.forEach((material) => material.resolution.set(width, height));
    this.render();
  }

  private render(): void {
    this.renderer.render(this.scene, this.camera);
  }
}

