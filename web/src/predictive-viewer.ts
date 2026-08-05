import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

import type {
  PredictiveFrame,
  PredictiveRun,
  PredictiveScenario,
  StaticNoFlyZone,
  Vec3,
} from "./predictive-schema";

type ViewPreset = "isometric" | "top" | "reset";

interface ZoneVisual {
  fill: THREE.Mesh<THREE.CylinderGeometry, THREE.MeshStandardMaterial>;
  wireframe: THREE.LineSegments<THREE.WireframeGeometry, THREE.LineBasicMaterial>;
}

interface ThemePalette {
  background: number;
  ground: number;
  gridMajor: number;
  gridMinor: number;
  building: number;
  buildingEdge: number;
  foreground: number;
  staticZone: number;
  temporaryZone: number;
  movingSphere: number;
  scheduledPath: number;
  currentPath: number;
  executedPath: number;
  goal: number;
}

const LIGHT_PALETTE: ThemePalette = {
  background: 0xf5f4ef,
  ground: 0xe9e8e2,
  gridMajor: 0xaaa8a0,
  gridMinor: 0xd0cec6,
  building: 0x999994,
  buildingEdge: 0x5f605d,
  foreground: 0x202224,
  staticZone: 0xa8423c,
  temporaryZone: 0xb06f28,
  movingSphere: 0x79558f,
  scheduledPath: 0x686b70,
  currentPath: 0x2d64a8,
  executedPath: 0x278064,
  goal: 0xeceae3,
};

const DARK_PALETTE: ThemePalette = {
  background: 0x20211f,
  ground: 0x292a27,
  gridMajor: 0x666760,
  gridMinor: 0x444540,
  building: 0x74756f,
  buildingEdge: 0xc0bfb7,
  foreground: 0xecebe5,
  staticZone: 0xd97a70,
  temporaryZone: 0xd9a15f,
  movingSphere: 0xb99bcc,
  scheduledPath: 0xa9abb2,
  currentPath: 0x7aa9e3,
  executedPath: 0x73bd9e,
  goal: 0x343531,
};

function enuToThree(point: Vec3): Vec3 {
  return [point[0], point[2], -point[1]];
}

function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    if ("geometry" in child && child.geometry instanceof THREE.BufferGeometry) {
      child.geometry.dispose();
    }
    if ("material" in child) {
      const materials = child.material as THREE.Material | THREE.Material[];
      for (const material of Array.isArray(materials) ? materials : [materials]) {
        material.dispose();
      }
    }
  });
}

function pointsGeometry(points: Vec3[]): THREE.BufferGeometry {
  const geometry = new THREE.BufferGeometry();
  geometry.setFromPoints(points.map((point) => new THREE.Vector3(...enuToThree(point))));
  return geometry;
}

export class PredictiveViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera();
  private readonly controls: OrbitControls;
  private readonly content = new THREE.Group();
  private readonly resizeObserver: ResizeObserver;
  private readonly colorScheme = window.matchMedia("(prefers-color-scheme: dark)");
  private readonly onColorSchemeChange = (): void => this.rebuild();
  private scenario: PredictiveScenario | null = null;
  private run: PredictiveRun | null = null;
  private frame: PredictiveFrame | null = null;
  private temporaryZones = new Map<string, ZoneVisual>();
  private movingSpheres = new Map<string, THREE.Mesh>();
  private vehicle: THREE.Mesh | null = null;
  private scheduledPath: THREE.Line | null = null;
  private currentPath: THREE.Line | null = null;
  private executedPath: THREE.Line | null = null;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.container.append(this.renderer.domElement);
    this.scene.add(this.content);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
    this.controls.addEventListener("change", () => this.render());
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.colorScheme.addEventListener("change", this.onColorSchemeChange);
  }

  setScenario(scenario: PredictiveScenario): void {
    this.scenario = scenario;
    this.run = null;
    this.frame = null;
    this.buildScene();
    this.setView("isometric");
  }

  setRun(run: PredictiveRun): void {
    this.run = run;
    this.replaceLine("scheduled", run.timedPath.map((waypoint) => waypoint.position));
    this.render();
  }

  setFrame(frame: PredictiveFrame): void {
    if (!this.scenario) return;
    this.frame = frame;
    this.vehicle?.position.fromArray(enuToThree(frame.vehicle));

    const activeZones = new Set(frame.activeTemporaryZoneIds);
    for (const [id, visual] of this.temporaryZones) {
      const active = activeZones.has(id);
      visual.fill.material.opacity = active ? 0.18 : 0.025;
      visual.wireframe.material.opacity = active ? 0.78 : 0.22;
    }

    const states = new Map(frame.movingSpheres.map((state) => [state.id, state]));
    for (const [id, mesh] of this.movingSpheres) {
      const state = states.get(id);
      mesh.visible = state !== undefined;
      if (state) mesh.position.fromArray(enuToThree(state.position));
    }

    this.replaceLine("current", frame.path);
    this.replaceLine("executed", frame.executedPath);
    this.render();
  }

  setView(preset: ViewPreset): void {
    if (!this.scenario) return;
    const { min, max } = this.scenario.bounds;
    const center = new THREE.Vector3((min[0] + max[0]) / 2, 0, -(min[1] + max[1]) / 2);
    const span = Math.max(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
    if (preset === "top") {
      this.camera.position.set(center.x, span * 2, center.z + 0.001);
      this.camera.up.set(0, 0, -1);
    } else {
      this.camera.position.set(center.x + span * 1.12, span * 0.9, center.z + span * 1.12);
      this.camera.up.set(0, 1, 0);
    }
    this.controls.target.set(center.x, min[2] + (max[2] - min[2]) * 0.3, center.z);
    this.controls.update();
    this.resize();
  }

  dispose(): void {
    this.resizeObserver.disconnect();
    this.colorScheme.removeEventListener("change", this.onColorSchemeChange);
    this.controls.dispose();
    this.clearContent();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private get palette(): ThemePalette {
    return this.colorScheme.matches ? DARK_PALETTE : LIGHT_PALETTE;
  }

  private rebuild(): void {
    if (!this.scenario) return;
    const run = this.run;
    const frame = this.frame;
    this.buildScene();
    if (run) this.setRun(run);
    if (frame) this.setFrame(frame);
  }

  private buildScene(): void {
    if (!this.scenario) return;
    this.clearContent();
    const palette = this.palette;
    this.scene.background = new THREE.Color(palette.background);
    this.scene.add(new THREE.HemisphereLight(0xffffff, palette.ground, 2.05));
    const sun = new THREE.DirectionalLight(0xffffff, 2.1);
    sun.position.set(-30, 90, 40);
    sun.name = "predictive-sun";
    this.scene.add(sun);

    this.addGround();
    for (const building of this.scenario.buildings) {
      const size = building.max.map((value, index) => value - building.min[index]!) as Vec3;
      const center = building.min.map((value, index) => value + size[index]! / 2) as Vec3;
      const geometry = new THREE.BoxGeometry(size[0], size[2], size[1]);
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({ color: palette.building, roughness: 0.94 }),
      );
      mesh.position.fromArray(enuToThree(center));
      this.content.add(mesh);
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geometry),
        new THREE.LineBasicMaterial({
          color: palette.buildingEdge,
          transparent: true,
          opacity: 0.58,
        }),
      );
      edges.position.copy(mesh.position);
      this.content.add(edges);
    }

    for (const zone of this.scenario.staticNoFlyZones) this.addZone(zone, "static");
    for (const zone of this.scenario.temporaryNoFlyZones) {
      this.temporaryZones.set(zone.id, this.addZone(zone, "temporary"));
    }
    for (const definition of this.scenario.movingSpheres) {
      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(definition.radiusM, 22, 14),
        new THREE.MeshStandardMaterial({
          color: palette.movingSphere,
          transparent: true,
          opacity: 0.82,
          roughness: 0.75,
        }),
      );
      mesh.visible = false;
      this.movingSpheres.set(definition.id, mesh);
      this.content.add(mesh);
    }

    this.addEndpoint(this.scenario.start, "start");
    this.addEndpoint(this.scenario.goal, "goal");
    this.vehicle = new THREE.Mesh(
      new THREE.ConeGeometry(1.9, 4.4, 5),
      new THREE.MeshStandardMaterial({
        color: palette.foreground,
        emissive: palette.background,
        emissiveIntensity: 0.16,
      }),
    );
    this.vehicle.rotation.z = -Math.PI / 2;
    this.content.add(this.vehicle);
  }

  private clearContent(): void {
    this.temporaryZones.clear();
    this.movingSpheres.clear();
    this.vehicle = null;
    this.scheduledPath = null;
    this.currentPath = null;
    this.executedPath = null;
    for (const child of [...this.content.children]) {
      this.content.remove(child);
      disposeObject(child);
    }
    for (const child of [...this.scene.children]) {
      if (child !== this.content && child instanceof THREE.Light) {
        this.scene.remove(child);
        disposeObject(child);
      }
    }
  }

  private addGround(): void {
    if (!this.scenario) return;
    const palette = this.palette;
    const width = this.scenario.bounds.max[0] - this.scenario.bounds.min[0];
    const depth = this.scenario.bounds.max[1] - this.scenario.bounds.min[1];
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(width, depth),
      new THREE.MeshStandardMaterial({ color: palette.ground, roughness: 1 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.set(
      this.scenario.bounds.min[0] + width / 2,
      this.scenario.bounds.min[2],
      -(this.scenario.bounds.min[1] + depth / 2),
    );
    this.content.add(ground);
    const grid = new THREE.GridHelper(
      Math.max(width, depth),
      12,
      palette.gridMajor,
      palette.gridMinor,
    );
    grid.position.copy(ground.position);
    grid.position.y += 0.02;
    this.content.add(grid);
  }

  private addZone(zone: StaticNoFlyZone, role: "static" | "temporary"): ZoneVisual {
    const palette = this.palette;
    const color = role === "static" ? palette.staticZone : palette.temporaryZone;
    const height = zone.zMaxM - zone.zMinM;
    const geometry = new THREE.CylinderGeometry(zone.radiusM, zone.radiusM, height, 40, 1, true);
    const fill = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color,
        transparent: true,
        opacity: role === "static" ? 0.12 : 0.025,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    fill.position.set(zone.center[0], zone.zMinM + height / 2, -zone.center[1]);
    this.content.add(fill);
    const wireframe = new THREE.LineSegments(
      new THREE.WireframeGeometry(geometry),
      new THREE.LineBasicMaterial({
        color,
        transparent: true,
        opacity: role === "static" ? 0.62 : 0.22,
      }),
    );
    wireframe.position.copy(fill.position);
    this.content.add(wireframe);
    return { fill, wireframe };
  }

  private addEndpoint(point: Vec3, role: "start" | "goal"): void {
    const palette = this.palette;
    const geometry =
      role === "start" ? new THREE.SphereGeometry(1.45, 18, 12) : new THREE.OctahedronGeometry(1.8);
    const material = new THREE.MeshStandardMaterial({
      color: role === "start" ? palette.executedPath : palette.goal,
      emissive: role === "start" ? palette.executedPath : palette.foreground,
      emissiveIntensity: role === "start" ? 0.08 : 0.04,
    });
    const marker = new THREE.Mesh(geometry, material);
    marker.position.fromArray(enuToThree(point));
    this.content.add(marker);
  }

  private replaceLine(role: "scheduled" | "current" | "executed", points: Vec3[]): void {
    const previous =
      role === "scheduled"
        ? this.scheduledPath
        : role === "current"
          ? this.currentPath
          : this.executedPath;
    if (previous) {
      this.content.remove(previous);
      disposeObject(previous);
    }
    const palette = this.palette;
    const material =
      role === "current"
        ? new THREE.LineDashedMaterial({
            color: palette.currentPath,
            dashSize: 4,
            gapSize: 2.4,
            linewidth: 1,
          })
        : new THREE.LineBasicMaterial({
            color: role === "scheduled" ? palette.scheduledPath : palette.executedPath,
            transparent: role === "scheduled",
            opacity: role === "scheduled" ? 0.52 : 1,
            linewidth: 1,
          });
    const line = new THREE.Line(pointsGeometry(points), material);
    line.visible = points.length >= 2;
    if (role === "current") line.computeLineDistances();
    this.content.add(line);
    if (role === "scheduled") this.scheduledPath = line;
    else if (role === "current") this.currentPath = line;
    else this.executedPath = line;
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
    const vertical = span * 0.76;
    this.camera.left = (-vertical * aspect) / 2;
    this.camera.right = (vertical * aspect) / 2;
    this.camera.top = vertical / 2;
    this.camera.bottom = -vertical / 2;
    this.camera.near = 0.1;
    this.camera.far = span * 10;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  private render(): void {
    this.renderer.render(this.scene, this.camera);
  }
}
