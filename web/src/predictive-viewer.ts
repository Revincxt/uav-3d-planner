import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

import type {
  MovingSphereDefinition,
  PredictivePathMode,
  PredictiveRun,
  PredictiveScenario,
  StaticNoFlyZone,
  TimedWaypoint,
  Vec3,
} from "./predictive-schema";

export type ViewPreset = "isometric" | "xy" | "xz" | "yz" | "fit";
export type ViewerLayer = "buildings" | "zones" | "dynamic" | "raw";

interface ZoneVisual {
  fill: THREE.Mesh<THREE.CylinderGeometry, THREE.MeshStandardMaterial>;
  outline: THREE.Group;
}

interface ThemePalette {
  background: number;
  ground: number;
  gridMajor: number;
  gridMinor: number;
  buildingLow: number;
  buildingHigh: number;
  buildingEdge: number;
  foreground: number;
  staticZone: number;
  temporaryZone: number;
  movingSphere: number;
  rawPath: number;
  plannedPath: number;
  executedPath: number;
  goal: number;
}

const LIGHT_PALETTE: ThemePalette = {
  background: 0xf7f7f3,
  ground: 0xe9ece8,
  gridMajor: 0xaeb5b2,
  gridMinor: 0xd5d9d6,
  buildingLow: 0xb8bfbd,
  buildingHigh: 0x697579,
  buildingEdge: 0x515b5f,
  foreground: 0x1d252c,
  staticZone: 0xa34a3f,
  temporaryZone: 0xb2742f,
  movingSphere: 0x76558f,
  rawPath: 0x71797e,
  plannedPath: 0x315f8c,
  executedPath: 0x15766b,
  goal: 0xf7f7f3,
};

const DARK_PALETTE: ThemePalette = {
  background: 0x1e2427,
  ground: 0x282f31,
  gridMajor: 0x697274,
  gridMinor: 0x424a4c,
  buildingLow: 0x687376,
  buildingHigh: 0xaab3b3,
  buildingEdge: 0xd1d5d3,
  foreground: 0xf0f1ed,
  staticZone: 0xe08779,
  temporaryZone: 0xe0ae69,
  movingSphere: 0xb69ac5,
  rawPath: 0xa9b0b3,
  plannedPath: 0x82acd4,
  executedPath: 0x6fc0ad,
  goal: 0x252c2e,
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

function timedPosition(waypoints: TimedWaypoint[], timeS: number): Vec3 {
  const first = waypoints[0]!;
  const last = waypoints.at(-1)!;
  if (timeS <= first.timeS) return first.position;
  if (timeS >= last.timeS) return last.position;
  for (let index = 1; index < waypoints.length; index += 1) {
    const right = waypoints[index]!;
    const left = waypoints[index - 1]!;
    if (timeS <= right.timeS) {
      const fraction = (timeS - left.timeS) / (right.timeS - left.timeS);
      return left.position.map(
        (coordinate, axis) => coordinate + (right.position[axis]! - coordinate) * fraction,
      ) as Vec3;
    }
  }
  return last.position;
}

function movingPosition(definition: MovingSphereDefinition, timeS: number): Vec3 {
  return timedPosition(definition.keyframes, timeS);
}

function pathPrefix(waypoints: TimedWaypoint[], timeS: number): Vec3[] {
  const prefix = waypoints
    .filter((waypoint) => waypoint.timeS <= timeS)
    .map((waypoint) => waypoint.position);
  const current = timedPosition(waypoints, timeS);
  const last = prefix.at(-1);
  if (!last || last.some((coordinate, axis) => Math.abs(coordinate - current[axis]!) > 1e-9)) {
    prefix.push(current);
  }
  return prefix;
}

function flattenPoints(points: Vec3[]): number[] {
  return points.flatMap((point) => enuToThree(point));
}

function ringPoints(radius: number, height: number, segments = 64): THREE.Vector3[] {
  return Array.from({ length: segments }, (_, index) => {
    const angle = (index / segments) * Math.PI * 2;
    return new THREE.Vector3(Math.cos(angle) * radius, height, Math.sin(angle) * radius);
  });
}

export class PredictiveViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera();
  private readonly controls: OrbitControls;
  private readonly content = new THREE.Group();
  private readonly buildingsGroup = new THREE.Group();
  private readonly zonesGroup = new THREE.Group();
  private readonly dynamicGroup = new THREE.Group();
  private readonly primaryPathGroup = new THREE.Group();
  private readonly rawPathGroup = new THREE.Group();
  private readonly resizeObserver: ResizeObserver;
  private readonly colorScheme = window.matchMedia("(prefers-color-scheme: dark)");
  private readonly onColorSchemeChange = (): void => this.rebuild();
  private readonly layerVisibility: Record<ViewerLayer, boolean> = {
    buildings: true,
    zones: true,
    dynamic: true,
    raw: false,
  };
  private scenario: PredictiveScenario | null = null;
  private run: PredictiveRun | null = null;
  private pathMode: PredictivePathMode = "certified";
  private currentTimeS = 0;
  private currentView: ViewPreset = "isometric";
  private temporaryZones = new Map<string, ZoneVisual>();
  private movingSpheres = new Map<string, THREE.Mesh>();
  private lineMaterials = new Set<LineMaterial>();
  private vehicle: THREE.Mesh | null = null;
  private primaryPath: Line2 | null = null;
  private executedPath: Line2 | null = null;
  private rawPath: Line2 | null = null;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.append(this.renderer.domElement);
    this.content.add(
      this.buildingsGroup,
      this.zonesGroup,
      this.dynamicGroup,
      this.rawPathGroup,
      this.primaryPathGroup,
    );
    this.scene.add(this.content);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
    this.controls.screenSpacePanning = true;
    this.controls.addEventListener("change", () => this.render());
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.colorScheme.addEventListener("change", this.onColorSchemeChange);
  }

  setScenario(scenario: PredictiveScenario): void {
    this.scenario = scenario;
    this.run = null;
    this.currentTimeS = 0;
    this.buildScene();
    this.setView("isometric");
  }

  setRun(run: PredictiveRun): void {
    this.run = run;
    this.replacePathLines();
    this.setTime(0);
  }

  setPathMode(mode: PredictivePathMode): void {
    if (this.pathMode === mode) return;
    this.pathMode = mode;
    this.replacePathLines();
    this.setTime(this.currentTimeS);
  }

  setLayerVisibility(layer: ViewerLayer, visible: boolean): void {
    this.layerVisibility[layer] = visible;
    this.updateLayerVisibility();
    this.render();
  }

  setTime(timeS: number): void {
    if (!this.scenario || !this.run) return;
    const path = this.activePath;
    const duration = path.at(-1)!.timeS;
    this.currentTimeS = Math.max(0, Math.min(timeS, duration));
    const vehiclePosition = timedPosition(path, this.currentTimeS);
    this.vehicle?.position.fromArray(enuToThree(vehiclePosition));
    this.orientVehicle(path, this.currentTimeS);

    for (const zone of this.scenario.temporaryNoFlyZones) {
      const visual = this.temporaryZones.get(zone.id);
      if (!visual) continue;
      const active = zone.activeFromS <= this.currentTimeS && this.currentTimeS < zone.activeUntilS;
      visual.fill.material.opacity = active ? 0.18 : 0.018;
      visual.outline.traverse((child) => {
        if (child instanceof THREE.Line && child.material instanceof THREE.LineBasicMaterial) {
          child.material.opacity = active ? 0.88 : 0.24;
        }
      });
    }

    for (const definition of this.scenario.movingSpheres) {
      const mesh = this.movingSpheres.get(definition.id);
      if (mesh) mesh.position.fromArray(enuToThree(movingPosition(definition, this.currentTimeS)));
    }

    this.executedPath = this.replaceLine(
      this.primaryPathGroup,
      this.executedPath,
      pathPrefix(path, this.currentTimeS),
      this.palette.executedPath,
      3.7,
      false,
      1,
    );
    this.render();
  }

  setView(preset: ViewPreset): void {
    if (!this.scenario) return;
    this.currentView = preset === "fit" ? "isometric" : preset;
    const { min, max } = this.scenario.bounds;
    const center = new THREE.Vector3(
      (min[0] + max[0]) / 2,
      (min[2] + max[2]) / 2,
      -(min[1] + max[1]) / 2,
    );
    const span = Math.max(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
    if (this.currentView === "xy") {
      this.camera.position.set(center.x, center.y + span * 2.2, center.z + 0.001);
      this.camera.up.set(0, 0, -1);
    } else if (this.currentView === "xz") {
      this.camera.position.set(center.x, center.y, center.z + span * 2.2);
      this.camera.up.set(0, 1, 0);
    } else if (this.currentView === "yz") {
      this.camera.position.set(center.x + span * 2.2, center.y, center.z);
      this.camera.up.set(0, 1, 0);
    } else {
      this.camera.position.set(
        center.x + span * 1.18,
        center.y + span * 0.94,
        center.z + span * 1.18,
      );
      this.camera.up.set(0, 1, 0);
    }
    this.controls.target.copy(center);
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

  private get activePath(): TimedWaypoint[] {
    if (!this.run) return [];
    return this.pathMode === "raw" ? this.run.rawTimedPath : this.run.timedPath;
  }

  private rebuild(): void {
    if (!this.scenario) return;
    const run = this.run;
    const timeS = this.currentTimeS;
    const view = this.currentView;
    this.buildScene();
    this.setView(view);
    if (run) {
      this.setRun(run);
      this.setTime(timeS);
    }
  }

  private buildScene(): void {
    if (!this.scenario) return;
    this.clearContent();
    const palette = this.palette;
    this.scene.background = new THREE.Color(palette.background);
    this.scene.add(new THREE.HemisphereLight(0xffffff, palette.ground, 1.75));
    const sun = new THREE.DirectionalLight(0xffffff, 2.3);
    const lightTarget = new THREE.Vector3(
      (this.scenario.bounds.min[0] + this.scenario.bounds.max[0]) / 2,
      (this.scenario.bounds.min[2] + this.scenario.bounds.max[2]) / 2,
      -(this.scenario.bounds.min[1] + this.scenario.bounds.max[1]) / 2,
    );
    sun.position.set(lightTarget.x - 50, lightTarget.y + 110, lightTarget.z + 70);
    sun.target.position.copy(lightTarget);
    this.content.add(sun.target);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024);
    sun.shadow.bias = -0.0004;
    const sceneSpan = Math.max(
      this.scenario.bounds.max[0] - this.scenario.bounds.min[0],
      this.scenario.bounds.max[1] - this.scenario.bounds.min[1],
    );
    sun.shadow.camera.left = -sceneSpan;
    sun.shadow.camera.right = sceneSpan;
    sun.shadow.camera.top = sceneSpan;
    sun.shadow.camera.bottom = -sceneSpan;
    this.scene.add(sun);

    this.addGround();
    const buildingHeights = this.scenario.buildings.map((building) => building.max[2]);
    const minHeight = buildingHeights.length > 0 ? Math.min(...buildingHeights) : 0;
    const maxHeight = buildingHeights.length > 0 ? Math.max(...buildingHeights) : 0;
    for (const building of this.scenario.buildings) {
      const size = building.max.map((value, index) => value - building.min[index]!) as Vec3;
      const center = building.min.map((value, index) => value + size[index]! / 2) as Vec3;
      const heightFraction =
        maxHeight > minHeight ? (building.max[2] - minHeight) / (maxHeight - minHeight) : 0.5;
      const color = new THREE.Color(palette.buildingLow).lerp(
        new THREE.Color(palette.buildingHigh),
        heightFraction,
      );
      const geometry = new THREE.BoxGeometry(size[0], size[2], size[1]);
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({ color, roughness: 0.92, metalness: 0.02 }),
      );
      mesh.position.fromArray(enuToThree(center));
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.buildingsGroup.add(mesh);
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geometry, 24),
        new THREE.LineBasicMaterial({
          color: palette.buildingEdge,
          transparent: true,
          opacity: 0.46,
        }),
      );
      edges.position.copy(mesh.position);
      this.buildingsGroup.add(edges);
    }

    for (const zone of this.scenario.staticNoFlyZones) this.addZone(zone, "static");
    for (const zone of this.scenario.temporaryNoFlyZones) {
      this.temporaryZones.set(zone.id, this.addZone(zone, "temporary"));
    }
    for (const definition of this.scenario.movingSpheres) this.addMovingSphere(definition);

    this.addEndpoint(this.scenario.start, "start");
    this.addEndpoint(this.scenario.goal, "goal");
    this.vehicle = new THREE.Mesh(
      new THREE.ConeGeometry(1.35, 3.8, 5),
      new THREE.MeshStandardMaterial({
        color: palette.foreground,
        emissive: palette.background,
        emissiveIntensity: 0.12,
      }),
    );
    this.vehicle.castShadow = true;
    this.primaryPathGroup.add(this.vehicle);
    this.updateLayerVisibility();
  }

  private clearContent(): void {
    this.temporaryZones.clear();
    this.movingSpheres.clear();
    this.lineMaterials.clear();
    this.vehicle = null;
    this.primaryPath = null;
    this.executedPath = null;
    this.rawPath = null;
    for (const group of [
      this.buildingsGroup,
      this.zonesGroup,
      this.dynamicGroup,
      this.rawPathGroup,
      this.primaryPathGroup,
    ]) {
      for (const child of [...group.children]) {
        group.remove(child);
        disposeObject(child);
      }
    }
    for (const child of [...this.content.children]) {
      if (![
        this.buildingsGroup,
        this.zonesGroup,
        this.dynamicGroup,
        this.rawPathGroup,
        this.primaryPathGroup,
      ].includes(child as THREE.Group)) {
        this.content.remove(child);
        disposeObject(child);
      }
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
    ground.receiveShadow = true;
    this.content.add(ground);
    const divisions = Math.max(8, Math.round(Math.max(width, depth) / 8));
    const grid = new THREE.GridHelper(
      Math.max(width, depth),
      divisions,
      palette.gridMajor,
      palette.gridMinor,
    );
    grid.position.copy(ground.position);
    grid.position.y += 0.025;
    this.content.add(grid);
  }

  private addZone(zone: StaticNoFlyZone, role: "static" | "temporary"): ZoneVisual {
    const palette = this.palette;
    const color = role === "static" ? palette.staticZone : palette.temporaryZone;
    const height = zone.zMaxM - zone.zMinM;
    const geometry = new THREE.CylinderGeometry(zone.radiusM, zone.radiusM, height, 64, 1, true);
    const fill = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color,
        transparent: true,
        opacity: role === "static" ? 0.12 : 0.018,
        side: THREE.DoubleSide,
        depthWrite: false,
        roughness: 0.8,
      }),
    );
    fill.position.set(zone.center[0], zone.zMinM + height / 2, -zone.center[1]);
    this.zonesGroup.add(fill);

    const outline = new THREE.Group();
    outline.position.copy(fill.position);
    const outlineMaterial = new THREE.LineBasicMaterial({
      color,
      transparent: true,
      opacity: role === "static" ? 0.82 : 0.24,
    });
    const lower = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(ringPoints(zone.radiusM, -height / 2)),
      outlineMaterial.clone(),
    );
    const upper = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(ringPoints(zone.radiusM, height / 2)),
      outlineMaterial.clone(),
    );
    outline.add(lower, upper);
    for (let index = 0; index < 4; index += 1) {
      const angle = (index / 4) * Math.PI * 2;
      const x = Math.cos(angle) * zone.radiusM;
      const z = Math.sin(angle) * zone.radiusM;
      outline.add(
        new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(x, -height / 2, z),
            new THREE.Vector3(x, height / 2, z),
          ]),
          outlineMaterial.clone(),
        ),
      );
    }
    outlineMaterial.dispose();
    this.zonesGroup.add(outline);
    return { fill, outline };
  }

  private addMovingSphere(definition: MovingSphereDefinition): void {
    const palette = this.palette;
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(definition.radiusM, 24, 16),
      new THREE.MeshStandardMaterial({
        color: palette.movingSphere,
        transparent: true,
        opacity: 0.78,
        roughness: 0.72,
      }),
    );
    mesh.castShadow = true;
    this.movingSpheres.set(definition.id, mesh);
    this.dynamicGroup.add(mesh);
    this.replaceLine(
      this.dynamicGroup,
      null,
      definition.keyframes.map((keyframe) => keyframe.position),
      palette.movingSphere,
      1.5,
      true,
      0.56,
    );
  }

  private addEndpoint(point: Vec3, role: "start" | "goal"): void {
    const palette = this.palette;
    const geometry =
      role === "start" ? new THREE.SphereGeometry(1.15, 20, 14) : new THREE.OctahedronGeometry(1.55);
    const material = new THREE.MeshStandardMaterial({
      color: role === "start" ? palette.executedPath : palette.goal,
      emissive: role === "start" ? palette.executedPath : palette.foreground,
      emissiveIntensity: role === "start" ? 0.08 : 0.04,
    });
    const marker = new THREE.Mesh(geometry, material);
    marker.position.fromArray(enuToThree(point));
    this.primaryPathGroup.add(marker);
  }

  private replacePathLines(): void {
    if (!this.run) return;
    const path = this.activePath;
    this.primaryPath = this.replaceLine(
      this.primaryPathGroup,
      this.primaryPath,
      path.map((waypoint) => waypoint.position),
      this.palette.plannedPath,
      2.5,
      false,
      0.78,
    );
    this.rawPath = this.replaceLine(
      this.rawPathGroup,
      this.rawPath,
      this.run.rawTimedPath.map((waypoint) => waypoint.position),
      this.palette.rawPath,
      1.55,
      true,
      0.72,
    );
    this.updateLayerVisibility();
  }

  private replaceLine(
    group: THREE.Group,
    previous: Line2 | null,
    points: Vec3[],
    color: number,
    widthPx: number,
    dashed: boolean,
    opacity: number,
  ): Line2 | null {
    if (previous) {
      if (points.length < 2) {
        previous.visible = false;
        return previous;
      }
      const geometry = previous.geometry as LineGeometry;
      const material = previous.material as LineMaterial;
      geometry.setPositions(flattenPoints(points));
      material.color.setHex(color);
      material.linewidth = widthPx;
      material.dashed = dashed;
      material.opacity = opacity;
      material.transparent = opacity < 1;
      material.needsUpdate = true;
      previous.visible = true;
      if (dashed) previous.computeLineDistances();
      return previous;
    }
    if (points.length < 2) return null;
    const geometry = new LineGeometry();
    geometry.setPositions(flattenPoints(points));
    const material = new LineMaterial({
      color,
      linewidth: widthPx,
      dashed,
      dashSize: 2.2,
      gapSize: 1.4,
      transparent: opacity < 1,
      opacity,
      depthTest: true,
      depthWrite: false,
    });
    material.resolution.set(
      Math.max(1, this.container.clientWidth),
      Math.max(1, this.container.clientHeight),
    );
    this.lineMaterials.add(material);
    const line = new Line2(geometry, material);
    if (dashed) line.computeLineDistances();
    line.renderOrder = dashed ? 3 : 4;
    group.add(line);
    return line;
  }

  private orientVehicle(path: TimedWaypoint[], timeS: number): void {
    if (!this.vehicle || path.length < 2) return;
    const duration = path.at(-1)!.timeS;
    const before = timedPosition(path, Math.max(0, timeS - 0.03));
    const after = timedPosition(path, Math.min(duration, timeS + 0.03));
    const direction = new THREE.Vector3(...enuToThree(after)).sub(
      new THREE.Vector3(...enuToThree(before)),
    );
    if (direction.lengthSq() <= 1e-10) return;
    direction.normalize();
    this.vehicle.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
  }

  private updateLayerVisibility(): void {
    this.buildingsGroup.visible = this.layerVisibility.buildings;
    this.zonesGroup.visible = this.layerVisibility.zones;
    this.dynamicGroup.visible = this.layerVisibility.dynamic;
    this.rawPathGroup.visible =
      this.layerVisibility.raw && this.pathMode === "certified" && Boolean(this.run?.smoothing.applied);
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    for (const material of this.lineMaterials) material.resolution.set(width, height);
    const aspect = width / height;
    const widthSpan = this.scenario
      ? this.scenario.bounds.max[0] - this.scenario.bounds.min[0]
      : 100;
    const depthSpan = this.scenario
      ? this.scenario.bounds.max[1] - this.scenario.bounds.min[1]
      : 100;
    const vertical = Math.max(widthSpan, depthSpan) * 0.78;
    this.camera.left = (-vertical * aspect) / 2;
    this.camera.right = (vertical * aspect) / 2;
    this.camera.top = vertical / 2;
    this.camera.bottom = -vertical / 2;
    this.camera.near = 0.1;
    this.camera.far = Math.max(widthSpan, depthSpan) * 12;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  private render(): void {
    this.renderer.render(this.scene, this.camera);
  }
}
