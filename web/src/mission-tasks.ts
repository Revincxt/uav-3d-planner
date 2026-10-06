import * as THREE from "three";
import { enuToThree } from "./coordinates";
import type { CityMission } from "./city-schema";
import { metresPerPixelAt, type DisplayCamera } from "./camera-scale";
import { GATE_SIZE_M, missionGateGeometry, missionGateMaterial, missionGateYaw } from "./mission-gate";

/** Open fly-through gates with ordinary depth testing and world-space orientation. */
export function addMissionTaskMarkers(host: THREE.Group, mission?: CityMission): void {
  const tasks = new THREE.Group();
  tasks.name = "mission-task-points";
  for (const task of mission?.taskPoints ?? []) {
    const group = new THREE.Group();
    group.name = `mission-task-${task.order}`;
    group.position.fromArray(enuToThree(task.position));
    group.userData = { taskId: task.id, order: task.order, position: task.position };
    const marker = new THREE.Mesh(missionGateGeometry(), missionGateMaterial());
    marker.name = "mission-gate";
    group.add(marker);
    const canvas = document.createElement("canvas");
    canvas.width = canvas.height = 96;
    const context = canvas.getContext("2d");
    if (context) {
      context.fillStyle = "#f0faf6";
      context.fillRect(13, 18, 70, 60);
      context.fillStyle = "#287c69"; context.font = "600 48px system-ui";
      context.textAlign = "center"; context.textBaseline = "middle";
      context.fillText(String(task.order), 48, 49);
      const texture = new THREE.CanvasTexture(canvas);
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.userData.viewerOwned = true;
      const number = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: true,
        depthWrite: false, alphaTest: 0.2, sizeAttenuation: true }));
      number.name = "task-number";
      number.position.y = GATE_SIZE_M / 2 + 4;
      number.scale.set(7, 7, 1);
      group.add(number);
    }
    tasks.add(group);
  }
  host.add(tasks);
}

/** Keep the gate opening visible while the drone passes; only its number yields to the body. */
export function avoidDroneMarkerOverlap(host: THREE.Object3D, drone?: THREE.Object3D): void {
  const tasks = host.getObjectByName("mission-task-points");
  if (!tasks) return;
  for (const task of tasks.children) {
    const near = drone && task.position.distanceTo(drone.position) < Math.max(35, drone.scale.x * 1.5);
    for (const marker of task.children) if (marker instanceof THREE.Sprite) marker.visible = !near;
  }
}

/** Gate dimensions stay in metres; only number annotations are capped in close views. */
export function sizeMissionTaskMarkers(host: THREE.Object3D, camera: DisplayCamera, height: number, closeView: boolean): void {
  const tasks = host.getObjectByName("mission-task-points");
  if (!tasks) return;
  for (const task of tasks.children) {
    const metresPerPixel = closeView ? metresPerPixelAt(camera, task.position, height) : 0;
    const labelSize = closeView ? Math.min(7, 20 * metresPerPixel) : 7;
    for (const marker of task.children) {
      if (marker instanceof THREE.Sprite) {
        marker.scale.set(labelSize, labelSize, 1);
        marker.position.y = GATE_SIZE_M / 2 + labelSize / 2 + 0.5;
      }
    }
  }
}

export function orientMissionTaskGates(host: THREE.Object3D, points: readonly (readonly number[])[], accent?: number): void {
  for (const task of host.getObjectByName("mission-task-points")?.children ?? []) {
    const gate = task.getObjectByName("mission-gate");
    if (gate) {
      gate.rotation.y = missionGateYaw(points, task.userData.position);
      if (gate instanceof THREE.Mesh && accent !== undefined) {
        const material = gate.material as THREE.MeshStandardMaterial;
        material.color.setHex(accent); material.emissive.setHex(accent);
      }
    }
  }
}
