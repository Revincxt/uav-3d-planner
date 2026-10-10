import * as THREE from "three";
import { describe, expect, it, vi } from "vitest";
import { createEndpointMarker } from "../src/scene-style";
import { createLaunchMarker } from "../src/mission-launch";
import { disposeRenderObject } from "../src/render-resources";
import { RouteOverview, routeColor, type OverviewRoute } from "../src/route-overview";

describe("physical launch platform", () => {
  it("uses three batched meshes with no sphere, image or floating HUD label", () => {
    const marker = createLaunchMarker(0x009b87);
    expect(marker.userData.kind).toBe("launch-marker");
    expect(marker.children).toHaveLength(3);
    marker.traverse(object => {
      if (!(object instanceof THREE.Mesh)) return;
      expect(object.geometry.type).not.toBe("SphereGeometry");
      const material = object.material as THREE.MeshStandardMaterial;
      expect(material.map).toBe(null);
      expect(material.transparent).toBe(false);
      expect(material.depthTest && material.depthWrite).toBe(true);
      expect(object.receiveShadow).toBe(true);
    });
    disposeRenderObject(marker);
  });
  it("keeps the pad beneath the recorded origin and avoids degenerate geometry", () => {
    const marker = createLaunchMarker(0x2563eb);
    marker.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(marker);
    expect(box.max.y).toBeLessThan(-2);
    expect(box.getSize(new THREE.Vector3()).x).toBeLessThan(14);
    for (const mesh of marker.children as THREE.Mesh[]) {
      const positions = mesh.geometry.getAttribute("position");
      expect(Array.from(positions.array).every(Number.isFinite)).toBe(true);
    }
    disposeRenderObject(marker);
  });
  it("uses the same endpoint factory for all scene scales", () => {
    const marker = createEndpointMarker("start", false, 0x009b87);
    expect(marker.userData.kind).toBe("launch-marker");
    expect(marker.scale.x).toBe(0.28);
    disposeRenderObject(marker);
  });
  it("points the departure chevron along the gate-aligned local forward axis", () => {
    const marker = createLaunchMarker(0x009b87);
    const frame = marker.getObjectByName("launch-pad-frame") as THREE.Mesh;
    const positions = frame.geometry.getAttribute("position");
    expect(Array.from({ length: positions.count }, (_, i) => i).some(i =>
      Math.abs(positions.getX(i)) < 0.2 && positions.getZ(i) > 5.1 && positions.getZ(i) < 5.55)).toBe(true);
    disposeRenderObject(marker);
  });
  it("orients each pad along its departure without changing route coordinates or playback", () => {
    const routes: OverviewRoute[] = Array.from({ length: 8 }, (_, i) => ({ id: String(i), label: "Mission",
      points: [[0, i * 30, 50], [100, i * 30, 50]],
      timedPath: [{ timeS: 0, position: [0, i * 30, 50] }, { timeS: 10, position: [100, i * 30, 50] }],
    }));
    const before = JSON.stringify(routes), overview = new RouteOverview();
    overview.setRoutes(routes, "0", 800, 600);
    routes.forEach((route, index) => {
      const start = overview.group.getObjectByName(`overview-start-${route.id}`)!;
      expect(start.userData.kind).toBe("launch-marker");
      expect(start.rotation.y).toBeCloseTo(Math.PI / 2);
      expect(start.position.toArray()).toEqual([0, 50, -index * 30]);
      const frame = start.getObjectByName("launch-pad-frame") as THREE.Mesh;
      expect((frame.material as THREE.MeshStandardMaterial).color.getHex()).toBe(routeColor(index));
    });
    overview.setTime(5);
    expect(overview.vehicle("0")!.position.toArray()).toEqual([50, 50, -0]);
    expect(JSON.stringify(routes)).toBe(before);
    overview.dispose();
  });
  it("releases each batched geometry and material exactly once", () => {
    const marker = createLaunchMarker(0xe78a08);
    const disposals = (marker.children as THREE.Mesh[]).map(mesh => [
      vi.spyOn(mesh.geometry, "dispose"), vi.spyOn(mesh.material as THREE.Material, "dispose"),
    ]);
    disposeRenderObject(marker);
    for (const spies of disposals) for (const spy of spies) expect(spy).toHaveBeenCalledOnce();
  });
});
