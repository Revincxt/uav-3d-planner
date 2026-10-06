import * as THREE from "three";
import { describe, expect, it, vi } from "vitest";
import { createFinishMarker } from "../src/mission-finish";
import { createEndpointMarker } from "../src/scene-style";
import { disposeRenderObject } from "../src/render-resources";
import { RouteOverview, routeColor, type OverviewRoute } from "../src/route-overview";

describe("Open finish target", () => {
  it("keeps the flight endpoint empty and places the target beneath the vehicle", () => {
    const marker = createFinishMarker(0x009b87); marker.updateMatrixWorld(true);
    expect(new THREE.Raycaster(new THREE.Vector3(0,20,0), new THREE.Vector3(0,-1,0)).intersectObject(marker, true)).toHaveLength(0);
    expect(new THREE.Raycaster(new THREE.Vector3(0,3.2,12), new THREE.Vector3(0,-3.2,-12).normalize()).intersectObject(marker, true)).toHaveLength(0);
    expect(marker.getObjectByName("finish-inner-ring")!.position.y).toBeLessThan(-2);
    expect(marker.getObjectByName("finish-checkered-flag")!.position.x).toBeGreaterThan(6);
    marker.traverse(object => {
      if (!(object instanceof THREE.Mesh)) return;
      expect(object.geometry.type).not.toBe("OctahedronGeometry");
      const material = object.material as THREE.MeshStandardMaterial;
      expect(material.depthTest && material.depthWrite).toBe(true);
      expect(material.transparent).toBe(false);
    });
    disposeRenderObject(marker);
  });
  it("uses a waved checkered flag with two contrasting vertex colors, no canvas or image dependency", () => {
    const marker = createFinishMarker(0x2563eb);
    const flag = marker.getObjectByName("finish-checkered-flag") as THREE.Mesh;
    const color = flag.geometry.getAttribute("color"), position = flag.geometry.getAttribute("position");
    const colors = new Set<string>();
    for (let i = 0; i < color.count; i++) colors.add([color.getX(i),color.getY(i),color.getZ(i)].join(","));
    expect(colors.size).toBe(2);
    expect(Array.from({ length: position.count }, (_,i) => position.getZ(i)).some(z => Math.abs(z) > 0.05)).toBe(true);
    const material = flag.material as THREE.MeshStandardMaterial;
    expect(material.vertexColors).toBe(true); expect(material.side).toBe(THREE.DoubleSide); expect(material.map).toBe(null);
    disposeRenderObject(marker);
  });
  it("releases shared tick geometry and ring materials exactly once", () => {
    const marker = createFinishMarker(0xe78a08);
    const tick = marker.getObjectByName("finish-target-tick-0") as THREE.Mesh;
    const geometry = vi.spyOn(tick.geometry, "dispose"), material = vi.spyOn(tick.material as THREE.Material, "dispose");
    disposeRenderObject(marker);
    expect(geometry).toHaveBeenCalledOnce(); expect(material).toHaveBeenCalledOnce();
  });
  it("shares finish identity across viewers and routes without moving recorded goals", () => {
    const endpoint = createEndpointMarker("goal", false, 0x009b87);
    expect(endpoint.userData.kind).toBe("finish-marker"); expect(endpoint.scale.x).toBe(0.28);
    disposeRenderObject(endpoint);
    const routes: OverviewRoute[] = Array.from({ length: 4 }, (_,i) => ({ id: String(i), label: "Mission",
      points: [[0,i * 30,50],[100,i * 30,50]], timedPath: [{ timeS: 0, position: [0,i * 30,50] }, { timeS: 10, position: [100,i * 30,50] }] }));
    const overview = new RouteOverview(); overview.setRoutes(routes, "0", 800, 600); overview.setTime(10);
    overview.updateSymbols(new THREE.OrthographicCamera(-160,160,80,-80), 600, "0");
    routes.forEach((route,i) => {
      const finish = overview.group.getObjectByName(`overview-goal-${route.id}`)!;
      expect(finish.userData.kind).toBe("finish-marker"); expect(finish.position.toArray()).toEqual([100,50,-i * 30]);
      expect(finish.visible).toBe(true);
      expect(finish.rotation.y).toBeCloseTo(Math.PI / 2);
      expect(((finish.getObjectByName("finish-inner-ring") as THREE.Mesh).material as THREE.MeshStandardMaterial).color.getHex()).toBe(routeColor(i));
      expect(overview.vehicle(route.id)!.position.toArray()).toEqual(finish.position.toArray());
    });
    overview.dispose();
  });
});
