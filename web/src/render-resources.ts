import * as THREE from "three";

/** Release a viewer-owned subtree; borrowed textures and lighting resources stay untouched. */
export function disposeRenderObject(object: THREE.Object3D): void {
  const textures = new Set<THREE.Texture>();
  const geometries = new Set<THREE.BufferGeometry>();
  const ownedMaterials = new Set<THREE.Material>();
  object.traverse((child) => {
    if (child instanceof THREE.InstancedMesh) child.dispose();
    if ("geometry" in child && child.geometry instanceof THREE.BufferGeometry) {
      if (!geometries.has(child.geometry)) {
        geometries.add(child.geometry); child.geometry.dispose();
      }
    }
    if ("material" in child) {
      const materials = child.material as THREE.Material | THREE.Material[];
      for (const material of Array.isArray(materials) ? materials : [materials]) {
        if (ownedMaterials.has(material)) continue;
        ownedMaterials.add(material);
        if ("map" in material && material.map instanceof THREE.Texture) {
          const texture = material.map;
          if (texture.userData.viewerOwned === true && !textures.has(texture)) {
            textures.add(texture);
            if (texture.userData.viewerDisposed !== true) {
              // Set before disposal so an asynchronous image completion cannot resurrect a
              // texture or trigger a render after its scene has already been replaced.
              texture.userData.viewerDisposed = true;
              texture.dispose();
            }
          }
        }
        material.dispose();
      }
    }
  });
}
