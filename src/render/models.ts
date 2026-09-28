import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";

const HALF_W = 0.95;

function extrudeProfile(points: [number, number][], width: number, bevel: number): THREE.BufferGeometry {
  const shape = new THREE.Shape();
  const first = points[0];
  if (!first) throw new Error("Empty profile");
  shape.moveTo(first[0], first[1]);
  for (const [x, y] of points.slice(1)) shape.lineTo(x, y);
  shape.closePath();
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: width - bevel * 2,
    bevelEnabled: bevel > 0,
    bevelThickness: bevel,
    bevelSize: bevel,
    bevelSegments: 3,
    curveSegments: 4,
  });
  geo.translate(0, 0, -(width - bevel * 2) / 2);
  geo.computeVertexNormals();
  return geo;
}

function box(w: number, h: number, d: number, x: number, y: number, z: number): THREE.BufferGeometry {
  const g = new THREE.BoxGeometry(w, h, d);
  g.translate(x, y, z);
  return g;
}

/** Geometries shared by every car instance; the car faces +x with its centre at the origin. */
export interface CarGeometry {
  body: THREE.BufferGeometry;
  cabin: THREE.BufferGeometry;
  wheels: THREE.BufferGeometry;
  tail: THREE.BufferGeometry;
  head: THREE.BufferGeometry;
  blinkL: THREE.BufferGeometry;
  blinkR: THREE.BufferGeometry;
}

export function createCarGeometry(): CarGeometry {
  const body = extrudeProfile(
    [
      [-2.33, 0.3],
      [2.3, 0.3],
      [2.36, 0.52],
      [2.28, 0.74],
      [1.9, 0.86],
      [0.95, 0.98],
      [-1.7, 1.0],
      [-2.28, 0.93],
      [-2.36, 0.7],
    ],
    HALF_W * 2,
    0.07,
  );
  const cabin = extrudeProfile(
    [
      [0.95, 0.97],
      [0.12, 1.43],
      [-1.05, 1.45],
      [-1.95, 1.0],
    ],
    1.62,
    0.05,
  );
  const wheelParts: THREE.BufferGeometry[] = [];
  for (const x of [1.42, -1.42]) {
    for (const z of [0.83, -0.83]) {
      const w = new THREE.CylinderGeometry(0.36, 0.36, 0.26, 18);
      w.rotateX(Math.PI / 2);
      w.translate(x, 0.36, z);
      wheelParts.push(w);
    }
  }
  const merge = (parts: THREE.BufferGeometry[]): THREE.BufferGeometry => {
    const m = mergeGeometries(parts);
    if (!m) throw new Error("Failed to merge car geometry");
    return m;
  };
  return {
    body,
    cabin,
    wheels: merge(wheelParts),
    tail: merge([box(0.06, 0.1, 0.5, -2.36, 0.84, 0.6), box(0.06, 0.1, 0.5, -2.36, 0.84, -0.6), box(0.05, 0.05, 1.3, -2.35, 0.84, 0)]),
    head: merge([box(0.06, 0.08, 0.46, 2.3, 0.72, 0.6), box(0.06, 0.08, 0.46, 2.3, 0.72, -0.6)]),
    blinkL: merge([box(0.12, 0.08, 0.14, 2.24, 0.72, -0.9), box(0.12, 0.08, 0.14, -2.3, 0.84, -0.9)]),
    blinkR: merge([box(0.12, 0.08, 0.14, 2.24, 0.72, 0.9), box(0.12, 0.08, 0.14, -2.3, 0.84, 0.9)]),
  };
}

export interface CarMaterials {
  glass: THREE.Material;
  wheel: THREE.Material;
  tailOff: THREE.Material;
  tailOn: THREE.Material;
  head: THREE.Material;
  blinkOn: THREE.Material;
  blinkOff: THREE.Material;
}

export function createCarMaterials(): CarMaterials {
  return {
    glass: new THREE.MeshStandardMaterial({ color: 0x0d1015, roughness: 0.15, metalness: 0.6 }),
    wheel: new THREE.MeshStandardMaterial({ color: 0x141518, roughness: 0.8 }),
    tailOff: new THREE.MeshBasicMaterial({ color: 0x5a1116 }),
    tailOn: new THREE.MeshBasicMaterial({ color: 0xff2a36 }),
    head: new THREE.MeshBasicMaterial({ color: 0xe8f0ff }),
    blinkOn: new THREE.MeshBasicMaterial({ color: 0xffa31a }),
    blinkOff: new THREE.MeshBasicMaterial({ color: 0x3b2a12 }),
  };
}

export class CarView {
  readonly group = new THREE.Group();
  readonly body: THREE.Mesh;
  private readonly tail: THREE.Mesh;
  private readonly blinkL: THREE.Mesh;
  private readonly blinkR: THREE.Mesh;

  constructor(
    geo: CarGeometry,
    private readonly mats: CarMaterials,
    bodyMat: THREE.Material,
  ) {
    this.body = new THREE.Mesh(geo.body, bodyMat);
    this.body.castShadow = true;
    const cabin = new THREE.Mesh(geo.cabin, mats.glass);
    cabin.castShadow = true;
    const wheels = new THREE.Mesh(geo.wheels, mats.wheel);
    this.tail = new THREE.Mesh(geo.tail, mats.tailOff);
    const head = new THREE.Mesh(geo.head, mats.head);
    this.blinkL = new THREE.Mesh(geo.blinkL, mats.blinkOff);
    this.blinkR = new THREE.Mesh(geo.blinkR, mats.blinkOff);
    this.group.add(this.body, cabin, wheels, this.tail, head, this.blinkL, this.blinkR);
  }

  /** blinker: -1 left, 1 right, 2 hazard, 0 off. */
  setLights(braking: boolean, blinker: number, blinkPhase: boolean): void {
    this.tail.material = braking ? this.mats.tailOn : this.mats.tailOff;
    const left = blinkPhase && (blinker === -1 || blinker === 2);
    const rightOn = blinkPhase && (blinker === 1 || blinker === 2);
    this.blinkL.material = left ? this.mats.blinkOn : this.mats.blinkOff;
    this.blinkR.material = rightOn ? this.mats.blinkOn : this.mats.blinkOff;
  }

  setBodyMaterial(m: THREE.Material): void {
    if (this.body.material !== m) this.body.material = m;
  }
}
