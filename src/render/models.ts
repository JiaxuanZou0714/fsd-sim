import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { VEHICLES, type VehicleKind } from "../sim/vehicles";

function extrudeProfile(points: [number, number][], width: number, bevel: number): THREE.BufferGeometry {
  const shape = new THREE.Shape();
  shape.moveTo(points[0][0], points[0][1]);
  for (const [x, y] of points.slice(1)) shape.lineTo(x, y);
  shape.closePath();
  const depth = Math.max(0.01, width - bevel * 2);
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth,
    bevelEnabled: bevel > 0,
    bevelThickness: bevel,
    bevelSize: bevel,
    bevelSegments: 2,
    curveSegments: 3,
  });
  geo.translate(0, 0, -depth / 2);
  geo.computeVertexNormals();
  return geo;
}

function box(w: number, h: number, d: number, x: number, y: number, z: number): THREE.BufferGeometry {
  const g = new THREE.BoxGeometry(w, h, d);
  g.translate(x, y, z);
  return g;
}

function merge(parts: THREE.BufferGeometry[]): THREE.BufferGeometry {
  const normalized = parts.map((p) => (p.index ? p.toNonIndexed() : p));
  for (const p of normalized) {
    if (p.getAttribute("uv")) p.deleteAttribute("uv");
  }
  const m = mergeGeometries(normalized);
  if (!m) throw new Error("Failed to merge vehicle geometry");
  return m;
}

function wheels(xs: number[], halfTrack: number, r: number, w: number): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  for (const x of xs) {
    for (const z of [halfTrack, -halfTrack]) {
      const c = new THREE.CylinderGeometry(r, r, w, 14);
      c.rotateX(Math.PI / 2);
      c.translate(x, r, z);
      parts.push(c);
    }
  }
  return merge(parts);
}

/** Geometry set for one vehicle type; vehicles face +x with their centre at the origin. */
export interface VehicleGeometry {
  body: THREE.BufferGeometry;
  glass: THREE.BufferGeometry | null;
  wheels: THREE.BufferGeometry;
  tail: THREE.BufferGeometry;
  head: THREE.BufferGeometry;
  blinkL: THREE.BufferGeometry;
  blinkR: THREE.BufferGeometry;
  accent: THREE.BufferGeometry | null;
}

function lights(L: number, W: number, yTail: number, yHead: number): Pick<VehicleGeometry, "tail" | "head" | "blinkL" | "blinkR"> {
  const hx = L / 2;
  const hz = W / 2;
  return {
    tail: merge([box(0.06, 0.12, W * 0.26, -hx - 0.01, yTail, hz * 0.62), box(0.06, 0.12, W * 0.26, -hx - 0.01, yTail, -hz * 0.62)]),
    head: merge([box(0.06, 0.09, W * 0.24, hx + 0.01, yHead, hz * 0.62), box(0.06, 0.09, W * 0.24, hx + 0.01, yHead, -hz * 0.62)]),
    blinkL: merge([box(0.12, 0.09, 0.14, hx - 0.05, yHead, -hz + 0.02), box(0.12, 0.09, 0.14, -hx + 0.02, yTail, -hz + 0.02)]),
    blinkR: merge([box(0.12, 0.09, 0.14, hx - 0.05, yHead, hz - 0.02), box(0.12, 0.09, 0.14, -hx + 0.02, yTail, hz - 0.02)]),
  };
}

function sedan(L: number, W: number, H: number, taxi: boolean): VehicleGeometry {
  const hx = L / 2;
  const belt = H * 0.64;
  const body = extrudeProfile(
    [
      [-hx + 0.03, 0.3],
      [hx - 0.06, 0.3],
      [hx, 0.5],
      [hx - 0.08, belt * 0.78],
      [hx - 0.45, belt * 0.9],
      [hx * 0.4, belt],
      [-hx * 0.74, belt * 1.02],
      [-hx + 0.08, belt * 0.95],
      [-hx, belt * 0.7],
    ],
    W,
    0.06,
  );
  const glass = extrudeProfile(
    [
      [hx * 0.4, belt - 0.01],
      [hx * 0.05, H],
      [-hx * 0.45, H + 0.01],
      [-hx * 0.84, belt + 0.01],
    ],
    W * 0.86,
    0.04,
  );
  return {
    body,
    glass,
    wheels: wheels([hx * 0.61, -hx * 0.61], W / 2 - 0.12, 0.34, 0.24),
    ...lights(L, W, belt * 0.9, belt * 0.72),
    accent: taxi ? box(0.34, 0.14, 0.62, -hx * 0.2, H + 0.08, 0) : null,
  };
}

function van(L: number, W: number, H: number): VehicleGeometry {
  const hx = L / 2;
  const body = extrudeProfile(
    [
      [-hx, 0.35],
      [hx - 0.05, 0.35],
      [hx, 0.6],
      [hx - 0.1, 1.05],
      [hx - 0.75, H * 0.78],
      [hx - 1.1, H],
      [-hx, H],
    ],
    W,
    0.05,
  );
  const glass = extrudeProfile(
    [
      [hx - 0.12, 1.07],
      [hx - 0.74, H * 0.77],
      [hx - 1.02, H * 0.92],
      [hx - 1.02, 1.1],
    ],
    W * 0.9,
    0.02,
  );
  return { body, glass, wheels: wheels([hx * 0.66, -hx * 0.62], W / 2 - 0.14, 0.36, 0.26), ...lights(L, W, 0.9, 0.8), accent: null };
}

function bus(L: number, W: number, H: number): VehicleGeometry {
  const hx = L / 2;
  const body = box(L, H - 0.35, W, 0, (H - 0.35) / 2 + 0.35, 0);
  const glass = merge([box(L * 0.92, H * 0.32, W + 0.02, -0.1, H * 0.62, 0), box(0.04, H * 0.45, W * 0.9, hx + 0.01, H * 0.58, 0)]);
  return {
    body,
    glass,
    wheels: wheels([hx * 0.66, -hx * 0.5], W / 2 - 0.2, 0.5, 0.32),
    ...lights(L, W, 0.95, 0.75),
    accent: box(L * 0.95, 0.12, W * 0.7, 0, H + 0.06, 0),
  };
}

function truck(L: number, W: number, H: number): VehicleGeometry {
  const hx = L / 2;
  const cabL = 2.1;
  const cab = box(cabL, 2.5, W, hx - cabL / 2, 1.25 + 0.35, 0);
  const cargo = box(L - cabL - 0.25, H - 0.6, W, -cabL / 2 - 0.12, (H - 0.6) / 2 + 0.6, 0);
  return {
    body: merge([cab, cargo]),
    glass: box(0.04, 0.8, W * 0.86, hx + 0.01, 2.2, 0),
    wheels: wheels([hx * 0.72, -hx * 0.45, -hx * 0.72], W / 2 - 0.2, 0.48, 0.34),
    ...lights(L, W, 0.9, 0.8),
    accent: null,
  };
}

function twoWheeler(L: number, W: number, motor: boolean): VehicleGeometry {
  const hx = L / 2;
  const frame = motor
    ? merge([box(L * 0.62, 0.34, W * 0.55, 0, 0.62, 0), box(0.5, 0.16, W * 0.5, -hx * 0.3, 0.86, 0)])
    : merge([box(L * 0.62, 0.05, 0.05, 0, 0.68, 0), box(0.05, 0.5, 0.05, -hx * 0.2, 0.62, 0), box(0.05, 0.55, 0.05, hx * 0.45, 0.66, 0)]);
  const rider = merge([
    box(0.3, 0.62, 0.4, -hx * 0.12, 1.22, 0),
    (() => {
      const g = new THREE.SphereGeometry(0.15, 10, 8);
      g.translate(-hx * 0.02, 1.68, 0);
      return g;
    })(),
  ]);
  const wheelR = motor ? 0.3 : 0.34;
  const wheelParts: THREE.BufferGeometry[] = [];
  for (const x of [hx - wheelR, -hx + wheelR]) {
    const c = new THREE.CylinderGeometry(wheelR, wheelR, motor ? 0.12 : 0.04, 16);
    c.rotateX(Math.PI / 2);
    c.translate(x, wheelR, 0);
    wheelParts.push(c);
  }
  return {
    body: frame,
    glass: null,
    wheels: merge(wheelParts),
    tail: box(0.05, 0.08, 0.14, -hx, 0.8, 0),
    head: box(0.05, 0.1, 0.14, hx, 0.85, 0),
    blinkL: box(0.08, 0.06, 0.06, hx - 0.1, 0.85, -W / 2),
    blinkR: box(0.08, 0.06, 0.06, hx - 0.1, 0.85, W / 2),
    accent: rider,
  };
}

export function createVehicleGeometry(kind: VehicleKind | "ego"): VehicleGeometry {
  if (kind === "ego") return sedan(4.7, 1.9, 1.44, false);
  const s = VEHICLES[kind];
  switch (kind) {
    case "car":
      return sedan(s.length, s.width, s.height, false);
    case "suv":
      return sedan(s.length, s.width, s.height, false);
    case "taxi":
      return sedan(s.length, s.width, s.height, true);
    case "van":
      return van(s.length, s.width, s.height);
    case "bus":
      return bus(s.length, s.width, s.height);
    case "truck":
      return truck(s.length, s.width, s.height);
    case "moto":
      return twoWheeler(s.length, s.width, true);
    case "bike":
      return twoWheeler(s.length, s.width, false);
  }
}

export interface VehicleMaterials {
  glass: THREE.Material;
  wheel: THREE.Material;
  tailOff: THREE.Material;
  tailOn: THREE.Material;
  head: THREE.Material;
  blinkOn: THREE.Material;
  blinkOff: THREE.Material;
  taxiSign: THREE.Material;
  rider: THREE.Material;
  busRoof: THREE.Material;
}

export function createVehicleMaterials(): VehicleMaterials {
  return {
    glass: new THREE.MeshStandardMaterial({ color: 0x0d1015, roughness: 0.15, metalness: 0.6 }),
    wheel: new THREE.MeshStandardMaterial({ color: 0x141518, roughness: 0.8 }),
    tailOff: new THREE.MeshBasicMaterial({ color: 0x5a1116 }),
    tailOn: new THREE.MeshBasicMaterial({ color: 0xff2a36 }),
    head: new THREE.MeshBasicMaterial({ color: 0xe8f0ff }),
    blinkOn: new THREE.MeshBasicMaterial({ color: 0xffa31a }),
    blinkOff: new THREE.MeshBasicMaterial({ color: 0x3b2a12 }),
    taxiSign: new THREE.MeshBasicMaterial({ color: 0x8fd18f }),
    rider: new THREE.MeshStandardMaterial({ color: 0x565b66, roughness: 0.8 }),
    busRoof: new THREE.MeshStandardMaterial({ color: 0xd8dde3, roughness: 0.6 }),
  };
}

export class VehicleView {
  readonly group = new THREE.Group();
  readonly body: THREE.Mesh;
  private readonly tail: THREE.Mesh;
  private readonly blinkL: THREE.Mesh;
  private readonly blinkR: THREE.Mesh;

  constructor(
    kind: VehicleKind | "ego",
    geo: VehicleGeometry,
    private readonly mats: VehicleMaterials,
    bodyMat: THREE.Material,
  ) {
    this.body = new THREE.Mesh(geo.body, bodyMat);
    this.body.castShadow = true;
    const wheelsMesh = new THREE.Mesh(geo.wheels, mats.wheel);
    this.tail = new THREE.Mesh(geo.tail, mats.tailOff);
    const head = new THREE.Mesh(geo.head, mats.head);
    this.blinkL = new THREE.Mesh(geo.blinkL, mats.blinkOff);
    this.blinkR = new THREE.Mesh(geo.blinkR, mats.blinkOff);
    this.group.add(this.body, wheelsMesh, this.tail, head, this.blinkL, this.blinkR);
    if (geo.glass) {
      const g = new THREE.Mesh(geo.glass, mats.glass);
      g.castShadow = true;
      this.group.add(g);
    }
    if (geo.accent) {
      const mat = kind === "taxi" ? mats.taxiSign : kind === "bus" ? mats.busRoof : mats.rider;
      const a = new THREE.Mesh(geo.accent, mat);
      a.castShadow = true;
      this.group.add(a);
    }
  }

  /** blinker: -1 left, 1 right, 2 hazard, 0 off. */
  setLights(braking: boolean, blinker: number, phase: boolean): void {
    this.tail.material = braking ? this.mats.tailOn : this.mats.tailOff;
    this.blinkL.material = phase && (blinker === -1 || blinker === 2) ? this.mats.blinkOn : this.mats.blinkOff;
    this.blinkR.material = phase && (blinker === 1 || blinker === 2) ? this.mats.blinkOn : this.mats.blinkOff;
  }

  setBodyMaterial(m: THREE.Material): void {
    if (this.body.material !== m) this.body.material = m;
  }
}
