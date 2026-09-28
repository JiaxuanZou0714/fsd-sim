import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { Rng, type Vec2, add, right, scale } from "../sim/geometry";
import {
  BLOCK,
  CROSS_IN,
  CROSS_OUT,
  DIRS,
  GRID,
  LANE_W,
  ROAD_HALF,
  STOP_OFF,
  type SignalColor,
  type Turn,
  World,
} from "../sim/world";

const MARK_Y = 0.015;

interface Strip {
  cx: number;
  cz: number;
  len: number;
  width: number;
  /** Angle of the strip's long axis around +y. */
  rot: number;
}

function stripMesh(strips: Strip[], color: number): THREE.InstancedMesh {
  const geo = new THREE.PlaneGeometry(1, 1);
  geo.rotateX(-Math.PI / 2);
  const mat = new THREE.MeshBasicMaterial({ color });
  const mesh = new THREE.InstancedMesh(geo, mat, Math.max(1, strips.length));
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const up = new THREE.Vector3(0, 1, 0);
  strips.forEach((s, i) => {
    q.setFromAxisAngle(up, s.rot);
    m.compose(new THREE.Vector3(s.cx, MARK_Y, s.cz), q, new THREE.Vector3(s.len, 1, s.width));
    mesh.setMatrixAt(i, m);
  });
  mesh.count = strips.length;
  mesh.instanceMatrix.needsUpdate = true;
  return mesh;
}

/** Strip along direction d starting at p, from along-offset a to b, at lateral offset lat. */
function along(p: Vec2, d: Vec2, a: number, b: number, lat: number, width: number): Strip {
  const r = right(d);
  const mid = add(add(p, scale(d, (a + b) / 2)), scale(r, lat));
  return { cx: mid.x, cz: mid.y, len: Math.abs(b - a), width, rot: -Math.atan2(d.y, d.x) };
}

interface LampRef {
  node: number;
  dirIn: number;
  kind: "R" | "Y" | "G" | "L";
}

export class CityView {
  readonly group = new THREE.Group();
  private readonly lamps: THREE.InstancedMesh;
  private readonly lampRefs: LampRef[] = [];
  private readonly colorTmp = new THREE.Color();

  constructor(private readonly world: World) {
    const rng = new Rng(99);
    this.buildGround();
    this.buildMarkings();
    this.buildBlocks(rng);
    this.lamps = this.buildSignals();
  }

  private buildGround(): void {
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(3000, 3000),
      new THREE.MeshStandardMaterial({ color: 0x17191d, roughness: 1 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.03;
    ground.receiveShadow = true;
    ground.name = "ground";
    this.group.add(ground);

    const roadMat = new THREE.MeshStandardMaterial({ color: 0x2a2d33, roughness: 0.92 });
    const span = (GRID - 1) * BLOCK;
    const parts: THREE.BufferGeometry[] = [];
    for (let k = 0; k < GRID; k++) {
      const h = new THREE.PlaneGeometry(span + ROAD_HALF * 2, ROAD_HALF * 2);
      h.rotateX(-Math.PI / 2);
      h.translate(span / 2, 0, k * BLOCK);
      const v = new THREE.PlaneGeometry(ROAD_HALF * 2, span + ROAD_HALF * 2);
      v.rotateX(-Math.PI / 2);
      v.translate(k * BLOCK, 0.001, span / 2);
      parts.push(h, v);
    }
    const merged = mergeGeometries(parts);
    if (!merged) throw new Error("Failed to merge road geometry");
    const roads = new THREE.Mesh(merged, roadMat);
    roads.receiveShadow = true;
    roads.name = "road";
    this.group.add(roads);
  }

  private buildMarkings(): void {
    const white: Strip[] = [];
    const yellow: Strip[] = [];
    const w = this.world;
    for (const n of w.nodes) {
      // Road segments to the east and south of each node (each undirected road once).
      for (const d of [0, 1]) {
        if ((n.out[d] as number) < 0) continue;
        const dv = DIRS[d] as Vec2;
        const a = STOP_OFF;
        const b = BLOCK - STOP_OFF;
        yellow.push(along(n.pos, dv, a, b, 0.14, 0.12), along(n.pos, dv, a, b, -0.14, 0.12));
        for (const side of [1, -1]) {
          white.push(along(n.pos, dv, CROSS_OUT, BLOCK - CROSS_OUT, side * (ROAD_HALF - 0.3), 0.15));
          for (let u = a + 1; u + 3 <= b; u += 8) white.push(along(n.pos, dv, u, u + 3, side * LANE_W, 0.13));
        }
      }
      for (let d = 0; d < 4; d++) {
        const armOut = n.out[d] as number;
        if (armOut < 0) continue;
        const dv = DIRS[d] as Vec2;
        const r = right(dv);
        // Crosswalk stripes on this arm, parallel to the arm.
        for (let lat = -ROAD_HALF + 0.6; lat <= ROAD_HALF - 0.5; lat += 1.1) {
          white.push(along(n.pos, dv, CROSS_IN, CROSS_OUT, lat, 0.55));
        }
        // Stop line for traffic approaching along -dv on this arm (its right side is -r).
        const stopC = add(add(n.pos, scale(dv, STOP_OFF + 0.25)), scale(r, -ROAD_HALF / 2));
        white.push({ cx: stopC.x, cz: stopC.y, len: 0.45, width: ROAD_HALF - 0.4, rot: -Math.atan2(dv.y, dv.x) });
      }
    }
    this.group.add(stripMesh(white, 0xc9ccd3), stripMesh(yellow, 0xd8ad3f));
  }

  private buildBlocks(rng: Rng): void {
    const walkMat = new THREE.MeshStandardMaterial({ color: 0x3a3e46, roughness: 0.95 });
    const lotMat = new THREE.MeshStandardMaterial({ color: 0x202328, roughness: 1 });
    const walkParts: THREE.BufferGeometry[] = [];
    const lotParts: THREE.BufferGeometry[] = [];
    const buildings: { x: number; z: number; w: number; d: number; h: number; c: number }[] = [];
    const palette = [0x2c3140, 0x333845, 0x2a2f3a, 0x3a3f4c, 0x262a33, 0x30384a];
    for (let bi = -1; bi < GRID; bi++) {
      for (let bj = -1; bj < GRID; bj++) {
        const x0 = bi * BLOCK + ROAD_HALF;
        const x1 = (bi + 1) * BLOCK - ROAD_HALF;
        const z0 = bj * BLOCK + ROAD_HALF;
        const z1 = (bj + 1) * BLOCK - ROAD_HALF;
        const slab = new THREE.BoxGeometry(x1 - x0, 0.16, z1 - z0);
        slab.translate((x0 + x1) / 2, 0.08 - 0.02, (z0 + z1) / 2);
        walkParts.push(slab);
        const inset = 4.4;
        const lot = new THREE.PlaneGeometry(x1 - x0 - inset * 2, z1 - z0 - inset * 2);
        lot.rotateX(-Math.PI / 2);
        lot.translate((x0 + x1) / 2, 0.145, (z0 + z1) / 2);
        lotParts.push(lot);
        const outer = bi < 0 || bj < 0 || bi >= GRID - 1 || bj >= GRID - 1;
        const lx0 = x0 + inset + 1.5;
        const lz0 = z0 + inset + 1.5;
        const size = x1 - x0 - (inset + 1.5) * 2;
        const cells = 2;
        const cell = size / cells;
        for (let a = 0; a < cells; a++) {
          for (let b = 0; b < cells; b++) {
            if (rng.chance(0.12)) continue;
            const w = cell * rng.range(0.62, 0.9);
            const d = cell * rng.range(0.62, 0.9);
            const h = outer ? rng.range(6, 18) : rng.range(8, 34);
            buildings.push({
              x: lx0 + cell * (a + 0.5),
              z: lz0 + cell * (b + 0.5),
              w,
              d,
              h,
              c: rng.pick(palette),
            });
          }
        }
      }
    }
    const walk = mergeGeometries(walkParts);
    const lots = mergeGeometries(lotParts);
    if (!walk || !lots) throw new Error("Failed to merge block geometry");
    const walkMesh = new THREE.Mesh(walk, walkMat);
    walkMesh.receiveShadow = true;
    const lotMesh = new THREE.Mesh(lots, lotMat);
    lotMesh.receiveShadow = true;
    this.group.add(walkMesh, lotMesh);

    const bGeo = new THREE.BoxGeometry(1, 1, 1);
    bGeo.translate(0, 0.5, 0);
    const bMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.85, metalness: 0.05 });
    const inst = new THREE.InstancedMesh(bGeo, bMat, buildings.length);
    const m = new THREE.Matrix4();
    const c = new THREE.Color();
    buildings.forEach((b, i) => {
      m.makeScale(b.w, b.h, b.d);
      m.setPosition(b.x, 0.14, b.z);
      inst.setMatrixAt(i, m);
      inst.setColorAt(i, c.setHex(b.c));
    });
    inst.castShadow = true;
    inst.receiveShadow = true;
    this.group.add(inst);

    // Faint roof outlines keep block shapes readable from the top-down camera.
    const edges = new THREE.EdgesGeometry(bGeo);
    const edgeParts = buildings.map((b) => {
      const g = edges.clone();
      g.scale(b.w, b.h, b.d);
      g.translate(b.x, 0.14, b.z);
      return g;
    });
    const mergedEdges = mergeGeometries(edgeParts);
    if (mergedEdges) {
      const lineMat = new THREE.LineBasicMaterial({ color: 0x4a5264, transparent: true, opacity: 0.55 });
      this.group.add(new THREE.LineSegments(mergedEdges, lineMat));
    }
  }

  private buildSignals(): THREE.InstancedMesh {
    const staticParts: THREE.BufferGeometry[] = [];
    const lampPositions: THREE.Vector3[] = [];
    const w = this.world;
    for (const n of w.nodes) {
      for (let d = 0; d < 4; d++) {
        // Traffic approaching the node travelling along d comes from arm (d+2)%4.
        if ((n.out[(d + 2) % 4] as number) < 0) continue;
        const dv = DIRS[d] as Vec2;
        const r = right(dv);
        const polePos = add(add(n.pos, scale(dv, ROAD_HALF + 2.2)), scale(r, ROAD_HALF + 1.2));
        const pole = new THREE.CylinderGeometry(0.12, 0.14, 6, 8);
        pole.translate(polePos.x, 3, polePos.y);
        staticParts.push(pole);
        const armLen = ROAD_HALF + 1.2 - 1.4;
        const armMid = add(polePos, scale(r, -armLen / 2));
        const arm = new THREE.BoxGeometry(0.12, 0.12, armLen);
        arm.rotateY(-Math.atan2(r.y, r.x) + Math.PI / 2);
        arm.translate(armMid.x, 5.8, armMid.y);
        staticParts.push(arm);
        const headPos = add(polePos, scale(r, -armLen + 0.3));
        const head = new THREE.BoxGeometry(0.4, 1.35, 0.5);
        head.rotateY(-Math.atan2(dv.y, dv.x));
        head.translate(headPos.x, 5.1, headPos.y);
        staticParts.push(head);
        const face = add(headPos, scale(dv, -0.22));
        const kinds: ["R" | "Y" | "G" | "L", number, number][] = [
          ["R", 5.55, 0],
          ["Y", 5.1, 0],
          ["G", 4.65, 0],
          ["L", 4.65, -0.45],
        ];
        for (const [kind, y, lat] of kinds) {
          const p = add(face, scale(r, lat));
          lampPositions.push(new THREE.Vector3(p.x, y, p.y));
          this.lampRefs.push({ node: n.id, dirIn: d, kind });
        }
        const leftHead = new THREE.BoxGeometry(0.36, 0.42, 0.42);
        leftHead.rotateY(-Math.atan2(dv.y, dv.x));
        const lh = add(headPos, scale(r, -0.45));
        leftHead.translate(lh.x, 4.65, lh.y);
        staticParts.push(leftHead);
      }
    }
    const merged = mergeGeometries(staticParts);
    if (!merged) throw new Error("Failed to merge signal geometry");
    const poles = new THREE.Mesh(merged, new THREE.MeshStandardMaterial({ color: 0x23262c, roughness: 0.6, metalness: 0.4 }));
    poles.castShadow = true;
    this.group.add(poles);

    const lampGeo = new THREE.SphereGeometry(0.15, 12, 8);
    const lamps = new THREE.InstancedMesh(lampGeo, new THREE.MeshBasicMaterial({ color: 0xffffff }), lampPositions.length);
    const m = new THREE.Matrix4();
    lampPositions.forEach((p, i) => {
      m.makeTranslation(p.x, p.y, p.z);
      lamps.setMatrixAt(i, m);
      lamps.setColorAt(i, new THREE.Color(0x222222));
    });
    this.group.add(lamps);
    return lamps;
  }

  updateSignals(t: number): void {
    const off: Record<LampRef["kind"], number> = { R: 0x3a1214, Y: 0x3a2e10, G: 0x0f2f1c, L: 0x0f2f1c };
    const on: Record<LampRef["kind"], number> = { R: 0xff3b3b, Y: 0xffc21a, G: 0x2cff7a, L: 0x2cff7a };
    const cache = new Map<string, SignalColor>();
    const sig = (node: number, dirIn: number, turn: Turn): SignalColor => {
      const key = `${node}:${dirIn}:${turn}`;
      let c = cache.get(key);
      if (!c) {
        c = this.world.signal(node, dirIn, turn, t);
        cache.set(key, c);
      }
      return c;
    };
    this.lampRefs.forEach((ref, i) => {
      let lit = false;
      let hex = on[ref.kind];
      if (ref.kind === "L") {
        const c = sig(ref.node, ref.dirIn, "left");
        lit = c !== "R";
        hex = c === "Y" ? on.Y : on.L;
      } else {
        lit = sig(ref.node, ref.dirIn, "straight") === ref.kind;
      }
      this.lamps.setColorAt(i, this.colorTmp.setHex(lit ? hex : off[ref.kind]));
    });
    if (this.lamps.instanceColor) this.lamps.instanceColor.needsUpdate = true;
  }
}
