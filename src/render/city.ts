import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { Polyline, type Vec2, add, lerp, polygonArea, scale } from "../sim/geometry";
import { type DirRoad, LANE_W, type Lane, type RoadNetwork, type SignalColor } from "../sim/map/network";

/** Triangle-strip ribbon along `pts`, spanning lateral offsets [left, right] (right-positive). */
function ribbon(pts: Vec2[], left: number, right: number, y: number, out: number[]): void {
  if (pts.length < 2) return;
  const n = pts.length;
  const verts: number[][] = [];
  for (let i = 0; i < n; i++) {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(n - 1, i + 1)];
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    const l = Math.hypot(dx, dy) || 1;
    dx /= l;
    dy /= l;
    const rx = -dy;
    const ry = dx;
    verts.push([pts[i].x + rx * left, pts[i].y + ry * left, pts[i].x + rx * right, pts[i].y + ry * right]);
  }
  for (let i = 0; i < n - 1; i++) {
    const [lx0, ly0, rx0, ry0] = verts[i];
    const [lx1, ly1, rx1, ry1] = verts[i + 1];
    out.push(lx0, y, ly0, rx0, y, ry0, lx1, y, ly1, rx0, y, ry0, rx1, y, ry1, lx1, y, ly1);
  }
}

function meshFrom(positions: number[], material: THREE.Material, receiveShadow = true): THREE.Mesh {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  g.computeVertexNormals();
  // Ribbons are built with consistent winding; make sure normals face up.
  const nrm = g.getAttribute("normal") as THREE.BufferAttribute;
  for (let i = 0; i < nrm.count; i++) nrm.setXYZ(i, 0, 1, 0);
  const m = new THREE.Mesh(g, material);
  m.receiveShadow = receiveShadow;
  return m;
}

function slice(poly: Polyline, s0: number, s1: number, step = 1): Vec2[] {
  const out: Vec2[] = [];
  for (let s = s0; s < s1; s += step) out.push(poly.sampleAt(s).p);
  out.push(poly.sampleAt(s1).p);
  return out;
}

/** Line halfway between two parallel lanes. */
function between(a: Lane, b: Lane): Vec2[] {
  const out: Vec2[] = [];
  const n = Math.max(2, Math.ceil(a.poly.length));
  for (let i = 0; i <= n; i++) {
    const pa = a.poly.sampleAt((a.poly.length * i) / n).p;
    const pb = b.poly.sampleAt((b.poly.length * i) / n).p;
    out.push(lerp(pa, pb, 0.5));
  }
  return out;
}

interface LampRef {
  road: DirRoad;
  kind: "R" | "Y" | "G";
}

export class CityView {
  readonly group = new THREE.Group();
  private lamps: THREE.InstancedMesh | null = null;
  private readonly lampRefs: LampRef[] = [];
  private readonly tmpColor = new THREE.Color();

  constructor(
    private readonly net: RoadNetwork,
    busStops: { lane: Lane; s: number }[],
  ) {
    this.buildGround();
    this.buildRoads();
    this.buildMarkings();
    this.buildBuildings();
    this.buildSignals();
    this.buildBusStops(busStops);
  }

  private buildGround(): void {
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(4000, 4000), new THREE.MeshStandardMaterial({ color: 0x191b1f, roughness: 1 }));
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.05;
    ground.receiveShadow = true;
    ground.name = "ground";
    this.group.add(ground);
    const parkMat = new THREE.MeshStandardMaterial({ color: 0x1f2b22, roughness: 1 });
    for (const p of this.net.parks) {
      const shape = new THREE.Shape(p.map((q) => new THREE.Vector2(q.x, -q.y)));
      const g = new THREE.ShapeGeometry(shape);
      g.rotateX(-Math.PI / 2);
      const m = new THREE.Mesh(g, parkMat);
      m.position.y = -0.02;
      m.receiveShadow = true;
      this.group.add(m);
    }
  }

  private buildRoads(): void {
    const road: number[] = [];
    const walk: number[] = [];
    for (const seg of this.net.segments) {
      const c = seg.center;
      const pts = slice(c, 0, c.length, 1.5);
      ribbon(pts, -seg.leftW - 0.35, seg.rightW + 0.35, 0.0, road);
      const s0 = Math.min(seg.trimA, c.length / 2);
      const s1 = Math.max(c.length - seg.trimB, c.length / 2);
      if (s1 - s0 > 1) {
        const trimmed = slice(c, s0, s1, 1.5);
        ribbon(trimmed, seg.rightW + 0.35, seg.rightW + 3.6, 0.1, walk);
        ribbon(trimmed, -seg.leftW - 3.6, -seg.leftW - 0.35, 0.1, walk);
      }
    }
    for (const j of this.net.junctions.values()) {
      const o = j.outline;
      if (o.length < 3) continue;
      const ccw = polygonArea(o) > 0 ? o : [...o].reverse();
      for (let i = 1; i < ccw.length - 1; i++) {
        const a = ccw[0];
        const b = ccw[i];
        const d = ccw[i + 1];
        road.push(a.x, 0.005, a.y, d.x, 0.005, d.y, b.x, 0.005, b.y);
      }
    }
    const roadMat = new THREE.MeshStandardMaterial({ color: 0x2b2e34, roughness: 0.92, side: THREE.DoubleSide });
    const walkMat = new THREE.MeshStandardMaterial({ color: 0x383c44, roughness: 0.95, side: THREE.DoubleSide });
    const rm = meshFrom(road, roadMat);
    rm.name = "road";
    this.group.add(rm, meshFrom(walk, walkMat));
  }

  private buildMarkings(): void {
    const white: number[] = [];
    const center: number[] = [];
    const Y = 0.03;
    const dashed = (pts: Vec2[], width: number, dash: number, gap: number, out: number[]): void => {
      const poly = new Polyline(pts);
      for (let s = 1; s + dash < poly.length - 1; s += dash + gap) ribbon(slice(poly, s, s + dash, 1), -width / 2, width / 2, Y, out);
    };
    for (const r of this.net.roads) {
      const lanes = r.lanes;
      for (let k = 0; k + 1 < lanes.length; k++) dashed(between(lanes[k], lanes[k + 1]), 0.13, 3, 5, white);
      // Right edge line.
      const right = lanes[0];
      if (right.poly.length > 2) ribbon(right.poly.pts, LANE_W / 2 - 0.2, LANE_W / 2 - 0.05, Y, white);
      // Divider between directions, drawn once per two-way segment on its forward road.
      const seg = r.seg;
      if (seg.forward === r && seg.backward) {
        const inner = lanes[lanes.length - 1];
        if (inner.poly.length > 2) {
          ribbon(inner.poly.pts, -LANE_W / 2 - 0.08, -LANE_W / 2 + 0.06, Y, center);
        }
      } else if (seg.oneway) {
        const left = lanes[lanes.length - 1];
        if (left.poly.length > 2) ribbon(left.poly.pts, -LANE_W / 2 + 0.05, -LANE_W / 2 + 0.2, Y, white);
      }
      // Stop line at signalised approaches.
      if (r.signalGroup >= 0) {
        for (const l of lanes) {
          const end = l.poly.sampleAt(l.poly.length - 0.3);
          const rr = { x: -end.dir.y, y: end.dir.x };
          ribbon([add(end.p, scale(rr, -LANE_W / 2)), add(end.p, scale(rr, LANE_W / 2))], -0.25, 0.25, Y, white);
        }
      }
    }
    for (const c of this.net.crossings) {
      const r = { x: -c.dir.y, y: c.dir.x };
      for (let lat = -c.seg.leftW + 0.3; lat <= c.seg.rightW - 0.3; lat += 1.0) {
        const p = add(c.pos, scale(r, lat));
        ribbon([add(p, scale(c.dir, -1.6)), add(p, scale(c.dir, 1.6))], -0.26, 0.26, Y + 0.002, white);
      }
    }
    const wm = new THREE.MeshBasicMaterial({ color: 0xc9ccd3, side: THREE.DoubleSide });
    const cm = new THREE.MeshBasicMaterial({ color: 0xd8ad3f, side: THREE.DoubleSide });
    this.group.add(meshFrom(white, wm, false), meshFrom(center, cm, false));
  }

  private buildBuildings(): void {
    const parts: THREE.BufferGeometry[] = [];
    const palette = [0x3a3d47, 0x41444f, 0x363943, 0x444855, 0x3d3a3f, 0x47464d];
    const color = new THREE.Color();
    for (const b of this.net.buildings) {
      if (b.pts.length < 3) continue;
      const shape = new THREE.Shape(b.pts.map((p) => new THREE.Vector2(p.x, -p.y)));
      let geo: THREE.BufferGeometry;
      try {
        geo = new THREE.ExtrudeGeometry(shape, { depth: b.h, bevelEnabled: false, curveSegments: 1 });
      } catch {
        continue;
      }
      geo.rotateX(-Math.PI / 2);
      const n = geo.getAttribute("position").count;
      const cols = new Float32Array(n * 3);
      color.setHex(palette[Math.abs(Math.floor(b.pts[0].x * 7 + b.pts[0].y * 13)) % palette.length]);
      for (let i = 0; i < n; i++) {
        cols[i * 3] = color.r;
        cols[i * 3 + 1] = color.g;
        cols[i * 3 + 2] = color.b;
      }
      geo.setAttribute("color", new THREE.BufferAttribute(cols, 3));
      geo.deleteAttribute("uv");
      parts.push(geo.index ? geo.toNonIndexed() : geo);
    }
    const merged = mergeGeometries(parts);
    if (!merged) return;
    merged.computeVertexNormals();
    const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.85, metalness: 0.05 });
    const mesh = new THREE.Mesh(merged, mat);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    this.group.add(mesh);
    const edges = new THREE.EdgesGeometry(merged, 30);
    this.group.add(new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x565d6e, transparent: true, opacity: 0.35 })));
  }

  private buildSignals(): void {
    const staticParts: THREE.BufferGeometry[] = [];
    const lampPos: THREE.Vector3[] = [];
    for (const r of this.net.roads) {
      if (r.signalGroup < 0) continue;
      const right = r.lanes[0];
      const end = right.poly.sampleAt(right.poly.length);
      const rr = { x: -end.dir.y, y: end.dir.x };
      const pole = add(add(end.p, scale(rr, LANE_W / 2 + 1.2)), scale(end.dir, -0.5));
      const g = new THREE.CylinderGeometry(0.1, 0.12, 4.2, 8);
      g.translate(pole.x, 2.1, pole.y);
      staticParts.push(g);
      const head = new THREE.BoxGeometry(0.34, 1.1, 0.4);
      head.rotateY(-Math.atan2(end.dir.y, end.dir.x));
      head.translate(pole.x, 4.6, pole.y);
      staticParts.push(head);
      const face = add(pole, scale(end.dir, -0.2));
      const kinds: ["R" | "Y" | "G", number][] = [
        ["R", 4.95],
        ["Y", 4.6],
        ["G", 4.25],
      ];
      for (const [kind, y] of kinds) {
        lampPos.push(new THREE.Vector3(face.x, y, face.y));
        this.lampRefs.push({ road: r, kind });
      }
    }
    if (staticParts.length === 0) return;
    const merged = mergeGeometries(staticParts);
    if (merged) {
      const m = new THREE.Mesh(merged, new THREE.MeshStandardMaterial({ color: 0x24272d, roughness: 0.6, metalness: 0.4 }));
      m.castShadow = true;
      this.group.add(m);
    }
    const lamps = new THREE.InstancedMesh(new THREE.SphereGeometry(0.13, 10, 8), new THREE.MeshBasicMaterial({ color: 0xffffff }), lampPos.length);
    const mat = new THREE.Matrix4();
    lampPos.forEach((p, i) => {
      mat.makeTranslation(p.x, p.y, p.z);
      lamps.setMatrixAt(i, mat);
      lamps.setColorAt(i, new THREE.Color(0x222222));
    });
    this.group.add(lamps);
    this.lamps = lamps;
  }

  private buildBusStops(stops: { lane: Lane; s: number }[]): void {
    const parts: THREE.BufferGeometry[] = [];
    for (const { lane, s } of stops) {
      const smp = lane.poly.sampleAt(s);
      const rr = { x: -smp.dir.y, y: smp.dir.x };
      const p = add(smp.p, scale(rr, LANE_W / 2 + 2.0));
      const roof = new THREE.BoxGeometry(4, 0.12, 1.4);
      roof.rotateY(-Math.atan2(smp.dir.y, smp.dir.x));
      roof.translate(p.x, 2.5, p.y);
      const back = new THREE.BoxGeometry(4, 2.3, 0.08);
      back.rotateY(-Math.atan2(smp.dir.y, smp.dir.x));
      const bp = add(p, scale(rr, 0.65));
      back.translate(bp.x, 1.3, bp.y);
      parts.push(roof, back);
    }
    const merged = mergeGeometries(parts);
    if (!merged) return;
    const m = new THREE.Mesh(merged, new THREE.MeshStandardMaterial({ color: 0x4d6a78, roughness: 0.4, metalness: 0.3, transparent: true, opacity: 0.85 }));
    m.castShadow = true;
    this.group.add(m);
  }

  updateSignals(t: number): void {
    if (!this.lamps) return;
    const off: Record<"R" | "Y" | "G", number> = { R: 0x3a1214, Y: 0x3a2e10, G: 0x0f2f1c };
    const on: Record<"R" | "Y" | "G", number> = { R: 0xff3b3b, Y: 0xffc21a, G: 0x2cff7a };
    const cache = new Map<DirRoad, SignalColor | null>();
    this.lampRefs.forEach((ref, i) => {
      let c = cache.get(ref.road);
      if (c === undefined) {
        c = this.net.laneSignal(ref.road.lanes[0], t);
        cache.set(ref.road, c);
      }
      this.lamps?.setColorAt(i, this.tmpColor.setHex(c === ref.kind ? on[ref.kind] : off[ref.kind]));
    });
    if (this.lamps.instanceColor) this.lamps.instanceColor.needsUpdate = true;
  }
}
