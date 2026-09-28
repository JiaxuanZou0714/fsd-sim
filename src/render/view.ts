import * as THREE from "three";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import type { Vec2 } from "../sim/geometry";
import { LANE_W } from "../sim/map/network";
import type { Simulation } from "../sim/simulation";
import type { VehicleKind } from "../sim/vehicles";
import { CityView } from "./city";
import { type VehicleGeometry, VehicleView, createVehicleGeometry, createVehicleMaterials } from "./models";

export type CameraMode = "follow" | "chase" | "top";
export const CAMERA_MODES: readonly CameraMode[] = ["follow", "chase", "top"];

const RIBBON_POINTS = 110;
const MAX_PEDS = 400;

const ribbonVertex = /* glsl */ `
  attribute float alpha;
  attribute float side;
  varying float vAlpha;
  varying float vSide;
  void main() {
    vAlpha = alpha;
    vSide = side;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;
const ribbonFragment = /* glsl */ `
  uniform vec3 color;
  varying float vAlpha;
  varying float vSide;
  void main() {
    float edge = smoothstep(1.0, 0.72, abs(vSide));
    float rim = smoothstep(0.78, 0.95, abs(vSide)) * (1.0 - smoothstep(0.95, 1.0, abs(vSide)));
    gl_FragColor = vec4(color + rim * 0.25, (edge * 0.55 + rim * 0.9) * vAlpha);
  }
`;

export class View {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  cameraMode: CameraMode = "follow";
  showCandidates = true;
  showPredictions = false;

  private readonly city: CityView;
  private readonly mats = createVehicleMaterials();
  private readonly geos = new Map<VehicleKind | "ego", VehicleGeometry>();
  private readonly bodyMats = new Map<number, THREE.Material>();
  private readonly leadMat = new THREE.MeshStandardMaterial({ color: 0x3f7cff, roughness: 0.35, metalness: 0.3, emissive: 0x0b2a80, emissiveIntensity: 0.6 });
  private readonly stalledMat = new THREE.MeshStandardMaterial({ color: 0xc27a2c, roughness: 0.5, metalness: 0.2 });
  private readonly ego: VehicleView;
  private readonly vehicles = new Map<number, VehicleView>();
  private readonly pedBody: THREE.InstancedMesh;
  private readonly pedHead: THREE.InstancedMesh;
  private readonly ribbon: THREE.Mesh;
  private readonly ribbonGeo: THREE.BufferGeometry;
  private readonly ribbonMat: THREE.ShaderMaterial;
  private readonly candLines: THREE.LineSegments;
  private readonly predLines: THREE.LineSegments;
  private readonly freeLines: THREE.LineSegments;
  private readonly destPin: THREE.Group;
  private readonly stopBar: THREE.Mesh;
  private readonly sun: THREE.DirectionalLight;
  private camHeading = 0;
  private readonly camPos = new THREE.Vector3();
  private readonly camTarget = new THREE.Vector3();
  private camInit = false;
  private readonly raycaster = new THREE.Raycaster();
  private readonly groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

  constructor(
    private readonly canvas: HTMLCanvasElement,
    sim: Simulation,
  ) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: "high-performance" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    const bg = new THREE.Color(0x14161a);
    this.scene.background = bg;
    this.scene.fog = new THREE.Fog(bg, 140, 520);
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environmentIntensity = 0.35;
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.5, 1400);

    this.scene.add(new THREE.HemisphereLight(0xc8d4ff, 0x1a1c22, 0.9));
    this.sun = new THREE.DirectionalLight(0xffffff, 1.6);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = -80;
    sc.right = 80;
    sc.top = 80;
    sc.bottom = -80;
    sc.near = 1;
    sc.far = 300;
    this.sun.shadow.bias = -0.0004;
    this.scene.add(this.sun, this.sun.target);

    this.city = new CityView(sim.net, sim.traffic.busStopsList());
    this.scene.add(this.city.group);

    const egoMat = new THREE.MeshPhysicalMaterial({ color: 0xf2f3f5, roughness: 0.28, metalness: 0.1, clearcoat: 1, clearcoatRoughness: 0.12 });
    this.ego = new VehicleView("ego", this.geometry("ego"), this.mats, egoMat);
    this.scene.add(this.ego.group);

    const bodyGeo = new THREE.CapsuleGeometry(0.22, 0.85, 4, 10);
    bodyGeo.translate(0, 0.72, 0);
    const headGeo = new THREE.SphereGeometry(0.14, 12, 10);
    headGeo.translate(0, 1.5, 0);
    const pedMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.7 });
    this.pedBody = new THREE.InstancedMesh(bodyGeo, pedMat, MAX_PEDS);
    this.pedHead = new THREE.InstancedMesh(headGeo, pedMat, MAX_PEDS);
    for (const m of [this.pedBody, this.pedHead]) {
      m.count = 0;
      m.frustumCulled = false;
      for (let i = 0; i < MAX_PEDS; i++) m.setColorAt(i, new THREE.Color(0xcfd3da));
    }
    this.pedBody.castShadow = true;
    this.scene.add(this.pedBody, this.pedHead);

    this.ribbonGeo = new THREE.BufferGeometry();
    const n = RIBBON_POINTS * 2;
    this.ribbonGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(n * 3), 3));
    this.ribbonGeo.setAttribute("alpha", new THREE.BufferAttribute(new Float32Array(n), 1));
    const side = new Float32Array(n);
    for (let i = 0; i < RIBBON_POINTS; i++) {
      side[i * 2] = -1;
      side[i * 2 + 1] = 1;
    }
    this.ribbonGeo.setAttribute("side", new THREE.BufferAttribute(side, 1));
    const idx: number[] = [];
    for (let i = 0; i < RIBBON_POINTS - 1; i++) {
      const a = i * 2;
      idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
    }
    this.ribbonGeo.setIndex(idx);
    this.ribbonMat = new THREE.ShaderMaterial({
      vertexShader: ribbonVertex,
      fragmentShader: ribbonFragment,
      uniforms: { color: { value: new THREE.Color(0x2f7bff) } },
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this.ribbon = new THREE.Mesh(this.ribbonGeo, this.ribbonMat);
    this.ribbon.frustumCulled = false;
    this.ribbon.renderOrder = 2;
    this.scene.add(this.ribbon);

    const lineMat = (opacity: number): THREE.LineBasicMaterial =>
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity, depthWrite: false });
    this.candLines = new THREE.LineSegments(new THREE.BufferGeometry(), lineMat(0.55));
    this.predLines = new THREE.LineSegments(new THREE.BufferGeometry(), lineMat(0.7));
    this.freeLines = new THREE.LineSegments(new THREE.BufferGeometry(), lineMat(0.95));
    for (const l of [this.candLines, this.predLines, this.freeLines]) {
      l.frustumCulled = false;
      l.renderOrder = 3;
      this.scene.add(l);
    }

    this.destPin = new THREE.Group();
    const pinMat = new THREE.MeshBasicMaterial({ color: 0x3f8cff });
    const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 3.2, 8), pinMat);
    stem.position.y = 1.6;
    const ball = new THREE.Mesh(new THREE.SphereGeometry(0.45, 18, 12), pinMat);
    ball.position.y = 3.4;
    const ring = new THREE.Mesh(new THREE.RingGeometry(1.1, 1.5, 40), new THREE.MeshBasicMaterial({ color: 0x3f8cff, transparent: true, opacity: 0.6, side: THREE.DoubleSide }));
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.06;
    ring.name = "ring";
    this.destPin.add(stem, ball, ring);
    this.destPin.visible = false;
    this.scene.add(this.destPin);

    this.stopBar = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.05, LANE_W - 0.3), new THREE.MeshBasicMaterial({ color: 0xff3344, transparent: true, opacity: 0.9 }));
    this.stopBar.visible = false;
    this.scene.add(this.stopBar);
  }

  private geometry(kind: VehicleKind | "ego"): VehicleGeometry {
    let g = this.geos.get(kind);
    if (!g) {
      g = createVehicleGeometry(kind);
      this.geos.set(kind, g);
    }
    return g;
  }

  private bodyMat(color: number): THREE.Material {
    let m = this.bodyMats.get(color);
    if (!m) {
      m = new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.15 });
      this.bodyMats.set(color, m);
    }
    return m;
  }

  resize(): void {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / Math.max(1, h);
    this.camera.updateProjectionMatrix();
  }

  pick(clientX: number, clientY: number): Vec2 | null {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    const hit = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.groundPlane, hit)) return null;
    return { x: hit.x, y: hit.z };
  }

  render(sim: Simulation, dt: number, wall: number): void {
    const ego = sim.ego;
    const phase = Math.floor(wall * 2.6) % 2 === 0;
    this.city.updateSignals(sim.t);

    this.ego.group.position.set(ego.pos.x, 0, ego.pos.y);
    this.ego.group.rotation.y = -ego.heading;
    this.ego.setLights(ego.accel < -0.6 || (Math.abs(ego.v) < 0.1 && ego.mode === "fsd"), ego.blinker, phase);

    const seen = new Set<number>();
    for (const car of sim.traffic.vehicles) {
      seen.add(car.id);
      let view = this.vehicles.get(car.id);
      if (!view) {
        view = new VehicleView(car.vkind, this.geometry(car.vkind), this.mats, this.bodyMat(car.color));
        this.vehicles.set(car.id, view);
        this.scene.add(view.group);
      }
      view.group.position.set(car.pos.x, 0, car.pos.y);
      view.group.rotation.y = -car.heading;
      const lead = ego.mode === "fsd" && ego.leadId === car.id;
      view.setBodyMaterial(lead ? this.leadMat : car.stalled ? this.stalledMat : this.bodyMat(car.color));
      view.setLights(car.accel < -0.8 || car.v < 0.1, car.blinker, phase);
    }
    for (const [id, view] of this.vehicles) {
      if (seen.has(id)) continue;
      this.scene.remove(view.group);
      this.vehicles.delete(id);
    }

    const m = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const up = new THREE.Vector3(0, 1, 0);
    const col = new THREE.Color();
    const tones = [0xcfd3da, 0xb9bec7, 0xa9afb9, 0xd9dce2];
    const peds = sim.peds.peds;
    const count = Math.min(peds.length, MAX_PEDS);
    for (let i = 0; i < count; i++) {
      const p = peds[i];
      const bob = p.v > 0.1 ? Math.abs(Math.sin(p.gait)) * 0.06 : 0;
      q.setFromAxisAngle(up, -p.heading);
      m.compose(new THREE.Vector3(p.pos.x, bob, p.pos.y), q, new THREE.Vector3(1, 1, 1));
      this.pedBody.setMatrixAt(i, m);
      this.pedHead.setMatrixAt(i, m);
      const yielding = ego.mode === "fsd" && ego.leadId === p.id;
      col.setHex(yielding ? 0xff5a3c : p.jaywalker ? 0xffb347 : tones[p.tone] ?? 0xcfd3da);
      this.pedBody.setColorAt(i, col);
      this.pedHead.setColorAt(i, col);
    }
    for (const mesh of [this.pedBody, this.pedHead]) {
      mesh.count = count;
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    }

    this.updateRibbon(sim, wall);
    this.updateOverlays(sim);
    this.updateMarkers(sim, wall);
    this.updateCamera(sim, dt);
    this.sun.position.set(ego.pos.x + 45, 110, ego.pos.y + 30);
    this.sun.target.position.set(ego.pos.x, 0, ego.pos.y);
    this.renderer.render(this.scene, this.camera);
  }

  private updateRibbon(sim: Simulation, wall: number): void {
    const ego = sim.ego;
    const plan = ego.plan;
    const path = ego.planPath;
    if (ego.mode !== "fsd" || !plan || !path || ego.fsd !== "lane") {
      this.ribbon.visible = false;
      return;
    }
    this.ribbon.visible = true;
    const pos = this.ribbonGeo.getAttribute("position") as THREE.BufferAttribute;
    const alpha = this.ribbonGeo.getAttribute("alpha") as THREE.BufferAttribute;
    const pr = path.project(ego.pos);
    const start = pr.s + 2.2;
    const planned = plan.s[plan.s.length - 1];
    const length = Math.max(0.6, Math.min(75, Math.max(planned - pr.s + 3, 0.6)));
    const pulse = 0.85 + 0.15 * Math.sin(wall * 3);
    for (let i = 0; i < RIBBON_POINTS; i++) {
      const t = i / (RIBBON_POINTS - 1);
      const smp = path.sampleAt(start + t * length);
      const rx = -smp.dir.y;
      const ry = smp.dir.x;
      pos.setXYZ(i * 2, smp.p.x - rx * 0.95, 0.05, smp.p.y - ry * 0.95);
      pos.setXYZ(i * 2 + 1, smp.p.x + rx * 0.95, 0.05, smp.p.y + ry * 0.95);
      const a = Math.min(1, t * 12) * (1 - Math.pow(t, 2.2)) * pulse;
      alpha.setX(i * 2, a);
      alpha.setX(i * 2 + 1, a);
    }
    pos.needsUpdate = true;
    alpha.needsUpdate = true;
    const stopping = ["red", "yellow", "emergency", "ped", "yield", "obstacle"].includes(ego.intent);
    (this.ribbonMat.uniforms.color as { value: THREE.Color }).value.setHex(stopping ? 0x5d7fb8 : 0x2f7bff);
  }

  private setLines(target: THREE.LineSegments, segs: number[], colors: number[]): void {
    const g = target.geometry;
    g.setAttribute("position", new THREE.Float32BufferAttribute(segs, 3));
    g.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    g.computeBoundingSphere();
  }

  private updateOverlays(sim: Simulation): void {
    const ego = sim.ego;
    const segs: number[] = [];
    const cols: number[] = [];
    const push = (a: Vec2, b: Vec2, y: number, c: THREE.Color): void => {
      segs.push(a.x, y, a.y, b.x, y, b.y);
      cols.push(c.r, c.g, c.b, c.r, c.g, c.b);
    };
    const ok = new THREE.Color(0x6f8fbf);
    const bad = new THREE.Color(0xd9534f);
    if (this.showCandidates && ego.mode === "fsd" && ego.plan && ego.fsd === "lane") {
      for (const c of ego.plan.candidates) {
        if (c.chosen) continue;
        for (let i = 0; i + 1 < c.pts.length; i++) push(c.pts[i], c.pts[i + 1], 0.08, c.feasible ? ok : bad);
      }
    }
    this.setLines(this.candLines, segs, cols);

    const ps: number[] = [];
    const pc: number[] = [];
    if (this.showPredictions && ego.mode === "fsd") {
      const c1 = new THREE.Color(0xf0c060);
      for (const p of ego.lastPreds) {
        if (p.still) continue;
        for (let k = 0; k + 1 < p.pts.length; k += 2) {
          const a = p.pts[k];
          const b = p.pts[Math.min(p.pts.length - 1, k + 2)];
          ps.push(a.x, 0.12, a.y, b.x, 0.12, b.y);
          pc.push(c1.r, c1.g, c1.b, c1.r, c1.g, c1.b);
        }
      }
    }
    this.setLines(this.predLines, ps, pc);

    const fs: number[] = [];
    const fc: number[] = [];
    if (ego.mode === "fsd" && ego.free) {
      const fwd = new THREE.Color(0x35d6e8);
      const rev = new THREE.Color(0xffa94d);
      const path = ego.free.path;
      for (let i = 0; i + 1 < path.length; i++) {
        const c = path[i].dir > 0 ? fwd : rev;
        fs.push(path[i].x, 0.1, path[i].y, path[i + 1].x, 0.1, path[i + 1].y);
        fc.push(c.r, c.g, c.b, c.r, c.g, c.b);
      }
    }
    this.setLines(this.freeLines, fs, fc);
  }

  private updateMarkers(sim: Simulation, wall: number): void {
    const ego = sim.ego;
    const dest = ego.dest;
    this.destPin.visible = !!dest;
    if (dest) {
      this.destPin.position.set(dest.pos.x, Math.sin(wall * 2.2) * 0.15, dest.pos.y);
      this.destPin.getObjectByName("ring")?.scale.setScalar(1 + 0.12 * Math.sin(wall * 3));
    }
    const show = ego.mode === "fsd" && ego.ref && ego.plan && (ego.intent === "red" || ego.intent === "yellow");
    this.stopBar.visible = false;
    if (show && ego.ref && ego.plan) {
      const s0 = ego.plan.s0;
      const gate = ego.ref.gates.find((g) => g.s > s0 && g.s - s0 < 80);
      if (gate) {
        const l = gate.lane;
        const end = l.poly.sampleAt(l.poly.length - 0.3);
        this.stopBar.position.set(end.p.x, 0.07, end.p.y);
        this.stopBar.rotation.y = -Math.atan2(end.dir.y, end.dir.x);
        (this.stopBar.material as THREE.MeshBasicMaterial).color.setHex(ego.intent === "yellow" ? 0xffc21a : 0xff3344);
        this.stopBar.visible = true;
      }
    }
  }

  private updateCamera(sim: Simulation, dt: number): void {
    const ego = sim.ego;
    let diff = ego.heading - this.camHeading;
    while (diff > Math.PI) diff -= Math.PI * 2;
    while (diff < -Math.PI) diff += Math.PI * 2;
    if (!this.camInit) this.camHeading = ego.heading;
    else this.camHeading += diff * Math.min(1, dt * 2.2);
    const fx = Math.cos(this.camHeading);
    const fy = Math.sin(this.camHeading);
    const desiredPos = new THREE.Vector3();
    const desiredTarget = new THREE.Vector3();
    if (this.cameraMode === "top") {
      desiredPos.set(ego.pos.x - fx * 6, 130, ego.pos.y - fy * 6 + 40);
      desiredTarget.set(ego.pos.x + fx * 6, 0, ego.pos.y + fy * 6);
    } else {
      const [back, height, ahead] = this.cameraMode === "chase" ? [8.5, 3.4, 10] : [16, 9.5, 15];
      desiredPos.set(ego.pos.x - fx * back, height, ego.pos.y - fy * back);
      desiredTarget.set(ego.pos.x + fx * ahead, 0.8, ego.pos.y + fy * ahead);
    }
    if (!this.camInit) {
      this.camPos.copy(desiredPos);
      this.camTarget.copy(desiredTarget);
      this.camInit = true;
    } else {
      const k = Math.min(1, dt * 6);
      this.camPos.lerp(desiredPos, k);
      this.camTarget.lerp(desiredTarget, k);
    }
    this.camera.position.copy(this.camPos);
    this.camera.lookAt(this.camTarget);
  }
}
