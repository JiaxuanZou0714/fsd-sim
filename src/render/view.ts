import * as THREE from "three";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import type { Vec2 } from "../sim/geometry";
import { ROAD_HALF, laneOffset } from "../sim/world";
import type { Simulation } from "../sim/simulation";
import { CityView } from "./city";
import { CarView, createCarGeometry, createCarMaterials } from "./models";

export type CameraMode = "follow" | "chase" | "top";
export const CAMERA_MODES: readonly CameraMode[] = ["follow", "chase", "top"];

const RIBBON_POINTS = 90;
const MAX_PEDS = 200;

const ribbonVertex = /* glsl */ `
  attribute float alpha;
  varying float vAlpha;
  varying float vSide;
  attribute float side;
  void main() {
    vAlpha = alpha;
    vSide = side;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;
const ribbonFragment = /* glsl */ `
  uniform vec3 color;
  uniform float opacity;
  varying float vAlpha;
  varying float vSide;
  void main() {
    float edge = smoothstep(1.0, 0.72, abs(vSide));
    float rim = smoothstep(0.78, 0.95, abs(vSide)) * (1.0 - smoothstep(0.95, 1.0, abs(vSide)));
    float a = (edge * 0.55 + rim * 0.9) * vAlpha * opacity;
    gl_FragColor = vec4(color + rim * 0.25, a);
  }
`;

export class View {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  cameraMode: CameraMode = "follow";

  private readonly city: CityView;
  private readonly carGeo = createCarGeometry();
  private readonly carMats = createCarMaterials();
  private readonly npcBodyMats = new Map<number, THREE.Material>();
  private readonly leadMat = new THREE.MeshStandardMaterial({ color: 0x3f7cff, roughness: 0.35, metalness: 0.3, emissive: 0x0b2a80, emissiveIntensity: 0.6 });
  private readonly stalledMat = new THREE.MeshStandardMaterial({ color: 0xc27a2c, roughness: 0.5, metalness: 0.2 });
  private readonly egoCar: CarView;
  private readonly npcViews = new Map<number, CarView>();

  private readonly pedBody: THREE.InstancedMesh;
  private readonly pedHead: THREE.InstancedMesh;

  private readonly ribbon: THREE.Mesh;
  private readonly ribbonGeo: THREE.BufferGeometry;
  private readonly ribbonMat: THREE.ShaderMaterial;
  private readonly destPin: THREE.Group;
  private readonly stopBar: THREE.Mesh;
  private readonly sun: THREE.DirectionalLight;

  private camHeading = 0;
  private camPos = new THREE.Vector3();
  private camTarget = new THREE.Vector3();
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
    this.scene.fog = new THREE.Fog(bg, 110, 360);
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environmentIntensity = 0.35;

    this.camera = new THREE.PerspectiveCamera(55, 1, 0.5, 900);

    const hemi = new THREE.HemisphereLight(0xc8d4ff, 0x1a1c22, 0.85);
    this.scene.add(hemi);
    this.sun = new THREE.DirectionalLight(0xffffff, 1.6);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = -70;
    sc.right = 70;
    sc.top = 70;
    sc.bottom = -70;
    sc.near = 1;
    sc.far = 260;
    this.sun.shadow.bias = -0.0004;
    this.scene.add(this.sun, this.sun.target);

    this.city = new CityView(sim.world);
    this.scene.add(this.city.group);

    const egoMat = new THREE.MeshPhysicalMaterial({
      color: 0xf2f3f5,
      roughness: 0.28,
      metalness: 0.1,
      clearcoat: 1,
      clearcoatRoughness: 0.12,
    });
    this.egoCar = new CarView(this.carGeo, this.carMats, egoMat);
    this.scene.add(this.egoCar.group);

    const bodyGeo = new THREE.CapsuleGeometry(0.22, 0.85, 4, 10);
    bodyGeo.translate(0, 0.72, 0);
    const headGeo = new THREE.SphereGeometry(0.14, 12, 10);
    headGeo.translate(0, 1.5, 0);
    const pedMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.7 });
    this.pedBody = new THREE.InstancedMesh(bodyGeo, pedMat, MAX_PEDS);
    this.pedHead = new THREE.InstancedMesh(headGeo, pedMat, MAX_PEDS);
    this.pedBody.castShadow = true;
    this.pedBody.count = 0;
    this.pedHead.count = 0;
    this.pedBody.frustumCulled = false;
    this.pedHead.frustumCulled = false;
    for (let i = 0; i < MAX_PEDS; i++) {
      this.pedBody.setColorAt(i, new THREE.Color(0xcfd3da));
      this.pedHead.setColorAt(i, new THREE.Color(0xcfd3da));
    }
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
      uniforms: { color: { value: new THREE.Color(0x2f7bff) }, opacity: { value: 1 } },
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this.ribbon = new THREE.Mesh(this.ribbonGeo, this.ribbonMat);
    this.ribbon.frustumCulled = false;
    this.ribbon.renderOrder = 2;
    this.scene.add(this.ribbon);

    this.destPin = new THREE.Group();
    const pinMat = new THREE.MeshBasicMaterial({ color: 0x3f8cff });
    const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 3.2, 8), pinMat);
    stem.position.y = 1.6;
    const ball = new THREE.Mesh(new THREE.SphereGeometry(0.45, 18, 12), pinMat);
    ball.position.y = 3.4;
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(1.1, 1.5, 40),
      new THREE.MeshBasicMaterial({ color: 0x3f8cff, transparent: true, opacity: 0.6, side: THREE.DoubleSide }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.05;
    ring.name = "ring";
    this.destPin.add(stem, ball, ring);
    this.destPin.visible = false;
    this.scene.add(this.destPin);

    this.stopBar = new THREE.Mesh(
      new THREE.BoxGeometry(0.5, 0.05, ROAD_HALF - 0.6),
      new THREE.MeshBasicMaterial({ color: 0xff3344, transparent: true, opacity: 0.9 }),
    );
    this.stopBar.visible = false;
    this.scene.add(this.stopBar);
  }

  resize(): void {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / Math.max(1, h);
    this.camera.updateProjectionMatrix();
  }

  /** Ground point under a canvas pixel, in simulation coordinates. */
  pick(clientX: number, clientY: number): Vec2 | null {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    const hit = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.groundPlane, hit)) return null;
    return { x: hit.x, y: hit.z };
  }

  private npcBodyMat(tint: number): THREE.Material {
    let m = this.npcBodyMats.get(tint);
    if (!m) {
      m = new THREE.MeshStandardMaterial({ color: tint, roughness: 0.55, metalness: 0.15 });
      this.npcBodyMats.set(tint, m);
    }
    return m;
  }

  render(sim: Simulation, dt: number, wallTime: number): void {
    const ego = sim.ego;
    const blinkPhase = Math.floor(wallTime * 2.6) % 2 === 0;
    this.city.updateSignals(sim.t);

    // Ego vehicle
    this.egoCar.group.position.set(ego.pos.x, 0, ego.pos.y);
    this.egoCar.group.rotation.y = -ego.heading;
    this.egoCar.setLights(ego.accel < -0.6 || (ego.v < 0.1 && ego.mode === "fsd"), ego.blinker, blinkPhase);

    // NPC vehicles
    const seen = new Set<number>();
    for (const car of sim.traffic.cars) {
      seen.add(car.id);
      let view = this.npcViews.get(car.id);
      if (!view) {
        view = new CarView(this.carGeo, this.carMats, this.npcBodyMat(car.bodyTint));
        this.npcViews.set(car.id, view);
        this.scene.add(view.group);
      }
      view.group.position.set(car.pos.x, 0, car.pos.y);
      view.group.rotation.y = -car.heading;
      const isLead = ego.mode === "fsd" && ego.leadId === car.id;
      view.setBodyMaterial(isLead ? this.leadMat : car.stalled ? this.stalledMat : this.npcBodyMat(car.bodyTint));
      view.setLights(car.accel < -0.8 || car.v < 0.1, car.blinker, blinkPhase);
    }
    for (const [id, view] of this.npcViews) {
      if (seen.has(id)) continue;
      this.scene.remove(view.group);
      this.npcViews.delete(id);
    }

    // Pedestrians
    const m = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const up = new THREE.Vector3(0, 1, 0);
    const color = new THREE.Color();
    const peds = sim.peds.peds;
    const count = Math.min(peds.length, MAX_PEDS);
    for (let i = 0; i < count; i++) {
      const p = peds[i];
      if (!p) continue;
      const bob = p.v > 0.1 ? Math.abs(Math.sin(p.gait)) * 0.06 : 0;
      q.setFromAxisAngle(up, -p.heading);
      m.compose(new THREE.Vector3(p.pos.x, bob, p.pos.y), q, new THREE.Vector3(1, 1, 1));
      this.pedBody.setMatrixAt(i, m);
      this.pedHead.setMatrixAt(i, m);
      const yielding = ego.mode === "fsd" && ego.leadId === p.id;
      const tones = [0xcfd3da, 0xb9bec7, 0xa9afb9, 0xd9dce2];
      color.setHex(yielding ? 0xff5a3c : p.jaywalker ? 0xffb347 : (tones[p.tone] ?? 0xcfd3da));
      this.pedBody.setColorAt(i, color);
      this.pedHead.setColorAt(i, color);
    }
    this.pedBody.count = count;
    this.pedHead.count = count;
    this.pedBody.instanceMatrix.needsUpdate = true;
    this.pedHead.instanceMatrix.needsUpdate = true;
    if (this.pedBody.instanceColor) this.pedBody.instanceColor.needsUpdate = true;
    if (this.pedHead.instanceColor) this.pedHead.instanceColor.needsUpdate = true;

    this.updateRibbon(sim, wallTime);
    this.updateMarkers(sim, wallTime);
    this.updateCamera(sim, dt);

    this.sun.position.set(ego.pos.x + 40, 90, ego.pos.y + 25);
    this.sun.target.position.set(ego.pos.x, 0, ego.pos.y);

    this.renderer.render(this.scene, this.camera);
  }

  private updateRibbon(sim: Simulation, wallTime: number): void {
    const ego = sim.ego;
    const path = ego.path;
    if (ego.mode !== "fsd" || !path) {
      this.ribbon.visible = false;
      return;
    }
    this.ribbon.visible = true;
    const pos = this.ribbonGeo.getAttribute("position") as THREE.BufferAttribute;
    const alpha = this.ribbonGeo.getAttribute("alpha") as THREE.BufferAttribute;
    const start = ego.pathS + 2.2;
    const lengthAhead = Math.max(0.5, Math.min(75, ego.destS - start));
    const halfW = 0.95;
    const pulse = 0.85 + 0.15 * Math.sin(wallTime * 3);
    for (let i = 0; i < RIBBON_POINTS; i++) {
      const t = i / (RIBBON_POINTS - 1);
      const smp = path.sampleAt(start + t * lengthAhead);
      const rx = -smp.dir.y;
      const ry = smp.dir.x;
      pos.setXYZ(i * 2, smp.p.x - rx * halfW, 0.04, smp.p.y - ry * halfW);
      pos.setXYZ(i * 2 + 1, smp.p.x + rx * halfW, 0.04, smp.p.y + ry * halfW);
      const fadeIn = Math.min(1, t * 12);
      const fadeOut = 1 - Math.pow(t, 2.2);
      const a = fadeIn * fadeOut * pulse;
      alpha.setX(i * 2, a);
      alpha.setX(i * 2 + 1, a);
    }
    pos.needsUpdate = true;
    alpha.needsUpdate = true;
    const stopping = ego.intent === "light" || ego.intent === "yellow" || ego.intent === "emergency" || ego.intent === "ped";
    (this.ribbonMat.uniforms.color as { value: THREE.Color }).value.setHex(stopping ? 0x5d7fb8 : 0x2f7bff);
  }

  private updateMarkers(sim: Simulation, wallTime: number): void {
    const ego = sim.ego;
    const dest = ego.dest;
    this.destPin.visible = !!dest;
    if (dest) {
      this.destPin.position.set(dest.pos.x, Math.sin(wallTime * 2.2) * 0.15, dest.pos.y);
      const ring = this.destPin.getObjectByName("ring");
      if (ring) ring.scale.setScalar(1 + 0.12 * Math.sin(wallTime * 3));
    }
    const stop = ego.activeStop;
    const show = ego.mode === "fsd" && stop && (ego.intent === "light" || ego.intent === "yellow");
    this.stopBar.visible = !!show;
    if (show && stop) {
      const { pos, dir } = stop.marker;
      this.stopBar.position.set(pos.x - dir.x * 0.25, 0.06, pos.y - dir.y * 0.25);
      // Centre the bar across the approach half of the road.
      const laneCentre = laneOffset(ego.lane);
      this.stopBar.position.x += dir.y * (laneCentre - 3.5);
      this.stopBar.position.z -= dir.x * (laneCentre - 3.5);
      this.stopBar.rotation.y = -Math.atan2(dir.y, dir.x);
      (this.stopBar.material as THREE.MeshBasicMaterial).color.setHex(stop.color === "Y" ? 0xffc21a : 0xff3344);
    }
  }

  private updateCamera(sim: Simulation, dt: number): void {
    const ego = sim.ego;
    const target = ego.heading;
    let diff = target - this.camHeading;
    while (diff > Math.PI) diff -= Math.PI * 2;
    while (diff < -Math.PI) diff += Math.PI * 2;
    if (!this.camInit) {
      this.camHeading = target;
    } else {
      this.camHeading += diff * Math.min(1, dt * 2.2);
    }
    const fx = Math.cos(this.camHeading);
    const fy = Math.sin(this.camHeading);
    let back = 15;
    let height = 8.5;
    let ahead = 14;
    if (this.cameraMode === "chase") {
      back = 8.5;
      height = 3.4;
      ahead = 10;
    }
    const desiredPos = new THREE.Vector3();
    const desiredTarget = new THREE.Vector3();
    if (this.cameraMode === "top") {
      desiredPos.set(ego.pos.x - fx * 6, 95, ego.pos.y - fy * 6 + 28);
      desiredTarget.set(ego.pos.x + fx * 6, 0, ego.pos.y + fy * 6);
    } else {
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
