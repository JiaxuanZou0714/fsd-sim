import * as THREE from "three";
import { Sky } from "three/examples/jsm/objects/Sky.js";
import { Rng } from "../sim/geometry";

export type TimeOfDay = "day" | "dusk" | "night";

export interface Lighting {
  /** 0 in daylight, 1 at night: drives windows, street lamps and headlights. */
  night: number;
  bloom: number;
}

interface Preset {
  elevation: number;
  azimuth: number;
  sunColor: number;
  sunIntensity: number;
  hemiSky: number;
  hemiGround: number;
  hemiIntensity: number;
  fog: number;
  exposure: number;
  turbidity: number;
  rayleigh: number;
  envIntensity: number;
  night: number;
  bloom: number;
  stars: number;
}

const PRESETS: Record<TimeOfDay, Preset> = {
  day: {
    elevation: 52,
    azimuth: 150,
    sunColor: 0xfff4e2,
    sunIntensity: 2.6,
    hemiSky: 0xcfe0ff,
    hemiGround: 0x7a6e5e,
    hemiIntensity: 0.9,
    fog: 0xc9d6e6,
    exposure: 0.62,
    turbidity: 4,
    rayleigh: 1.4,
    envIntensity: 0.5,
    night: 0,
    bloom: 0.12,
    stars: 0,
  },
  dusk: {
    elevation: 5,
    azimuth: 255,
    sunColor: 0xffa066,
    sunIntensity: 1.9,
    hemiSky: 0x8e9ccc,
    hemiGround: 0x4a3a3a,
    hemiIntensity: 0.7,
    fog: 0x9a8a9a,
    exposure: 0.55,
    turbidity: 9,
    rayleigh: 3,
    envIntensity: 0.35,
    night: 0.65,
    bloom: 0.55,
    stars: 0.15,
  },
  night: {
    elevation: -12,
    azimuth: 60,
    sunColor: 0x8fa6ff,
    sunIntensity: 0.35,
    hemiSky: 0x33406a,
    hemiGround: 0x1a1a22,
    hemiIntensity: 0.55,
    fog: 0x0d1220,
    exposure: 0.85,
    turbidity: 2,
    rayleigh: 0.3,
    envIntensity: 0.18,
    night: 1,
    bloom: 0.85,
    stars: 1,
  },
};

export class Environment {
  readonly sky = new Sky();
  readonly sun = new THREE.DirectionalLight(0xffffff, 2);
  readonly hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 0.8);
  private readonly stars: THREE.Points;
  private readonly sunDir = new THREE.Vector3();
  time: TimeOfDay = "dusk";

  constructor(
    private readonly scene: THREE.Scene,
    private readonly renderer: THREE.WebGLRenderer,
  ) {
    this.sky.scale.setScalar(4000);
    scene.add(this.sky, this.hemi, this.sun, this.sun.target);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = -90;
    sc.right = 90;
    sc.top = 90;
    sc.bottom = -90;
    sc.near = 1;
    sc.far = 400;
    this.sun.shadow.bias = -0.0005;
    this.sun.shadow.normalBias = 0.02;

    const rng = new Rng(21);
    const pos: number[] = [];
    for (let i = 0; i < 1500; i++) {
      const th = rng.range(0, Math.PI * 2);
      const ph = rng.range(0.08, Math.PI / 2);
      pos.push(Math.cos(th) * Math.cos(ph) * 1800, Math.sin(ph) * 1800, Math.sin(th) * Math.cos(ph) * 1800);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    this.stars = new THREE.Points(g, new THREE.PointsMaterial({ color: 0xffffff, size: 2.2, sizeAttenuation: false, transparent: true, opacity: 0, fog: false, depthWrite: false }));
    scene.add(this.stars);
  }

  apply(time: TimeOfDay): Lighting {
    this.time = time;
    const p = PRESETS[time];
    const u = this.sky.material.uniforms;
    u.turbidity.value = p.turbidity;
    u.rayleigh.value = p.rayleigh;
    u.mieCoefficient.value = 0.005;
    u.mieDirectionalG.value = 0.8;
    const phi = THREE.MathUtils.degToRad(90 - p.elevation);
    const theta = THREE.MathUtils.degToRad(p.azimuth);
    this.sunDir.setFromSphericalCoords(1, phi, theta);
    u.sunPosition.value.copy(this.sunDir);
    this.sun.color.setHex(p.sunColor);
    this.sun.intensity = p.sunIntensity;
    this.hemi.color.setHex(p.hemiSky);
    this.hemi.groundColor.setHex(p.hemiGround);
    this.hemi.intensity = p.hemiIntensity;
    const fogColor = new THREE.Color(p.fog);
    this.scene.fog = new THREE.Fog(fogColor, 180, 900);
    this.scene.background = fogColor;
    this.renderer.toneMappingExposure = p.exposure;
    this.scene.environmentIntensity = p.envIntensity;
    (this.stars.material as THREE.PointsMaterial).opacity = p.stars;
    this.stars.visible = p.stars > 0;
    return { night: p.night, bloom: p.bloom };
  }

  /** Keeps the shadow camera centred on the area around `focus`. */
  follow(focus: THREE.Vector3): void {
    // Night light comes from the moon on the opposite side, never from below the horizon.
    const dir = this.sunDir.clone();
    if (dir.y < 0.25) dir.y = 0.25 + Math.abs(dir.y);
    dir.normalize();
    this.sun.position.copy(focus).addScaledVector(dir, 220);
    this.sun.target.position.copy(focus);
    this.stars.position.copy(focus);
    this.sky.position.copy(focus);
  }
}
