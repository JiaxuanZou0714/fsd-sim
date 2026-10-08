import * as THREE from "three";
import { Rng } from "../sim/geometry";

function canvas(w: number, h: number): [HTMLCanvasElement, CanvasRenderingContext2D] {
  const c = document.createElement("canvas");
  c.width = w;
  c.height = h;
  const g = c.getContext("2d");
  if (!g) throw new Error("2D canvas unavailable");
  return [c, g];
}

function toTexture(c: HTMLCanvasElement, srgb: boolean): THREE.CanvasTexture {
  const t = new THREE.CanvasTexture(c);
  t.wrapS = THREE.RepeatWrapping;
  t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

/** Asphalt with fine aggregate speckle and faint patches; tiles every 8 m. */
export function asphaltTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(512, 512);
  const rng = new Rng(11);
  g.fillStyle = "#56595e";
  g.fillRect(0, 0, 512, 512);
  for (let i = 0; i < 40; i++) {
    g.fillStyle = `rgba(${rng.chance(0.5) ? "30,32,36" : "90,92,96"},${rng.range(0.03, 0.08)})`;
    g.beginPath();
    g.ellipse(rng.range(0, 512), rng.range(0, 512), rng.range(30, 120), rng.range(20, 80), rng.range(0, 3), 0, Math.PI * 2);
    g.fill();
  }
  const img = g.getImageData(0, 0, 512, 512);
  for (let i = 0; i < img.data.length; i += 4) {
    const n = (rng.next() - 0.5) * 38;
    img.data[i] += n;
    img.data[i + 1] += n;
    img.data[i + 2] += n;
  }
  g.putImageData(img, 0, 0);
  return toTexture(c, true);
}

/** Pale limestone paving slabs, 1 m grid; tiles every 4 m. */
export function pavingTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(256, 256);
  const rng = new Rng(12);
  g.fillStyle = "#b9b2a6";
  g.fillRect(0, 0, 256, 256);
  for (let y = 0; y < 4; y++) {
    for (let x = 0; x < 4; x++) {
      const v = rng.range(-14, 14);
      g.fillStyle = `rgb(${185 + v},${178 + v},${166 + v})`;
      g.fillRect(x * 64 + 1, y * 64 + 1, 62, 62);
    }
  }
  g.strokeStyle = "rgba(90,84,76,0.55)";
  g.lineWidth = 1.5;
  for (let k = 0; k <= 4; k++) {
    g.beginPath();
    g.moveTo(k * 64, 0);
    g.lineTo(k * 64, 256);
    g.moveTo(0, k * 64);
    g.lineTo(256, k * 64);
    g.stroke();
  }
  return toTexture(c, true);
}

export interface FacadeTextures {
  map: THREE.CanvasTexture;
  lit: THREE.CanvasTexture;
  /** Metres covered by one texture repeat, horizontally and vertically. */
  size: [number, number];
}

/**
 * Haussmann-style limestone facade: tall windows with wrought-iron balcony rails, a stone
 * cornice on every floor. The emissive twin lights a random subset of windows for dusk and night.
 */
export function facadeTextures(): FacadeTextures {
  const cols = 8;
  const rows = 8;
  const cw = 64;
  const ch = 64;
  const [c, g] = canvas(cols * cw, rows * ch);
  const [lc, lg] = canvas(cols * cw, rows * ch);
  const rng = new Rng(13);
  g.fillStyle = "#e4d9c4";
  g.fillRect(0, 0, c.width, c.height);
  lg.fillStyle = "#000";
  lg.fillRect(0, 0, lc.width, lc.height);
  for (let r = 0; r < rows; r++) {
    // Floor cornice.
    g.fillStyle = "rgba(120,104,82,0.35)";
    g.fillRect(0, r * ch + ch - 5, c.width, 3);
    g.fillStyle = "rgba(255,250,240,0.35)";
    g.fillRect(0, r * ch + ch - 8, c.width, 2);
    for (let k = 0; k < cols; k++) {
      const x = k * cw + 18;
      const y = r * ch + 12;
      const w = 28;
      const h = 40;
      g.fillStyle = "#b8ab94";
      g.fillRect(x - 3, y - 3, w + 6, h + 5);
      const glass = g.createLinearGradient(x, y, x + w, y + h);
      glass.addColorStop(0, "#2d3844");
      glass.addColorStop(1, "#1b222b");
      g.fillStyle = glass;
      g.fillRect(x, y, w, h);
      g.fillStyle = "rgba(200,215,230,0.18)";
      g.fillRect(x + 2, y + 2, w / 2 - 3, h / 2);
      g.fillStyle = "#e9e1d0";
      g.fillRect(x + w / 2 - 1, y, 2, h);
      g.fillRect(x, y + h * 0.45, w, 2);
      // Balcony rail.
      g.fillStyle = "#23262a";
      g.fillRect(x - 4, y + h - 9, w + 8, 2);
      for (let b = 0; b < 8; b++) g.fillRect(x - 3 + b * 5, y + h - 9, 1, 8);
      if (rng.chance(0.38)) {
        const warm = rng.chance(0.75);
        lg.fillStyle = warm ? `rgb(255,${rng.int(190, 220)},${rng.int(130, 170)})` : "rgb(200,220,255)";
        lg.fillRect(x, y, w, h - 10);
      }
    }
  }
  const map = toTexture(c, true);
  const lit = toTexture(lc, true);
  return { map, lit, size: [cols * 3.6, rows * 3.3] };
}

/** Zinc mansard roof with dormer windows; one repeat is 4 dormers across. */
export function zincTextures(): FacadeTextures {
  const [c, g] = canvas(256, 64);
  const [lc, lg] = canvas(256, 64);
  const rng = new Rng(14);
  g.fillStyle = "#6f7880";
  g.fillRect(0, 0, 256, 64);
  for (let x = 0; x < 256; x += 6) {
    g.fillStyle = "rgba(40,48,56,0.35)";
    g.fillRect(x, 0, 1, 64);
  }
  lg.fillStyle = "#000";
  lg.fillRect(0, 0, 256, 64);
  for (let k = 0; k < 4; k++) {
    const x = k * 64 + 20;
    g.fillStyle = "#d9cfbc";
    g.fillRect(x - 3, 16, 30, 40);
    g.fillStyle = "#1f262e";
    g.fillRect(x, 22, 24, 30);
    if (rng.chance(0.3)) {
      lg.fillStyle = "rgb(255,205,150)";
      lg.fillRect(x, 22, 24, 30);
    }
  }
  return { map: toTexture(c, true), lit: toTexture(lc, true), size: [14.4, 3.5] };
}

/** Soft radial glow for light sprites. */
export function glowTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(128, 128);
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  grad.addColorStop(0, "rgba(255,255,255,1)");
  grad.addColorStop(0.25, "rgba(255,255,255,0.55)");
  grad.addColorStop(1, "rgba(255,255,255,0)");
  g.fillStyle = grad;
  g.fillRect(0, 0, 128, 128);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

/** Grass with slight mottling; tiles every 6 m. */
export function grassTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(256, 256);
  const rng = new Rng(15);
  g.fillStyle = "#4f6b3a";
  g.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 2500; i++) {
    g.fillStyle = `rgba(${rng.int(40, 110)},${rng.int(80, 140)},${rng.int(30, 70)},0.35)`;
    g.fillRect(rng.range(0, 256), rng.range(0, 256), 2, 2);
  }
  return toTexture(c, true);
}
