/**
 * 2D math in the simulation frame. x points east, y points south
 * (it maps to three.js +z), so the right-hand side of heading (dx, dy) is (-dy, dx).
 */
export interface Vec2 {
  x: number;
  y: number;
}

export const vec = (x: number, y: number): Vec2 => ({ x, y });
export const add = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y });
export const sub = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y });
export const scale = (a: Vec2, k: number): Vec2 => ({ x: a.x * k, y: a.y * k });
export const dot = (a: Vec2, b: Vec2): number => a.x * b.x + a.y * b.y;
export const cross = (a: Vec2, b: Vec2): number => a.x * b.y - a.y * b.x;
export const len = (a: Vec2): number => Math.hypot(a.x, a.y);
export const dist = (a: Vec2, b: Vec2): number => Math.hypot(a.x - b.x, a.y - b.y);
export const dist2 = (a: Vec2, b: Vec2): number => (a.x - b.x) ** 2 + (a.y - b.y) ** 2;
export const lerp = (a: Vec2, b: Vec2, t: number): Vec2 => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
});
export const norm = (a: Vec2): Vec2 => {
  const l = len(a);
  return l > 1e-9 ? { x: a.x / l, y: a.y / l } : { x: 1, y: 0 };
};
export const right = (d: Vec2): Vec2 => ({ x: -d.y, y: d.x });
export const fromAngle = (h: number): Vec2 => ({ x: Math.cos(h), y: Math.sin(h) });
export const angleOf = (d: Vec2): number => Math.atan2(d.y, d.x);
export const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v));
export const wrapAngle = (a: number): number => {
  let r = a;
  while (r > Math.PI) r -= Math.PI * 2;
  while (r < -Math.PI) r += Math.PI * 2;
  return r;
};
export const smoothstep = (t: number): number => {
  const c = clamp(t, 0, 1);
  return c * c * (3 - 2 * c);
};

export function cubicBezier(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: number): Vec2 {
  const u = 1 - t;
  const a = u * u * u;
  const b = 3 * u * u * t;
  const c = 3 * u * t * t;
  const d = t * t * t;
  return {
    x: a * p0.x + b * p1.x + c * p2.x + d * p3.x,
    y: a * p0.y + b * p1.y + c * p2.y + d * p3.y,
  };
}

export interface PathSample {
  p: Vec2;
  dir: Vec2;
  s: number;
  kappa: number;
}

export interface Projection {
  s: number;
  idx: number;
  /** Signed lateral offset, positive to the right of the path. */
  lateral: number;
  dist: number;
}

/** Arc-length parameterised polyline with per-vertex curvature. */
export class Polyline {
  readonly pts: Vec2[];
  readonly cum: number[];
  readonly kappa: number[];
  readonly length: number;

  constructor(points: Vec2[]) {
    const pts: Vec2[] = [];
    for (const p of points) {
      const last = pts[pts.length - 1];
      if (!last || dist(last, p) > 1e-4) pts.push(p);
    }
    if (pts.length === 1) {
      const only = pts[0] as Vec2;
      pts.push({ x: only.x + 0.01, y: only.y });
    }
    this.pts = pts;
    this.cum = [0];
    for (let i = 1; i < pts.length; i++) {
      this.cum.push((this.cum[i - 1] as number) + dist(pts[i - 1] as Vec2, pts[i] as Vec2));
    }
    this.length = this.cum[this.cum.length - 1] as number;
    this.kappa = computeCurvature(pts, this.cum);
  }

  /** Index of the segment containing arc length s (binary search). */
  segmentAt(s: number): number {
    const n = this.pts.length;
    if (s <= 0) return 0;
    if (s >= this.length) return n - 2;
    let lo = 0;
    let hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if ((this.cum[mid] as number) <= s) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  sampleAt(s: number): PathSample {
    const cs = clamp(s, 0, this.length);
    const i = this.segmentAt(cs);
    const a = this.pts[i] as Vec2;
    const b = this.pts[i + 1] as Vec2;
    const segLen = (this.cum[i + 1] as number) - (this.cum[i] as number);
    const t = segLen > 1e-9 ? (cs - (this.cum[i] as number)) / segLen : 0;
    const ka = this.kappa[i] as number;
    const kb = this.kappa[i + 1] as number;
    const base = lerp(a, b, t);
    // Linear extrapolation beyond the ends keeps look-ahead points meaningful.
    const dir = norm(sub(b, a));
    const extra = s - cs;
    return {
      p: extra !== 0 ? add(base, scale(dir, extra)) : base,
      dir,
      s,
      kappa: ka + (kb - ka) * t,
    };
  }

  /** Closest point search. With a hint, only a window of segments around it is scanned. */
  project(p: Vec2, hintIdx = -1, window = 40): Projection {
    const n = this.pts.length;
    let lo = 0;
    let hi = n - 2;
    if (hintIdx >= 0) {
      lo = Math.max(0, hintIdx - window);
      hi = Math.min(n - 2, hintIdx + window);
    }
    let best: Projection = { s: 0, idx: 0, lateral: 0, dist: Infinity };
    for (let i = lo; i <= hi; i++) {
      const a = this.pts[i] as Vec2;
      const b = this.pts[i + 1] as Vec2;
      const ab = sub(b, a);
      const l2 = dot(ab, ab);
      const t = l2 > 1e-12 ? clamp(dot(sub(p, a), ab) / l2, 0, 1) : 0;
      const q = add(a, scale(ab, t));
      const d = dist(p, q);
      if (d < best.dist) {
        const dir = norm(ab);
        best = {
          s: (this.cum[i] as number) + t * Math.sqrt(l2),
          idx: i,
          lateral: dot(sub(p, q), right(dir)),
          dist: d,
        };
      }
    }
    return best;
  }
}

function computeCurvature(pts: Vec2[], cum: number[]): number[] {
  const n = pts.length;
  const raw = new Array<number>(n).fill(0);
  for (let i = 1; i < n - 1; i++) {
    const d1 = norm(sub(pts[i] as Vec2, pts[i - 1] as Vec2));
    const d2 = norm(sub(pts[i + 1] as Vec2, pts[i] as Vec2));
    const dTheta = Math.abs(Math.atan2(cross(d1, d2), dot(d1, d2)));
    const ds = ((cum[i + 1] as number) - (cum[i - 1] as number)) / 2;
    raw[i] = ds > 1e-6 ? dTheta / ds : 0;
  }
  const out = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) {
    let sum = 0;
    let cnt = 0;
    for (let k = -2; k <= 2; k++) {
      const v = raw[i + k];
      if (v !== undefined) {
        sum += v;
        cnt++;
      }
    }
    out[i] = cnt > 0 ? sum / cnt : 0;
  }
  return out;
}

/** Deterministic PRNG so headless tests are reproducible. */
export class Rng {
  private state: number;
  constructor(seed: number) {
    this.state = seed >>> 0;
  }
  next(): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0;
    let t = this.state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  range(lo: number, hi: number): number {
    return lo + (hi - lo) * this.next();
  }
  int(lo: number, hiExclusive: number): number {
    return Math.floor(this.range(lo, hiExclusive));
  }
  pick<T>(arr: readonly T[]): T {
    const v = arr[this.int(0, arr.length)];
    if (v === undefined) throw new Error("Rng.pick called with an empty array");
    return v;
  }
  chance(p: number): boolean {
    return this.next() < p;
  }
}
