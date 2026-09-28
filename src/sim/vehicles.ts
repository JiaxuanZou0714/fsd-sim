import type { Rng } from "./geometry";

export type VehicleKind = "car" | "suv" | "taxi" | "van" | "bus" | "truck" | "moto" | "bike";

export interface VehicleSpec {
  kind: VehicleKind;
  label: string;
  length: number;
  width: number;
  height: number;
  /** Desired speed as a fraction of the posted limit. */
  speedFactor: number;
  /** Absolute speed cap in m/s. */
  vMax: number;
  aMax: number;
  bComf: number;
  /** Desired time headway in seconds. */
  headway: number;
  /** Standstill gap in metres. */
  s0: number;
  colors: number[];
  /** Relative spawn frequency. */
  weight: number;
}

export const VEHICLES: Record<VehicleKind, VehicleSpec> = {
  car: {
    kind: "car",
    label: "轿车",
    length: 4.6,
    width: 1.85,
    height: 1.45,
    speedFactor: 0.95,
    vMax: 20,
    aMax: 2.2,
    bComf: 3.0,
    headway: 1.4,
    s0: 2.4,
    colors: [0x9aa0a8, 0xa7adb5, 0x8c9299, 0xb3b8bf, 0x6e747c, 0x2f3238, 0xd9dadc],
    weight: 46,
  },
  suv: {
    kind: "suv",
    label: "SUV",
    length: 4.9,
    width: 1.98,
    height: 1.75,
    speedFactor: 0.95,
    vMax: 20,
    aMax: 2.0,
    bComf: 3.0,
    headway: 1.5,
    s0: 2.5,
    colors: [0x7c828a, 0x3a3e45, 0xa9aeb6, 0x5b6a7a],
    weight: 16,
  },
  taxi: {
    kind: "taxi",
    label: "出租车",
    length: 4.7,
    width: 1.86,
    height: 1.5,
    speedFactor: 1.02,
    vMax: 20,
    aMax: 2.5,
    bComf: 3.2,
    headway: 1.1,
    s0: 2.0,
    colors: [0x25282d, 0x30343a],
    weight: 9,
  },
  van: {
    kind: "van",
    label: "厢式货车",
    length: 5.6,
    width: 2.05,
    height: 2.4,
    speedFactor: 0.88,
    vMax: 17,
    aMax: 1.6,
    bComf: 2.6,
    headway: 1.6,
    s0: 2.6,
    colors: [0xe6e6e3, 0xd0d3d6, 0xc9cfd6],
    weight: 9,
  },
  bus: {
    kind: "bus",
    label: "公交车",
    length: 12,
    width: 2.55,
    height: 3.1,
    speedFactor: 0.85,
    vMax: 14,
    aMax: 1.1,
    bComf: 2.2,
    headway: 1.8,
    s0: 3.0,
    colors: [0x4f7f8f, 0x5c8c7a],
    weight: 4,
  },
  truck: {
    kind: "truck",
    label: "卡车",
    length: 9,
    width: 2.5,
    height: 3.4,
    speedFactor: 0.82,
    vMax: 13,
    aMax: 1.0,
    bComf: 2.2,
    headway: 2.0,
    s0: 3.0,
    colors: [0x9b5c2e, 0x40566e, 0xb9b9b3],
    weight: 3,
  },
  moto: {
    kind: "moto",
    label: "摩托车",
    length: 2.1,
    width: 0.8,
    height: 1.35,
    speedFactor: 1.1,
    vMax: 22,
    aMax: 3.5,
    bComf: 4.0,
    headway: 0.9,
    s0: 1.5,
    colors: [0x1f2226, 0x8c1e22, 0x2d4f8a],
    weight: 7,
  },
  bike: {
    kind: "bike",
    label: "自行车",
    length: 1.8,
    width: 0.65,
    height: 1.7,
    speedFactor: 0.4,
    vMax: 5.5,
    aMax: 1.0,
    bComf: 2.0,
    headway: 1.2,
    s0: 1.5,
    colors: [0x2f7a4a, 0x3a3f46, 0x9a3b3b],
    weight: 6,
  },
};

export type DriverStyle = "normal" | "aggressive" | "cautious";

export interface DriverProfile {
  style: DriverStyle;
  speedMul: number;
  headwayMul: number;
  /** Multiplier on required lane-change gaps (smaller = accepts tighter gaps). */
  gapMul: number;
  /** Overtakes vehicles that are slower than desired. */
  overtakes: boolean;
}

export function randomProfile(rng: Rng, kind: VehicleKind): DriverProfile {
  if (kind === "bus" || kind === "truck" || kind === "bike") {
    return { style: "normal", speedMul: 1, headwayMul: 1, gapMul: 1, overtakes: false };
  }
  const r = rng.next();
  if (r < 0.2 || kind === "moto") {
    return { style: "aggressive", speedMul: rng.range(1.08, 1.2), headwayMul: 0.65, gapMul: 0.6, overtakes: true };
  }
  if (r < 0.35) return { style: "cautious", speedMul: rng.range(0.78, 0.88), headwayMul: 1.35, gapMul: 1.3, overtakes: false };
  return { style: "normal", speedMul: rng.range(0.93, 1.03), headwayMul: 1, gapMul: 1, overtakes: rng.chance(0.3) };
}

export function pickKind(rng: Rng): VehicleKind {
  const kinds = Object.values(VEHICLES);
  const total = kinds.reduce((a, k) => a + k.weight, 0);
  let r = rng.next() * total;
  for (const k of kinds) {
    r -= k.weight;
    if (r <= 0) return k.kind;
  }
  return "car";
}
