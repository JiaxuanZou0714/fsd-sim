import "./style.css";
import { CAMERA_MODES, View } from "./render/view";
import type { ManualInput } from "./sim/ego";
import type { Vec2 } from "./sim/geometry";
import { Simulation } from "./sim/simulation";
import { Hud } from "./ui/hud";
import { Minimap } from "./ui/minimap";

const STEP = 1 / 60;
const SPEEDS = [1, 2, 4];
const CAMERA_LABEL: Record<string, string> = { follow: "跟随视角", chase: "近距视角", top: "俯视视角" };
const DRIVE_KEYS = new Set(["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space"]);

function byId<T extends HTMLElement>(id: string): T {
  const e = document.getElementById(id);
  if (!e) throw new Error(`Missing element #${id}`);
  return e as T;
}

function label(id: string, text: string): void {
  const l = byId(id).querySelector(".label");
  if (l) l.textContent = text;
}

const canvas = byId<HTMLCanvasElement>("view");
let sim = new Simulation(Math.floor(Math.random() * 1e9));
const view = new View(canvas, sim);
const hud = new Hud();
const minimap = new Minimap(byId<HTMLCanvasElement>("minimap"), sim.net, (p) => setDestinationAt(p));

let paused = false;
let speedIdx = 0;
let accumulator = 0;
const keys = new Set<string>();

function setDestinationAt(p: Vec2): void {
  const d = sim.destinationNear(p);
  if (d) sim.setDestination(d);
  else hud.toast("附近没有可到达的车道", "info");
}

function manualInput(): ManualInput {
  const has = (...k: string[]): boolean => k.some((x) => keys.has(x));
  return {
    throttle: has("KeyW", "ArrowUp") ? 1 : 0,
    brake: has("KeyS", "ArrowDown", "Space") ? 1 : 0,
    steer: (has("KeyD", "ArrowRight") ? 1 : 0) - (has("KeyA", "ArrowLeft") ? 1 : 0),
  };
}

function toggleFsd(): void {
  if (sim.ego.mode === "fsd") sim.takeover();
  else sim.engageFsd();
}

function cycleCamera(): void {
  const i = CAMERA_MODES.indexOf(view.cameraMode);
  view.cameraMode = CAMERA_MODES[(i + 1) % CAMERA_MODES.length];
  label("btn-cam", CAMERA_LABEL[view.cameraMode] ?? "");
}

function cycleSpeed(): void {
  speedIdx = (speedIdx + 1) % SPEEDS.length;
  label("btn-speed", `${SPEEDS[speedIdx]}× 速度`);
}

function togglePause(): void {
  paused = !paused;
  byId("btn-pause").classList.toggle("on", paused);
  label("btn-pause", paused ? "继续" : "暂停");
}

function randomDestination(): void {
  const d = sim.randomDestination();
  if (sim.ego.mode === "fsd") sim.engageFsd(d);
  else sim.setDestination(d);
}

function applySettings(): void {
  sim.settings.autoplay = byId<HTMLInputElement>("opt-autoplay").checked;
  sim.settings.randomEvents = byId<HTMLInputElement>("opt-events").checked;
  sim.settings.trafficCount = Number(byId<HTMLInputElement>("opt-traffic").value);
  sim.settings.pedCount = Number(byId<HTMLInputElement>("opt-peds").value);
  view.showCandidates = byId<HTMLInputElement>("opt-cands").checked;
  view.showPredictions = byId<HTMLInputElement>("opt-preds").checked;
  byId("opt-traffic-val").textContent = String(sim.settings.trafficCount);
  byId("opt-peds-val").textContent = String(sim.settings.pedCount);
}

function reset(): void {
  sim = new Simulation(Math.floor(Math.random() * 1e9));
  applySettings();
  sim.engageFsd();
}

const attempt = (fn: () => boolean, fail: string) => (): void => {
  if (!fn()) hud.toast(fail, "info");
};

const actions: Record<string, () => void> = {
  KeyF: toggleFsd,
  KeyC: cycleCamera,
  KeyR: randomDestination,
  KeyJ: attempt(() => sim.spawnJaywalkerAhead(), "前方空间不足，无法生成横穿行人"),
  KeyB: attempt(() => sim.spawnStalledAhead(), "前方空间不足，无法生成违停车辆"),
  KeyK: attempt(() => sim.spawnCyclistAhead(), "前方空间不足，无法生成自行车"),
  KeyO: attempt(() => sim.placeOffRoad(), "未找到合适的非道路位置"),
  KeyV: attempt(() => sim.placeWrongWay(), "未找到合适的单行道位置"),
  KeyT: cycleSpeed,
  KeyP: togglePause,
};

window.addEventListener("keydown", (ev) => {
  if (ev.target instanceof HTMLInputElement) return;
  if (DRIVE_KEYS.has(ev.code)) {
    ev.preventDefault();
    if (!keys.has(ev.code) && sim.ego.mode === "fsd") sim.takeover();
    keys.add(ev.code);
    return;
  }
  const action = actions[ev.code];
  if (action && !ev.repeat) {
    ev.preventDefault();
    action();
  }
});
window.addEventListener("keyup", (ev) => keys.delete(ev.code));
window.addEventListener("blur", () => keys.clear());

const buttons: Record<string, () => void> = {
  "btn-fsd": toggleFsd,
  "btn-cam": cycleCamera,
  "btn-dest": randomDestination,
  "btn-jay": actions.KeyJ,
  "btn-stall": actions.KeyB,
  "btn-bike": actions.KeyK,
  "btn-offroad": actions.KeyO,
  "btn-wrongway": actions.KeyV,
  "btn-speed": cycleSpeed,
  "btn-pause": togglePause,
  "btn-reset": reset,
};
for (const [id, fn] of Object.entries(buttons)) {
  byId(id).addEventListener("click", (ev) => {
    fn();
    (ev.currentTarget as HTMLElement).blur();
  });
}
for (const id of ["opt-autoplay", "opt-events", "opt-traffic", "opt-peds", "opt-cands", "opt-preds"]) {
  byId(id).addEventListener("input", applySettings);
}

let downAt: { x: number; y: number } | null = null;
canvas.addEventListener("pointerdown", (ev) => {
  downAt = { x: ev.clientX, y: ev.clientY };
});
canvas.addEventListener("pointerup", (ev) => {
  if (!downAt) return;
  const moved = Math.hypot(ev.clientX - downAt.x, ev.clientY - downAt.y);
  downAt = null;
  if (moved > 6) return;
  const p = view.pick(ev.clientX, ev.clientY);
  if (p) setDestinationAt(p);
});

byId("btn-start").addEventListener("click", () => byId("intro").classList.add("hidden"));
if (location.hash === "#play") byId("intro").classList.add("hidden");

function resize(): void {
  view.resize();
  minimap.resize();
}
window.addEventListener("resize", resize);
resize();
applySettings();
sim.engageFsd();

let last = performance.now();
function frame(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  const wall = now / 1000;
  if (!paused) {
    accumulator += dt * SPEEDS[speedIdx];
    let steps = 0;
    const input = manualInput();
    while (accumulator >= STEP && steps < 16) {
      sim.step(STEP, input);
      accumulator -= STEP;
      steps++;
    }
    if (steps >= 16) accumulator = 0;
  }
  for (const e of sim.drainEvents()) hud.toast(e.text, e.level);
  view.render(sim, dt, wall);
  hud.update(sim, wall);
  minimap.draw(sim, wall);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

declare global {
  interface Window {
    __fsd?: { sim: () => Simulation; view: View };
  }
}
window.__fsd = { sim: () => sim, view };
