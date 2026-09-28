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

const canvas = byId<HTMLCanvasElement>("view");
let sim = new Simulation(Math.floor(Math.random() * 1e9));
const view = new View(canvas, sim);
const hud = new Hud();
const minimap = new Minimap(byId<HTMLCanvasElement>("minimap"), (p) => setDestinationAt(p));

let paused = false;
let speedIdx = 0;
let accumulator = 0;
const keys = new Set<string>();

function setDestinationAt(p: Vec2): void {
  sim.setDestination(sim.destinationNear(p));
}

function manualInput(): ManualInput {
  const throttle = keys.has("KeyW") || keys.has("ArrowUp") ? 1 : 0;
  const brake = keys.has("KeyS") || keys.has("ArrowDown") || keys.has("Space") ? 1 : 0;
  const left = keys.has("KeyA") || keys.has("ArrowLeft") ? 1 : 0;
  const rightKey = keys.has("KeyD") || keys.has("ArrowRight") ? 1 : 0;
  return { throttle, brake, steer: rightKey - left };
}

function toggleFsd(): void {
  if (sim.ego.mode === "fsd") sim.takeover();
  else sim.engageFsd();
}

function cycleCamera(): void {
  const i = CAMERA_MODES.indexOf(view.cameraMode);
  view.cameraMode = CAMERA_MODES[(i + 1) % CAMERA_MODES.length] ?? "follow";
  byId("btn-cam").querySelector(".label")!.textContent = CAMERA_LABEL[view.cameraMode] ?? "";
}

function cycleSpeed(): void {
  speedIdx = (speedIdx + 1) % SPEEDS.length;
  byId("btn-speed").querySelector(".label")!.textContent = `${SPEEDS[speedIdx]}× 速度`;
}

function togglePause(): void {
  paused = !paused;
  byId("btn-pause").classList.toggle("on", paused);
  byId("btn-pause").querySelector(".label")!.textContent = paused ? "继续" : "暂停";
}

function randomDestination(): void {
  const d = sim.randomDestination();
  if (sim.ego.mode === "fsd") sim.engageFsd(d);
  else sim.setDestination(d);
}

function applySettingsFromUi(): void {
  sim.settings.autoplay = byId<HTMLInputElement>("opt-autoplay").checked;
  sim.settings.randomEvents = byId<HTMLInputElement>("opt-events").checked;
  sim.settings.trafficCount = Number(byId<HTMLInputElement>("opt-traffic").value);
  sim.settings.pedCount = Number(byId<HTMLInputElement>("opt-peds").value);
  byId("opt-traffic-val").textContent = String(sim.settings.trafficCount);
  byId("opt-peds-val").textContent = String(sim.settings.pedCount);
}

function reset(): void {
  sim = new Simulation(Math.floor(Math.random() * 1e9));
  applySettingsFromUi();
  sim.engageFsd();
}

const actions: Record<string, () => void> = {
  KeyF: toggleFsd,
  KeyC: cycleCamera,
  KeyR: randomDestination,
  KeyJ: () => {
    if (!sim.spawnJaywalkerAhead()) hud.toast("前方空间不足，无法生成横穿行人", "info");
  },
  KeyB: () => {
    if (!sim.spawnStalledAhead()) hud.toast("前方空间不足，无法生成故障车辆", "info");
  },
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
  "btn-jay": actions.KeyJ as () => void,
  "btn-stall": actions.KeyB as () => void,
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
for (const id of ["opt-autoplay", "opt-events", "opt-traffic", "opt-peds"]) {
  byId(id).addEventListener("input", applySettingsFromUi);
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

byId("btn-start").addEventListener("click", () => {
  byId("intro").classList.add("hidden");
});
if (location.hash === "#play") byId("intro").classList.add("hidden");

function resize(): void {
  view.resize();
  minimap.resize();
}
window.addEventListener("resize", resize);
resize();

applySettingsFromUi();
sim.engageFsd();

let last = performance.now();
function frame(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  const wall = now / 1000;
  if (!paused) {
    accumulator += dt * (SPEEDS[speedIdx] ?? 1);
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
