import { type Edge, World, turnOf } from "./world";

const TURN_COST = { straight: 0, right: 8, left: 20, uturn: Infinity } as const;
const FROM_START = -1;

/**
 * Shortest route over directed edges. The route always begins with `start`.
 * When the goal lies behind the vehicle on the same edge, the route loops around a block.
 */
export function planRoute(world: World, start: Edge, startU: number, goal: Edge, goalU: number): Edge[] | null {
  if (start.id === goal.id && goalU > startU + 6) return [start];

  const n = world.edges.length;
  const distArr = new Array<number>(n).fill(Infinity);
  const prev = new Array<number>(n).fill(-2);
  const done = new Array<boolean>(n).fill(false);
  const baseCost = start.length - startU;

  for (const s of world.successors(start)) {
    const c = baseCost + TURN_COST[turnOf(start.dir, s.dir)] + (s.id === goal.id ? goalU : s.length);
    if (c < (distArr[s.id] as number)) {
      distArr[s.id] = c;
      prev[s.id] = FROM_START;
    }
  }

  for (;;) {
    let best = -1;
    let bestD = Infinity;
    for (let i = 0; i < n; i++) {
      if (!done[i] && (distArr[i] as number) < bestD) {
        bestD = distArr[i] as number;
        best = i;
      }
    }
    if (best < 0) return null;
    if (best === goal.id) break;
    done[best] = true;
    const e = world.edge(best);
    for (const s of world.successors(e)) {
      if (done[s.id]) continue;
      const c = bestD + TURN_COST[turnOf(e.dir, s.dir)] + (s.id === goal.id ? goalU : s.length);
      if (c < (distArr[s.id] as number)) {
        distArr[s.id] = c;
        prev[s.id] = best;
      }
    }
  }

  const chain: Edge[] = [];
  let cur = goal.id;
  for (let guard = 0; guard <= n; guard++) {
    chain.push(world.edge(cur));
    const p = prev[cur] as number;
    if (p === FROM_START) break;
    if (p < 0) return null;
    cur = p;
  }
  chain.reverse();
  return [start, ...chain];
}
