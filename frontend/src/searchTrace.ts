/**
 * Replaying an SST search trace.
 *
 * The server hands back every event in the order it happened — see `search_trace.rs`. This
 * module turns a prefix of that list into the tree as it stood at that moment, which is the
 * one thing a scrubber needs and the one thing worth testing on its own.
 *
 * Kept free of React and of SVG for that reason: the reduction is where a replay goes wrong
 * (an edge drawn after its parent was culled, a prune applied to nothing), and none of that is
 * easier to see through a component.
 */

import type { Pose, SearchEvent } from "./types";

/** One edge of the tree, as drawn: the poses from a node's parent to the node itself. */
export interface TraceEdge {
  id: number;
  parent: number;
  path: Pose[];
  /** Cost-to-come in seconds, for shading by depth if a view wants it. */
  cost: number;
}

/** The tree as it stood after some number of events. */
export interface TraceFrame {
  /** Edges still in the tree. */
  live: TraceEdge[];
  /**
   * Edges sparsification has culled, in the order they went.
   *
   * Kept rather than dropped because they are the point: the difference between SST and a
   * plain kinodynamic RRT *is* what gets thrown away, and a view that only ever shows the
   * surviving tree cannot show it happening.
   */
  pruned: TraceEdge[];
  /** Witness centres opened so far, in metres. */
  witnesses: [number, number][];
  /** Cost of the best solution found by this point, or null before the first one. */
  bestCost: number | null;
  /** How many events have been applied. */
  applied: number;
}

const EMPTY: TraceFrame = {
  live: [],
  pruned: [],
  witnesses: [],
  bestCost: null,
  applied: 0,
};

/**
 * The tree after the first `frame` events.
 *
 * Recomputed from the start each time rather than stepped incrementally. That is O(n) per
 * redraw against a few thousand events, which is nothing next to laying out the SVG — and it
 * makes scrubbing *backwards* exactly as correct as scrubbing forwards, which an incremental
 * version only manages by keeping an undo log.
 *
 * A prune names a node that must already be live; the server guarantees that ordering and the
 * Rust side has a test pinning it. Here a prune that finds nothing is simply ignored rather
 * than throwing, because a half-drawn tree is a better failure than a blank screen.
 */
export function treeAt(events: SearchEvent[], frame: number): TraceFrame {
  if (events.length === 0) return EMPTY;

  const limit = Math.max(0, Math.min(frame, events.length));
  const live = new Map<number, TraceEdge>();
  const pruned: TraceEdge[] = [];
  const witnesses: [number, number][] = [];
  let bestCost: number | null = null;

  for (let i = 0; i < limit; i += 1) {
    const event = events[i];
    switch (event.type) {
      case "node_added":
        live.set(event.id, {
          id: event.id,
          parent: event.parent,
          path: event.path,
          cost: event.cost,
        });
        break;
      case "node_pruned": {
        const edge = live.get(event.id);
        if (edge) {
          live.delete(event.id);
          pruned.push(edge);
        }
        break;
      }
      case "witness_added":
        witnesses.push([event.x, event.y]);
        break;
      case "solution_improved":
        bestCost = event.cost;
        break;
    }
  }

  return { live: [...live.values()], pruned, witnesses, bestCost, applied: limit };
}

/**
 * The iteration an event belongs to, for showing progress in the search's own units rather
 * than in event counts — an iteration may emit several events or none.
 */
export function iterationAt(events: SearchEvent[], frame: number): number {
  if (events.length === 0) return 0;
  const index = Math.max(0, Math.min(frame, events.length) - 1);
  return events[index]?.iteration ?? 0;
}
