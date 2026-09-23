import { describe, expect, it } from "vitest";
import { iterationAt, treeAt } from "./searchTrace";
import type { Pose, SearchEvent } from "./types";

/**
 * The replay reduction, tested without React or SVG.
 *
 * This is where a scrubber goes wrong — an edge still drawn after its parent was culled, a
 * prune applied to nothing, a frame that differs depending on which direction you arrived at
 * it — and none of those are visible through a component.
 */

const path = (n: number): Pose[] => [
  [n, n, 0],
  [n + 1, n + 1, 0],
];

const added = (iteration: number, id: number, parent: number): SearchEvent => ({
  type: "node_added",
  iteration,
  id,
  parent,
  path: path(id),
  cost: id * 0.1,
});

const pruned = (iteration: number, id: number): SearchEvent => ({
  type: "node_pruned",
  iteration,
  id,
});

/** A small search: three nodes, one culled, a witness, and a solution. */
const TRACE: SearchEvent[] = [
  { type: "witness_added", iteration: 0, x: 1, y: 1 },
  added(1, 1, 0),
  added(2, 2, 1),
  { type: "witness_added", iteration: 2, x: 3, y: 3 },
  added(3, 3, 1),
  pruned(4, 2),
  { type: "solution_improved", iteration: 5, cost: 4.25 },
];

describe("treeAt", () => {
  it("builds the tree up as events are applied", () => {
    expect(treeAt(TRACE, 0).live).toHaveLength(0);
    expect(treeAt(TRACE, 2).live.map((e) => e.id)).toEqual([1]);
    expect(treeAt(TRACE, 3).live.map((e) => e.id)).toEqual([1, 2]);
    expect(treeAt(TRACE, 5).live.map((e) => e.id)).toEqual([1, 2, 3]);
  });

  it("moves a culled edge out of the tree rather than dropping it", () => {
    // Pruned branches are the point: what SST throws away is the whole difference between it
    // and a plain kinodynamic RRT, so a view that forgot them could not show it happening.
    const frame = treeAt(TRACE, 6);
    expect(frame.live.map((e) => e.id)).toEqual([1, 3]);
    expect(frame.pruned.map((e) => e.id)).toEqual([2]);
    // And it keeps the geometry, or there would be nothing to grey out.
    expect(frame.pruned[0].path).toEqual(path(2));
  });

  it("gives the same frame whichever direction it was reached from", () => {
    // The property that lets the scrubber be dragged backwards. An incremental replay only
    // manages this by keeping an undo log; recomputing from the start gets it for free, and
    // this is what pins that it stays that way.
    const forwards = treeAt(TRACE, 4);
    const backwards = treeAt(TRACE, 4);
    expect(backwards).toEqual(forwards);

    const ids = (frame: number) => treeAt(TRACE, frame).live.map((e) => e.id);
    expect(ids(7)).toEqual([1, 3]);
    expect(ids(3)).toEqual([1, 2]);
    expect(ids(7)).toEqual([1, 3]);
  });

  it("reports the best cost only once a solution has been reached", () => {
    expect(treeAt(TRACE, 6).bestCost).toBeNull();
    expect(treeAt(TRACE, 7).bestCost).toBe(4.25);
  });

  it("collects witnesses as they are opened", () => {
    expect(treeAt(TRACE, 1).witnesses).toEqual([[1, 1]]);
    expect(treeAt(TRACE, 7).witnesses).toEqual([
      [1, 1],
      [3, 3],
    ]);
  });

  it("clamps a frame outside the trace instead of throwing", () => {
    // The scrubber's range and the event list can disagree for a render or two while a new
    // trace loads, and a blank canvas is a worse answer than the nearest valid frame.
    expect(treeAt(TRACE, -5).applied).toBe(0);
    expect(treeAt(TRACE, 999).applied).toBe(TRACE.length);
    expect(treeAt(TRACE, 999).live.map((e) => e.id)).toEqual([1, 3]);
    expect(treeAt([], 3)).toEqual({
      live: [],
      pruned: [],
      witnesses: [],
      bestCost: null,
      applied: 0,
    });
  });

  it("ignores a prune for a node that is not in the tree", () => {
    // The server guarantees a prune always names a live node and has a test pinning it. If
    // that ever broke, a half-drawn tree is a better failure than a blank screen.
    expect(() => treeAt([pruned(0, 99)], 1)).not.toThrow();
    expect(treeAt([pruned(0, 99)], 1).pruned).toHaveLength(0);
  });
});

describe("iterationAt", () => {
  it("reports the iteration of the last applied event", () => {
    // Events are not evenly spread over iterations — one iteration can emit several or none —
    // so the playhead counts events while the readout has to speak in the search's own units.
    expect(iterationAt(TRACE, 0)).toBe(0);
    expect(iterationAt(TRACE, 3)).toBe(2);
    expect(iterationAt(TRACE, TRACE.length)).toBe(5);
  });

  it("is zero for an empty trace", () => {
    expect(iterationAt([], 5)).toBe(0);
  });
});
