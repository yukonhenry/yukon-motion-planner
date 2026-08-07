import { describe, expect, it } from 'vitest';
import { rasterizePolygon } from './geometry';
import type { Vertex } from './types';

/**
 * Pins the TypeScript rasterizer to the Rust one.
 *
 * `rasterizePolygon` mirrors `rasterize_polygon` in src/models/grid_world_manager.rs so
 * the canvas can shade the cells a route is really avoiding. A mirror that drifts is
 * worse than no mirror — it draws a confident, wrong picture of why a route looks
 * suboptimal — so every expectation below is *output of the Rust*, produced by
 * tests/raster_fixtures.rs. Regenerate with:
 *
 *   cargo test --test raster_fixtures -- --ignored --nocapture
 *
 * If one of these fails, the question is which side changed, not which side is right.
 */

interface Fixture {
  polygon: Vertex[];
  grid: { width: number; height: number };
  blocked: Vertex[];
}

const fixtures: Record<string, Fixture> = {
  triangle: {
    polygon: [[2,1],[9,4],[3,8]],
    grid: { width: 12, height: 10 },
    blocked: [[2,1],[3,1],[2,2],[3,2],[4,2],[5,2],[2,3],[3,3],[4,3],[5,3],[6,3],[7,3],[2,4],[3,4],[4,4],[5,4],[6,4],[7,4],[8,4],[9,4],[3,5],[4,5],[5,5],[6,5],[7,5],[8,5],[3,6],[4,6],[5,6],[6,6],[3,7],[4,7],[5,7],[3,8]],
  },

  // The UI stores polygons closed, so this is the shape every drawn obstacle takes.
  triangle_closed: {
    polygon: [[2,1],[9,4],[3,8],[2,1]],
    grid: { width: 12, height: 10 },
    blocked: [[2,1],[3,1],[2,2],[3,2],[4,2],[5,2],[2,3],[3,3],[4,3],[5,3],[6,3],[7,3],[2,4],[3,4],[4,4],[5,4],[6,4],[7,4],[8,4],[9,4],[3,5],[4,5],[5,5],[6,5],[7,5],[8,5],[3,6],[4,6],[5,6],[6,6],[3,7],[4,7],[5,7],[3,8]],
  },

  square: {
    polygon: [[2,2],[6,2],[6,6],[2,6]],
    grid: { width: 10, height: 10 },
    blocked: [[2,2],[3,2],[4,2],[5,2],[6,2],[2,3],[3,3],[4,3],[5,3],[6,3],[2,4],[3,4],[4,4],[5,4],[6,4],[2,5],[3,5],[4,5],[5,5],[6,5],[2,6],[3,6],[4,6],[5,6],[6,6]],
  },

  // Where Math.trunc and Math.floor part company: spans start a cell late and the edge
  // pass is what covers the gap.
  left_leaning: {
    polygon: [[9,1],[2,5],[8,10]],
    grid: { width: 12, height: 12 },
    blocked: [[9,1],[7,2],[8,2],[9,2],[5,3],[6,3],[7,3],[8,3],[9,3],[3,4],[4,4],[5,4],[6,4],[7,4],[8,4],[9,4],[2,5],[3,5],[4,5],[5,5],[6,5],[7,5],[8,5],[9,5],[3,6],[4,6],[5,6],[6,6],[7,6],[8,6],[4,7],[5,7],[6,7],[7,7],[8,7],[5,8],[6,8],[7,8],[8,8],[6,9],[7,9],[8,9],[8,10]],
  },

  concave_v: {
    polygon: [[1,1],[6,8],[11,1],[11,10],[1,10]],
    grid: { width: 14, height: 12 },
    blocked: [[1,1],[11,1],[1,2],[2,2],[10,2],[11,2],[1,3],[2,3],[9,3],[10,3],[11,3],[1,4],[2,4],[3,4],[8,4],[9,4],[10,4],[11,4],[1,5],[2,5],[3,5],[4,5],[8,5],[9,5],[10,5],[11,5],[1,6],[2,6],[3,6],[4,6],[5,6],[7,6],[8,6],[9,6],[10,6],[11,6],[1,7],[2,7],[3,7],[4,7],[5,7],[6,7],[7,7],[8,7],[9,7],[10,7],[11,7],[1,8],[2,8],[3,8],[4,8],[5,8],[6,8],[7,8],[8,8],[9,8],[10,8],[11,8],[1,9],[2,9],[3,9],[4,9],[5,9],[6,9],[7,9],[8,9],[9,9],[10,9],[11,9],[1,10],[2,10],[3,10],[4,10],[5,10],[6,10],[7,10],[8,10],[9,10],[10,10],[11,10]],
  },

  // A horizontal edge contributes no scanline crossing at all — the fill relies on the
  // edge pass for those rows.
  horizontal_edge: {
    polygon: [[1,2],[8,2],[8,5],[1,5]],
    grid: { width: 10, height: 8 },
    blocked: [[1,2],[2,2],[3,2],[4,2],[5,2],[6,2],[7,2],[8,2],[1,3],[2,3],[3,3],[4,3],[5,3],[6,3],[7,3],[8,3],[1,4],[2,4],[3,4],[4,4],[5,4],[6,4],[7,4],[8,4],[1,5],[2,5],[3,5],[4,5],[5,5],[6,5],[7,5],[8,5]],
  },

  collinear: {
    polygon: [[1,1],[5,1],[9,1],[9,5],[1,5]],
    grid: { width: 12, height: 8 },
    blocked: [[1,1],[2,1],[3,1],[4,1],[5,1],[6,1],[7,1],[8,1],[9,1],[1,2],[2,2],[3,2],[4,2],[5,2],[6,2],[7,2],[8,2],[9,2],[1,3],[2,3],[3,3],[4,3],[5,3],[6,3],[7,3],[8,3],[9,3],[1,4],[2,4],[3,4],[4,4],[5,4],[6,4],[7,4],[8,4],[9,4],[1,5],[2,5],[3,5],[4,5],[5,5],[6,5],[7,5],[8,5],[9,5]],
  },

  // Bresenham's tie-breaking, with almost no interior to hide a disagreement.
  thin_diagonal: {
    polygon: [[0,0],[9,9],[8,9]],
    grid: { width: 10, height: 10 },
    blocked: [[0,0],[1,1],[2,2],[3,3],[4,4],[4,5],[5,5],[5,6],[6,6],[6,7],[7,7],[7,8],[8,8],[8,9],[9,9]],
  },

  // Zero area: the interior pass finds nothing and the edges are the whole shape.
  degenerate_line: {
    polygon: [[1,1],[7,4],[1,1]],
    grid: { width: 10, height: 10 },
    blocked: [[1,1],[2,1],[2,2],[3,2],[4,2],[4,3],[5,3],[6,3],[6,4],[7,4]],
  },

  // Negative coordinates must be rejected, not wrapped — `try_id` takes isize for
  // exactly this reason, and the mirror tests x >= 0 rather than trusting array bounds.
  partly_offgrid: {
    polygon: [[-3,2],[5,-1],[6,6]],
    grid: { width: 8, height: 8 },
    blocked: [[2,0],[3,0],[4,0],[5,0],[0,1],[1,1],[2,1],[3,1],[4,1],[5,1],[0,2],[1,2],[2,2],[3,2],[4,2],[5,2],[0,3],[1,3],[2,3],[3,3],[4,3],[5,3],[6,3],[1,4],[2,4],[3,4],[4,4],[5,4],[6,4],[3,5],[4,5],[5,5],[6,5],[5,6],[6,6]],
  },
};

/** Row-major, matching the order the Rust fixture dump walks the grid in. */
const sorted = (cells: Vertex[]) =>
  [...cells].sort((a, b) => a[1] - b[1] || a[0] - b[0]);

describe('rasterizePolygon matches the Rust rasterizer', () => {
  for (const [name, { polygon, grid, blocked }] of Object.entries(fixtures)) {
    it(name, () => {
      expect(sorted(rasterizePolygon(polygon, grid))).toEqual(sorted(blocked));
    });
  }
});

describe('rasterizePolygon', () => {
  it('is unchanged by closing the ring', () => {
    const grid = { width: 12, height: 10 };
    const open: Vertex[] = [[2, 1], [9, 4], [3, 8]];
    expect(sorted(rasterizePolygon([...open, open[0]], grid))).toEqual(
      sorted(rasterizePolygon(open, grid)),
    );
  });

  it('reports every blocked cell once', () => {
    const cells = rasterizePolygon([[1, 1], [8, 3], [2, 7], [1, 1]], { width: 10, height: 10 });
    expect(new Set(cells.map(String)).size).toBe(cells.length);
  });

  it('claims cells the drawn outline only clips', () => {
    // The reason this view exists: a shallow triangle covers a hair of row 2, and the
    // planner still refuses the whole row.
    const cells = rasterizePolygon([[1, 1], [9, 2], [1, 2]], { width: 12, height: 6 });
    expect(cells).toContainEqual([9, 2]);
    expect(cells).toContainEqual([5, 1]);
  });
});