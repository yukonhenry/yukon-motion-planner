import type {GridSize, Vertex} from './types';

/**
 * Pure coordinate math — no React, no SVG, no DOM.
 *
 * Everything the renderer needs to place shapes lives here, so swapping SVG for a
 * canvas library later means rewriting only the component that draws, not the rules
 * about where things go.
 */

/** Pixels per grid cell. */
export const CELL_SIZE = 28;

/**
 * The API rejects polygons with fewer corners — see `validate_polygons` in
 * src/handlers/grid_crud.rs.
 */
export const MIN_VERTICES = 3;

export const gridPixelWidth = (grid: GridSize) => grid.width * CELL_SIZE;
export const gridPixelHeight = (grid: GridSize) => grid.height * CELL_SIZE;

/**
 * Cell indices address cells, not corners, so a vertex is drawn at the cell's center.
 * That also makes cell `width - 1` visibly inside the grid, matching the backend's
 * exclusive upper bound.
 */
export const cellToPixel = (cell: number) => cell * CELL_SIZE + CELL_SIZE / 2;

/** Inverse of {@link cellToPixel}: which cell does this pixel offset fall in? */
export const pixelToCell = (pixel: number) => Math.floor(pixel / CELL_SIZE);

export const verticesToPixels = (vertices: Vertex[]): [number, number][] =>
    vertices.map(([x, y]) => [cellToPixel(x), cellToPixel(y)]);

const clamp = (value: number, min: number, max: number) =>
    Math.min(Math.max(value, min), max);

/** Pins a cell inside the grid, so a drag that leaves the canvas still yields a legal cell. */
export const clampCell = (cell: Vertex, grid: GridSize): Vertex => [
    clamp(cell[0], 0, grid.width - 1),
    clamp(cell[1], 0, grid.height - 1),
];

/**
 * Mirrors `out_of_bounds` in src/handlers/grid_crud.rs — vertices are cell indices, so
 * the far edge is out of range: a 10-wide grid addresses columns 0..=9.
 */
export function outOfBounds(vertices: Vertex[], grid: GridSize): Vertex | null {
    return (
        vertices.find(
            ([x, y]) => x < 0 || x >= grid.width || y < 0 || y >= grid.height,
        ) ?? null
    );
}

/**
 * Mirrors `validate_polygons` in src/handlers/grid_crud.rs, so the UI can disable an
 * impossible action instead of round-tripping to be told no. The server stays
 * authoritative — its rejection is still surfaced if the two ever disagree.
 *
 * @returns an error message, or `null` when the polygon is acceptable.
 */
export function validateVertices(vertices: Vertex[], grid: GridSize): string | null {
    if (vertices.length < MIN_VERTICES) {
        return `an obstacle needs at least ${MIN_VERTICES} vertices, got ${vertices.length}`;
    }

    const stray = outOfBounds(vertices, grid);
    if (stray) {
        return `vertex [${stray[0]}, ${stray[1]}] is outside the ${grid.width}x${grid.height} grid`;
    }

    return null;
}

/**
 * The first problem across a whole obstacle set, or `null` when all of them are fine.
 *
 * Shrinking a grid is what makes this more than a per-shape check: obstacles that were
 * legal at 20x15 can be stranded out of bounds at 10x10, and the server refuses the
 * whole save rather than clipping them.
 */
export function validateObstacles(
    polygons: Vertex[][],
    grid: GridSize,
): string | null {
    for (const [index, vertices] of polygons.entries()) {
        const problem = validateVertices(vertices, grid);
        if (problem) return `obstacle ${index}: ${problem}`;
    }
    return null;
}

/** The smallest cell-space box containing the polygon. */
export function polygonBounds(vertices: Vertex[]) {
    const xs = vertices.map(([x]) => x);
    const ys = vertices.map(([, y]) => y);
    return {
        minX: Math.min(...xs),
        maxX: Math.max(...xs),
        minY: Math.min(...ys),
        maxY: Math.max(...ys),
    };
}

/**
 * Shifts a whole polygon by a cell delta, trimming the delta first so every vertex
 * lands in bounds. Clamping the *translation* rather than each vertex keeps the shape
 * rigid — clamping per-vertex would deform it against the wall.
 */
export function translatePolygon(
    vertices: Vertex[],
    dx: number,
    dy: number,
    grid: GridSize,
): Vertex[] {
    const {minX, maxX, minY, maxY} = polygonBounds(vertices);
    const limitedDx = clamp(dx, -minX, grid.width - 1 - maxX);
    const limitedDy = clamp(dy, -minY, grid.height - 1 - maxY);

    return vertices.map(([x, y]) => [x + limitedDx, y + limitedDy]);
}

/** Two cells are equal — used to avoid re-committing a drag that never moved. */
export const sameCell = (a: Vertex, b: Vertex) => a[0] === b[0] && a[1] === b[1];

/**
 * Whether a polygon repeats its first corner as its last entry.
 *
 * Obstacles are stored closed, so `vertices.length` is one more than the number of
 * corners. The backend closes implicitly instead — `(i + 1) % len` in
 * src/models/grid_world_manager.rs — so it accepts either form, which is why grids
 * saved before this convention can still come back open.
 */
export const isClosedRing = (vertices: Vertex[]) =>
    vertices.length > 1 && sameCell(vertices[0], vertices[vertices.length - 1]);

/** Repeats the first corner at the end, unless the polygon already ends there. */
export const closeRing = (vertices: Vertex[]): Vertex[] =>
    vertices.length === 0 || isClosedRing(vertices) ? vertices : [...vertices, vertices[0]];

/**
 * Replaces one vertex, keeping the result inside the grid.
 *
 * The first and last entries of a closed ring are one corner held twice, so dragging
 * either handle has to move both — moving one alone tears the outline open.
 */
export function replaceVertex(
    vertices: Vertex[],
    index: number,
    cell: Vertex,
    grid: GridSize,
): Vertex[] {
    const next = clampCell(cell, grid);
    const last = vertices.length - 1;
    const paired = isClosedRing(vertices) && (index === 0 || index === last);

    return vertices.map((v, i) =>
        i === index || (paired && (i === 0 || i === last)) ? next : v,
    );
}

/**
 * Which cells the planner treats as blocked by one polygon.
 *
 * A deliberate mirror of `rasterize_polygon` in src/models/grid_world_manager.rs, kept
 * split into the same two passes so the two can be diffed by eye. It exists because the
 * drawn outline understates the obstacle: vertices sit at cell centers and the stroke is
 * infinitely thin, but rasterization marks whole cells, so a route that looks like it
 * should hug an edge is really routing around a fatter shape. Drawing this is what makes
 * a "suboptimal" path legible as the optimal path it actually is.
 *
 * Kept in step with the Rust by tests in geometry.raster.test.ts, which pin the cases
 * where the two could plausibly diverge.
 */
export function rasterizePolygon(vertices: Vertex[], grid: GridSize): Vertex[] {
    // Deduped by cell index: the interior and edge passes overlap, and an obstacle drawn
    // as a closed ring walks its final degenerate edge twice.
    const blocked = new Set<number>();
    const mark = (x: number, y: number) => {
        // Mirrors `try_id` — negatives included, which is why this tests x >= 0 rather
        // than relying on the array bounds.
        if (x >= 0 && y >= 0 && x < grid.width && y < grid.height) {
            blocked.add(y * grid.width + x);
        }
    };

    fillPolygonInterior(vertices, grid, mark);
    for (let i = 0; i < vertices.length; i++) {
        drawBresenhamEdge(vertices[i], vertices[(i + 1) % vertices.length], mark);
    }

    return [...blocked].map((id) => [id % grid.width, Math.floor(id / grid.width)]);
}

/** Scanline fill of the polygon interior — boundary cells are the edge pass's job. */
function fillPolygonInterior(
    vertices: Vertex[],
    grid: GridSize,
    mark: (x: number, y: number) => void,
) {
    let minY = Infinity;
    let maxY = -Infinity;
    for (const [, y] of vertices) {
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
    }
    minY = Math.max(minY, 0);
    maxY = Math.min(maxY, grid.height - 1);

    for (let y = minY; y <= maxY; y++) {
        const intersections: number[] = [];
        for (let i = 0; i < vertices.length; i++) {
            const [x1, y1] = vertices[i];
            const [x2, y2] = vertices[(i + 1) % vertices.length];

            // Half-open in y: a vertex counts for the span below it and not the one
            // above, which is what stops a shared corner being counted twice.
            if ((y1 <= y && y < y2) || (y2 <= y && y < y1)) {
                // Rust's integer division truncates toward zero, so this is Math.trunc
                // and not Math.floor — they differ on left-leaning edges, where the span
                // starts a cell late and the edge pass covers the gap.
                intersections.push(x1 + Math.trunc(((y - y1) * (x2 - x1)) / (y2 - y1)));
            }
        }

        // Numeric, not JS's lexicographic default: [2, 10] must not sort as [10, 2].
        intersections.sort((a, b) => a - b);

        // An odd count leaves a lone trailing value that Rust's `chunks(2)` slice pattern
        // silently drops, so this stops one short rather than filling to the grid edge.
        for (let i = 0; i + 1 < intersections.length; i += 2) {
            for (let x = intersections[i]; x <= intersections[i + 1]; x++) {
                mark(x, y);
            }
        }
    }
}

function drawBresenhamEdge(
    [x0, y0]: Vertex,
    [x1, y1]: Vertex,
    mark: (x: number, y: number) => void,
) {
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;
    let x = x0;
    let y = y0;

    for (;;) {
        mark(x, y);
        if (x === x1 && y === y1) break;
        const e2 = 2 * err;
        // Both branches can fire on the same step — that diagonal move is what keeps the
        // stair-stepping to one cell per axis instead of cutting a corner.
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

/** Deterministic color per obstacle so shapes stay visually distinct between renders. */
export function obstacleHue(id: number) {
    // Golden-angle stepping spreads consecutive ids far apart on the color wheel.
    return (id * 137.508) % 360;
}
