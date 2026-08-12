/**
 * A vertex is a pair of *cell indices*.
 *
 * The working copy and every geometry helper use this tuple form; the API's own `{x, y}`
 * objects are converted at the boundary in `useGridDraft`. Keeping the tuple internally is
 * what lets the footprint rasterizer stay a line-for-line mirror of the Rust definition, which
 * has a type `[i32; 2]`.
 */
export type Vertex = [number, number];

/** One vertex as the API spells it — `ObstaclePoly::vertices` in src/models/obstacle.rs. */
export interface WireVertex {
    x: number;
    y: number;
}

/**
 * One obstacle as the API defines it, mirroring `ObstaclePoly` in src/models/obstacle.rs.
 *
 * `id` and `dynamic` sit on the polygon rather than on each vertex: they describe the shape
 * as a whole, and repeating them per vertex would allow a shape that disagrees with itself.
 */
export interface WireObstacle {
    id: number;
    dynamic: boolean;
    vertices: WireVertex[];
}

/**
 * Enough of a grid to place cells on it.
 *
 * Split out because a grid being composed has dimensions but no id yet — nothing is
 * written to the server until the user confirms.
 */
export interface GridSize {
    width: number;
    height: number;
}

/**
 * A row of `GET /grids`. Deliberately lean: the listing carries no obstacle geometry,
 * so a page of grids stays small. Use {@link GridDetail} when the shapes are needed.
 */
export interface Grid extends GridSize {
    id: number;
    name: string;
    /**
     * Which snapshot of `name` this is. A grid row is immutable once a plan has been
     * computed against it; editing one then writes a new row at `version + 1`, so
     * `(name, version)` identifies a snapshot and is unique in the database.
     */
    version: number;
}

/** `GET /grids/{id}` — the whole snapshot, obstacles included. */
export interface GridDetail extends Grid {
    obs_polygons: WireObstacle[];
}

/** Body of `POST /grids`, `PUT /grids/{id}` and `POST /grids/{id}/versions`. */
export interface GridInput {
    name: string;
    width: number;
    height: number;
    obs_polygons: WireObstacle[];
}

/**
 * One polygon in the working copy.
 *
 * `id` keeps selection, color and drag targeting stable across edits to the list, where an
 * array index would reshuffle all three whenever a shape ahead of it is deleted. It is also
 * stored now — the API round-trips it — so a shape can be followed from one snapshot to the
 * next, which is what a moving obstacle needs.
 */
export interface Obstacle {
    id: number;
    /**
     * Whether a simulation may move or reshape this obstacle. Fixed environment is `false`.
     */
    dynamic: boolean;
    vertices: Vertex[];
}

/** Which end of a route a click is placing. `src`/`dest` match the API's field names. */
export type Endpoint = 'src' | 'dest';

/**
 * Body of `POST /grids/{id}/replan` — one tick of a moving-obstacle simulation.
 *
 * The obstacles are sent every tick because the server keeps no session: tick 5 has to perturb
 * the shapes as tick 4 left them, and the stored grid row is only the initial condition.
 */
export interface ReplanInput {
    src_vertex: Vertex;
    dest_vertex: Vertex;
    obs_polygons: WireObstacle[];
    /** Omit on the first tick; then pass back {@link ReplanResult.next_seed}. */
    seed?: number;
}

/** What one tick answers with. Nothing is stored, so this is the only record of it. */
export interface ReplanResult {
    /** The obstacles after the tick — draw these, and send them on to the next one. */
    obs_polygons: WireObstacle[];
    /** The recomputed route, empty when the goal has been walled off. */
    vertices: Vertex[];
    reachable: boolean;
    cost: number;
    /** How many obstacles actually moved; 0 means the tick changed nothing. */
    moved: number;
    /**
     * Feed back as `seed` to continue the run.
     *
     * 32 bits wide on purpose — a `u64` would exceed what a JSON number can hold exactly, and
     * the chain would break without anything looking wrong.
     */
    next_seed: number;
    planner: string;
}

/** Written by `generate_grid_plan` in src/handlers/planner_crud.rs. */
export interface PlanMeta {
    planner: string;
    src_vertex: Vertex;
    dest_vertex: Vertex;
    reachable: boolean;
    /**
     * Scaled by 10 so a diagonal stays an integer: an orthogonal step costs 10, a
     * diagonal 14. See `ORTHOGONAL_COST` in src/models/planners/movement_model.rs.
     */
    cost: number;
}

/** Mirrors `plans::Model` in src/entities/plans.rs. */
export interface Plan {
    id: number;
    grid_id: number;
    name: string;
    /**
     * The cells the route runs through, start first — empty when there is no route.
     *
     * An unreachable goal comes back as a *saved plan* with no cells rather than an
     * error, because it is a fact about the grid rather than a bad request. Only a
     * malformed endpoint (off-grid, or inside an obstacle) is a 4xx.
     */
    vertices: Vertex[];
    meta: PlanMeta;
}