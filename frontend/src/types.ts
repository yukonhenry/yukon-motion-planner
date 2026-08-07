/** A vertex is a pair of *cell indices*, matching the `Vec<[i32; 2]>` the API takes. */
export type Vertex = [number, number];

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
    obs_polygons: Vertex[][];
}

/** Body of `POST /grids`, `PUT /grids/{id}` and `POST /grids/{id}/versions`. */
export interface GridInput {
    name: string;
    width: number;
    height: number;
    obs_polygons: Vertex[][];
}

/**
 * One polygon in the working copy.
 *
 * `id` is client-side only — the API stores obstacles as a bare list on the grid, so
 * nothing identifies a shape across a save. It exists so that selection, color and
 * drag targeting survive edits to the list; an array index would reshuffle all three
 * every time a shape ahead of it is deleted.
 */
export interface Obstacle {
    id: number;
    vertices: Vertex[];
}

/** Which end of a route a click is placing. `src`/`dest` match the API's field names. */
export type Endpoint = 'src' | 'dest';

/** Written by `generate_grid_plan` in src/handlers/planner_crud.rs. */
export interface PlanMeta {
    planner: string;
    src_vertex: Vertex;
    dest_vertex: Vertex;
    reachable: boolean;
    /**
     * Scaled by 10 so a diagonal stays an integer: an orthogonal step costs 10, a
     * diagonal 14. See `ORTHOGONAL_COST` in src/models/planners/grid_cost.rs.
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