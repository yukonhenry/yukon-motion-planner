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
  /** Cells translated per tick, `[dx, dy]`. `[0, 0]` keeps the original random jitter. */
  velocity: Vertex;
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
  /**
   * Seconds between environment ticks when this grid is simulated by the backend.
   *
   * Absent from {@link Grid} because the listing is a partial model that does not select
   * it — a run reads it from the stored row, not from anything the client sends.
   */
  sim_interval: number;
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
  /**
   * Cells travelled per tick, `[dx, dy]`.
   *
   * `[0, 0]` — the default — keeps the original behavior: a dynamic obstacle with no velocity
   * jitters one corner at random rather than travelling. Whole cells, because obstacles
   * round-trip through the API every tick and a fractional speed would need its unspent
   * remainder carried on the wire too.
   *
   * A translation that would overlap another obstacle, run over the robot, or leave the grid
   * is refused for that tick and retried on the next one; nothing latches.
   */
  velocity: Vertex;
  vertices: Vertex[];
}

/** Which end of a route a click is placing. `src`/`dest` match the API's field names. */
export type Endpoint = "src" | "dest";

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
  /** Ids of obstacles whose translation was refused: something, or the grid edge, was there. */
  blocked: number[];
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

/**
 * Written by `generate_grid_plan` in src/handlers/plan_crud.rs.
 *
 * The endpoints used to live here; they are columns on `route_plans` now, so what is left is
 * how the route was found and what it cost. See {@link Plan.src_vertex}.
 */
export interface PlanMeta {
  planner: string;
  reachable: boolean;
  /**
   * Scaled by 10 so a diagonal stays an integer: an orthogonal step costs 10, a
   * diagonal 14. See `ORTHOGONAL_COST` in src/models/planners/movement_model.rs.
   */
  cost: number;
}

/** Mirrors `RoutePlanOutput` in src/handlers/plan_crud.rs. */
export interface Plan {
  id: number;
  /**
   * Which grid this route crosses. Not a column: a route plan points at the *world* it was
   * planned in, and the server resolves the grid behind it so a client that asked about a
   * grid gets an answer in those terms.
   */
  grid_id: number;
  /** The world this route was planned in — the row it actually references. */
  grid_world_state_id: number;
  /** Who drives this route. Every plan has one — a route is planned *for* a machine. */
  robot_id: number;
  name: string;
  src_vertex: Vertex;
  dest_vertex: Vertex;
  /**
   * The cells the route runs through, start first — empty when there is no route.
   *
   * An unreachable goal comes back as a *saved plan* with no cells rather than an
   * error, because it is a fact about the grid rather than a bad request. Only a
   * malformed endpoint (off-grid, or inside an obstacle) is a 4xx.
   */
  route_vertices: Vertex[];
  meta: PlanMeta;
}

// --- robots --------------------------------------------------------------

/**
 * What a robot can do. Free-form on the wire so a new capability is a new key rather than a
 * migration, but `max_velocity` is the one the server insists on — see `validate_robot` in
 * src/handlers/robot_crud.rs.
 */
export interface RobotCapabilities {
  /** Cells travelled per tick of the robot's own clock. Finite and greater than zero. */
  max_velocity: number;
  /**
   * Seconds between one move-and-replan and the next.
   *
   * The robot's own clock, independent of `grid_worlds.sim_interval`: how often the world
   * moves and how often the machine thinks are the experiment. A run reads it from here
   * rather than from the start request, so the same robot keeps its cadence everywhere.
   */
  task_interval: number;
  [key: string]: unknown;
}

/** Mirrors `robots::Model` in src/entities/robots.rs. */
export interface Robot {
  id: number;
  name: string;
  capabilities: RobotCapabilities;
}

/** Body of `POST /robots` and `PUT /robots/{id}`. */
export interface RobotInput {
  name: string;
  capabilities: RobotCapabilities;
}

// --- backend-scheduled simulation ----------------------------------------

/**
 * A run in progress, as `POST /grids/{id}/sim/start` and `GET /grids/{id}/sim` report it.
 *
 * Carries the opening snapshot as well as the parameters, so the canvas is correct the
 * instant a run starts rather than one tick later. The event stream carries it forward from
 * this point.
 */
export interface SimStatus {
  grid_id: number;
  plan_id: number;
  /** Which robot is driving, so a client that joined late can name it. */
  robot_id: number;
  /** Seconds between environment ticks — `grid_worlds.sim_interval`. */
  env_interval: number;
  /**
   * Seconds between the robot's moves, from its own `capabilities.task_interval`.
   *
   * A property of the machine rather than of the run: the same robot keeps its cadence
   * across every experiment, instead of taking it from whoever pressed Run.
   */
  robot_interval: number;
  env_tick: number;
  /** Where the robot is standing right now. */
  robot_position: Vertex;
  obs_polygons: WireObstacle[];
  /** Where the walk is up to, for replaying a run that turned out interesting. */
  seed: number;
}

/**
 * One event off `GET /grids/{id}/sim/stream`.
 *
 * Every variant carries a *whole* snapshot rather than a delta, which is what makes a client
 * that fell behind self-correcting: the next event puts it right, with no resync protocol.
 */
export type SimEvent =
  | {
      type: "environment";
      tick: number;
      /** How many obstacles moved; 0 means every draw clamped against an edge. */
      moved: number;
      /** Ids of obstacles that could not move: another obstacle, or the grid edge, was there. */
      blocked: number[];
      obs_polygons: WireObstacle[];
    }
  | {
      /**
       * A translating obstacle stopped rather than run the robot over.
       *
       * Its own event rather than a field on `environment`, because it is a warning about the
       * machine and not a description of the scenery — a client can listen for it alone.
       */
      type: "collision";
      tick: number;
      obstacle_ids: number[];
      robot_position: Vertex;
    }
  | {
      type: "plan";
      /** The robot's own count, independent of the environment's. */
      tick: number;
      /**
       * Which environment tick this route was planned against. The gap to the latest
       * `environment` tick is how far the planner is running behind the world.
       */
      env_tick: number;
      /** Where the robot is standing after this tick's move. */
      position: Vertex;
      /** How many cells it covered getting there; 0 means it banked a fractional step. */
      moved: number;
      /** The route from `position` onward, empty when the goal is walled off. */
      vertices: Vertex[];
      reachable: boolean;
      cost: number;
      planner: string;
      elapsed_ms: number;
    }
  | { type: "stopped"; reason: string };

/** Body of `POST /grids/{id}/sim/start`. */
export interface StartSimInput {
  /**
   * The plan to run: it supplies the endpoints, the world to start from, and — through its
   * robot — how fast and how often the machine moves. A plan with no robot is refused.
   */
  plan_id: number;
  /** Omit to start somewhere arbitrary — the response reports where. */
  seed?: number;
}

// --- SST search traces ----------------------------------------------------

/**
 * One pose along a trajectory: `[x, y, theta]`, metres and radians.
 *
 * Metric, not cells. A kinodynamic plan is a curve through continuous space, and the whole
 * reason to draw it is the part a cell index cannot express — where between two cells the
 * robot is, and which way it is pointing. {@link SearchTrace.meters_per_cell} is what puts it
 * back on the grid.
 */
export type Pose = [number, number, number];

/**
 * One thing that happened to the search tree, mirroring `SearchEvent` in
 * src/models/planners/rrt/sst.rs.
 *
 * In the order it happened, and complete: every node arrives after its parent and every prune
 * names a node added earlier, so applying a prefix in order always yields a drawable tree.
 */
export type SearchEvent =
  | {
      type: "node_added";
      iteration: number;
      id: number;
      parent: number;
      /** Thinned to a handful of poses — enough for the curve, not every integration step. */
      path: Pose[];
      /** Cost-to-come, in seconds. */
      cost: number;
    }
  | {
      /**
       * Sparsification culled a node: something cheaper took its witness and nothing was
       * descended from it. This is the event that distinguishes SST from a plain kinodynamic
       * RRT, and the one the player exists to show.
       */
      type: "node_pruned";
      iteration: number;
      id: number;
    }
  | { type: "witness_added"; iteration: number; x: number; y: number }
  | { type: "solution_improved"; iteration: number; cost: number };

/** What one search cost, mirroring `TraceStats` in src/handlers/search_trace.rs. */
export interface TraceStats {
  iterations: number;
  nodes: number;
  nodes_created: number;
  nodes_pruned: number;
  witnesses: number;
  collision_queries: number;
  extensions_failed: number;
  dominated: number;
  selection_fallbacks: number;
  selection_ms: number;
  propagation_ms: number;
  bookkeeping_ms: number;
  total_ms: number;
  /** Metres. Well above the goal radius on a search that never arrived. */
  closest_approach: number;
  /** `[iteration, elapsed_ms, cost_seconds]` each time the best solution improved. */
  improvements: [number, number, number][];
}

/** Body of `POST /grids/{id}/search-trace`. */
export interface SearchTraceInput {
  src_vertex: Vertex;
  dest_vertex: Vertex;
  /** Sent rather than read from the stored row, so a moving world can be traced as drawn. */
  obs_polygons: WireObstacle[];
  robot_id?: number;
  /** Omit for an arbitrary search; the response reports which seed was used. */
  seed?: number;
  iterations?: number;
}

/** What `POST /grids/{id}/search-trace` answers with. Nothing is stored. */
export interface SearchTrace {
  events: SearchEvent[];
  stats: TraceStats;
  /** The best trajectory found. Empty when the goal was never reached. */
  solution: Pose[];
  solution_cost: number;
  reachable: boolean;
  start: Pose;
  goal: [number, number];
  /** Metres per cell, for placing everything above on the grid this app already draws. */
  meters_per_cell: number;
  /** The seed actually used, so this exact picture can be asked for again. */
  seed: number;
}
