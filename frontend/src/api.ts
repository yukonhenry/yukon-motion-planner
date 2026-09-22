import type {
  Grid,
  GridDetail,
  GridInput,
  Plan,
  ReplanInput,
  ReplanResult,
  Robot,
  RobotInput,
  SearchTrace,
  SearchTraceInput,
  SimStatus,
  StartSimInput,
  Vertex,
  WireObstacle,
} from "./types";

/**
 * The API's error responses are plain text, and deliberately human-readable
 * ("vertex [10, 0] is outside grid 3 (10x10)"). Carrying the status alongside lets
 * callers distinguish "you typed a bad id" from "the server fell over".
 */
export class ApiError extends Error {
  constructor(
    readonly status: number,
    message: string,
  ) {
    super(message);
    this.name = "ApiError";
  }
}

/** All paths are relative: Vite proxies `/api` to the backend, so this stays same-origin. */
async function request<T>(path: string, init?: RequestInit): Promise<T> {
  let res: Response;
  try {
    res = await fetch(`/api${path}`, {
      headers: init?.body ? { "content-type": "application/json" } : undefined,
      ...init,
    });
  } catch {
    // fetch only rejects when the request never completed — the API is down or
    // unreachable, which is worth saying plainly rather than as "Failed to fetch".
    throw new ApiError(0, "cannot reach the API — is `cargo run` running on port 3000?");
  }

  if (!res.ok) {
    throw new ApiError(res.status, (await res.text()) || res.statusText);
  }

  // 204 No Content has an empty body, so parsing it as JSON would throw.
  return res.status === 204 ? (undefined as T) : ((await res.json()) as T);
}

/** The listing carries no obstacles — see {@link showGrid} for a whole snapshot. */
export const listGrids = () => request<Grid[]>("/grids");

export const showGrid = (gridId: number) => request<GridDetail>(`/grids/${gridId}`);

/** Starts a new grid at version 0. 409s if the name is already in use. */
export const createGrid = (input: GridInput) =>
  request<GridDetail>("/grids", {
    method: "POST",
    body: JSON.stringify(input),
  });

/**
 * Rewrites a grid in place, obstacles included.
 *
 * Only legal while nothing has been planned against it. Once a plan exists the row is
 * frozen and this 409s, pointing at {@link createGridVersion} — see `ensure_unfrozen`
 * in src/handlers/helpers.rs.
 */
export const updateGrid = (gridId: number, input: GridInput) =>
  request<GridDetail>(`/grids/${gridId}`, {
    method: "PUT",
    body: JSON.stringify(input),
  });

/**
 * Saves an edited grid as the next version of it, leaving the original row untouched
 * so the plans that froze it stay meaningful. Returns a *different* grid id.
 */
export const createGridVersion = (gridId: number, input: GridInput) =>
  request<GridDetail>(`/grids/${gridId}/versions`, {
    method: "POST",
    body: JSON.stringify(input),
  });

/** 409s while plans still reference the grid: delete those first, deliberately. */
export const deleteGrid = (gridId: number) =>
  request<void>(`/grids/${gridId}`, { method: "DELETE" });

/**
 * The plans computed against one grid — and how the UI knows the grid is frozen:
 * a non-empty list means edits have to fork a new version.
 */
export const listPlans = (gridId: number) => request<Plan[]>(`/grids/${gridId}/plans`);

/**
 * Plans an optimal route across one grid with A*, blocking until the search finishes.
 *
 * The obstacles are sent rather than read from the stored grid, and the saved plan keeps
 * its own copy of them. That is what lets a route be planned against a world in motion:
 * the grid row is only ever the initial condition, and a plan generated at tick 37 records
 * tick 37's geometry instead of pointing at obstacles that have since moved on.
 *
 * So `obs_polygons` is the world this route is a route *through*, and it is stored as such —
 * an empty list is a claim that nothing was in the way, not a way of saying "use the grid's".
 */
export const generatePlan = (
  gridId: number,
  src: Vertex,
  dest: Vertex,
  obstacles: WireObstacle[],
  robotId: number,
) =>
  request<Plan>(`/grids/${gridId}/plans`, {
    method: "POST",
    body: JSON.stringify({
      src_vertex: src,
      dest_vertex: dest,
      obs_polygons: obstacles,
      // Required: the plan is where the simulator learns how fast and how often the
      // machine moves, so a route with no driver could never be run.
      robot_id: robotId,
    }),
  });

/** Deleting the last plan of a grid unfreezes it for editing again. */
export const deletePlan = (planId: number) =>
  request<void>(`/plans/${planId}`, { method: "DELETE" });

/**
 * Advances a moving-obstacle simulation one tick and replans, with D* Lite.
 *
 * Stores nothing: no plan row, and the grid's own geometry is left as the initial condition. So
 * unlike {@link generatePlan} this neither freezes the grid nor shows up in {@link listPlans} —
 * the returned result is the only record of the tick, and the caller owns the run's state.
 */
export const replan = (gridId: number, input: ReplanInput) =>
  request<ReplanResult>(`/grids/${gridId}/replan`, {
    method: "POST",
    body: JSON.stringify(input),
  });

// --- search traces -------------------------------------------------------

/**
 * Runs one SST search and returns how it went, without storing or moving anything.
 *
 * A debugging and teaching call rather than part of a run: the response is a fixed timeline a
 * client scrubs over, not a stream. Passing the same `seed` back draws the identical picture,
 * which is what makes a search worth studying shareable.
 *
 * Takes noticeably longer than the other calls — the server runs a real search, a few hundred
 * milliseconds at the default budget — so a caller should show that it is working.
 */
export const searchTrace = (gridId: number, input: SearchTraceInput) =>
  request<SearchTrace>(`/grids/${gridId}/search-trace`, {
    method: "POST",
    body: JSON.stringify(input),
  });

// --- robots --------------------------------------------------------------

/**
 * The whole fleet. Unscoped on purpose: a robot is a spec, not something a grid owns, so
 * this is the same list whatever grid is on screen.
 */
export const listRobots = () => request<Robot[]>("/robots");

/** 400s on an empty name, or a `max_velocity` that is missing, non-numeric, or <= 0. */
export const createRobot = (input: RobotInput) =>
  request<Robot>("/robots", { method: "POST", body: JSON.stringify(input) });

/** Replaces the whole spec — a robot has no freeze rule, so this is always allowed. */
export const updateRobot = (robotId: number, input: RobotInput) =>
  request<Robot>(`/robots/${robotId}`, {
    method: "PUT",
    body: JSON.stringify(input),
  });

/** Takes the robot's route plans with it, by the cascade `route_plans` declares. */
export const deleteRobot = (robotId: number) =>
  request<void>(`/robots/${robotId}`, { method: "DELETE" });

// --- backend-scheduled simulation ----------------------------------------

/**
 * Hands a grid and one of its plans to the backend scheduler.
 *
 * From here the server owns the clock: an environment task jitters the obstacles every
 * `grid_worlds.sim_interval`, a replanner task recomputes the route every
 * `replan_interval` seconds, and both push to {@link simStreamUrl}. Unlike {@link replan},
 * the caller no longer holds the run's state — the server does, and the browser only draws
 * it.
 *
 * 409s if a run is already going on this grid, 400 if nothing on it is dynamic.
 */
export const startSim = (gridId: number, input: StartSimInput) =>
  request<SimStatus>(`/grids/${gridId}/sim/start`, {
    method: "POST",
    body: JSON.stringify(input),
  });

/** Ends the run. Subscribers get a final `stopped` event before the stream closes. */
export const stopSim = (gridId: number) =>
  request<void>(`/grids/${gridId}/sim/stop`, { method: "POST" });

/**
 * The run in progress, or a 404 if there is none.
 *
 * How a reloaded page finds its way back into a run it started before the refresh: the
 * status is the whole picture as of now, and the stream continues from there.
 */
export const simStatus = (gridId: number) => request<SimStatus>(`/grids/${gridId}/sim`);

/**
 * Where to point an `EventSource` for one run's events.
 *
 * A URL rather than a wrapper, because the lifetime of the connection belongs to the
 * component that opens it — see `useSimRun`. Same-origin through the Vite proxy, like every
 * other path here.
 */
export const simStreamUrl = (gridId: number) => `/api/grids/${gridId}/sim/stream`;
