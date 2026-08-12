import type {
  Grid,
  GridDetail,
  GridInput,
  Plan,
  ReplanInput,
  ReplanResult,
  Vertex,
} from './types';

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
    this.name = 'ApiError';
  }
}

/** All paths are relative: Vite proxies `/api` to the backend, so this stays same-origin. */
async function request<T>(path: string, init?: RequestInit): Promise<T> {
  let res: Response;
  try {
    res = await fetch(`/api${path}`, {
      headers: init?.body ? { 'content-type': 'application/json' } : undefined,
      ...init,
    });
  } catch {
    // fetch only rejects when the request never completed — the API is down or
    // unreachable, which is worth saying plainly rather than as "Failed to fetch".
    throw new ApiError(0, 'cannot reach the API — is `cargo run` running on port 3000?');
  }

  if (!res.ok) {
    throw new ApiError(res.status, (await res.text()) || res.statusText);
  }

  // 204 No Content has an empty body, so parsing it as JSON would throw.
  return res.status === 204 ? (undefined as T) : ((await res.json()) as T);
}

/** The listing carries no obstacles — see {@link showGrid} for a whole snapshot. */
export const listGrids = () => request<Grid[]>('/grids');

export const showGrid = (gridId: number) => request<GridDetail>(`/grids/${gridId}`);

/** Starts a new grid at version 0. 409s if the name is already in use. */
export const createGrid = (input: GridInput) =>
  request<GridDetail>('/grids', { method: 'POST', body: JSON.stringify(input) });

/**
 * Rewrites a grid in place, obstacles included.
 *
 * Only legal while nothing has been planned against it. Once a plan exists the row is
 * frozen and this 409s, pointing at {@link createGridVersion} — see `ensure_unfrozen`
 * in src/handlers/helpers.rs.
 */
export const updateGrid = (gridId: number, input: GridInput) =>
  request<GridDetail>(`/grids/${gridId}`, { method: 'PUT', body: JSON.stringify(input) });

/**
 * Saves an edited grid as the next version of it, leaving the original row untouched
 * so the plans that froze it stay meaningful. Returns a *different* grid id.
 */
export const createGridVersion = (gridId: number, input: GridInput) =>
  request<GridDetail>(`/grids/${gridId}/versions`, {
    method: 'POST',
    body: JSON.stringify(input),
  });

/** 409s while plans still reference the grid: delete those first, deliberately. */
export const deleteGrid = (gridId: number) =>
  request<void>(`/grids/${gridId}`, { method: 'DELETE' });

/**
 * The plans computed against one grid — and how the UI knows the grid is frozen:
 * a non-empty list means edits have to fork a new version.
 */
export const listPlans = (gridId: number) => request<Plan[]>(`/grids/${gridId}/plans`);

/**
 * Plans an optimal route across one grid, blocking until the search finishes.
 *
 * The server rasterizes the obstacles *as stored* and runs A* per request, so this
 * plans against the saved grid rather than whatever is on the canvas — which is why
 * the UI refuses to call it while there are unsaved edits.
 */
export const generatePlan = (gridId: number, src: Vertex, dest: Vertex) =>
  request<Plan>(`/grids/${gridId}/plans`, {
    method: 'POST',
    body: JSON.stringify({ src_vertex: src, dest_vertex: dest }),
  });

/** Deleting the last plan of a grid unfreezes it for editing again. */
export const deletePlan = (planId: number) =>
  request<void>(`/plans/${planId}`, { method: 'DELETE' });

/**
 * Advances a moving-obstacle simulation one tick and replans, with D* Lite.
 *
 * Stores nothing: no plan row, and the grid's own geometry is left as the initial condition. So
 * unlike {@link generatePlan} this neither freezes the grid nor shows up in {@link listPlans} —
 * the returned result is the only record of the tick, and the caller owns the run's state.
 */
export const replan = (gridId: number, input: ReplanInput) =>
  request<ReplanResult>(`/grids/${gridId}/replan`, {
    method: 'POST',
    body: JSON.stringify(input),
  });
