//! `POST /grids/{id}/search-trace` — run one SST search and hand back how it went.
//!
//! A debugging and teaching endpoint rather than part of the simulation. It stores nothing,
//! moves nothing, and drives no robot: it runs a search over the geometry it is given and
//! returns every node the tree gained, every branch sparsification culled, and the trajectory
//! it settled on, in the order they happened. A client can then scrub back and forth over a
//! fixed timeline.
//!
//! One-shot rather than streamed on purpose. The search is a few hundred milliseconds, so
//! there is nothing to watch live that a scrubber does not show better, and a request-response
//! needs no backpressure, no resync and no per-tick reset. A seed replays the same picture
//! exactly, which is what makes a run worth studying shareable.

use crate::handlers::helpers::{AppError, find_grid, validate_polygons};
use crate::models::obstacle::ObstaclePoly;
use crate::models::planners::rrt::sst::SearchEvent;
use crate::models::planners::rrt::trace::{
    DEFAULT_TRACE_ITERATIONS, MAX_TRACE_ITERATIONS, trace_search,
};
use crate::models::robot::RobotSpec;
use crate::models::robots::unicycle_spec::UnicycleSpec;
use crate::models::scale::METERS_PER_CELL;
use crate::router::AppState;
use axum::Json;
use axum::extract::{Path, State};
use sea_orm::EntityTrait;
use serde::{Deserialize, Serialize};

#[derive(Deserialize)]
pub(crate) struct SearchTraceInput {
    pub src_vertex: [i32; 2],
    pub dest_vertex: [i32; 2],
    /// The geometry to search, sent rather than read from the stored row for the same reason
    /// `POST /grids/{id}/replan` takes it: a client watching a moving world wants to trace the
    /// arrangement on screen now, not the one the grid was saved with.
    pub obs_polygons: Vec<ObstaclePoly>,
    /// Whose body to sweep. Omitted means the default machine.
    #[serde(default)]
    pub robot_id: Option<i32>,
    /// Omit for an arbitrary search; the response reports which seed was used, so an
    /// interesting one can be asked for again.
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default)]
    pub iterations: Option<usize>,
}

/// What a search cost, flattened for a client that wants to show it beside the picture.
#[derive(Serialize)]
pub(crate) struct TraceStats {
    pub iterations: usize,
    pub nodes: usize,
    pub nodes_created: usize,
    pub nodes_pruned: usize,
    pub witnesses: usize,
    pub collision_queries: u64,
    pub extensions_failed: usize,
    pub dominated: usize,
    pub selection_fallbacks: usize,
    /// Milliseconds, split by phase — selection is the nearest-neighbor scan, propagation is
    /// forward integration and collision checking.
    pub selection_ms: f64,
    pub propagation_ms: f64,
    pub bookkeeping_ms: f64,
    pub total_ms: f64,
    /// The closest any node came to the goal, in metres. Well above the goal radius on a
    /// search that never arrived.
    pub closest_approach: f64,
    /// `(iteration, elapsed_ms, cost_seconds)` each time the best solution improved.
    pub improvements: Vec<(usize, f64, f64)>,
}

#[derive(Serialize)]
pub(crate) struct SearchTraceOutput {
    pub events: Vec<SearchEvent>,
    pub stats: TraceStats,
    /// The best trajectory found, as `[x, y, theta]`. Empty when the goal was not reached.
    pub solution: Vec<[f64; 3]>,
    pub solution_cost: f64,
    pub reachable: bool,
    pub start: [f64; 3],
    pub goal: [f64; 2],
    /// Everything above is in metres; this is what a cell is worth, so a client can place it
    /// on the same grid it already draws without hardcoding the conversion.
    pub meters_per_cell: f64,
    /// The seed actually used, so this exact picture can be asked for again.
    pub seed: u64,
}

pub(crate) async fn search_trace(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<SearchTraceInput>,
) -> Result<Json<SearchTraceOutput>, AppError> {
    let grid = find_grid(&state.db, id).await?;
    validate_polygons(&payload.obs_polygons, grid.width, grid.height)?;

    // A named robot has to exist, and its capabilities have to describe a machine — the same
    // check a run makes, so a trace cannot show a body no simulation would accept.
    let robot = match payload.robot_id {
        None => UnicycleSpec::default(),
        Some(robot_id) => {
            let row = crate::entities::robots::Entity::find_by_id(robot_id)
                .one(&state.db)
                .await?
                .ok_or_else(|| AppError::NotFound(format!("robot {robot_id} not found")))?;
            RobotSpec::from_capabilities(&row.capabilities)
                .map_err(|err| AppError::Invalid(err.to_string()))?
                .body
        }
    };

    let seed = payload.seed.unwrap_or_else(seed_from_clock);
    let iterations = payload
        .iterations
        .unwrap_or(DEFAULT_TRACE_ITERATIONS)
        .clamp(1, MAX_TRACE_ITERATIONS);

    let (width, height) = (grid.width, grid.height);
    let (src, dest) = (payload.src_vertex, payload.dest_vertex);
    let obstacles = payload.obs_polygons;

    // On the blocking pool, not inline. A search is hundreds of milliseconds of pure CPU, and
    // holding a runtime worker for that would stall every simulation tick sharing it — which
    // is exactly why the grid planners, which return in microseconds, can stay inline and this
    // cannot.
    let traced = tokio::task::spawn_blocking(move || {
        trace_search(width, height, &obstacles, src, dest, robot, seed, iterations)
    })
    .await
    .map_err(|err| AppError::Invalid(format!("the search did not finish: {err}")))?;

    let stats = &traced.stats;
    let ms = |d: std::time::Duration| d.as_secs_f64() * 1e3;

    Ok(Json(SearchTraceOutput {
        stats: TraceStats {
            iterations: stats.iterations,
            nodes: stats.nodes,
            nodes_created: stats.nodes_created,
            nodes_pruned: stats.nodes_pruned,
            witnesses: stats.witnesses,
            collision_queries: stats.collision_queries,
            extensions_failed: stats.extensions_failed,
            dominated: stats.dominated,
            selection_fallbacks: stats.selection_fallbacks,
            selection_ms: ms(stats.selection),
            propagation_ms: ms(stats.propagation),
            bookkeeping_ms: ms(stats.bookkeeping),
            total_ms: ms(stats.total),
            closest_approach: stats.closest_approach,
            improvements: stats
                .improvements
                .iter()
                .map(|i| (i.iteration, ms(i.elapsed), i.cost))
                .collect(),
        },
        reachable: !traced.solution.is_empty(),
        solution: traced.solution,
        solution_cost: traced.solution_cost,
        start: traced.start,
        goal: traced.goal,
        meters_per_cell: METERS_PER_CELL,
        seed,
        events: traced.events,
    }))
}

/// A seed when the caller did not choose one. Reported back, so an interesting search can be
/// replayed even though it was started arbitrarily.
fn seed_from_clock() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x2026_0922)
}
