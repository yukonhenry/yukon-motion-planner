//! Driving a backend-scheduled simulation: start it, stop it, and watch it.
//!
//! A run is anchored to a *saved plan*. That plan supplies the endpoints to keep replanning
//! between; the grid supplies the obstacles and, through `grid_worlds.sim_interval`, how
//! often the world moves. So building a grid and computing its first plan stay exactly as
//! static as they were — pressing Start is what hands that finished situation to the
//! scheduler and lets it run.
//!
//! How often to replan is a property of the *run*, not of the plan row, which is why it
//! arrives in the request beside the seed. The eventual answer is the robot's: a replan is
//! due once the robot has moved far enough that what it sensed last time is stale, so the
//! cadence falls out of `robots.capabilities` and the cells it covers per second.
//! [`DEFAULT_REPLAN_SECS`] is the placeholder standing where that computation goes.
//!
//! Everything the run needs is resolved here, once, and passed to
//! [`SimRegistry::start`](crate::scheduler::SimRegistry::start) as plain values. The tasks
//! never touch the database, so deleting the plan mid-run does not disturb it — and does not
//! silently change it either.

use crate::entities::plans;
use crate::handlers::helpers::{AppError, find_grid};
use crate::models::obstacle::ObstaclePoly;
use crate::router::AppState;
use crate::scheduler::{SimStatus, StartError};
use axum::Json;
use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::Sse;
use axum::response::sse::{Event, KeepAlive};
use sea_orm::EntityTrait;
use serde::Deserialize;
use std::convert::Infallible;
use std::time::Duration;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::{Stream, StreamExt};

const DEFAULT_REPLAN_SECS: f64 = 1.0;

/// Body of `POST /grids/{id}/sim/start`.
#[derive(Debug, Deserialize)]
pub(crate) struct StartInput {
    /// The plan to keep replanning. Its `meta` holds the endpoints.
    plan_id: i32,
    /// Seconds between replans, defaulting to [`DEFAULT_REPLAN_SECS`].
    /// Per-run rather than stored; bounds checked by scheduler.
    replan_interval: Option<f64>,
    /// A starting seed, for replaying a run that turned out to be interesting. Omitted, the
    /// run starts somewhere arbitrary and reports where in its status.
    ///
    /// `u32` for the same reason [`crate::handlers::planner_crud::ReplanInput::seed`] is: the
    /// client is JavaScript, where a `u64` past 2^53 loses its low bits in transit and the
    /// run stops replaying with nothing looking wrong.
    seed: Option<u32>,
}

/// Route endpoints extracted from plan's `meta`.
#[derive(Debug, Deserialize)]
struct PlanEndpoints {
    src_vertex: [i32; 2],
    dest_vertex: [i32; 2],
}

impl From<StartError> for AppError {
    fn from(err: StartError) -> Self {
        match err {
            StartError::AlreadyRunning => AppError::Conflict(err.to_string()),
            StartError::BadInterval(_) | StartError::NothingDynamic => {
                AppError::Invalid(err.to_string())
            }
        }
    }
}

// POST /grids/{id}/sim/start — hand the grid and one of its plans to the scheduler.
pub(crate) async fn start_sim(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<StartInput>,
) -> Result<Json<SimStatus>, AppError> {
    let grid = find_grid(&state.db, id).await?;

    let plan = plans::Entity::find_by_id(payload.plan_id)
        .one(&state.db)
        .await?
        .ok_or_else(|| AppError::NotFound(format!("plan {} not found", payload.plan_id)))?;

    // Ensure correct plan
    if plan.grid_id != id {
        return Err(AppError::Invalid(format!(
            "plan {} belongs to grid {}, not grid {id}",
            plan.id, plan.grid_id,
        )));
    }

    let endpoints: PlanEndpoints = serde_json::from_value(plan.meta.clone()).map_err(|e| {
        AppError::Invalid(format!(
            "plan {} has no usable endpoints in its meta: {e}",
            plan.id,
        ))
    })?;

    // Ensure entire set of obstacles are properly decoded
    let obstacles: Vec<ObstaclePoly> = serde_json::from_value(grid.obs_polygons.clone())
        .map_err(|e| AppError::Invalid(format!("stored obstacles are malformed: {e}")))?;

    let status = state.sims.start(
        id,
        plan.id,
        grid.width,
        grid.height,
        obstacles,
        endpoints.src_vertex,
        endpoints.dest_vertex,
        payload.seed.map_or_else(seed_from_clock, u64::from),
        grid.sim_interval,
        payload.replan_interval.unwrap_or(DEFAULT_REPLAN_SECS),
    )?;

    Ok(Json(status))
}

// POST /grids/{id}/sim/stop — end the run.
pub(crate) async fn stop_sim(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<StatusCode, AppError> {
    if state.sims.stop(id, "stopped by request") {
        Ok(StatusCode::NO_CONTENT)
    } else {
        Err(AppError::NotFound(format!(
            "no simulation is running on grid {id}"
        )))
    }
}

// GET /grids/{id}/sim — get current sim state
pub(crate) async fn show_sim(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<Json<SimStatus>, AppError> {
    state
        .sims
        .status(id)
        .map(Json)
        .ok_or_else(|| AppError::NotFound(format!("no simulation is running on grid {id}")))
}

// GET /grids/{id}/sim/stream — server-sent events for one run.
//
// Send simulation events to frontend using SSE events.
pub(crate) async fn stream_sim(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<Sse<impl Stream<Item=Result<Event, Infallible>>>, AppError> {
    let receiver = state
        .sims
        .subscribe(id)
        .ok_or_else(|| AppError::NotFound(format!("no simulation is running on grid {id}")))?;

    let stream = BroadcastStream::new(receiver).filter_map(move |event| match event {
        Ok(event) => {
            let name = event.name();
            Event::default().event(name).json_data(&event).ok().map(Ok)
        }
        // The client fell behind the buffer. Nothing to repair
        Err(err) => {
            tracing::debug!("sim stream for grid {id} lagged: {err}");
            None
        }
    });

    // Keep-alive for connection
    Ok(Sse::new(stream).keep_alive(KeepAlive::new().interval(Duration::from_secs(15))))
}

/// A starting seed for a run that did not bring one.
fn seed_from_clock() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x2026_0814)
}
