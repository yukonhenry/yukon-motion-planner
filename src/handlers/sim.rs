//! Driving a backend-scheduled simulation: start it, stop it, and watch it.
//!
//! A run is anchored to a *saved route plan*. That plan supplies the endpoints to keep
//! replanning between and, through the world it points at, the obstacles to start from; the
//! grid supplies the dimensions and, through `grid_worlds.sim_interval`, how often the world
//! moves. So building a grid and computing its first plan stay exactly as
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

use crate::entities::{grid_world_states, robots, route_plans};
use crate::handlers::helpers::{AppError, find_grid};
use crate::models::obstacle::ObstaclePoly;
use crate::models::robot::RobotSpec;
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

/// Body of `POST /grids/{id}/sim/start`.
#[derive(Debug, Deserialize)]
pub(crate) struct StartInput {
    /// The route plan to keep replanning. Its `src_vertex`/`dest_vertex` are the endpoints,
    /// and the world it points at is where the run starts.
    plan_id: i32,
    /// A starting seed, for replaying a run that turned out to be interesting. Omitted, the
    /// run starts somewhere arbitrary and reports where in its status.
    ///
    /// `u32` for the same reason [`crate::handlers::plan_crud::ReplanInput::seed`] is: the
    /// client is JavaScript, where a `u64` past 2^53 loses its low bits in transit and the
    /// run stops replaying with nothing looking wrong.
    seed: Option<u32>,
}


impl From<StartError> for AppError {
    fn from(err: StartError) -> Self {
        match err {
            StartError::AlreadyRunning => AppError::Conflict(err.to_string()),
            StartError::BadInterval(_)
            | StartError::NothingDynamic
            | StartError::AlreadyArrived => AppError::Invalid(err.to_string()),
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

    let plan = route_plans::Entity::find_by_id(payload.plan_id)
        .one(&state.db)
        .await?
        .ok_or_else(|| AppError::NotFound(format!("plan {} not found", payload.plan_id)))?;

    // The world the plan was computed in, which is also where the run starts. Fetched rather
    // than read off the plan, because the plan points at it instead of carrying a copy.
    let world = grid_world_states::Entity::find_by_id(plan.grid_world_state_id)
        .one(&state.db)
        .await?
        .ok_or_else(|| {
            AppError::NotFound(format!(
                "plan {} points at world {}, which no longer exists",
                plan.id, plan.grid_world_state_id,
            ))
        })?;

    // Ensure correct plan
    if world.grid_world_id != id {
        return Err(AppError::Invalid(format!(
            "plan {} belongs to grid {}, not grid {id}",
            plan.id, world.grid_world_id,
        )));
    }

    let src_vertex: [i32; 2] = serde_json::from_value(plan.src_vertex.clone())
        .map_err(|e| AppError::Invalid(format!("plan {} has an unusable start: {e}", plan.id)))?;
    let dest_vertex: [i32; 2] = serde_json::from_value(plan.dest_vertex.clone())
        .map_err(|e| AppError::Invalid(format!("plan {} has an unusable goal: {e}", plan.id)))?;

    // Ensure entire set of obstacles are properly decoded
    let obstacles: Vec<ObstaclePoly> = serde_json::from_value(world.obs_polygons.clone())
        .map_err(|e| AppError::Invalid(format!("stored obstacles are malformed: {e}")))?;

    // Every plan has a driver — the column is NOT NULL — so this is a lookup rather than a
    // check. It can still miss: the row is only gone if the robot was deleted, and the
    // cascade would have taken this plan with it, so a miss here means a genuinely odd state.
    let robot_id = plan.robot_id;
    let robot = robots::Entity::find_by_id(robot_id)
        .one(&state.db)
        .await?
        .ok_or_else(|| AppError::NotFound(format!("robot {robot_id} not found")))?;
    let spec = RobotSpec::from_capabilities(&robot.capabilities)
        .map_err(|err| AppError::Invalid(format!("robot {robot_id} cannot be run: {err}")))?;

    // The route the plan already found, so the run opens on exactly the saved plan and the
    // robot's first act is to move rather than to re-derive what is already on screen.
    let route: Vec<(usize, usize)> = serde_json::from_value(plan.route_vertices.clone())
        .map_err(|e| AppError::Invalid(format!("plan {} has an unusable route: {e}", plan.id)))?;

    let status = state.sims.start(
        id,
        plan.id,
        robot_id,
        spec,
        grid.width,
        grid.height,
        obstacles,
        src_vertex,
        dest_vertex,
        route,
        payload.seed.map_or_else(seed_from_clock, u64::from),
        grid.sim_interval,
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
