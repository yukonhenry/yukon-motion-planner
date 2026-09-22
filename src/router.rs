use crate::handlers::grid_crud::{
    create_grid, create_grid_version, delete_grid, list_grids, show_grid, update_grid,
};
use crate::handlers::plan_crud::{
    delete_plan, generate_grid_plan, list_grid_plans, replan_grid,
};
use crate::handlers::robot_crud::{
    create_robot, delete_robot, list_robots, show_robot, update_robot,
};
use crate::handlers::search_trace::search_trace;
use crate::handlers::sim::{show_sim, start_sim, stop_sim, stream_sim};
use crate::scheduler::SimRegistry;
use axum::{Router, routing};
use sea_orm::DatabaseConnection;
use std::sync::Arc;

// App state shared with every handler. Axum clones this per request, so all
// fields must be cheap to clone. `DatabaseConnection` wraps an `sqlx::Pool`,
// which is a reference-counted handle — cloning is one refcount bump, and
// every clone shares the same pool.
#[derive(Clone)]
pub(crate) struct AppState {
    pub(crate) db: DatabaseConnection,
    // The running simulations. Behind an `Arc` because, unlike the pool, a registry is a
    // plain map with no interior sharing of its own — and every request has to reach the
    // *same* one or a start and its stream would find different worlds.
    pub(crate) sims: Arc<SimRegistry>,
}

pub async fn route(db: DatabaseConnection) -> Router {
    let state = AppState {
        db,
        sims: Arc::new(SimRegistry::new()),
    };

    Router::new()
        .route(
            "/",
            routing::get(|| async { "Hello, Yukon Motion Planner!" }),
        )
        .route("/grids", routing::get(list_grids).post(create_grid))
        .route(
            "/grids/{id}",
            routing::get(show_grid).put(update_grid).delete(delete_grid),
        )
        // Editing a grid that plans depend on forks a new snapshot rather than
        // rewriting the one those plans were computed against.
        .route("/grids/{id}/versions", routing::post(create_grid_version))
        // Plans are created against a grid but addressed on their own — a plan id is
        // not a grid id, and the nested delete used to conflate the two.
        .route(
            "/grids/{id}/plans",
            routing::get(list_grid_plans).post(generate_grid_plan),
        )
        // One tick of a moving-obstacle simulation. Stores nothing — the caller holds the
        // evolving geometry and the grid row stays the initial condition, so a run does not
        // freeze the grid or leave a plan row per frame.
        .route("/grids/{id}/replan", routing::post(replan_grid))
        // Not part of a run: one SST search, recorded, so a client can scrub through how the
        // tree grew and what sparsification culled. Stores nothing and moves nothing.
        .route("/grids/{id}/search-trace", routing::post(search_trace))
        // A backend-driven run of the same simulation: the environment and the replanner as
        // two tasks on two frequencies, rather than one tick per click. `start` hands a saved
        // grid and one of its plans to the scheduler; `stream` is how the browser watches.
        .route("/grids/{id}/sim", routing::get(show_sim))
        .route("/grids/{id}/sim/start", routing::post(start_sim))
        .route("/grids/{id}/sim/stop", routing::post(stop_sim))
        .route("/grids/{id}/sim/stream", routing::get(stream_sim))
        .route("/plans/{plan_id}", routing::delete(delete_plan))
        // A robot is a spec — capabilities, no home cell — so it has no scoping parent to
        // nest under. Tying it to a grid would pin it to one frozen snapshot and empty the
        // fleet on the next edit; which world it is in is a property of the scenario.
        .route("/robots", routing::get(list_robots).post(create_robot))
        .route(
            "/robots/{id}",
            routing::get(show_robot).put(update_robot).delete(delete_robot),
        )
        .with_state(state)
}
