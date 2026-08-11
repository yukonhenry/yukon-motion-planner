use crate::handlers::grid_crud::{
    create_grid, create_grid_version, delete_grid, list_grids, show_grid, update_grid,
};
use crate::handlers::planner_crud::{delete_plan, generate_grid_plan, list_grid_plans};
use axum::{Router, routing};
use sea_orm::DatabaseConnection;

// App state shared with every handler. Axum clones this per request, so all
// fields must be cheap to clone. `DatabaseConnection` wraps an `sqlx::Pool`,
// which is a reference-counted handle — cloning is one refcount bump, and
// every clone shares the same pool.
#[derive(Clone)]
pub(crate) struct AppState {
    pub(crate) db: DatabaseConnection,
}

pub async fn route(db: DatabaseConnection) -> Router {
    let state = AppState { db };

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
        .route("/plans/{plan_id}", routing::delete(delete_plan))
        .with_state(state)
}
