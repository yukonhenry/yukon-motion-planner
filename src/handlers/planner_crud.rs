use crate::entities::plans;
use crate::handlers::helpers::{AppError, find_grid};
use crate::models::cell::Cell;
use crate::models::grid_world_manager::GridWorldManager;
use crate::models::planners::PlanError;
use crate::router::AppState;
use axum::Json;
use axum::extract::{Path, State};
use axum::http::StatusCode;
use sea_orm::{ActiveModelTrait, ColumnTrait, EntityTrait, QueryFilter, Set};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub(crate) struct PlanInput {
    src_vertex: [i32; 2],
    dest_vertex: [i32; 2],
}

// GET /grids/{id}/plans — the plans computed against this grid.
//
// Also how a client learns the grid is frozen: a non-empty list means edits have to
// fork a new version. Deriving it from the plans themselves keeps the client's idea of
// "frozen" and the server's the same thing rather than two flags to keep in step.
pub(crate) async fn list_grid_plans(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<Json<Vec<plans::Model>>, AppError> {
    // An unknown grid is a 404 rather than an empty list, so a stale id can't read as
    // "this grid is editable".
    find_grid(&state.db, id).await?;

    let found = plans::Entity::find()
        .filter(plans::Column::GridId.eq(id))
        .all(&state.db)
        .await?;
    Ok(Json(found))
}

pub(crate) async fn generate_grid_plan(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<PlanInput>,
) -> Result<(StatusCode, Json<plans::Model>), AppError> {
    let grid = find_grid(&state.db, id).await?;
    let mut grid_world = GridWorldManager::<Cell>::new(grid.width as usize, grid.height as usize);

    // Decoding here rather than inside the grid keeps the model free of serde/axum
    // types. A row we can't read is fatal: planning around obstacles we silently
    // dropped would route straight through them.
    let polygons = grid.obs_polygons
        .as_array()
        .unwrap_or(&vec![])
        .iter()
        .map(|obstacle| serde_json::from_value::<Vec<[i32; 2]>>(obstacle.clone()))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| AppError::Invalid(format!("obstacle vertices are malformed: {e}")))?;

    grid_world.rasterize_polygons(&polygons, |cell| cell.blocked = true);

    // An unreachable goal is a fact about the world, not a bad request: the grid and
    // both endpoints were fine, there is just no route, and an empty plan says so. A
    // malformed endpoint is the caller's mistake and has to be a 400 — saving it as an
    // empty plan would report a typo'd coordinate back as 201 Created.
    let optimal_path = match grid_world.find_optimal_plan(payload.src_vertex, payload.dest_vertex) {
        Ok(path) => path,
        Err(PlanError::Unreachable) => Vec::new(),
        Err(err) => return Err(AppError::Invalid(err.to_string())),
    };
    let vertices = optimal_path.iter()
        .map(|cell| grid_world.xy(*cell)).collect::<Vec<_>>();

    // `meta` is NOT NULL, so it has to be set here rather than left to a column default
    // that the migration does not define. What belongs in it is how the route was
    // produced: a stored plan is only interpretable alongside the planner and the
    // endpoints that produced it, and `vertices` alone cannot distinguish "no route
    // exists" from a row written before the planner understood obstacles.
    let meta = serde_json::json!({
        "planner": "a_star",
        "src_vertex": payload.src_vertex,
        "dest_vertex": payload.dest_vertex,
        "reachable": !vertices.is_empty(),
        "cost": grid_world.path_cost(&optimal_path),
    });

    let new_plan = plans::ActiveModel {
        grid_id: Set(id),
        name: Set(String::from("Prototype")),
        vertices: Set(serde_json::to_value(vertices).map_err(|e| {
            AppError::Invalid(format!("failed to serialize plan vertices: {e}"))
        })?),
        meta: Set(meta),
        ..Default::default() // leaves `id` unset so the DB generates it
    };

    let saved = new_plan.insert(&state.db).await?;
    Ok((StatusCode::CREATED, Json(saved)))
}

// DELETE /plans/{plan_id} — delete one plan.
//
// Top level rather than nested under the grid: a plan is identified by its own id, and
// nesting it invited the previous bug where the *grid* id from the path was deleted as
// though it were a plan id.
pub(crate) async fn delete_plan(
    State(state): State<AppState>,
    Path(plan_id): Path<i32>,
) -> Result<StatusCode, AppError> {
    // `rows_affected` doubles as the existence check, so this stays one round trip.
    let res = plans::Entity::delete_by_id(plan_id).exec(&state.db).await?;
    if res.rows_affected == 0 {
        return Err(AppError::NotFound(format!("plan {plan_id} not found")));
    }

    Ok(StatusCode::NO_CONTENT)
}