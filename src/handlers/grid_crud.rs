//! Grid CRUD.
//!
//! A grid row is a *snapshot*: name, dimensions and obstacle geometry together. It is
//! editable in place right up until a plan is computed against it, at which point the
//! row freezes and further edits fork a new row at `version + 1`. See
//! [`ensure_unfrozen`] for why, and `POST /grids/{id}/versions` for the fork.

use crate::entities::grid_worlds;
use crate::handlers::helpers::{AppError, ensure_unfrozen, find_grid};
use crate::models::obstacle::{CellVertex, ObstaclePoly};
use crate::router::AppState;
use axum::extract::Path;
use axum::{Json, extract::State, http::StatusCode};
use sea_orm::{ActiveModelTrait, ColumnTrait, DerivePartialModel, EntityTrait, QueryFilter, Set};
use serde::{Deserialize, Serialize};

// Request body for creating or replacing a grid.
// For Update, id comes from path.
#[derive(Debug, Deserialize)]
pub(crate) struct GridInput {
    name: String,
    width: i32,
    height: i32,
    obs_polygons: Vec<ObstaclePoly>,
}

#[derive(Debug, Serialize, DerivePartialModel)]
#[sea_orm(entity = "grid_worlds::Entity")]
pub(crate) struct GridOutput {
    id: i32,
    name: String,
    width: i32,
    height: i32,
    version: i32,
}

// --- shared helpers ------------------------------------------------------

// The first vertex outside a `width` x `height` grid, if there is one. Vertices are
// cell indices, so the far edge is out of range: a 10-wide grid addresses columns 0..=9.
fn out_of_bounds(vertices: &[CellVertex], width: i32, height: i32) -> Option<CellVertex> {
    vertices
        .iter()
        .copied()
        .find(|v| !(0..width).contains(&v.x) || !(0..height).contains(&v.y))
}

// Every polygon needs three corners, all of them on the grid, and an id no sibling shares.
//
// Obstacles arrive with the grid rather than through routes of their own, so this is
// the only gate: a payload that fails here is rejected whole, leaving the stored grid
// exactly as it was.
pub(crate) fn validate_polygons(
    polygons: &[ObstaclePoly],
    width: i32,
    height: i32,
) -> Result<(), AppError> {
    for (index, obstacle) in polygons.iter().enumerate() {
        if obstacle.vertices.len() < 3 {
            return Err(AppError::Invalid(format!(
                "obstacle {index} needs at least 3 vertices, got {}",
                obstacle.vertices.len()
            )));
        }

        if let Some(vertex) = out_of_bounds(&obstacle.vertices, width, height) {
            return Err(AppError::Invalid(format!(
                "obstacle {index} has vertex [{}, {}] outside the {width}x{height} grid",
                vertex.x, vertex.y,
            )));
        }

        // Detect a shared id by scanning the earlier obstacles for one with the same id. The
        // earlier one is the one that "owns" the id, and the later one is the one that is trying to use it.
        if let Some(earlier) = polygons[..index].iter().position(|o| o.id == obstacle.id) {
            return Err(AppError::Invalid(format!(
                "obstacles {earlier} and {index} share id {} — ids must be distinct",
                obstacle.id,
            )));
        }
    }

    Ok(())
}

// `to_value` rather than a hand-built `Value`: for a struct of `i32`, `bool` and `Vec`,
// serialization cannot fail — the error cases are non-string map keys and non-finite floats,
// neither of which this type can produce.
fn polygons_to_json(polygons: &[ObstaclePoly]) -> serde_json::Value {
    serde_json::to_value(polygons).expect("obstacle geometry is always serializable")
}
// --- grids ---------------------------------------------------------------

// GET /grids — list all grids.
pub(crate) async fn list_grids(
    State(state): State<AppState>,
) -> Result<Json<Vec<GridOutput>>, AppError> {
    let grids = grid_worlds::Entity::find()
        .into_partial_model::<GridOutput>()
        .all(&state.db)
        .await?;
    Ok(Json(grids))
}

// POST /grids — start a new grid at version 0.
//
// Obstacles arrive with the grid rather than through routes of their own, so the client
// is expected to hold the whole drawing locally and post it once, on confirm.
pub(crate) async fn create_grid(
    State(state): State<AppState>,
    Json(payload): Json<GridInput>,
) -> Result<(StatusCode, Json<grid_worlds::Model>), AppError> {
    validate_polygons(&payload.obs_polygons, payload.width, payload.height)?;

    // The name is the lineage: every version of a grid shares it. Checking here turns
    // the constraint's generic complaint into advice — the caller wanted the grid they
    // already have, and needs to edit or version it rather than start a second one.
    let taken = grid_worlds::Entity::find()
        .filter(grid_worlds::Column::Name.eq(&payload.name))
        .one(&state.db)
        .await?;
    if let Some(existing) = taken {
        return Err(AppError::Conflict(format!(
            "a grid named \"{}\" already exists (id {}, version {}) — \
             edit it, or save a new version of it",
            payload.name, existing.id, existing.version
        )));
    }

    let new_grid = grid_worlds::ActiveModel {
        name: Set(payload.name),
        width: Set(payload.width),
        height: Set(payload.height),
        obs_polygons: Set(polygons_to_json(&payload.obs_polygons)),
        version: Set(0),
        ..Default::default() // leaves `id` unset so the DB generates it
    };
    let saved = new_grid.insert(&state.db).await?;
    Ok((StatusCode::CREATED, Json(saved)))
}

// POST /grids/{id}/versions — save an edited grid as the next version of it.
//
// The parent row is left exactly as it was, so the plans that froze it stay meaningful.
// Version numbers come from the parent rather than from a scan of the name, which is
// what lets `idx-grid_worlds-name-version` catch two clients forking the same snapshot:
// both compute the same next version and the loser gets a 409.
pub(crate) async fn create_grid_version(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<GridInput>,
) -> Result<(StatusCode, Json<grid_worlds::Model>), AppError> {
    let parent = find_grid(&state.db, id).await?;
    validate_polygons(&payload.obs_polygons, payload.width, payload.height)?;

    let new_grid = grid_worlds::ActiveModel {
        name: Set(payload.name),
        width: Set(payload.width),
        height: Set(payload.height),
        obs_polygons: Set(polygons_to_json(&payload.obs_polygons)),
        version: Set(parent.version + 1),
        ..Default::default()
    };
    let saved = new_grid.insert(&state.db).await?;
    Ok((StatusCode::CREATED, Json(saved)))
}

// GET /grids/{id} — show one grid.
pub(crate) async fn show_grid(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<Json<grid_worlds::Model>, AppError> {
    let grid = find_grid(&state.db, id).await?;
    Ok(Json(grid))
}

// PUT /grids/{id} — replace a grid's fields.
pub(crate) async fn update_grid(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<GridInput>,
) -> Result<Json<grid_worlds::Model>, AppError> {
    let grid = find_grid(&state.db, id).await?;

    // The whole row is frozen once anything has been planned against it, dimensions
    // included: resizing a grid invalidates a stored route at least as thoroughly as
    // moving an obstacle does.
    ensure_unfrozen(
        &state.db,
        id,
        "edited",
        &format!("POST /grids/{id}/versions to save a new version, or delete the plans first"),
    )
        .await?;

    // PUT replaces the whole grid, obstacles included — with the `/obstacles` routes
    // gone this is the only way to edit them. Validating against the *requested*
    // bounds is also what stops a shrink from stranding obstacles out of bounds: a
    // caller who resends the existing obstacles alongside smaller dimensions gets a
    // 400 here rather than a grid whose obstacles no longer fit.
    validate_polygons(&payload.obs_polygons, payload.width, payload.height)?;

    let mut grid: grid_worlds::ActiveModel = grid.into();
    grid.name = Set(payload.name);
    grid.width = Set(payload.width);
    grid.height = Set(payload.height);
    grid.obs_polygons = Set(polygons_to_json(&payload.obs_polygons));

    Ok(Json(grid.update(&state.db).await?))
}

// DELETE /grids/{id} — delete a grid_world
pub(crate) async fn delete_grid(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<StatusCode, AppError> {
    // The schema would happily cascade the plans away. If a plan is important enough to
    // freeze the grid against edits, it is important enough not to be destroyed as a
    // side effect of deleting the grid — so the plans have to go first, deliberately.
    // An unknown grid counts zero plans and falls through to the 404 below.
    ensure_unfrozen(&state.db, id, "deleted", "delete its plans first").await?;

    // `rows_affected` doubles as the existence check, so this stays one round trip.
    let res = grid_worlds::Entity::delete_by_id(id)
        .exec(&state.db)
        .await?;
    if res.rows_affected == 0 {
        return Err(AppError::NotFound(format!("grid {id} not found")));
    }

    Ok(StatusCode::NO_CONTENT)
}
