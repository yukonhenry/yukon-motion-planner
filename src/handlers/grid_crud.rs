//! Grid CRUD.
//!
//! A grid is two rows now: `grid_worlds` holds the name and dimensions, and its
//! `sequence_id = 0` row in `grid_world_states` holds the obstacles it starts from. Later
//! sequence numbers are where a simulation has carried that world; grid CRUD only ever reads
//! and rewrites the opening one. The wire shape is unchanged — a client still sees one
//! object, stitched back together by [`GridDetail`].
//!
//! The pair is editable in place right up until a plan is computed against it, at which point
//! it freezes and further edits fork a new grid at `version + 1`. See [`ensure_unfrozen`] for
//! why, and `POST /grids/{id}/versions` for the fork.

use crate::entities::{grid_world_states, grid_worlds};
use crate::handlers::helpers::{
    AppError, ensure_unfrozen, find_grid, find_initial_state, insert_initial_state,
    polygons_to_json, validate_polygons,
};
use crate::models::obstacle::ObstaclePoly;
use crate::router::AppState;
use axum::extract::Path;
use axum::{Json, extract::State, http::StatusCode};
use sea_orm::{
    ActiveModelTrait, ColumnTrait, DerivePartialModel, EntityTrait, QueryFilter, Set,
    TransactionTrait,
};
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

/// One whole grid: the row, plus the world it starts from.
///
/// Assembled here rather than serialized straight off `grid_worlds::Model`, because the
/// obstacles moved to their own table and a caller should not have to fetch two things to
/// draw one grid.
#[derive(Debug, Serialize)]
pub(crate) struct GridDetail {
    id: i32,
    name: String,
    width: i32,
    height: i32,
    version: i32,
    sim_interval: f64,
    obs_polygons: serde_json::Value,
}

impl GridDetail {
    fn new(grid: grid_worlds::Model, obs_polygons: serde_json::Value) -> Self {
        Self {
            id: grid.id,
            name: grid.name,
            width: grid.width,
            height: grid.height,
            version: grid.version,
            sim_interval: grid.sim_interval,
            obs_polygons,
        }
    }
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
) -> Result<(StatusCode, Json<GridDetail>), AppError> {
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

    // Both rows or neither: a grid whose opening world never landed is one every reader
    // downstream would have to special-case, and `find_initial_state` treats as corruption.
    let txn = state.db.begin().await?;

    let new_grid = grid_worlds::ActiveModel {
        name: Set(payload.name),
        width: Set(payload.width),
        height: Set(payload.height),
        version: Set(0),
        ..Default::default() // leaves `id` unset so the DB generates it
    };
    let saved = new_grid.insert(&txn).await?;
    let state_row = insert_initial_state(&txn, saved.id, &payload.obs_polygons).await?;

    txn.commit().await?;
    Ok((
        StatusCode::CREATED,
        Json(GridDetail::new(saved, state_row.obs_polygons)),
    ))
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
) -> Result<(StatusCode, Json<GridDetail>), AppError> {
    let parent = find_grid(&state.db, id).await?;
    validate_polygons(&payload.obs_polygons, payload.width, payload.height)?;

    // The fork gets its own opening world from the payload — the parent's states are the
    // parent's, and a version is a new lineage of them rather than a continuation.
    let txn = state.db.begin().await?;

    let new_grid = grid_worlds::ActiveModel {
        name: Set(payload.name),
        width: Set(payload.width),
        height: Set(payload.height),
        version: Set(parent.version + 1),
        ..Default::default()
    };
    let saved = new_grid.insert(&txn).await?;
    let state_row = insert_initial_state(&txn, saved.id, &payload.obs_polygons).await?;

    txn.commit().await?;
    Ok((
        StatusCode::CREATED,
        Json(GridDetail::new(saved, state_row.obs_polygons)),
    ))
}

// GET /grids/{id} — show one grid.
pub(crate) async fn show_grid(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<Json<GridDetail>, AppError> {
    let grid = find_grid(&state.db, id).await?;
    let state_row = find_initial_state(&state.db, id).await?;
    Ok(Json(GridDetail::new(grid, state_row.obs_polygons)))
}

// PUT /grids/{id} — replace a grid's fields.
pub(crate) async fn update_grid(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<GridInput>,
) -> Result<Json<GridDetail>, AppError> {
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

    // Rewrites the opening world in place rather than appending a sequence: this is an edit
    // to the definition, not a step of a simulation. Later states are left alone — an
    // unfrozen grid has no plans, so there is no run whose history this could contradict.
    let existing_state = find_initial_state(&state.db, id).await?;

    let txn = state.db.begin().await?;

    let mut grid: grid_worlds::ActiveModel = grid.into();
    grid.name = Set(payload.name);
    grid.width = Set(payload.width);
    grid.height = Set(payload.height);
    let saved = grid.update(&txn).await?;

    let mut state_row: grid_world_states::ActiveModel = existing_state.into();
    state_row.obs_polygons = Set(polygons_to_json(&payload.obs_polygons));
    let saved_state = state_row.update(&txn).await?;

    txn.commit().await?;
    Ok(Json(GridDetail::new(saved, saved_state.obs_polygons)))
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
