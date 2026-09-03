use crate::entities::{grid_world_states, grid_worlds, route_plans};
use crate::models::obstacle::{CellVertex, ObstaclePoly};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use sea_orm::{
    ActiveModelTrait, ColumnTrait, ConnectionTrait, DatabaseConnection, DbErr, EntityTrait,
    JoinType, PaginatorTrait, QueryFilter, QueryOrder, QuerySelect, RelationTrait, Set,
};

/// The sequence number of a grid's opening world, before any simulation has advanced it.
///
/// Later rows in `grid_world_states` are where a run has got to; this one is the definition,
/// and it is what grid CRUD reads and rewrites.
pub(crate) const INITIAL_SEQUENCE: i32 = 0;

// Every nested route needs the parent grid, and a bad id is always a 404.
pub(crate) async fn find_grid(
    db: &DatabaseConnection,
    id: i32,
) -> Result<grid_worlds::Model, AppError> {
    grid_worlds::Entity::find_by_id(id)
        .one(db)
        .await?
        .ok_or_else(|| AppError::NotFound(format!("grid {id} not found")))
}

/// The world a grid starts from: its `sequence_id = 0` state.
///
/// Every grid is created with one, so a missing row is a broken invariant rather than a grid
/// that merely has no obstacles yet — "nothing in the way" is an empty list in a row that
/// exists. Hence a 500 and not a default: silently substituting an empty world is exactly the
/// failure that plans a route straight through a wall and records it as fact.
pub(crate) async fn find_initial_state(
    db: &DatabaseConnection,
    grid_id: i32,
) -> Result<grid_world_states::Model, AppError> {
    grid_world_states::Entity::find()
        .filter(grid_world_states::Column::GridWorldId.eq(grid_id))
        .filter(grid_world_states::Column::SequenceId.eq(INITIAL_SEQUENCE))
        .one(db)
        .await?
        .ok_or_else(|| {
            AppError::Db(DbErr::Custom(format!(
                "grid {grid_id} has no sequence {INITIAL_SEQUENCE} state — \
                 it was created without an initial world"
            )))
        })
}

/// Writes one world for a grid at the sequence number given. Takes any connection so it can
/// share a caller's transaction — a grid and its opening state have to arrive together or not
/// at all, and so do a route plan and the world it was planned in.
pub(crate) async fn insert_state<C: ConnectionTrait>(
    db: &C,
    grid_id: i32,
    sequence_id: i32,
    polygons: &[ObstaclePoly],
) -> Result<grid_world_states::Model, AppError> {
    let state = grid_world_states::ActiveModel {
        grid_world_id: Set(grid_id),
        obs_polygons: Set(polygons_to_json(polygons)),
        sequence_id: Set(sequence_id),
        // `timestamp` left unset so the column default stamps it, keeping the clock the
        // database's rather than this process's.
        ..Default::default()
    };
    Ok(state.insert(db).await?)
}

/// The world a grid starts from, written at creation. See [`insert_state`].
pub(crate) async fn insert_initial_state<C: ConnectionTrait>(
    db: &C,
    grid_id: i32,
    polygons: &[ObstaclePoly],
) -> Result<grid_world_states::Model, AppError> {
    insert_state(db, grid_id, INITIAL_SEQUENCE, polygons).await
}

/// The newest world recorded for this grid, or `None` for a grid that has none yet.
pub(crate) async fn latest_state<C: ConnectionTrait>(
    db: &C,
    grid_id: i32,
) -> Result<Option<grid_world_states::Model>, AppError> {
    Ok(grid_world_states::Entity::find()
        .filter(grid_world_states::Column::GridWorldId.eq(grid_id))
        .order_by_desc(grid_world_states::Column::SequenceId)
        .one(db)
        .await?)
}

/// Records `polygons` as this grid's newest world, and answers with the row to point at.
///
/// Appends only when the world has actually moved. A state row is *the world at a moment*, so
/// geometry identical to the newest row is not a new moment — it is the same one, and every
/// route planned in it should reference the one row rather than a pile of indistinguishable
/// copies. Planning three times on an unedited grid leaves one state, not three.
///
/// Compared against the newest row only, never an older match. If a dynamic obstacle wanders
/// away and happens to wander back, that is a *later* moment that resembles an earlier one —
/// attaching the new plan to the old row would put it before states that already preceded it
/// and make the sequence lie about the order things happened in.
///
/// Equality is over the encoded JSON, which is exact because both sides come from
/// [`polygons_to_json`] — the same struct always serializes the same way. It follows that the
/// comparison is order-sensitive: the same obstacles sent in a different order append a new
/// row. That is the honest reading, since nothing here can tell a reordered payload from a
/// deliberate one, and both `advance_one_tick` and the client preserve list order anyway.
pub(crate) async fn record_state<C: ConnectionTrait>(
    db: &C,
    grid_id: i32,
    polygons: &[ObstaclePoly],
) -> Result<grid_world_states::Model, AppError> {
    let encoded = polygons_to_json(polygons);

    // Racing callers can both read the same newest row and both append; the index on
    // `(grid_world_id, sequence_id)` is not unique, so that costs a duplicate step rather
    // than an error — worth revisiting when states start arriving concurrently.
    let sequence_id = match latest_state(db, grid_id).await? {
        Some(latest) if latest.obs_polygons == encoded => return Ok(latest),
        Some(latest) => latest.sequence_id + 1,
        None => INITIAL_SEQUENCE,
    };

    insert_state(db, grid_id, sequence_id, polygons).await
}

/// How many stored route plans were computed against this grid.
///
/// This is the freeze rule: a grid with plans is immutable, because each of those routes was
/// planned in a world derived from this one, and editing the row in place would silently
/// invalidate them. Deleting the plans unfreezes the grid — which is why the count is derived
/// on demand rather than kept as a column that could drift out of step.
///
/// Counted through `grid_world_states` because a route plan names the *world* it was planned
/// in, not the grid: the grid is one join away, and that indirection is what lets a route be
/// planned mid-simulation without inventing a grid version for the tick it happened on.
pub(crate) async fn plan_count(db: &DatabaseConnection, grid_id: i32) -> Result<u64, AppError> {
    Ok(route_plans::Entity::find()
        .join(JoinType::InnerJoin, route_plans::Relation::GridWorldStates.def())
        .filter(grid_world_states::Column::GridWorldId.eq(grid_id))
        .count(db)
        .await?)
}

/// Refuses to touch a grid that plans depend on, naming the way forward.
///
/// The message matters: the same `PUT` succeeds before a plan exists and fails after,
/// so a caller that hits this needs to be told what to do instead rather than left to
/// guess why an edit that worked a minute ago stopped working.
pub(crate) async fn ensure_unfrozen(
    db: &DatabaseConnection,
    grid_id: i32,
    action: &str,
    remedy: &str,
) -> Result<(), AppError> {
    let plans = plan_count(db, grid_id).await?;
    if plans > 0 {
        return Err(AppError::Conflict(format!(
            "grid {grid_id} has {plans} plan(s) and cannot be {action}; {remedy}"
        )));
    }
    Ok(())
}

// --- errors --------------------------------------------------------------

// Minimal error type so `?` works in handlers and failures become responses
// instead of panicking the whole request.
pub(crate) enum AppError {
    Db(sea_orm::DbErr),
    NotFound(String),
    Invalid(String),
    Conflict(String),
}

impl From<sea_orm::DbErr> for AppError {
    fn from(err: sea_orm::DbErr) -> Self {
        AppError::Db(err)
    }
}

// Postgres reports a breach of `idx-grid_worlds-name-version` as a plain query error;
// sea-orm passes the driver's message straight through. Matching on it is what turns
// the race two clients can lose — both forking the same snapshot to the same version —
// into a 409 rather than an opaque 500.
fn is_unique_violation(err: &sea_orm::DbErr) -> bool {
    err.to_string().contains("duplicate key value")
}

impl IntoResponse for AppError {
    fn into_response(self) -> Response {
        match self {
            AppError::NotFound(msg) => (StatusCode::NOT_FOUND, msg).into_response(),
            AppError::Invalid(msg) => (StatusCode::BAD_REQUEST, msg).into_response(),
            AppError::Conflict(msg) => (StatusCode::CONFLICT, msg).into_response(),
            // sea-orm reports an UPDATE/DELETE that matched no row this way —
            // still the caller naming an id that isn't there, so still a 404.
            AppError::Db(sea_orm::DbErr::RecordNotFound(msg)) => {
                (StatusCode::NOT_FOUND, msg).into_response()
            }
            AppError::Db(err) if is_unique_violation(&err) => (
                StatusCode::CONFLICT,
                "that grid name and version already exists — \
                 someone else saved this version first"
                    .to_string(),
            )
                .into_response(),
            AppError::Db(err) => {
                tracing::error!("database error: {:?}", err);
                (
                    StatusCode::INTERNAL_SERVER_ERROR,
                    format!("database error: {err}"),
                )
                    .into_response()
            }
        }
    }
}

// `to_value` rather than a hand-built `Value`: for a struct of `i32`, `bool` and `Vec`,
// serialization cannot fail — the error cases are non-string map keys and non-finite floats,
// neither of which this type can produce.
pub(crate) fn polygons_to_json(polygons: &[ObstaclePoly]) -> serde_json::Value {
    serde_json::to_value(polygons).expect("obstacle geometry is always serializable")
}

// The first vertex outside a `width` x `height` grid, if there is one. Vertices are
// cell indices, so the far edge is out of range: a 10-wide grid addresses columns 0..=9.
pub(crate) fn out_of_bounds(vertices: &[CellVertex], width: i32, height: i32) -> Option<CellVertex> {
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
