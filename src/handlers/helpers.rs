use crate::entities::{grid_worlds, plans};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use sea_orm::{ColumnTrait, DatabaseConnection, EntityTrait, PaginatorTrait, QueryFilter};

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

/// How many stored plans were computed against this grid.
///
/// This is the freeze rule: a grid with plans is immutable, because each of those
/// routes was planned against this exact geometry and editing the row in place would
/// silently invalidate them. Deleting the plans unfreezes the grid — which is why the
/// count is derived on demand rather than kept as a column that could drift out of
/// step with the plans table.
pub(crate) async fn plan_count(db: &DatabaseConnection, grid_id: i32) -> Result<u64, AppError> {
    Ok(plans::Entity::find()
        .filter(plans::Column::GridId.eq(grid_id))
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