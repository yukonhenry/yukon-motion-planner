//! Robot CRUD.
//!
//! A robot is a *spec* — a name and a bag of capabilities — and carries nothing about
//! where it is. That is why these routes sit at the top level rather than under a grid:
//! a grid row is a frozen snapshot that forks a new id on every edit (see
//! [`grid_crud`](crate::handlers::grid_crud)), so `POST /grids/{id}/robots` would tie a
//! robot to one snapshot and empty the fleet as soon as an obstacle moved.
//!
//! Where a robot *is* belongs to the scenario, and is recorded by
//! [`plan_robot_crud`](crate::handlers::plan_robot_crud).

use crate::entities::robots;
use crate::handlers::helpers::AppError;
use crate::models::robot::RobotSpec;
use crate::router::AppState;
use axum::Json;
use axum::extract::{Path, State};
use axum::http::StatusCode;
use sea_orm::{ActiveModelTrait, EntityTrait, Set};
use serde::Deserialize;

/// Body of `POST /robots` and `PUT /robots/{id}`.
#[derive(Debug, Deserialize)]
pub(crate) struct RobotInput {
    name: String,
    /// Free-form while the shape of a capability is still being settled — footprint, speed
    /// and kinematics all live in here before any of them earns a column of its own.
    capabilities: serde_json::Value,
}

// An object and not a bare number or list, so that adding a second capability later is a new
// key rather than a migration of every stored row into a different shape.
//
// The two keys a run needs are checked through [`RobotSpec`] rather than re-listed here, so
// "what makes a robot runnable" has exactly one definition. Validating at the form is the
// point: the alternative is a robot that stores fine and fails at the moment someone presses
// Run, which is what this check exists to prevent.
fn validate_robot(input: &RobotInput) -> Result<(), AppError> {
    if input.name.trim().is_empty() {
        return Err(AppError::Invalid("robot name cannot be empty".into()));
    }
    if !input.capabilities.is_object() {
        return Err(AppError::Invalid(
            "capabilities must be a JSON object".into(),
        ));
    }

    RobotSpec::from_capabilities(&input.capabilities)
        .map(|_| ())
        .map_err(|err| AppError::Invalid(err.to_string()))
}

// GET /robots — the whole fleet.
//
// Unfiltered because a robot is not scoped to anything: the fleet is the same list whatever
// grid or plan the caller is looking at.
pub(crate) async fn list_robots(
    State(state): State<AppState>,
) -> Result<Json<Vec<robots::Model>>, AppError> {
    Ok(Json(robots::Entity::find().all(&state.db).await?))
}

// POST /robots — add a robot to the fleet.
pub(crate) async fn create_robot(
    State(state): State<AppState>,
    Json(payload): Json<RobotInput>,
) -> Result<(StatusCode, Json<robots::Model>), AppError> {
    validate_robot(&payload)?;

    let new_robot = robots::ActiveModel {
        name: Set(payload.name),
        capabilities: Set(payload.capabilities),
        ..Default::default() // leaves `id` unset so the DB generates it
    };

    let saved = new_robot.insert(&state.db).await?;
    Ok((StatusCode::CREATED, Json(saved)))
}

// GET /robots/{id}
pub(crate) async fn show_robot(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<Json<robots::Model>, AppError> {
    Ok(Json(find_robot(&state.db, id).await?))
}

// PUT /robots/{id} — replace a robot's spec.
//
// Editing a robot may invalidate a plan, including the plan's route vertices.
// todo: determine whether to enforce a freeze rule here like grids have, or to let the caller
// handle the consequences of changing a robot that a plan depends on.
pub(crate) async fn update_robot(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<RobotInput>,
) -> Result<Json<robots::Model>, AppError> {
    validate_robot(&payload)?;

    let mut robot: robots::ActiveModel = find_robot(&state.db, id).await?.into();
    robot.name = Set(payload.name);
    robot.capabilities = Set(payload.capabilities);

    Ok(Json(robot.update(&state.db).await?))
}

// DELETE /robots/{id} — retire a robot.
//
// Its route plans go with it, by the cascade `route_plans` declares: a route with no robot
// to drive it is a row pointing at nothing. So this can quietly shrink a scenario, which is
// the intended trade — the alternative is a robot that cannot be retired until every route
// ever planned for it is deleted.
pub(crate) async fn delete_robot(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<StatusCode, AppError> {
    // `rows_affected` doubles as the existence check, so this stays one round trip.
    let res = robots::Entity::delete_by_id(id).exec(&state.db).await?;
    if res.rows_affected == 0 {
        return Err(AppError::NotFound(format!("robot {id} not found")));
    }

    Ok(StatusCode::NO_CONTENT)
}

async fn find_robot(db: &sea_orm::DatabaseConnection, id: i32) -> Result<robots::Model, AppError> {
    robots::Entity::find_by_id(id)
        .one(db)
        .await?
        .ok_or_else(|| AppError::NotFound(format!("robot {id} not found")))
}
