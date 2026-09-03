use crate::entities::{grid_world_states, robots, route_plans};
use crate::handlers::helpers::validate_polygons;
use crate::handlers::helpers::{AppError, find_grid, record_state};
use crate::models::cell::Cell;
use crate::models::grid_world_manager::GridWorldManager;
use crate::models::obstacle::{ObstaclePoly, advance_one_tick};
use crate::models::planners::{PlanError, PlannerKind};
use crate::models::rng::Xorshift;
use crate::models::simulation::plan_route;
use crate::router::AppState;
use axum::Json;
use axum::extract::{Path, State};
use axum::http::StatusCode;
use sea_orm::{
    ActiveModelTrait, ColumnTrait, EntityTrait, JoinType, QueryFilter, QuerySelect, RelationTrait,
    Set, TransactionTrait,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize)]
pub(crate) struct PlanInput {
    src_vertex: [i32; 2],
    dest_vertex: [i32; 2],
    obs_polygons: Vec<ObstaclePoly>,
    /// Who will drive this route.
    ///
    /// Required: a route is planned *for* a machine, and a run takes its speed and cadence
    /// from that machine. Accepting a driverless plan would only defer the problem to the
    /// moment someone pressed Run.
    robot_id: i32,
}

/// One stored route, as a client sees it.
///
/// Carries `grid_id` even though the row does not: a route plan names the world it was
/// planned in, and the grid is a join behind that. The client asked about a grid and should
/// get an answer in those terms rather than having to resolve a state id of its own.
#[derive(Debug, Serialize)]
pub(crate) struct RoutePlanOutput {
    id: i32,
    grid_id: i32,
    /// Which world this route was planned in — the row it actually points at.
    grid_world_state_id: i32,
    /// Who drives this route.
    robot_id: i32,
    name: String,
    src_vertex: serde_json::Value,
    dest_vertex: serde_json::Value,
    /// The cells the route runs through, start first — empty when the goal is unreachable.
    route_vertices: serde_json::Value,
    meta: serde_json::Value,
}

impl RoutePlanOutput {
    fn new(plan: route_plans::Model, grid_id: i32) -> Self {
        Self {
            id: plan.id,
            grid_id,
            grid_world_state_id: plan.grid_world_state_id,
            robot_id: plan.robot_id,
            name: plan.name,
            src_vertex: plan.src_vertex,
            dest_vertex: plan.dest_vertex,
            route_vertices: plan.route_vertices,
            meta: plan.meta,
        }
    }
}

// GET /grids/{id}/plans — the route plans computed against this grid.
//
// Client must determine that a grid is frozen: a non-empty list means edits have to
// fork a new version. Deriving it from the plans themselves keeps the client's idea of
// "frozen" and the server's the same thing rather than two flags to keep in step.
//
// Still addressed by grid even though a route plan names a *world*: the grid is one join
// away, and "the routes across this grid" is the question a client actually has. A
// state-shaped route is the eventual home for this.
pub(crate) async fn list_grid_plans(
    State(state): State<AppState>,
    Path(id): Path<i32>,
) -> Result<Json<Vec<RoutePlanOutput>>, AppError> {
    // An unknown grid is a 404 rather than an empty list, so a stale id can't read as
    // "this grid is editable".
    find_grid(&state.db, id).await?;

    let found = route_plans::Entity::find()
        .join(
            JoinType::InnerJoin,
            route_plans::Relation::GridWorldStates.def(),
        )
        .filter(grid_world_states::Column::GridWorldId.eq(id))
        .all(&state.db)
        .await?;
    Ok(Json(
        found.into_iter().map(|plan| RoutePlanOutput::new(plan, id)).collect(),
    ))
}

pub(crate) async fn generate_grid_plan(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<PlanInput>,
) -> Result<(StatusCode, Json<RoutePlanOutput>), AppError> {
    let grid = find_grid(&state.db, id).await?;
    let mut grid_world = GridWorldManager::<Cell>::new(grid.width as usize, grid.height as usize);

    // Decode the stored obstacles. The DB column is `jsonb`, so sea-orm hands it over as an
    // untyped `Value` and the shape is re-established here — see
    // [`ObstaclePoly`](crate::models::obstacle::ObstaclePoly) for what it is.
    //
    // Decoded whole rather than per element, so one malformed obstacle rejects the request
    // instead of leaving a grid rasterized from the shapes that happened to parse.
    let obstacles: Vec<ObstaclePoly> = payload.obs_polygons;
    validate_polygons(&obstacles, grid.width, grid.height)?;

    // Checked rather than left to the foreign key, which reports an unknown robot as a
    // driver-level error and would surface as a 500 instead of naming the bad id.
    if robots::Entity::find_by_id(payload.robot_id)
        .one(&state.db)
        .await?
        .is_none()
    {
        return Err(AppError::NotFound(format!(
            "robot {} not found",
            payload.robot_id
        )));
    }

    // Only the geometry reaches the rasterizer: `id` and `dynamic` say nothing about which
    // cells a shape covers right now.
    let polygons: Vec<Vec<[i32; 2]>> = obstacles.iter().map(ObstaclePoly::cells).collect();
    grid_world.rasterize_polygons(&polygons, |cell| cell.blocked = true);

    // One value picks both the search and the name recorded below. Once `PlanInput` carries a
    // planner this is the field, and nothing else in here changes.
    let kind = PlannerKind::AStar;
    let mut planner = kind.planner();

    let optimal_path =
        match grid_world.find_plan(payload.src_vertex, payload.dest_vertex, planner.as_mut()) {
            Ok(path) => path,
            Err(PlanError::Unreachable) => Vec::new(),
            Err(err) => return Err(AppError::Invalid(err.to_string())),
        };
    let route_vertices = optimal_path
        .iter()
        .map(|cell| grid_world.xy(*cell))
        .collect::<Vec<_>>();

    // What is left once the endpoints have columns of their own: how the route was found,
    // and what it cost. `reachable` is derived rather than sent, so it cannot disagree with
    // the route beside it.
    let meta = serde_json::json!({
        "planner": kind.name(),
        "reachable": !route_vertices.is_empty(),
        "cost": grid_world.path_cost(&optimal_path),
    });

    // The world is written first and the route points at it, rather than the route carrying
    // a copy. One row is then the single account of "the obstacles at this moment", shared by
    // every route planned in it, and the two can never drift apart.
    //
    // Appended rather than overwriting the opening state, because the obstacles are the
    // caller's — mid-run they are not the grid's initial condition, and rewriting sequence 0
    // would rewrite history the earlier plans still refer to. Appended only when the world
    // actually moved: planning twice against unchanged geometry points both routes at the
    // same row. See [`record_state`].
    let txn = state.db.begin().await?;

    let world = record_state(&txn, id, &obstacles).await?;

    let new_plan = route_plans::ActiveModel {
        grid_world_state_id: Set(world.id),
        robot_id: Set(payload.robot_id),
        name: Set(String::from("Prototype")),
        src_vertex: Set(serde_json::json!(payload.src_vertex)),
        dest_vertex: Set(serde_json::json!(payload.dest_vertex)),
        route_vertices: Set(serde_json::to_value(route_vertices)
            .map_err(|e| AppError::Invalid(format!("failed to serialize plan vertices: {e}")))?),
        meta: Set(meta),
        ..Default::default() // leaves `id` unset so the DB generates it
    };

    let saved = new_plan.insert(&txn).await?;
    txn.commit().await?;

    Ok((StatusCode::CREATED, Json(RoutePlanOutput::new(saved, id))))
}

// Request body for `POST /grids/{id}/replan` — one tick of a simulation.
//
// The obstacles come from the *client*, not the stored grid row, because they are the evolving
// state: tick 5 has to perturb the shapes as tick 4 left them. The stored row is the initial
// condition and stays untouched, which is what keeps this from colliding with the freeze rule —
// a run of a hundred ticks would otherwise be a hundred grid versions.
#[derive(Debug, Deserialize)]
pub(crate) struct ReplanInput {
    src_vertex: [i32; 2],
    dest_vertex: [i32; 2],
    /// Where the obstacles are *now*, as the previous response left them.
    obs_polygons: Vec<ObstaclePoly>,
    /// Omit on the first tick; subsequently pass back the `next_seed` from the last response.
    ///
    /// Chaining the seed rather than sending a tick number is what makes a whole run replayable
    /// from its first value: consecutive small integers are not independent xorshift seeds, so
    /// `seed = tick` would produce a correlated, uninteresting walk.
    ///
    /// `u32` and not `u64` because the client is JavaScript, where every JSON number is a
    /// double: a `u64` above 2^53 would come back with its low bits rounded away, and the chain
    /// would break silently — the run would still look fine and simply not replay.
    seed: Option<u32>,
}

#[derive(Debug, Serialize)]
pub(crate) struct ReplanOutput {
    /// The obstacles *after* the tick — what the caller should draw, and send back next time.
    obs_polygons: Vec<ObstaclePoly>,
    /// The recomputed route as `[x, y]` cells, empty when the goal is walled off.
    vertices: Vec<(usize, usize)>,
    reachable: bool,
    cost: u32,
    /// How many obstacles actually moved. Zero means the tick was a no-op — every obstacle is
    /// static, or every draw clamped against an edge — so the route is unchanged by construction.
    moved: usize,
    /// Pass as `seed` on the next tick to continue this run. See [`ReplanInput::seed`] for why
    /// this is 32 bits wide.
    next_seed: u32,
    planner: &'static str,
}

// POST /grids/{id}/replan — advance the simulation one tick and replan.
//
// Deliberately does *not* store a plan. A stored plan is what freezes a grid, and a per-tick row
// would both freeze the row this run is exploring and bury the user's real saved routes under
// simulation debris. Ticks are ephemeral; `POST /grids/{id}/plans` is still how a plan is kept.
pub(crate) async fn replan_grid(
    State(state): State<AppState>,
    Path(id): Path<i32>,
    Json(payload): Json<ReplanInput>,
) -> Result<Json<ReplanOutput>, AppError> {
    // The grid supplies the dimensions the jitter clamps against and the plan runs on. Obstacle
    // geometry is the caller's; everything else is still the stored snapshot's.
    let grid = find_grid(&state.db, id).await?;

    // Revalidate the obstacles against the grid's dimensions.
    let mut obstacles = payload.obs_polygons;
    validate_polygons(&obstacles, grid.width, grid.height)?;

    let mut rng = Xorshift::new(payload.seed.map_or_else(seed_from_clock, u64::from));
    let moved = advance_one_tick(&mut obstacles, &mut rng, grid.width, grid.height);

    // Replan against the new geometry. The route is computed from the obstacles the caller
    // just sent, not the stored row, so a run can explore a moving world without freezing the
    // grid or leaving a plan row per tick.
    let route = plan_route(
        grid.width,
        grid.height,
        &obstacles,
        payload.src_vertex,
        payload.dest_vertex,
        PlannerKind::DStarLite,
    )
        .map_err(|err| AppError::Invalid(err.to_string()))?;

    Ok(Json(ReplanOutput {
        obs_polygons: obstacles,
        vertices: route.vertices,
        reachable: route.reachable,
        cost: route.cost,
        moved,
        // The high half: xorshift's low bits are the weaker ones, so truncating from the top
        // gives a better next seed than masking off the bottom would.
        next_seed: (rng.next_u64() >> 32) as u32,
        planner: route.planner,
    }))
}

/// A starting seed for a caller that did not bring one.
///
/// Only ever used for the first tick of a run: from then on the seed is chained through the
/// response, so the run stays replayable even though it began somewhere arbitrary.
fn seed_from_clock() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x2026_0811)
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
    let res = route_plans::Entity::delete_by_id(plan_id)
        .exec(&state.db)
        .await?;
    if res.rows_affected == 0 {
        return Err(AppError::NotFound(format!("plan {plan_id} not found")));
    }

    Ok(StatusCode::NO_CONTENT)
}
