//! Integration tests for the routes in `src/router.rs`.
//!
//! These talk to a real Postgres — the one `docker compose up -d` starts — but in a
//! dedicated `yukon_motion_planner_test` database, so they never touch dev data. The
//! database is created and migrated once per test binary.
//!
//! Tests only assert about rows they created themselves, which keeps them correct
//! under `cargo test`'s default parallelism and across repeat runs. `(name, version)`
//! is unique, so a test that renames a grid to a fixed string would pass once and then
//! collide with its own leftovers — renames use [`unique_name`], and edits that are not
//! about the name resend the grid's existing one.
//!
//! Obstacles have no routes of their own: they are a `obs_polygons` field on the grid,
//! written by `POST /grids` and replaced wholesale by `PUT /grids/{id}`.
//!
//! A grid freezes once a plan is computed against it: `PUT` and `DELETE` then 409, and
//! edits go through `POST /grids/{id}/versions`. Deleting the plans unfreezes it.

use std::env;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use migration::{Migrator, MigratorTrait};
use reqwest::Client;
use sea_orm::{ConnectionTrait, DatabaseConnection, EntityTrait};
use serde_json::{Value, json};
use yukon_motion_planner::db::{self, connect, options};
use yukon_motion_planner::entities::plans;

const TEST_DB: &str = "yukon_motion_planner_test";

/// `(maintenance url, test database url)` derived from `DATABASE_URL`.
///
/// The database name is swapped, not appended — and the whole authority is rebuilt to do
/// it, because `rsplit_once('/')` finds the `//` of the scheme on a URL with no path at
/// all and would quietly produce `postgres://yukon_motion_planner_test`.
fn database_urls() -> Result<(String, String), String> {
    dotenvy::dotenv().ok();
    let base = env::var("DATABASE_URL")
        .unwrap_or_else(|_| "postgres://postgres:postgres@localhost:5432/postgres".to_string());

    let malformed = || {
        format!(
            "DATABASE_URL must name a database, as in \
             postgres://postgres:postgres@localhost:5432/postgres — got `{}`.",
            db::endpoint(&base)
        )
    };
    let (scheme, rest) = base.split_once("://").ok_or_else(malformed)?;
    let (authority, path) = rest.split_once('/').ok_or_else(malformed)?;
    let name = db::database_name(&base).ok_or_else(malformed)?;
    // Anything trailing the name — `?sslmode=require` and the like — belongs to the
    // connection, not to the database, so it survives the swap.
    let params = &path[name.len()..];

    Ok((base.clone(), format!("{scheme}://{authority}/{TEST_DB}{params}")))
}

/// A pool for the test database, sized so the parallel tests together stay well under
/// Postgres's connection limit.
///
/// Pools are bound to the runtime that created them, so callers must not share one
/// between tests.
async fn pool(url: &str) -> Result<DatabaseConnection, String> {
    let mut opts = options(url);
    opts.max_connections(2).min_connections(0);
    connect(opts).await
}

/// Creates and migrates the test database, once per test binary, and answers with its URL.
///
/// Every `#[tokio::test]` builds its own runtime, so this gets a throwaway runtime on
/// a thread of its own rather than borrowing whichever test happened to arrive first.
///
/// The *outcome* is what's cached, failure included: a `OnceLock` whose closure panics
/// stays empty and runs again for the next caller, which with the database down means
/// all fifty tests taking turns waiting for the same connection. Cached, the first test
/// reports why and the rest fail instantly with the same message.
fn ensure_database() -> String {
    static INIT: OnceLock<Result<String, String>> = OnceLock::new();

    let outcome = INIT.get_or_init(|| {
        std::thread::spawn(|| {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async {
                    let (admin_url, test_url) = database_urls()?;

                    // `CREATE DATABASE` has to be issued from some *other* database.
                    let admin = pool(&admin_url).await?;
                    // Fails harmlessly when the database already exists, i.e. every run but the first.
                    let _ = admin
                        .execute_unprepared(&format!("CREATE DATABASE {TEST_DB}"))
                        .await;
                    let _ = admin.close().await;

                    let db = pool(&test_url).await?;
                    Migrator::up(&db, None)
                        .await
                        .map_err(|err| format!("Could not migrate {TEST_DB}: {err}"))?;
                    let _ = db.close().await;
                    Ok(test_url)
                })
        })
        .join()
        .unwrap_or_else(|_| Err("test database setup panicked".to_string()))
    });

    match outcome {
        Ok(test_url) => test_url.clone(),
        Err(message) => panic!("{message}"),
    }
}

/// A connection pool belonging to *this* test's runtime.
async fn db() -> DatabaseConnection {
    let test_url = ensure_database();
    pool(&test_url)
        .await
        .unwrap_or_else(|message| panic!("{message}"))
}

/// Serves the real router on an ephemeral port; returns its base URL.
async fn spawn_app() -> String {
    let app = yukon_motion_planner::router::route(db().await).await;
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, app).await.unwrap() });
    format!("http://{addr}")
}

/// Grid names double as a marker for "this test's row", so they must not collide.
fn unique_name() -> String {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("grid-{nanos}-{n}")
}

/// The body both `POST /grids` and `PUT /grids/{id}` take — a whole grid, obstacles
/// included. `obs_polygons` is a list of polygons, each a list of `[x, y]` cells.
fn grid_body(name: &str, width: i32, height: i32, obs_polygons: Value) -> Value {
    json!({ "name": name, "width": width, "height": height, "obs_polygons": obs_polygons })
}

async fn post_grid(client: &Client, base: &str, body: Value) -> reqwest::Response {
    client
        .post(format!("{base}/grids"))
        .json(&body)
        .send()
        .await
        .unwrap()
}

async fn put_grid(client: &Client, base: &str, grid_id: i64, body: Value) -> reqwest::Response {
    client
        .put(format!("{base}/grids/{grid_id}"))
        .json(&body)
        .send()
        .await
        .unwrap()
}

async fn post_grid_version(
    client: &Client,
    base: &str,
    grid_id: i64,
    body: Value,
) -> reqwest::Response {
    client
        .post(format!("{base}/grids/{grid_id}/versions"))
        .json(&body)
        .send()
        .await
        .unwrap()
}

/// The body for editing a grid without renaming it — the ordinary case, and the one
/// that keeps `(name, version)` unique when a test runs twice.
fn edited(grid: &Value, width: i32, height: i32, obs_polygons: Value) -> Value {
    grid_body(grid["name"].as_str().unwrap(), width, height, obs_polygons)
}

/// Freezes `grid_id` by planning a route across it, and returns the plan's id.
///
/// Routes the top-left corner, so grids frozen this way must leave `(0, 0)`–`(1, 1)`
/// clear — planning from inside an obstacle is a 400, not a plan.
async fn freeze_with_plan(client: &Client, base: &str, grid_id: i64) -> i64 {
    let res = post_plan(client, base, grid_id, [0, 0], [1, 1]).await;
    assert_eq!(res.status(), 201, "plan setup should succeed");
    let plan: Value = res.json().await.unwrap();
    plan["id"].as_i64().unwrap()
}

async fn list_plans(client: &Client, base: &str, grid_id: i64) -> Vec<Value> {
    let res = client
        .get(format!("{base}/grids/{grid_id}/plans"))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), 200);
    res.json().await.unwrap()
}

async fn delete_plan(client: &Client, base: &str, plan_id: i64) -> reqwest::Response {
    client
        .delete(format!("{base}/plans/{plan_id}"))
        .send()
        .await
        .unwrap()
}

/// Creates a grid that is expected to succeed, and returns the saved row.
async fn create_grid(client: &Client, base: &str, width: i32, height: i32, obs: Value) -> Value {
    let res = post_grid(client, base, grid_body(&unique_name(), width, height, obs)).await;
    assert_eq!(res.status(), 201, "grid setup should succeed");
    res.json().await.unwrap()
}

/// The common setup: a grid with nothing in it.
async fn create_empty_grid(client: &Client, base: &str, width: i32, height: i32) -> Value {
    create_grid(client, base, width, height, json!([])).await
}

/// Reads a grid back through `GET /grids/{id}` — the only route that returns obstacles.
async fn show_grid(client: &Client, base: &str, grid_id: i64) -> Value {
    let res = client
        .get(format!("{base}/grids/{grid_id}"))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), 200, "grid {grid_id} should be readable");
    res.json().await.unwrap()
}

/// Consumes a response body and reports whether it mentions `needle`.
async fn res_contains(res: reqwest::Response, needle: &str) -> bool {
    let body = res.text().await.unwrap();
    if !body.contains(needle) {
        eprintln!("expected {needle:?} in response body: {body}");
        return false;
    }
    true
}

async fn list_grids(client: &Client, base: &str) -> Vec<Value> {
    let res = client.get(format!("{base}/grids")).send().await.unwrap();
    assert_eq!(res.status(), 200);
    res.json().await.unwrap()
}

// --- grids: create -------------------------------------------------------

#[tokio::test]
async fn create_grid_returns_201_and_echoes_the_grid() {
    let client = Client::new();
    let base = spawn_app().await;
    let name = unique_name();

    let res = post_grid(&client, &base, grid_body(&name, 10, 20, json!([]))).await;

    assert_eq!(res.status(), 201);
    let body: Value = res.json().await.unwrap();
    assert_eq!(body["name"], name.as_str());
    assert_eq!(body["width"], 10);
    assert_eq!(body["height"], 20);
    assert_eq!(body["obs_polygons"], json!([]));
    assert_eq!(body["version"], 0, "a fresh grid starts at version 0");
    // The DB assigns the id, so we only care that we got a real one back.
    assert!(body["id"].as_i64().unwrap() > 0);
}

#[tokio::test]
async fn create_grid_saves_the_obstacles_it_was_given() {
    let client = Client::new();
    let base = spawn_app().await;
    let polygons = json!([
        [[0, 0], [3, 0], [3, 2]],
        [[5, 5], [6, 5], [6, 6], [5, 6]],
    ]);

    let created = create_grid(&client, &base, 10, 10, polygons.clone()).await;

    assert_eq!(created["obs_polygons"], polygons, "the response echoes them");
    // And they are stored, not just echoed.
    let stored = show_grid(&client, &base, created["id"].as_i64().unwrap()).await;
    assert_eq!(stored["obs_polygons"], polygons);
}

#[tokio::test]
async fn an_obstacle_may_use_the_highest_valid_cell_index() {
    let client = Client::new();
    let base = spawn_app().await;

    // Vertices are cell indices, so 9 is in range on a 10-wide grid.
    let res = post_grid(
        &client,
        &base,
        grid_body(&unique_name(), 10, 10, json!([[[0, 0], [9, 0], [9, 9]]])),
    )
        .await;

    assert_eq!(res.status(), 201);
}

// --- grids: read ---------------------------------------------------------

#[tokio::test]
async fn list_grids_includes_a_newly_created_grid() {
    let client = Client::new();
    let base = spawn_app().await;
    let created = create_empty_grid(&client, &base, 5, 5).await;

    let body = list_grids(&client, &base).await;

    assert!(
        body.iter().any(|g| g["id"] == created["id"]),
        "grid {} missing from the listing",
        created["id"]
    );
}

#[tokio::test]
async fn the_listing_leaves_obstacles_out() {
    let client = Client::new();
    let base = spawn_app().await;
    let created =
        create_grid(&client, &base, 10, 10, json!([[[0, 0], [3, 0], [3, 2]]])).await;
    let grid_id = created["id"].as_i64().unwrap();

    let listed = list_grids(&client, &base).await;
    let listed = listed.iter().find(|g| g["id"] == grid_id).unwrap();

    // Deliberate: the listing is a partial model, so a page of grids stays small.
    // Obstacles come from `GET /grids/{id}`.
    assert_eq!(listed["name"], created["name"]);
    assert!(
        listed.get("obs_polygons").is_none(),
        "the listing should not carry obstacle geometry: {listed}"
    );
}

#[tokio::test]
async fn show_grid_returns_the_grid_and_its_obstacles() {
    let client = Client::new();
    let base = spawn_app().await;
    let polygons = json!([[[1, 1], [4, 1], [4, 3]]]);
    let created = create_grid(&client, &base, 8, 8, polygons.clone()).await;
    let grid_id = created["id"].as_i64().unwrap();

    let body = show_grid(&client, &base, grid_id).await;

    assert_eq!(body["id"], grid_id);
    assert_eq!(body["name"], created["name"]);
    assert_eq!(body["width"], 8);
    assert_eq!(body["height"], 8);
    assert_eq!(body["obs_polygons"], polygons);
}

#[tokio::test]
async fn showing_an_unknown_grid_is_404() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = client
        .get(format!("{base}/grids/999999"))
        .send()
        .await
        .unwrap();

    assert_eq!(res.status(), 404);
    assert!(res.text().await.unwrap().contains("grid 999999 not found"));
}

// --- grids: obstacle validation -----------------------------------------

#[tokio::test]
async fn an_obstacle_needs_at_least_three_vertices() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = post_grid(
        &client,
        &base,
        grid_body(&unique_name(), 10, 10, json!([[[0, 0], [1, 1]]])),
    )
        .await;

    assert_eq!(res.status(), 400);
    assert!(res.text().await.unwrap().contains("at least 3 vertices"));
}

#[tokio::test]
async fn vertices_past_the_grid_bounds_are_rejected() {
    let client = Client::new();
    let base = spawn_app().await;

    // 10 is one past the last cell index on a 10-wide grid.
    let res = post_grid(
        &client,
        &base,
        grid_body(&unique_name(), 10, 10, json!([[[0, 0], [10, 0], [0, 5]]])),
    )
        .await;

    assert_eq!(res.status(), 400);
    assert!(
        res.text()
            .await
            .unwrap()
            .contains("obstacle 0 has vertex [10, 0] outside")
    );
}

#[tokio::test]
async fn negative_vertices_are_rejected() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = post_grid(
        &client,
        &base,
        grid_body(&unique_name(), 10, 10, json!([[[0, 0], [-1, 0], [0, 5]]])),
    )
        .await;

    assert_eq!(res.status(), 400);
    assert!(
        res.text()
            .await
            .unwrap()
            .contains("obstacle 0 has vertex [-1, 0] outside")
    );
}

#[tokio::test]
async fn a_bad_obstacle_is_named_by_its_position_in_the_list() {
    let client = Client::new();
    let base = spawn_app().await;

    // The first polygon is fine; the second is not. Callers get told which.
    let res = post_grid(
        &client,
        &base,
        grid_body(
            &unique_name(),
            10,
            10,
            json!([[[0, 0], [3, 0], [3, 2]], [[0, 0], [1, 1]]]),
        ),
    )
        .await;

    assert_eq!(res.status(), 400);
    assert!(res.text().await.unwrap().contains("obstacle 1"));
}

#[tokio::test]
async fn a_rejected_obstacle_saves_no_grid_at_all() {
    let client = Client::new();
    let base = spawn_app().await;
    let name = unique_name();

    let res = post_grid(
        &client,
        &base,
        grid_body(&name, 10, 10, json!([[[0, 0], [1, 1]]])),
    )
        .await;
    assert_eq!(res.status(), 400);

    // Obstacles ride along with the grid, so a bad one has to take the whole insert
    // down — a saved grid missing its obstacles would plan straight through them.
    let listed = list_grids(&client, &base).await;
    assert!(
        !listed.iter().any(|g| g["name"] == name.as_str()),
        "the grid was saved despite its obstacle being rejected"
    );
}

#[tokio::test]
async fn a_malformed_vertex_pair_is_rejected_before_the_handler() {
    let client = Client::new();
    let base = spawn_app().await;

    // `Vec<Vec<[i32; 2]>>` makes serde reject this during extraction, hence 422 not 400.
    let res = post_grid(
        &client,
        &base,
        grid_body(&unique_name(), 10, 10, json!([[[0, 0], [3], [3, 2]]])),
    )
        .await;

    assert_eq!(res.status(), 422);
}

#[tokio::test]
async fn obs_polygons_is_a_required_field() {
    let client = Client::new();
    let base = spawn_app().await;

    // A grid with no obstacles is spelled `[]`. Omitting the field is a malformed
    // body rather than a shorthand, so it never reaches the handler.
    let res = post_grid(
        &client,
        &base,
        json!({ "name": unique_name(), "width": 10, "height": 10 }),
    )
        .await;

    assert_eq!(res.status(), 422);
}

// --- grids: update -------------------------------------------------------

#[tokio::test]
async fn update_grid_replaces_every_field() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_grid(&client, &base, 10, 10, json!([[[0, 0], [3, 0], [3, 2]]])).await;
    let grid_id = grid["id"].as_i64().unwrap();
    let replacement = json!([[[1, 1], [4, 1], [4, 3], [1, 3]]]);
    let renamed = unique_name();

    let res = put_grid(
        &client,
        &base,
        grid_id,
        grid_body(&renamed, 20, 30, replacement.clone()),
    )
        .await;

    assert_eq!(res.status(), 200);
    let body: Value = res.json().await.unwrap();
    assert_eq!(body["id"], grid_id, "an unfrozen edit stays on the same row");
    assert_eq!(body["version"], 0, "and does not spend a version");
    assert_eq!(body["name"], renamed.as_str());
    assert_eq!(body["width"], 20);
    assert_eq!(body["height"], 30);
    assert_eq!(body["obs_polygons"], replacement);

    // And the change is durable, not just echoed back.
    let stored = show_grid(&client, &base, grid_id).await;
    assert_eq!(stored["name"], renamed.as_str());
    assert_eq!(stored["width"], 20);
    assert_eq!(stored["obs_polygons"], replacement);
}

#[tokio::test]
async fn update_grid_can_clear_the_obstacles() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_grid(&client, &base, 10, 10, json!([[[0, 0], [3, 0], [3, 2]]])).await;
    let grid_id = grid["id"].as_i64().unwrap();

    // PUT replaces rather than merges, so an empty list means "no obstacles" — not
    // "leave them alone".
    let res = put_grid(&client, &base, grid_id, edited(&grid, 10, 10, json!([]))).await;

    assert_eq!(res.status(), 200);
    assert_eq!(show_grid(&client, &base, grid_id).await["obs_polygons"], json!([]));
}

#[tokio::test]
async fn updating_an_unknown_grid_is_404() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = put_grid(&client, &base, 999999, grid_body("nope", 5, 5, json!([]))).await;

    assert_eq!(res.status(), 404);
    assert!(res.text().await.unwrap().contains("grid 999999 not found"));
}

#[tokio::test]
async fn a_grid_may_grow_while_holding_obstacles() {
    let client = Client::new();
    let base = spawn_app().await;
    let obstacles = json!([[[8, 8], [9, 8], [9, 9]]]);
    let grid = create_grid(&client, &base, 10, 10, obstacles.clone()).await;
    let grid_id = grid["id"].as_i64().unwrap();

    let res = put_grid(&client, &base, grid_id, edited(&grid, 50, 50, obstacles)).await;

    assert_eq!(res.status(), 200);
}

#[tokio::test]
async fn a_grid_cannot_shrink_below_its_obstacles() {
    let client = Client::new();
    let base = spawn_app().await;
    let obstacles = json!([[[8, 8], [9, 8], [9, 9]]]);
    let grid = create_grid(&client, &base, 10, 10, obstacles.clone()).await;
    let grid_id = grid["id"].as_i64().unwrap();

    // Shrinking to 5x5 while keeping the obstacles would strand them out of bounds.
    let res = put_grid(&client, &base, grid_id, edited(&grid, 5, 5, obstacles)).await;

    assert_eq!(res.status(), 400);
    assert!(res.text().await.unwrap().contains("obstacle 0 has vertex [8, 8]"));

    // The grid is untouched, not partially applied.
    let stored = show_grid(&client, &base, grid_id).await;
    assert_eq!(stored["width"], 10);
    assert_eq!(stored["name"], grid["name"]);
}

#[tokio::test]
async fn a_grid_may_shrink_once_the_obstacles_go_with_it() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_grid(&client, &base, 10, 10, json!([[[8, 8], [9, 8], [9, 9]]])).await;
    let grid_id = grid["id"].as_i64().unwrap();

    // The same shrink, with obstacles that fit the new bounds, is fine.
    let res = put_grid(
        &client,
        &base,
        grid_id,
        edited(&grid, 5, 5, json!([[[1, 1], [3, 1], [3, 3]]])),
    )
        .await;

    assert_eq!(res.status(), 200);
}

#[tokio::test]
async fn update_grid_enforces_the_same_obstacle_validation_as_create() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();

    let too_few = put_grid(
        &client,
        &base,
        grid_id,
        edited(&grid, 10, 10, json!([[[0, 0], [1, 1]]])),
    )
        .await;
    assert_eq!(too_few.status(), 400);

    let out_of_bounds = put_grid(
        &client,
        &base,
        grid_id,
        edited(&grid, 10, 10, json!([[[0, 0], [10, 0], [0, 5]]])),
    )
        .await;
    assert_eq!(out_of_bounds.status(), 400);
}

// --- grids: the freeze rule ----------------------------------------------

#[tokio::test]
async fn a_grid_is_editable_until_something_is_planned_against_it() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();

    // Free editing while nothing depends on the geometry — this is the drawing phase,
    // and it must not burn a version per tweak.
    for size in [11, 12, 13] {
        let res = put_grid(&client, &base, grid_id, edited(&grid, size, size, json!([]))).await;
        assert_eq!(res.status(), 200);
    }
    assert_eq!(show_grid(&client, &base, grid_id).await["version"], 0);

    freeze_with_plan(&client, &base, grid_id).await;

    let res = put_grid(&client, &base, grid_id, edited(&grid, 14, 14, json!([]))).await;
    assert_eq!(res.status(), 409);
    let message = res.text().await.unwrap();
    assert!(message.contains("1 plan(s)"), "{message}");
    assert!(
        message.contains(&format!("/grids/{grid_id}/versions")),
        "the 409 must say what to do instead: {message}"
    );

    // The refusal is total, not partial.
    assert_eq!(show_grid(&client, &base, grid_id).await["width"], 13);
}

#[tokio::test]
async fn the_freeze_covers_dimensions_as_well_as_obstacles() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_grid(&client, &base, 10, 10, json!([[[5, 5], [7, 5], [7, 7]]])).await;
    let grid_id = grid["id"].as_i64().unwrap();
    freeze_with_plan(&client, &base, grid_id).await;

    // Resizing invalidates a stored route as thoroughly as moving an obstacle does, so
    // it is refused even though the obstacles are untouched.
    let same_obstacles = grid["obs_polygons"].clone();
    let res = put_grid(&client, &base, grid_id, edited(&grid, 40, 40, same_obstacles)).await;
    assert_eq!(res.status(), 409);
}

#[tokio::test]
async fn a_rename_is_also_blocked_once_plans_exist() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();
    freeze_with_plan(&client, &base, grid_id).await;

    // A rename does not invalidate a route, but the name *is* the lineage — so it goes
    // through a fork like any other edit rather than rewriting history in place.
    let res = put_grid(
        &client,
        &base,
        grid_id,
        grid_body(&unique_name(), 10, 10, json!([])),
    )
        .await;
    assert_eq!(res.status(), 409);
}

#[tokio::test]
async fn deleting_the_plans_unfreezes_the_grid() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();
    let plan_id = freeze_with_plan(&client, &base, grid_id).await;

    assert_eq!(
        put_grid(&client, &base, grid_id, edited(&grid, 12, 12, json!([])))
            .await
            .status(),
        409
    );

    assert_eq!(delete_plan(&client, &base, plan_id).await.status(), 204);

    // Nothing depends on the geometry any more, so it is ordinary editable data again.
    // A grid frozen forever by a route you threw away would be the wrong answer.
    let res = put_grid(&client, &base, grid_id, edited(&grid, 12, 12, json!([]))).await;
    assert_eq!(res.status(), 200);
    assert_eq!(show_grid(&client, &base, grid_id).await["width"], 12);
}

// --- grids: versions -----------------------------------------------------

#[tokio::test]
async fn a_new_version_is_a_new_row_leaving_the_original_intact() {
    let client = Client::new();
    let base = spawn_app().await;
    // Clear of the origin, so the freezing plan below has somewhere to start.
    let grid = create_grid(&client, &base, 10, 10, json!([[[4, 4], [6, 4], [6, 6]]])).await;
    let grid_id = grid["id"].as_i64().unwrap();
    freeze_with_plan(&client, &base, grid_id).await;
    let replacement = json!([[[6, 6], [8, 6], [8, 8]]]);

    let res = post_grid_version(
        &client,
        &base,
        grid_id,
        edited(&grid, 10, 10, replacement.clone()),
    )
        .await;

    assert_eq!(res.status(), 201);
    let v1: Value = res.json().await.unwrap();
    assert_ne!(v1["id"], grid_id, "a version is a new row, not an update");
    assert_eq!(v1["version"], 1);
    assert_eq!(v1["name"], grid["name"], "and stays in the same lineage");
    assert_eq!(v1["obs_polygons"], replacement);

    // The frozen original is exactly as its plan left it.
    let v0 = show_grid(&client, &base, grid_id).await;
    assert_eq!(v0["version"], 0);
    assert_eq!(v0["obs_polygons"], grid["obs_polygons"]);
}

#[tokio::test]
async fn versions_may_be_stacked() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let mut id = grid["id"].as_i64().unwrap();

    for expected in 1..=3 {
        freeze_with_plan(&client, &base, id).await;
        let res = post_grid_version(&client, &base, id, edited(&grid, 10, 10, json!([]))).await;
        assert_eq!(res.status(), 201);
        let next: Value = res.json().await.unwrap();
        assert_eq!(next["version"], expected);
        id = next["id"].as_i64().unwrap();
    }
}

#[tokio::test]
async fn two_forks_of_one_snapshot_cannot_both_claim_the_version() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();
    freeze_with_plan(&client, &base, grid_id).await;
    let body = edited(&grid, 10, 10, json!([]));

    let first = post_grid_version(&client, &base, grid_id, body.clone()).await;
    assert_eq!(first.status(), 201);

    // Both forks compute v1 from the same parent. `(name, version)` is unique, so the
    // second is refused rather than silently producing two different grids that each
    // claim to be v1.
    let second = post_grid_version(&client, &base, grid_id, body).await;
    assert_eq!(second.status(), 409);
    assert!(res_contains(second, "already exists").await);
}

#[tokio::test]
async fn a_version_may_be_taken_of_an_unfrozen_grid_too() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();

    // Nothing forces a fork here — but "save as a new version" is a reasonable thing to
    // ask for deliberately, so the route does not require the grid to be frozen.
    let res = post_grid_version(&client, &base, grid_id, edited(&grid, 20, 20, json!([]))).await;
    assert_eq!(res.status(), 201);
    assert_eq!(res.json::<Value>().await.unwrap()["version"], 1);
}

#[tokio::test]
async fn a_new_version_enforces_the_same_obstacle_validation() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();

    let res = post_grid_version(
        &client,
        &base,
        grid_id,
        edited(&grid, 10, 10, json!([[[0, 0], [10, 0], [0, 5]]])),
    )
        .await;
    assert_eq!(res.status(), 400);
}

#[tokio::test]
async fn versioning_an_unknown_grid_is_404() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = post_grid_version(
        &client,
        &base,
        999999,
        grid_body(&unique_name(), 10, 10, json!([])),
    )
        .await;

    assert_eq!(res.status(), 404);
}

#[tokio::test]
async fn a_second_grid_cannot_take_an_existing_name() {
    let client = Client::new();
    let base = spawn_app().await;
    let name = unique_name();

    let first = post_grid(&client, &base, grid_body(&name, 10, 10, json!([]))).await;
    assert_eq!(first.status(), 201);

    // The name is the lineage, so a second `POST /grids` under it would be a rival
    // history rather than a new grid.
    let second = post_grid(&client, &base, grid_body(&name, 5, 5, json!([]))).await;
    assert_eq!(second.status(), 409);
    assert!(res_contains(second, "already exists").await);
}

// --- grids: delete -------------------------------------------------------

#[tokio::test]
async fn delete_grid_returns_204_and_removes_it() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();

    let res = client
        .delete(format!("{base}/grids/{grid_id}"))
        .send()
        .await
        .unwrap();

    assert_eq!(res.status(), 204);

    let listed = list_grids(&client, &base).await;
    assert!(!listed.iter().any(|g| g["id"] == grid_id));
}

#[tokio::test]
async fn deleting_a_grid_that_plans_depend_on_is_409() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();
    let plan_id = freeze_with_plan(&client, &base, grid_id).await;

    let res = client
        .delete(format!("{base}/grids/{grid_id}"))
        .send()
        .await
        .unwrap();

    // The FK would happily cascade the plan away. A plan worth freezing the grid over
    // is worth deleting deliberately rather than as a side effect.
    assert_eq!(res.status(), 409);
    assert!(res.text().await.unwrap().contains("1 plan(s)"));

    // Both rows survived the refusal.
    assert_eq!(show_grid(&client, &base, grid_id).await["id"], grid_id);
    let db = db().await;
    assert!(
        plans::Entity::find_by_id(plan_id as i32)
            .one(&db)
            .await
            .unwrap()
            .is_some()
    );
}

#[tokio::test]
async fn deleting_the_plans_lets_the_grid_be_deleted() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();
    let plan_id = freeze_with_plan(&client, &base, grid_id).await;

    assert_eq!(delete_plan(&client, &base, plan_id).await.status(), 204);

    let res = client
        .delete(format!("{base}/grids/{grid_id}"))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), 204);
}

#[tokio::test]
async fn deleting_an_unknown_grid_is_404() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = client
        .delete(format!("{base}/grids/999999"))
        .send()
        .await
        .unwrap();

    assert_eq!(res.status(), 404);
}

#[tokio::test]
async fn deleting_a_grid_twice_is_404_the_second_time() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 10, 10).await;
    let grid_id = grid["id"].as_i64().unwrap();

    let first = client
        .delete(format!("{base}/grids/{grid_id}"))
        .send()
        .await
        .unwrap();
    assert_eq!(first.status(), 204);

    let second = client
        .delete(format!("{base}/grids/{grid_id}"))
        .send()
        .await
        .unwrap();
    assert_eq!(second.status(), 404, "delete is not silently idempotent");
}

// --- plans ---------------------------------------------------------------

async fn post_plan(
    client: &Client,
    base: &str,
    grid_id: i64,
    src: [i32; 2],
    dest: [i32; 2],
) -> reqwest::Response {
    client
        .post(format!("{base}/grids/{grid_id}/plans"))
        .json(&json!({ "src_vertex": src, "dest_vertex": dest }))
        .send()
        .await
        .unwrap()
}

/// Reads a plan's cells as `(x, y)` pairs, so assertions read like the grid does.
fn plan_cells(plan: &Value) -> Vec<(i64, i64)> {
    plan["vertices"]
        .as_array()
        .expect("vertices must be an array")
        .iter()
        .map(|cell| (cell[0].as_i64().unwrap(), cell[1].as_i64().unwrap()))
        .collect()
}

#[tokio::test]
async fn planning_across_an_empty_grid_returns_the_direct_route() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();

    let res = post_plan(&client, &base, grid_id, [0, 0], [3, 3]).await;
    assert_eq!(res.status(), 201);

    let plan: Value = res.json().await.unwrap();
    assert_eq!(plan_cells(&plan), vec![(0, 0), (1, 1), (2, 2), (3, 3)]);
    assert_eq!(plan["grid_id"].as_i64(), Some(grid_id));
    // `meta` is NOT NULL in the schema, so a plan that fails to populate it never
    // reaches the client at all — the insert fails first, as a 500.
    assert_eq!(plan["meta"]["planner"], "a_star");
    assert_eq!(plan["meta"]["reachable"], true);
    assert_eq!(plan["meta"]["cost"], 42, "3 diagonal steps at 14 each");
    assert_eq!(plan["meta"]["src_vertex"], json!([0, 0]));
    assert_eq!(plan["meta"]["dest_vertex"], json!([3, 3]));
}

#[tokio::test]
async fn planning_routes_around_a_saved_obstacle() {
    let client = Client::new();
    let base = spawn_app().await;

    // A wall down column 4, open only along the bottom row.
    let grid_id = create_grid(&client, &base, 9, 5, json!([[[4, 0], [4, 3], [4, 3], [4, 0]]]))
        .await["id"]
        .as_i64()
        .unwrap();

    let res = post_plan(&client, &base, grid_id, [0, 0], [8, 0]).await;
    assert_eq!(res.status(), 201);

    let plan: Value = res.json().await.unwrap();
    let cells = plan_cells(&plan);
    assert_eq!(cells.first(), Some(&(0, 0)));
    assert_eq!(cells.last(), Some(&(8, 0)));
    assert!(
        cells.iter().all(|&(x, y)| !(x == 4 && y <= 3)),
        "the route crossed the wall: {cells:?}",
    );
    assert!(
        cells.iter().any(|&(_, y)| y == 4),
        "the only way past the wall is the bottom row: {cells:?}",
    );
}

#[tokio::test]
async fn a_new_version_plans_against_its_own_obstacles() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid = create_empty_grid(&client, &base, 9, 5).await;
    let grid_id = grid["id"].as_i64().unwrap();

    // The empty grid routes straight down row 0 — and planning freezes it.
    let before: Value = post_plan(&client, &base, grid_id, [0, 0], [8, 0])
        .await
        .json()
        .await
        .unwrap();
    assert!(plan_cells(&before).iter().all(|&(_, y)| y == 0));

    // Adding a wall now has to fork, which is the point: the route above stays valid
    // against v0 forever, and the wall lives in v1.
    let res = post_grid_version(
        &client,
        &base,
        grid_id,
        edited(&grid, 9, 5, json!([[[4, 0], [4, 3], [4, 3], [4, 0]]])),
    )
        .await;
    assert_eq!(res.status(), 201);
    let v1_id = res.json::<Value>().await.unwrap()["id"].as_i64().unwrap();

    let after: Value = post_plan(&client, &base, v1_id, [0, 0], [8, 0])
        .await
        .json()
        .await
        .unwrap();
    let cells = plan_cells(&after);
    assert!(
        cells.iter().any(|&(_, y)| y == 4),
        "the new wall was ignored: {cells:?}",
    );

    // The original route is untouched by the fork — it still describes v0's world.
    let v0_plans: Vec<Value> = client
        .get(format!("{base}/grids/{grid_id}/plans"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(v0_plans.len(), 1);
    assert_eq!(v0_plans[0]["id"], before["id"]);
}

#[tokio::test]
async fn an_unreachable_goal_is_a_saved_plan_with_no_cells() {
    let client = Client::new();
    let base = spawn_app().await;

    // A wall clean across the grid, sealing the two halves off from each other.
    let grid_id = create_grid(&client, &base, 5, 3, json!([[[2, 0], [2, 2], [2, 2], [2, 0]]]))
        .await["id"]
        .as_i64()
        .unwrap();

    let res = post_plan(&client, &base, grid_id, [0, 1], [4, 1]).await;
    // Deliberately not an error: the request was well formed and the answer — that no
    // route exists — is a fact about the grid worth recording.
    assert_eq!(res.status(), 201);

    let plan: Value = res.json().await.unwrap();
    assert_eq!(plan_cells(&plan), Vec::new());
    assert_eq!(plan["meta"]["reachable"], false);
    assert_eq!(plan["meta"]["cost"], 0);
}

#[tokio::test]
async fn planning_from_inside_an_obstacle_is_400() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_grid(&client, &base, 10, 10, json!([[[2, 2], [5, 2], [5, 5], [2, 5]]]))
        .await["id"]
        .as_i64()
        .unwrap();

    let res = post_plan(&client, &base, grid_id, [3, 3], [9, 9]).await;
    assert_eq!(res.status(), 400, "a start inside an obstacle is the caller's mistake");
    assert!(res.text().await.unwrap().contains("start"));

    let res = post_plan(&client, &base, grid_id, [0, 0], [3, 3]).await;
    assert_eq!(res.status(), 400);
    assert!(res.text().await.unwrap().contains("goal"));
}

#[tokio::test]
async fn planning_to_a_cell_outside_the_grid_is_400() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_empty_grid(&client, &base, 6, 4).await["id"]
        .as_i64()
        .unwrap();

    // Cell indices, so the far edge is out of range: a 6-wide grid addresses 0..=5.
    let res = post_plan(&client, &base, grid_id, [0, 0], [6, 0]).await;
    assert_eq!(res.status(), 400);
    assert!(res.text().await.unwrap().contains("outside the grid"));

    let res = post_plan(&client, &base, grid_id, [-1, 0], [1, 1]).await;
    assert_eq!(res.status(), 400);
}

#[tokio::test]
async fn planning_on_an_unknown_grid_is_404() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = post_plan(&client, &base, 999999, [0, 0], [1, 1]).await;
    assert_eq!(res.status(), 404);
}

// --- plans: read ---------------------------------------------------------

#[tokio::test]
async fn list_plans_returns_only_the_grids_own_plans() {
    let client = Client::new();
    let base = spawn_app().await;
    let mine = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();
    let other = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();
    let expected = freeze_with_plan(&client, &base, mine).await;
    freeze_with_plan(&client, &base, other).await;

    let listed = list_plans(&client, &base, mine).await;

    assert_eq!(listed.len(), 1);
    assert_eq!(listed[0]["id"], expected);
    assert_eq!(listed[0]["grid_id"], mine);
}

#[tokio::test]
async fn an_empty_plan_list_is_how_a_client_knows_a_grid_is_editable() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();

    assert_eq!(list_plans(&client, &base, grid_id).await, Vec::<Value>::new());

    freeze_with_plan(&client, &base, grid_id).await;
    assert_eq!(list_plans(&client, &base, grid_id).await.len(), 1);
}

#[tokio::test]
async fn listing_plans_of_an_unknown_grid_is_404() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = client
        .get(format!("{base}/grids/999999/plans"))
        .send()
        .await
        .unwrap();

    // Not an empty list: a stale grid id must not read as "editable".
    assert_eq!(res.status(), 404);
}

// --- plans: delete -------------------------------------------------------

#[tokio::test]
async fn delete_plan_returns_204_and_removes_only_that_plan() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();
    let doomed: Value = post_plan(&client, &base, grid_id, [0, 0], [3, 3])
        .await
        .json()
        .await
        .unwrap();
    let survivor: Value = post_plan(&client, &base, grid_id, [1, 1], [5, 5])
        .await
        .json()
        .await
        .unwrap();
    let doomed_id = doomed["id"].as_i64().unwrap();
    let survivor_id = survivor["id"].as_i64().unwrap() as i32;

    let res = client
        .delete(format!("{base}/plans/{doomed_id}"))
        .send()
        .await
        .unwrap();

    assert_eq!(res.status(), 204);

    let db = db().await;
    assert!(
        plans::Entity::find_by_id(doomed_id as i32)
            .one(&db)
            .await
            .unwrap()
            .is_none()
    );
    assert!(
        plans::Entity::find_by_id(survivor_id)
            .one(&db)
            .await
            .unwrap()
            .is_some(),
        "the grid's other plan was taken with it"
    );
}

#[tokio::test]
async fn deleting_a_plan_leaves_its_grid_alone() {
    let client = Client::new();
    let base = spawn_app().await;
    let grid_id = create_empty_grid(&client, &base, 10, 10).await["id"]
        .as_i64()
        .unwrap();
    let plan: Value = post_plan(&client, &base, grid_id, [0, 0], [3, 3])
        .await
        .json()
        .await
        .unwrap();

    let res = client
        .delete(format!("{base}/plans/{}", plan["id"].as_i64().unwrap()))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), 204);

    // The old nested route took a *grid* id and deleted whichever plan shared that
    // number; this pins that a plan delete never reaches across to the grid.
    assert_eq!(show_grid(&client, &base, grid_id).await["id"], grid_id);
}

#[tokio::test]
async fn deleting_an_unknown_plan_is_404() {
    let client = Client::new();
    let base = spawn_app().await;

    let res = client
        .delete(format!("{base}/plans/999999"))
        .send()
        .await
        .unwrap();

    assert_eq!(res.status(), 404);
    assert!(res.text().await.unwrap().contains("plan 999999 not found"));
}