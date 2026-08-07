use migration::MigratorTrait;
use std::env;
use std::process::exit;
use yukon_motion_planner::{db, router};

#[tokio::main]
async fn main() {
    dotenvy::dotenv().ok();

    let database_url = env::var("DATABASE_URL")
        .unwrap_or_else(|_| "postgres://postgres:postgres@localhost:5432/postgres".to_string());
    println!("Database: {}", db::endpoint(&database_url));

    // A misconfigured or absent database is the ordinary startup failure, and there is
    // nothing to serve without one — so it exits with the explanation rather than a
    // panic, whose backtrace note only distracts from an answer that is already plain.
    let db = db::connect(db::options(&database_url))
        .await
        .unwrap_or_else(|message| {
            eprintln!("{message}");
            exit(1);
        });

    tracing_subscriber::fmt::init();

    migration::Migrator::up(&db, None)
        .await
        .expect("Failed to run migrations");

    let app = router::route(db).await;

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await.unwrap();
    println!("Listening on http://localhost:3000");
    axum::serve(listener, app).await.unwrap();
}