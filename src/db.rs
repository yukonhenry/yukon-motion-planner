//! Connecting to Postgres, and failing usefully when that doesn't work.
//!
//! [`Database::connect`] is a poor first contact with a database. It hands its work to a
//! pool that retries until its acquire timeout expires, so every way of being wrong —
//! container stopped, database misnamed, password changed — arrives thirty seconds later
//! as the same `PoolTimedOut`. [`connect`] checks the cheap, specific things first and
//! only then builds the pool, so the common mistakes are reported in milliseconds and
//! named for what they are.

use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use sea_orm::sqlx::postgres::PgConnection;
use sea_orm::sqlx::{Connection, Error as SqlxError};
use sea_orm::{ConnectOptions, Database, DatabaseConnection};

/// Bounds every wait on the database. Long enough for a container that is still
/// starting up, short enough that a wrong `DATABASE_URL` is not mistaken for slowness.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// [`ConnectOptions`] with that timeout applied. Callers add their own pool sizing.
pub fn options(url: &str) -> ConnectOptions {
    let mut opts = ConnectOptions::new(url.to_string());
    opts.connect_timeout(CONNECT_TIMEOUT);
    opts
}

/// Connects, or explains why it couldn't in terms of the thing to go fix.
///
/// The error is a finished sentence meant for a terminal, not a `Debug` dump: print it
/// and exit rather than wrapping it in further context.
pub async fn connect(opts: ConnectOptions) -> Result<DatabaseConnection, String> {
    let url = opts.get_url().to_string();
    reachable(&url)?;
    handshake(&url).await?;

    Database::connect(opts)
        .await
        .map_err(|err| format!("Could not connect to Postgres at {}: {err}", endpoint(&url)))
}

/// The `host:port` a Postgres URL points at.
///
/// Deliberately partial: a shape this can't read comes back as `None`, and the caller
/// falls back to simply connecting rather than acting on a guess.
pub fn host_port(url: &str) -> Option<(String, u16)> {
    let authority = url.split_once("://")?.1.split(['/', '?']).next()?;
    let host_port = authority.rsplit_once('@').map_or(authority, |(_, h)| h);

    // An IPv6 literal is itself full of colons, so it arrives bracketed and any port
    // follows the `]`. The brackets are URL syntax and no part of the address.
    if let Some(rest) = host_port.strip_prefix('[') {
        let (host, after) = rest.split_once(']')?;
        let port = match after.strip_prefix(':') {
            Some(port) => port.parse().ok()?,
            None => 5432,
        };
        return Some((host.to_string(), port));
    }

    match host_port.rsplit_once(':') {
        Some((host, port)) if !host.is_empty() => Some((host.to_string(), port.parse().ok()?)),
        _ => Some((host_port.to_string(), 5432)),
    }
}

/// The database a Postgres URL names, if it names one.
pub fn database_name(url: &str) -> Option<&str> {
    let path = url.split_once("://")?.1.split_once('/')?.1;
    let name = path.split(['?', '#']).next()?;
    (!name.is_empty()).then_some(name)
}

/// `host:port/database`, with any credentials dropped — safe to print.
pub fn endpoint(url: &str) -> String {
    let Some((_, rest)) = url.split_once("://") else {
        return url.to_string();
    };
    let without_credentials = rest.rsplit_once('@').map_or(rest, |(_, tail)| tail);
    without_credentials
        .split(['?', '#'])
        .next()
        .unwrap_or(without_credentials)
        .to_string()
}

/// Checks that *something* is listening, before any pool gets involved.
///
/// A stopped container refuses the connection instantly, and this is the only check that
/// can say so instantly: everything below it has to wait out a timeout to be sure.
fn reachable(url: &str) -> Result<(), String> {
    let Some((host, port)) = host_port(url) else {
        return Ok(());
    };
    let Ok(addrs) = (host.as_str(), port).to_socket_addrs() else {
        return Ok(());
    };

    let mut refusal = None;
    for addr in addrs {
        match TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT) {
            Ok(_) => return Ok(()),
            Err(err) => refusal = Some(err),
        }
    }
    match refusal {
        Some(err) => Err(format!(
            "Postgres is not reachable at {host}:{port} ({err}). \
             Start it with `docker compose up -d`, or point DATABASE_URL somewhere else."
        )),
        // No addresses to try at all; let the pool produce the error.
        None => Ok(()),
    }
}

/// One direct connection, made only to get a real answer out of Postgres.
///
/// The pool can't provide one. It treats a refused login the way it treats a busy server
/// — as something to retry — so the specific error is spent inside the acquire timeout
/// and `PoolTimedOut` is all that comes back out.
async fn handshake(url: &str) -> Result<(), String> {
    match tokio::time::timeout(CONNECT_TIMEOUT, PgConnection::connect(url)).await {
        Ok(Ok(conn)) => {
            let _ = conn.close().await;
            Ok(())
        }
        Ok(Err(err)) => Err(explain(url, &err)),
        Err(_) => Err(format!(
            "Postgres at {} accepted a connection but did not finish the handshake within {}s.",
            endpoint(url),
            CONNECT_TIMEOUT.as_secs()
        )),
    }
}

/// Turns the errors people actually hit into instructions.
fn explain(url: &str, err: &SqlxError) -> String {
    let SqlxError::Database(db_err) = err else {
        return format!("Could not connect to Postgres at {}: {err}", endpoint(url));
    };

    let (host, port) = host_port(url).unwrap_or_else(|| ("localhost".to_string(), 5432));
    match db_err.code().as_deref() {
        // The server is fine; it just has no database by that name. Migrations create
        // tables inside a database, never the database itself.
        Some("3D000") => {
            let name = database_name(url).unwrap_or("<none>");
            format!(
                "Postgres at {host}:{port} has no database named `{name}`. \
                 Create it (`docker compose exec db createdb -U postgres {name}`), \
                 or correct the database name in DATABASE_URL."
            )
        }
        // Wrong password, or a role that doesn't exist.
        Some("28P01" | "28000") => format!(
            "Postgres at {host}:{port} rejected the credentials in DATABASE_URL ({}).",
            db_err.message()
        ),
        _ => format!("Postgres at {host}:{port} refused the connection: {db_err}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const URL: &str = "postgres://postgres:secret@db.example:6543/planner";

    #[test]
    fn reads_the_parts_of_a_full_url() {
        assert_eq!(host_port(URL), Some(("db.example".to_string(), 6543)));
        assert_eq!(database_name(URL), Some("planner"));
    }

    #[test]
    fn defaults_the_port_when_the_url_leaves_it_out() {
        let url = "postgres://postgres@localhost/planner";
        assert_eq!(host_port(url), Some(("localhost".to_string(), 5432)));
    }

    #[test]
    fn unwraps_a_bracketed_ipv6_literal() {
        // The brackets have to go: `to_socket_addrs` resolves `::1`, not `[::1]`.
        assert_eq!(
            host_port("postgres://postgres@[::1]:5432/planner"),
            Some(("::1".to_string(), 5432))
        );
        assert_eq!(
            host_port("postgres://postgres@[::1]/planner"),
            Some(("::1".to_string(), 5432))
        );
    }

    #[test]
    fn declines_to_guess_at_shapes_it_cannot_read() {
        assert_eq!(host_port("localhost:5432"), None);
        assert_eq!(host_port("postgres://localhost:not-a-port/planner"), None);
        assert_eq!(database_name("postgres://localhost:5432"), None);
        assert_eq!(database_name("postgres://localhost:5432/"), None);
    }

    #[test]
    fn the_printable_endpoint_leaves_the_password_out() {
        let printable = endpoint(URL);
        assert_eq!(printable, "db.example:6543/planner");
        assert!(!printable.contains("secret"));
    }

    #[test]
    fn query_parameters_belong_to_neither_the_name_nor_the_endpoint() {
        let url = "postgres://postgres@localhost:5432/planner?sslmode=require";
        assert_eq!(database_name(url), Some("planner"));
        assert_eq!(endpoint(url), "localhost:5432/planner");
    }
}
