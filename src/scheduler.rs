//! Backend-driven simulation: the environment and the replanner as two independently
//! scheduled tasks, and the broadcast that tells the frontend what they did.
//!
//! # The shape of a run
//!
//! Starting a run spawns two tokio tasks over one shared [`SimWorld`]:
//!
//! * the **environment** task, ticking at `grid_worlds.sim_interval`, jitters every dynamic
//!   obstacle;
//! * the **replanner** task, ticking at the interval the run was started with, recomputes
//!   the route against whatever the obstacles happen to be at that moment.
//!
//! Two tasks rather than one loop over both clocks, because the frequencies are the
//! experiment. A planner that runs at a tenth of the world's rate should visibly lag it, and
//! a single loop would have to invent a policy for what to do when both are due — which is
//! precisely the coupling worth not having.
//!
//! # What is *not* here
//!
//! No database access after start. The tasks read the grid's dimensions, its obstacles and
//! the plan's endpoints once, into memory, and never look again — so a tick costs a lock, a
//! `Vec` clone and a search, with no query in the hot loop. Nothing is written back either:
//! ticks stay ephemeral, exactly as `POST /grids/{id}/replan` already had them. A run
//! therefore neither freezes the grid it is exploring nor buries the user's saved routes
//! under a plan row per frame, and a server restart simply ends it.
//!
//! # Consistency
//!
//! The world is behind a [`std::sync::Mutex`], never held across an `await`. The environment
//! mutates it and releases; the replanner clones out of it and releases, then searches with
//! the lock free. So the planner always sees a whole world — never a shape mid-jitter — and
//! a slow search never stalls the environment behind it.

use crate::models::obstacle::ObstaclePoly;
use crate::models::planners::PlannerKind;
use crate::models::simulation::{SimWorld, plan_route};
use serde::Serialize;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;

/// How many events a subscriber may fall behind before it starts losing them.
///
/// Small on purpose. Every event carries a *whole* snapshot — all the obstacles, or the whole
/// route — so a client that misses some is corrected by the next one rather than left with a
/// hole in its state. Buffering deeply would only mean showing a slow client a longer stretch
/// of stale frames.
const EVENT_BUFFER: usize = 32;

/// The bound on both intervals, in seconds.
///
/// The floor keeps a typo like `0.0001` from spawning a task that saturates a core and floods
/// the stream; the ceiling is a sanity check, not a real limit.
const MIN_INTERVAL_SECS: f64 = 0.02;
const MAX_INTERVAL_SECS: f64 = 3600.0;

/// One thing that happened in a run, as pushed to the frontend.
///
/// Tagged as well as carrying an SSE event name so the same payload would serve a WebSocket
/// unchanged — the transport is a delivery choice, not part of what an event *is*.
#[derive(Clone, Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub(crate) enum SimEvent {
    /// The environment moved. Carries every obstacle, not a delta: the client redraws from
    /// this and needs no history to do it.
    Environment {
        tick: u64,
        /// How many obstacles actually moved. Zero means every draw clamped against an edge.
        moved: usize,
        obs_polygons: Vec<ObstaclePoly>,
    },
    /// The replanner finished a search.
    Plan {
        /// The replanner's own count, which advances independently of `env_tick`.
        tick: u64,
        /// Which environment tick this route was computed against. The gap between this and
        /// the latest `Environment.tick` is how far the planner is running behind the world.
        env_tick: u64,
        vertices: Vec<(usize, usize)>,
        reachable: bool,
        cost: u32,
        planner: &'static str,
        /// How long the search took. The reason a frequency might be the wrong one.
        elapsed_ms: u64,
    },
    /// The run ended. Always the last event on the stream.
    Stopped { reason: String },
}

impl SimEvent {
    /// The SSE `event:` name, which is what the frontend adds listeners for.
    pub(crate) fn name(&self) -> &'static str {
        match self {
            SimEvent::Environment { .. } => "environment",
            SimEvent::Plan { .. } => "plan",
            SimEvent::Stopped { .. } => "stopped",
        }
    }
}

/// A run in progress: its shared world, its subscribers, and the tasks driving it.
struct Run {
    plan_id: i32,
    world: Arc<Mutex<SimWorld>>,
    events: broadcast::Sender<SimEvent>,
    env_interval: f64,
    replan_interval: f64,
    /// Aborted on drop — which is what makes removing a run from the registry stop it, with
    /// no separate shutdown handshake to get wrong.
    tasks: Vec<JoinHandle<()>>,
}

impl Drop for Run {
    fn drop(&mut self) {
        for task in &self.tasks {
            task.abort();
        }
    }
}

/// Every run currently going, keyed by grid.
///
/// One run per grid, because that is what the UI offers and what a user means by "start the
/// simulation": a second run over the same grid would be two sets of obstacles claiming to be
/// the same world. Starting one where one already exists is a conflict, not a replacement —
/// silently discarding someone else's run would be the worse surprise.
#[derive(Default)]
pub(crate) struct SimRegistry {
    runs: Mutex<HashMap<i32, Run>>,
}

/// What starting a run answers with, and what `GET /grids/{id}/sim` reports about one.
///
/// Carries the opening snapshot as well as the parameters, so the frontend has a complete
/// picture the instant it starts and does not have to render an empty canvas until the first
/// tick arrives. The stream then carries changes from this point on.
#[derive(Debug, Serialize)]
pub(crate) struct SimStatus {
    pub(crate) grid_id: i32,
    pub(crate) plan_id: i32,
    pub(crate) env_interval: f64,
    pub(crate) replan_interval: f64,
    pub(crate) env_tick: u64,
    /// The obstacles as they stand right now.
    pub(crate) obs_polygons: Vec<ObstaclePoly>,
    /// Where the walk will go next, for replaying a run that turned out to be interesting.
    pub(crate) seed: u32,
}

/// Why a run could not be started.
#[derive(Debug)]
pub(crate) enum StartError {
    /// A run over this grid is already going.
    AlreadyRunning,
    /// An interval outside [`MIN_INTERVAL_SECS`]..=[`MAX_INTERVAL_SECS`], or not a number.
    BadInterval(String),
    /// Nothing on the grid can move, so the run would tick forever changing nothing.
    NothingDynamic,
}

impl std::fmt::Display for StartError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StartError::AlreadyRunning => {
                f.write_str("a simulation is already running on this grid — stop it first")
            }
            StartError::BadInterval(message) => f.write_str(message),
            StartError::NothingDynamic => f.write_str(
                "no obstacle on this grid is marked dynamic — the environment would never change",
            ),
        }
    }
}

/// Rejects an interval that would make a useless or abusive task.
///
/// Checked here rather than by a database constraint because it is a property of the
/// *scheduler* — what this process is willing to spawn a task for — and because an override
/// arriving in a request never touches the column at all.
fn checked_interval(seconds: f64, which: &str) -> Result<Duration, StartError> {
    if !seconds.is_finite() || !(MIN_INTERVAL_SECS..=MAX_INTERVAL_SECS).contains(&seconds) {
        return Err(StartError::BadInterval(format!(
            "{which} interval {seconds} is not between {MIN_INTERVAL_SECS} and \
             {MAX_INTERVAL_SECS} seconds",
        )));
    }
    Ok(Duration::from_secs_f64(seconds))
}

impl SimRegistry {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Starts a run and spawns its two tasks.
    ///
    /// Everything the tasks need is passed in already resolved — the caller has read the grid
    /// and the plan and turned them into geometry, endpoints and intervals — so this function
    /// makes no decisions about *what* is being simulated, only about scheduling it.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn start(
        &self,
        grid_id: i32,
        plan_id: i32,
        width: i32,
        height: i32,
        obstacles: Vec<ObstaclePoly>,
        src: [i32; 2],
        dest: [i32; 2],
        seed: u64,
        env_interval: f64,
        replan_interval: f64,
    ) -> Result<SimStatus, StartError> {
        let env_period = checked_interval(env_interval, "environment")?;
        let replan_period = checked_interval(replan_interval, "replanner")?;

        let world = SimWorld::new(width, height, obstacles, seed);
        if !world.has_dynamic_obstacles() {
            return Err(StartError::NothingDynamic);
        }

        // Held across the whole insert so two starts racing on one grid cannot both find the
        // slot empty and both spawn tasks.
        let mut runs = self.runs.lock().expect("sim registry poisoned");
        if runs.contains_key(&grid_id) {
            return Err(StartError::AlreadyRunning);
        }

        let world = Arc::new(Mutex::new(world));
        let (events, _) = broadcast::channel(EVENT_BUFFER);

        let environment = tokio::spawn(environment_task(
            Arc::clone(&world),
            events.clone(),
            env_period,
        ));
        let replanner = tokio::spawn(replanner_task(
            Arc::clone(&world),
            events.clone(),
            replan_period,
            src,
            dest,
        ));

        let status = {
            let world = world.lock().expect("sim world poisoned");
            let (obs_polygons, env_tick) = world.snapshot();
            SimStatus {
                grid_id,
                plan_id,
                env_interval,
                replan_interval,
                env_tick,
                obs_polygons,
                seed: world.peek_seed(),
            }
        };

        runs.insert(
            grid_id,
            Run {
                plan_id,
                world,
                events,
                env_interval,
                replan_interval,
                tasks: vec![environment, replanner],
            },
        );

        tracing::info!(
            grid_id,
            plan_id,
            env_interval,
            replan_interval,
            "simulation started"
        );
        Ok(status)
    }

    /// Stops a run, telling its subscribers why. `false` if nothing was running.
    ///
    /// The `Stopped` event goes out *before* the run is dropped, because dropping it aborts
    /// the tasks and closes the channel — a client would otherwise see the stream end with no
    /// explanation and have to guess between "stopped" and "the server fell over".
    pub(crate) fn stop(&self, grid_id: i32, reason: &str) -> bool {
        let mut runs = self.runs.lock().expect("sim registry poisoned");
        match runs.remove(&grid_id) {
            Some(run) => {
                let _ = run.events.send(SimEvent::Stopped {
                    reason: reason.to_string(),
                });
                tracing::info!(grid_id, reason, "simulation stopped");
                // `run` drops here, aborting both tasks.
                true
            }
            None => false,
        }
    }

    /// A stream of everything a run does from now on, or `None` if it is not running.
    ///
    /// Only events sent after this call arrive, which is why [`SimStatus`] carries the
    /// opening snapshot: the client's picture is "the status, then every event since".
    pub(crate) fn subscribe(&self, grid_id: i32) -> Option<broadcast::Receiver<SimEvent>> {
        let runs = self.runs.lock().expect("sim registry poisoned");
        runs.get(&grid_id).map(|run| run.events.subscribe())
    }

    /// What a run currently looks like, or `None` if it is not running.
    ///
    /// Also how a reloaded page finds its way back into a run it started before the refresh.
    pub(crate) fn status(&self, grid_id: i32) -> Option<SimStatus> {
        let runs = self.runs.lock().expect("sim registry poisoned");
        let run = runs.get(&grid_id)?;
        let world = run.world.lock().expect("sim world poisoned");
        let (obs_polygons, env_tick) = world.snapshot();
        Some(SimStatus {
            grid_id,
            plan_id: run.plan_id,
            env_interval: run.env_interval,
            replan_interval: run.replan_interval,
            env_tick,
            obs_polygons,
            seed: world.peek_seed(),
        })
    }
}

/// A ticker on a fixed period that does not try to catch up.
///
/// `Delay` rather than tokio's default `Burst`: if a search overruns its interval, bursting
/// would fire the missed ticks back to back and turn a planner that is merely too slow into
/// one that never stops searching. Falling behind at a steady rate is the honest behavior,
/// and the `env_tick` gap on each `Plan` event is what shows it happening.
fn ticker(period: Duration) -> tokio::time::Interval {
    let mut ticker = tokio::time::interval(period);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    ticker
}

/// Jitters the dynamic obstacles, once per environment interval.
async fn environment_task(
    world: Arc<Mutex<SimWorld>>,
    events: broadcast::Sender<SimEvent>,
    period: Duration,
) {
    let mut ticker = ticker(period);
    // `interval` fires immediately; the caller already has tick 0 in its `SimStatus`, so the
    // first *change* belongs one period in rather than at once.
    ticker.tick().await;

    loop {
        ticker.tick().await;

        let tick = {
            let mut world = world.lock().expect("sim world poisoned");
            world.advance()
        };

        // `send` failing means nobody is listening. Not a reason to stop: a run is the
        // server's, and a user closing the tab and reopening it should find it still going.
        let _ = events.send(SimEvent::Environment {
            tick: tick.tick,
            moved: tick.moved,
            obs_polygons: tick.obstacles,
        });
    }
}

/// Replans against the world as it stands, once per replanner interval.
async fn replanner_task(
    world: Arc<Mutex<SimWorld>>,
    events: broadcast::Sender<SimEvent>,
    period: Duration,
    src: [i32; 2],
    dest: [i32; 2],
) {
    let mut ticker = ticker(period);
    // The initial plan is already on screen — it is what the run was started from — so the
    // first *re*plan is one period in.
    ticker.tick().await;

    let mut tick = 0u64;
    loop {
        ticker.tick().await;
        tick += 1;

        let (obstacles, env_tick, width, height) = {
            let world = world.lock().expect("sim world poisoned");
            let (obstacles, env_tick) = world.snapshot();
            (obstacles, env_tick, world.width, world.height)
        };

        // A search is CPU-bound and unbounded in principle, so it goes to the blocking pool
        // rather than parking a runtime worker — otherwise one large grid would stall every
        // other run's environment along with it.
        let started = std::time::Instant::now();
        let outcome = tokio::task::spawn_blocking(move || {
            // D* Lite specifically: replanning a world that just changed is what it is for.
            // It still plans from scratch per tick — making it incremental means keeping the
            // planner itself in `SimWorld` across ticks, which is the natural next step now
            // that a run finally has somewhere to keep state.
            plan_route(width, height, &obstacles, src, dest, PlannerKind::DStarLite)
        })
            .await;

        let elapsed_ms = started.elapsed().as_millis() as u64;

        match outcome {
            Ok(Ok(route)) => {
                let _ = events.send(SimEvent::Plan {
                    tick,
                    env_tick,
                    vertices: route.vertices,
                    reachable: route.reachable,
                    cost: route.cost,
                    planner: route.planner,
                    elapsed_ms,
                });
            }
            // The endpoints were legal when the run started and cannot move, so this means an
            // obstacle has drifted over the start or the goal. Reported once per tick rather
            // than ending the run: the obstacle may well drift off again.
            Ok(Err(err)) => {
                tracing::debug!("replan tick {tick} could not plan: {err}");
            }
            // The blocking task panicked, or was cancelled by a stop landing mid-search.
            Err(err) => {
                if !err.is_cancelled() {
                    tracing::error!("replan tick {tick} panicked: {err}");
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::obstacle::CellVertex;

    fn square(dynamic: bool) -> ObstaclePoly {
        ObstaclePoly {
            id: 1,
            dynamic,
            vertices: vec![
                CellVertex { x: 3, y: 3 },
                CellVertex { x: 6, y: 3 },
                CellVertex { x: 6, y: 6 },
                CellVertex { x: 3, y: 6 },
                CellVertex { x: 3, y: 3 },
            ],
        }
    }

    fn start(
        registry: &SimRegistry,
        grid_id: i32,
        env: f64,
        replan: f64,
    ) -> Result<SimStatus, StartError> {
        registry.start(
            grid_id,
            1,
            10,
            10,
            vec![square(true)],
            [0, 0],
            [9, 9],
            42,
            env,
            replan,
        )
    }

    #[tokio::test]
    async fn a_second_run_on_one_grid_is_refused() {
        // Two runs over one grid would be two sets of obstacles both claiming to be that
        // world — and the second would silently orphan the first one's subscribers.
        let registry = SimRegistry::new();
        assert!(start(&registry, 1, 1.0, 1.0).is_ok());
        assert!(matches!(
            start(&registry, 1, 1.0, 1.0),
            Err(StartError::AlreadyRunning)
        ));
        // A different grid is a different world, and is fine.
        assert!(start(&registry, 2, 1.0, 1.0).is_ok());
    }

    #[tokio::test]
    async fn stopping_frees_the_grid_to_run_again() {
        let registry = SimRegistry::new();
        assert!(start(&registry, 1, 1.0, 1.0).is_ok());
        assert!(registry.stop(1, "test"));
        assert!(!registry.stop(1, "test"), "stopped twice");
        assert!(start(&registry, 1, 1.0, 1.0).is_ok());
    }

    #[tokio::test]
    async fn an_unusable_interval_is_refused_before_a_task_is_spawned() {
        // Nothing is registered on failure, so a rejected start leaves the grid startable.
        let registry = SimRegistry::new();
        for bad in [0.0, -1.0, f64::NAN, 0.000_1, 99_999.0] {
            assert!(
                matches!(
                    start(&registry, 1, bad, 1.0),
                    Err(StartError::BadInterval(_))
                ),
                "{bad} was accepted as an environment interval",
            );
        }
        assert!(
            registry.status(1).is_none(),
            "a rejected start registered a run"
        );
    }

    #[tokio::test]
    async fn a_run_over_scenery_alone_is_refused() {
        let registry = SimRegistry::new();
        let result = registry.start(
            1,
            1,
            10,
            10,
            vec![square(false)],
            [0, 0],
            [9, 9],
            42,
            1.0,
            1.0,
        );
        assert!(matches!(result, Err(StartError::NothingDynamic)));
    }

    #[tokio::test]
    async fn the_two_clocks_advance_independently() {
        // The whole point of the feature: a fast world under a slow planner. Over the same
        // stretch of time the environment must produce strictly more events than the planner.
        let registry = SimRegistry::new();
        let mut stream = {
            start(&registry, 1, 0.02, 0.2).expect("startable");
            registry.subscribe(1).expect("running")
        };

        let (mut env, mut plans) = (0, 0);
        let deadline = tokio::time::Instant::now() + Duration::from_millis(500);
        while let Ok(Ok(event)) = tokio::time::timeout_at(deadline, stream.recv()).await {
            match event {
                SimEvent::Environment { .. } => env += 1,
                SimEvent::Plan { .. } => plans += 1,
                SimEvent::Stopped { .. } => break,
            }
        }

        assert!(env > 0, "the environment never ticked");
        assert!(
            env > plans,
            "a 20ms world under a 200ms planner produced {env} environment and {plans} plan events",
        );
        registry.stop(1, "test");
    }

    #[tokio::test]
    async fn subscribers_are_told_why_the_stream_ended() {
        // Otherwise a client cannot tell a deliberate stop from the server falling over.
        let registry = SimRegistry::new();
        start(&registry, 1, 0.02, 0.02).expect("startable");
        let mut stream = registry.subscribe(1).expect("running");
        registry.stop(1, "the user pressed stop");

        // The buffered environment ticks come first; the stop is the last event before close.
        let mut last = None;
        while let Ok(event) = stream.recv().await {
            last = Some(event);
        }
        assert!(
            matches!(last, Some(SimEvent::Stopped { ref reason }) if reason == "the user pressed stop"),
            "the stream ended without a reason: {last:?}",
        );
    }

    #[tokio::test]
    async fn a_stopped_run_has_no_status_and_no_stream() {
        let registry = SimRegistry::new();
        start(&registry, 1, 1.0, 1.0).expect("startable");
        assert!(registry.status(1).is_some());
        registry.stop(1, "test");
        assert!(registry.status(1).is_none());
        assert!(registry.subscribe(1).is_none());
    }

    #[tokio::test]
    async fn the_environment_actually_moves_the_world() {
        // Guards the wiring between the task and the shared world: a run whose tasks held
        // their own copy would emit events forever while `status` never changed.
        let registry = SimRegistry::new();
        let before = start(&registry, 1, 0.02, 60.0).expect("startable");

        let mut stream = registry.subscribe(1).expect("running");
        for _ in 0..20 {
            match tokio::time::timeout(Duration::from_secs(2), stream.recv()).await {
                Ok(Ok(SimEvent::Environment { moved, .. })) if moved > 0 => break,
                Ok(Ok(_)) => continue,
                _ => panic!("the environment stopped ticking"),
            }
        }

        let after = registry.status(1).expect("still running");
        assert!(after.env_tick > before.env_tick);
        assert_ne!(
            after.obs_polygons, before.obs_polygons,
            "the shared world never changed",
        );
        registry.stop(1, "test");
    }
}
