//! The state one simulation run evolves, and the two operations that evolve it.
//!
//! Deliberately synchronous and free of tokio, axum and sea-orm. A run's *schedule* — two
//! tasks on two intervals — lives in [`crate::scheduler`]; what a tick actually does lives
//! here, where it can be tested without a runtime or a database.
//!
//! The split matters because the two operations have different owners in a running
//! simulation. [`SimWorld::advance`] is the environment's, [`plan_route`] is the planner's,
//! and they run on independent clocks. Keeping the mutable state in one small struct is what
//! lets the planner take a consistent copy of the world without blocking the environment for
//! the length of a search.

use crate::models::cell::Cell;
use crate::models::grid_world_manager::GridWorldManager;
use crate::models::obstacle::{ObstaclePoly, advance_one_tick};
use crate::models::planners::{PlanError, PlannerKind};
use crate::models::rng::Xorshift;

/// Everything a run mutates: where the obstacles are, and where the random walk is up to.
///
/// The grid's dimensions ride along because both operations need them — jitter clamps
/// against them and the rasterizer is sized by them — and reading them from the database
/// every tick would put a query in the hot loop for a value that cannot change during a run.
pub(crate) struct SimWorld {
    pub(crate) width: i32,
    pub(crate) height: i32,
    obstacles: Vec<ObstaclePoly>,
    rng: Xorshift,
    /// How many environment ticks have run. Distinct from the planner's count on purpose:
    /// the two clocks are independent, and a plan labelled with the environment tick it was
    /// computed against is what makes "the planner is three ticks behind" legible.
    env_tick: u64,
}

impl SimWorld {
    /// Starts a run from a stored grid's obstacles.
    ///
    /// The seed is the run's identity as far as reproducibility goes — the same seed over the
    /// same starting geometry replays the same walk — so it is taken rather than invented
    /// here, and reported back to the caller by the handler that starts the run.
    pub(crate) fn new(width: i32, height: i32, obstacles: Vec<ObstaclePoly>, seed: u64) -> Self {
        Self {
            width,
            height,
            obstacles,
            rng: Xorshift::new(seed),
            env_tick: 0,
        }
    }

    /// One environment tick: jitters every dynamic obstacle, and reports how many moved.
    ///
    /// The tick counter advances whether or not anything moved, because it counts *time*, not
    /// change — a tick in which every draw clamped against a wall still happened.
    pub(crate) fn advance(&mut self) -> EnvironmentTick {
        let moved = advance_one_tick(&mut self.obstacles, &mut self.rng, self.width, self.height);
        self.env_tick += 1;
        EnvironmentTick {
            tick: self.env_tick,
            moved,
            obstacles: self.obstacles.clone(),
        }
    }

    /// The obstacles as they stand, and the environment tick they belong to.
    ///
    /// Cloned so the caller can plan against them with the lock released. A search is far
    /// longer than a `Vec` copy of a few dozen small polygons, and holding the world for its
    /// duration would stall the environment behind the planner — which is exactly the
    /// coupling two separate schedules exist to avoid.
    pub(crate) fn snapshot(&self) -> (Vec<ObstaclePoly>, u64) {
        (self.obstacles.clone(), self.env_tick)
    }

    /// Whether any obstacle can move at all. A run over nothing but scenery would tick
    /// forever without changing a thing, so the handler refuses to start one.
    pub(crate) fn has_dynamic_obstacles(&self) -> bool {
        self.obstacles.iter().any(|o| o.dynamic)
    }

    /// The next seed in the chain, for reporting a run's continuation point.
    ///
    /// The high half, because xorshift's low bits are its weaker ones.
    pub(crate) fn peek_seed(&self) -> u32 {
        (self.rng.clone().next_u64() >> 32) as u32
    }
}

/// What one environment tick produced.
pub(crate) struct EnvironmentTick {
    pub(crate) tick: u64,
    pub(crate) moved: usize,
    pub(crate) obstacles: Vec<ObstaclePoly>,
}

/// One route, as a replan leaves it.
pub(crate) struct RouteOutcome {
    /// The cells the route runs through, empty when the goal is walled off.
    pub(crate) vertices: Vec<(usize, usize)>,
    pub(crate) reachable: bool,
    pub(crate) cost: u32,
    pub(crate) planner: &'static str,
}

/// Rasterizes the given obstacles and generate route.
///
/// The one place a route is computed from a set of polygons, shared by the manual
/// `POST /grids/{id}/replan` and the scheduled replanner so the two provably agree — a
/// simulation whose stepped and scheduled results differed would be a very confusing thing
/// to debug.
///
/// An unreachable goal is a `RouteOutcome` with no cells rather than an error: an obstacle
/// sealing the goal off is an expected outcome of a run, and the caller wants to see it
/// happen and keep going. Only a malformed endpoint is an `Err`.
pub(crate) fn plan_route(
    width: i32,
    height: i32,
    obstacles: &[ObstaclePoly],
    src: [i32; 2],
    dest: [i32; 2],
    kind: PlannerKind,
) -> Result<RouteOutcome, PlanError> {
    let mut grid_world = GridWorldManager::<Cell>::new(width as usize, height as usize);
    let polygons: Vec<Vec<[i32; 2]>> = obstacles.iter().map(ObstaclePoly::cells).collect();
    grid_world.rasterize_polygons(&polygons, |cell| cell.blocked = true);

    let mut planner = kind.planner();
    let route = match grid_world.find_plan(src, dest, planner.as_mut()) {
        Ok(route) => route,
        Err(PlanError::Unreachable) => Vec::new(),
        Err(err) => return Err(err),
    };

    Ok(RouteOutcome {
        vertices: route.iter().map(|&cell| grid_world.xy(cell)).collect(),
        reachable: !route.is_empty(),
        cost: grid_world.path_cost(&route),
        planner: kind.name(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::obstacle::CellVertex;

    fn square(id: i32, dynamic: bool) -> ObstaclePoly {
        ObstaclePoly {
            id,
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

    #[test]
    fn the_environment_tick_counts_time_rather_than_movement() {
        // A world of pure scenery still advances its clock. The distinction is what lets a
        // client tell "the simulation is running and nothing is moving" from "the simulation
        // has stalled".
        let mut world = SimWorld::new(10, 10, vec![square(1, false)], 7);
        let first = world.advance();
        assert_eq!(first.tick, 1);
        assert_eq!(first.moved, 0);
        assert_eq!(world.advance().tick, 2);
    }

    #[test]
    fn a_run_replays_from_its_seed() {
        // The property the whole seed-chaining design exists for: a run that exposes a
        // planner bug can be reported as one number.
        let run = |seed: u64| {
            let mut world = SimWorld::new(10, 10, vec![square(1, true)], seed);
            for _ in 0..20 {
                world.advance();
            }
            world.snapshot().0
        };
        assert_eq!(run(4242), run(4242));
        assert_ne!(run(4242), run(4243), "the seed made no difference");
    }

    #[test]
    fn a_snapshot_does_not_advance_the_world() {
        // The planner takes snapshots on its own schedule. If taking one moved the walk, the
        // run would no longer replay from its seed, and the two frequencies would be coupled
        // through the random number generator of all things.
        let mut world = SimWorld::new(10, 10, vec![square(1, true)], 99);
        world.advance();
        let (obstacles, tick) = world.snapshot();
        assert_eq!(world.snapshot().0, obstacles);
        assert_eq!(world.snapshot().1, tick);
    }

    #[test]
    fn a_walled_off_goal_is_a_route_with_no_cells() {
        // A wall clean across a narrow grid. Not an error — the client wants to watch the
        // goal get sealed off and keep ticking.
        let wall = ObstaclePoly {
            id: 1,
            dynamic: false,
            vertices: vec![
                CellVertex { x: 0, y: 4 },
                CellVertex { x: 9, y: 4 },
                CellVertex { x: 9, y: 5 },
                CellVertex { x: 0, y: 5 },
                CellVertex { x: 0, y: 4 },
            ],
        };
        let outcome = plan_route(10, 10, &[wall], [0, 0], [9, 9], PlannerKind::DStarLite)
            .expect("legal ends");
        assert!(!outcome.reachable);
        assert!(outcome.vertices.is_empty());
    }

    #[test]
    fn a_malformed_endpoint_is_still_an_error() {
        // The line between "the world did this" and "you asked for something impossible".
        let result = plan_route(10, 10, &[], [0, 0], [99, 99], PlannerKind::DStarLite);
        assert_eq!(result.err(), Some(PlanError::DestOffGrid));
    }

    #[test]
    fn a_world_of_scenery_reports_nothing_dynamic() {
        assert!(!SimWorld::new(10, 10, vec![square(1, false)], 1).has_dynamic_obstacles());
        assert!(SimWorld::new(10, 10, vec![square(1, true)], 1).has_dynamic_obstacles());
    }
}
