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
use crate::models::obstacle::{ObstaclePoly, advance_one_tick, footprint};
use crate::models::robot::RobotBody;
use crate::models::planners::{PlanError, PlannerContext, PlannerKind};
use crate::models::scale::inflation_cells;
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
    /// The machine crossing this world.
    ///
    /// Shared state rather than the robot task's own, because a run's status has to report
    /// where the robot is to a client that joined late — the same reason the obstacles live
    /// here rather than in the environment task.
    robot: RobotBody,
}

impl SimWorld {
    /// Starts a run from a stored grid's obstacles.
    ///
    /// The seed is the run's identity as far as reproducibility goes — the same seed over the
    /// same starting geometry replays the same walk — so it is taken rather than invented
    /// here, and reported back to the caller by the handler that starts the run.
    pub(crate) fn new(
        width: i32,
        height: i32,
        obstacles: Vec<ObstaclePoly>,
        seed: u64,
        robot: RobotBody,
    ) -> Self {
        Self {
            width,
            height,
            obstacles,
            rng: Xorshift::new(seed),
            env_tick: 0,
            robot,
        }
    }

    /// Replaces the robot's route after a replan. It does not move; it just knows more.
    pub(crate) fn follow_route(&mut self, route: Vec<(usize, usize)>) {
        self.robot.follow(route);
    }

    /// Walks the robot along its route, stopping short of anything now in the way.
    ///
    /// The obstacle check lives here rather than in the caller because this is the one place
    /// that holds both the robot and the world it is crossing — and the cells it tests are
    /// rasterized by [`footprint`], the same fill the planner blocks on, so the robot can
    /// never stop at a cell the planner would have routed it through.
    pub(crate) fn step_robot(&mut self, max_velocity: f64) -> usize {
        let (width, height) = (self.width, self.height);
        let occupied: std::collections::HashSet<(usize, usize)> = self
            .obstacles
            .iter()
            .flat_map(|o| footprint(o, width, height))
            .collect();

        self.robot.advance(max_velocity, |[x, y]| {
            x >= 0 && y >= 0 && occupied.contains(&(x as usize, y as usize))
        })
    }

    /// Where the robot is standing, and whether that is its goal.
    pub(crate) fn robot_at(&self) -> ([i32; 2], bool) {
        (self.robot.position, self.robot.arrived())
    }

    /// Where the robot is trying to get to. Fixed for the life of a run.
    pub(crate) fn robot_dest(&self) -> [i32; 2] {
        self.robot.dest
    }

    /// One environment tick: jitters every dynamic obstacle, and reports how many moved.
    ///
    /// The tick counter advances whether or not anything moved, because it counts *time*, not
    /// change — a tick in which every draw clamped against a wall still happened.
    pub(crate) fn advance(&mut self) -> EnvironmentTick {
        // The robot's cell is passed in so a translating obstacle refuses to drive over it.
        let report = advance_one_tick(
            &mut self.obstacles,
            &mut self.rng,
            self.width,
            self.height,
            Some(self.robot.position),
        );
        self.env_tick += 1;
        EnvironmentTick {
            tick: self.env_tick,
            moved: report.moved,
            blocked: report.blocked,
            robot_hits: report.robot_hits,
            robot_position: self.robot.position,
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
    /// Obstacles that could not move: something was in the way, or the grid edge was.
    pub(crate) blocked: Vec<i32>,
    /// Obstacles that stopped short of running the robot over.
    pub(crate) robot_hits: Vec<i32>,
    /// Where the robot was standing during this tick.
    pub(crate) robot_position: [i32; 2],
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
/// Grows every blocked region by `cells` in each direction, turning the world into the robot's
/// configuration space.
///
/// Once the obstacles carry the robot's radius, a *point* searching this grid is an exact model
/// of the body searching the original one — which is what lets a grid planner and a
/// kinodynamic planner be compared, rather than quietly solving different problems. See
/// [`inflation_cells`](crate::models::scale::inflation_cells) for where the number comes from;
/// at the current scale it is zero for the default robot, and this is a no-op.
///
/// Reads the blocked set out before writing any of it back, because dilating in place would
/// feed freshly-inflated cells back into the same pass and grow the obstacle without bound.
fn inflate(grid_world: &mut GridWorldManager<Cell>, cells: i32) {
    if cells <= 0 {
        return;
    }

    let blocked: Vec<(usize, usize)> = grid_world
        .iter()
        .filter(|(_, cell)| cell.blocked)
        .map(|(id, _)| grid_world.xy(id))
        .collect();

    for (x, y) in blocked {
        for dy in -cells..=cells {
            for dx in -cells..=cells {
                let (nx, ny) = (x as isize + dx as isize, y as isize + dy as isize);
                if let Some(id) = grid_world.try_id(nx, ny) {
                    grid_world[id].blocked = true;
                }
            }
        }
    }
}

pub(crate) fn plan_route(
    width: i32,
    height: i32,
    obstacles: &[ObstaclePoly],
    src: [i32; 2],
    dest: [i32; 2],
    kind: PlannerKind,
    context: PlannerContext,
) -> Result<RouteOutcome, PlanError> {
    let mut grid_world = GridWorldManager::<Cell>::new(width as usize, height as usize);
    let polygons: Vec<Vec<[i32; 2]>> = obstacles.iter().map(ObstaclePoly::cells).collect();
    grid_world.rasterize_polygons(&polygons, |cell| cell.blocked = true);

    // Clearance is the planner's to ask for, not the caller's to remember: a point searcher
    // needs it baked into the world, and one that sweeps a real body would be paying for the
    // same radius twice. Deciding it here means neither caller can get the pairing wrong.
    if kind.inflates_obstacles() {
        let clearance = context
            .robot
            .map_or(0, |robot| inflation_cells(robot.radius()));
        inflate(&mut grid_world, clearance);
    }

    let mut planner = kind.planner(context);
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
    use crate::models::robots::unicycle_spec::UnicycleSpec;
    use crate::models::scale::METERS_PER_CELL;

    /// The manual-replan context: nothing to clear obstacles by.
    fn no_robot() -> PlannerContext {
        PlannerContext {
            robot: None,
            seed: 0,
        }
    }

    /// A robot whose radius rounds to exactly `clearance` cells of inflation.
    ///
    /// Built backwards from the wanted clearance rather than from a plausible machine, because
    /// these tests are about what inflation does to a search and the robot is only how it gets
    /// asked for. See `inflation_cells` for the half-cell the radius has to clear first.
    fn robot_clearing(clearance: i32) -> PlannerContext {
        let radius = if clearance <= 0 {
            0.1
        } else {
            METERS_PER_CELL * (clearance as f64 - 0.5) + METERS_PER_CELL / 2.0 + 1e-9
        };
        PlannerContext {
            robot: Some(UnicycleSpec {
                track_width: radius * 2.0,
                ..UnicycleSpec::default()
            }),
            seed: 0,
        }
    }

    /// A robot that is not the subject of these tests: `SimWorld` needs one, but nothing here
    /// moves it, so it sits at the origin with somewhere else to be.
    fn parked() -> RobotBody {
        RobotBody::new([0, 0], [9, 9], Vec::new())
    }

    fn square(id: i32, dynamic: bool) -> ObstaclePoly {
        ObstaclePoly {
            id,
            dynamic,
            velocity: [0, 0],
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
        let mut world = SimWorld::new(10, 10, vec![square(1, false)], 7, parked());
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
            let mut world = SimWorld::new(10, 10, vec![square(1, true)], seed, parked());
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
        let mut world = SimWorld::new(10, 10, vec![square(1, true)], 99, parked());
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
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 0, y: 4 },
                CellVertex { x: 9, y: 4 },
                CellVertex { x: 9, y: 5 },
                CellVertex { x: 0, y: 5 },
                CellVertex { x: 0, y: 4 },
            ],
        };
        let outcome = plan_route(10, 10, &[wall], [0, 0], [9, 9], PlannerKind::DStarLite, no_robot())
            .expect("legal ends");
        assert!(!outcome.reachable);
        assert!(outcome.vertices.is_empty());
    }

    #[test]
    fn clearance_seals_a_gap_too_narrow_for_the_body_to_fit_through() {
        // What inflation buys, stated as the behavior rather than as a cell count: a pillar
        // leaving a one-cell doorway is a route for a point and a wall for anything wider
        // than the doorway. Without this the planner hands a fat robot a path through a gap
        // it would wedge in, and the run only discovers it by driving into the shape.
        //
        // Two pillars leaving a single free cell at x == 4 on row 4.
        let pillar = |id: i32, x0: i32, x1: i32| ObstaclePoly {
            id,
            dynamic: false,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: x0, y: 4 },
                CellVertex { x: x1, y: 4 },
                CellVertex { x: x1, y: 5 },
                CellVertex { x: x0, y: 5 },
                CellVertex { x: x0, y: 4 },
            ],
        };
        let wall = [pillar(1, 0, 3), pillar(2, 5, 9)];
        let plan = |clearance| {
            plan_route(10, 10, &wall, [0, 0], [9, 9], PlannerKind::DStarLite, robot_clearing(clearance))
                .expect("legal ends")
        };

        // A point robot walks through the doorway.
        let open = plan(0);
        assert!(open.reachable, "a point should fit through a one-cell gap");
        assert!(
            open.vertices.contains(&(4, 4)),
            "the only way through is the gap itself, got {:?}",
            open.vertices,
        );

        // A body needing a cell of clearance does not: inflating both pillars closes the gap
        // from either side, and the goal is genuinely unreachable rather than merely dearer.
        let sealed = plan(1);
        assert!(!sealed.reachable, "a one-cell gap cannot pass a body that needs one cell either side");
        assert!(sealed.vertices.is_empty());
    }

    #[test]
    fn only_the_point_searchers_are_handed_inflated_obstacles() {
        // `PlannerKind::inflates_obstacles` is a flag; this is `plan_route` obeying it. Without
        // that the kinodynamic planner clears the robot's radius twice — once in the grid it is
        // given, once by sweeping the body — and quietly seals gaps the robot fits through.
        //
        // Read off the endpoint checks rather than off a route, so it is exact and costs no
        // search at all. A wall one cell wide at x = 5, a start beside it, and a goal inside
        // it: `find_plan` tests src before dest, so which error comes back says which world
        // the planner was handed.
        let wall = ObstaclePoly {
            id: 1,
            dynamic: false,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 5, y: 0 },
                CellVertex { x: 5, y: 9 },
                CellVertex { x: 5, y: 9 },
                CellVertex { x: 5, y: 0 },
            ],
        };
        // Radius 0.6 m, which is one cell of inflation: enough to swallow the start.
        let context = PlannerContext {
            robot: Some(UnicycleSpec {
                track_width: 1.2,
                ..UnicycleSpec::default()
            }),
            seed: 0,
        };
        let plan = |kind| plan_route(10, 10, &[wall.clone()], [4, 1], [5, 5], kind, context);

        assert_eq!(
            plan(PlannerKind::DStarLite).err(),
            Some(PlanError::SrcBlocked),
            "a point searcher should have had the wall grown over its start",
        );
        assert_eq!(
            plan(PlannerKind::Sst).err(),
            Some(PlanError::DestBlocked),
            "SST was handed inflated obstacles: its start should still be clear, \
             leaving the goal inside the wall as the first real problem",
        );
    }

    #[test]
    fn clearance_keeps_a_route_that_is_merely_dearer() {
        // The other half: inflation must not turn every obstacle into a wall. A pillar in
        // open ground still leaves a way round, just a longer one.
        let pillar = ObstaclePoly {
            id: 1,
            dynamic: false,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 4, y: 4 },
                CellVertex { x: 5, y: 4 },
                CellVertex { x: 5, y: 5 },
                CellVertex { x: 4, y: 5 },
                CellVertex { x: 4, y: 4 },
            ],
        };
        let plan = |clearance| {
            plan_route(20, 20, &[pillar.clone()], [0, 0], [19, 19], PlannerKind::DStarLite, robot_clearing(clearance))
                .expect("legal ends")
        };

        let (tight, roomy) = (plan(0), plan(2));
        assert!(roomy.reachable, "a pillar in open ground is not a wall");
        assert!(
            roomy.cost >= tight.cost,
            "giving the robot room can only cost more, not less: {} < {}",
            roomy.cost,
            tight.cost,
        );
    }

    #[test]
    fn inflating_does_not_grow_an_obstacle_without_bound() {
        // Dilating in place would feed each freshly-blocked cell back into the same pass and
        // swallow the grid. Pinned by the reachability of a corner far from the only shape:
        // a runaway inflation blocks it, a correct one leaves it untouched.
        let pillar = ObstaclePoly {
            id: 1,
            dynamic: false,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 5, y: 5 },
                CellVertex { x: 6, y: 5 },
                CellVertex { x: 6, y: 6 },
                CellVertex { x: 5, y: 6 },
                CellVertex { x: 5, y: 5 },
            ],
        };
        let outcome = plan_route(20, 20, &[pillar], [0, 0], [19, 19], PlannerKind::DStarLite, robot_clearing(3))
            .expect("legal ends");
        assert!(outcome.reachable, "inflation swallowed the grid");
    }

    #[test]
    fn a_malformed_endpoint_is_still_an_error() {
        // The line between "the world did this" and "you asked for something impossible".
        let result = plan_route(10, 10, &[], [0, 0], [99, 99], PlannerKind::DStarLite, no_robot());
        assert_eq!(result.err(), Some(PlanError::DestOffGrid));
    }

    #[test]
    fn a_world_of_scenery_reports_nothing_dynamic() {
        assert!(!SimWorld::new(10, 10, vec![square(1, false)], 1, parked()).has_dynamic_obstacles());
        assert!(SimWorld::new(10, 10, vec![square(1, true)], 1, parked()).has_dynamic_obstacles());
    }
}
