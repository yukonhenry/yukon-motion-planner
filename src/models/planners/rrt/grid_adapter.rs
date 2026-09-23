//! SST behind the [`Planner`](crate::models::planners::Planner) trait.
//!
//! The trait is written for grid search — cells in, cells out — and SST is neither. This
//! module is the seam, and it is worth being explicit about what each side gives up.
//!
//! **Going in**, the trait hands over a rasterized grid rather than the polygons behind it, so
//! the collision world is rebuilt from the blocked cells. That matches what the grid planners
//! see, which is the point.
//!
//! **Coming out**, a trajectory is projected onto the cells it passes through and the rest —
//! heading, speed, sub-cell position — is dropped on the floor. That is a real loss, and it is
//! deliberate for now: it keeps every existing caller, route row and frontend view working
//! unchanged while the planner itself is proven. Carrying the trajectory through to the client
//! is the animation work, and it is where the discard should stop.

use crate::models::cell::Cell;
use crate::models::grid_world_manager::{GridWorldManager, NodeId};
use crate::models::planners::rrt::kino_dynamic_edge_generator::KinodynamicEdgeGenerator;
use crate::models::planners::rrt::planning_world::PlanningWorld;
use crate::models::planners::rrt::sst::{Bounds, SearchStats, SstParams, SstPlanner, SstSolution};
use crate::models::planners::{PlanError, Planner};
use crate::models::robots::unicycle_spec::UnicycleSpec;
use crate::models::robots::unicycle_state::{SecondOrderUnicycle, UnicycleState};
use crate::models::scale::{METERS_PER_CELL, cell_to_point, point_to_cell};
use rand::SeedableRng;
use rand::rngs::StdRng;

/// The integration step the tree is grown with, in seconds.
///
/// 20ms: fine enough that a step at full speed covers 4cm — a twenty-fifth of a cell — so the
/// swept body cannot tunnel through anything the collision world holds.
const DT: f64 = 0.02;

pub(crate) struct Sst {
    planner: SstPlanner,
    rng: StdRng,
    /// The trajectory behind the most recent plan, kept whole.
    ///
    /// `&mut self` on the trait exists because D\* Lite carries state between calls; this uses
    /// the same door for a different reason. The cells returned to the caller are a projection
    /// of this, and once the wire format can carry poses it is this that should travel, not the
    /// cells.
    last_trajectory: Option<SstSolution>,
    /// Where the last search went, for the benchmarks below and for anything that wants to
    /// report planner effort alongside a route.
    pub(crate) last_stats: Option<SearchStats>,
}

impl Sst {
    pub(crate) fn new(robot: UnicycleSpec, seed: u64) -> Self {
        Self {
            planner: SstPlanner::new(
                {
                    let mut generator =
                        KinodynamicEdgeGenerator::new(SecondOrderUnicycle { spec: robot }, DT);
                    generator.num_control_samples = 3;
                    generator
                },
                SstParams {
                    // One cell. Tighter than this — half a cell, so a solution ends standing
                    // exactly on `dest` — measured no better and is markedly harder to hit,
                    // since a rollout has to *land* in the ball rather than pass through it.
                    // `plan` walks the last cell instead.
                    goal_radius: METERS_PER_CELL,
                    // Measured, not guessed. Per unit of wall-clock, a cheap iteration beats a
                    // thorough one: on a three-cell doorway, three control samples at 30k
                    // iterations solves 7/8 seeds in ~1.1s where fifteen samples at 5k solves
                    // 6/8 in ~0.6s and needs 15k and ~1.9s to reach 8/8. That also happens to
                    // be the shape SST is specified in — a single Monte-Carlo propagation per
                    // iteration — rather than best-of-N, which came from the edge generator.
                    iterations: 30_000,
                    ..SstParams::default()
                },
            ),
            rng: StdRng::seed_from_u64(seed),
            last_trajectory: None,
            last_stats: None,
        }
    }

    /// The trajectory behind the last plan, if there was one.
    pub(crate) fn last_trajectory(&self) -> Option<&SstSolution> {
        self.last_trajectory.as_ref()
    }

    /// Projects a trajectory onto the cells it crosses.
    ///
    /// Consecutive duplicates are dropped, so the result is a path rather than one entry per
    /// 20ms tick. At full speed a step covers a twenty-fifth of a cell, so successive kept
    /// cells are always neighbors and the path never teleports.
    ///
    /// Not guaranteed to satisfy the grid's corner rule, though: a trajectory that rounds the
    /// outside of an obstacle legitimately, with the body clearing it, can project onto a
    /// diagonal pair the 8-connected model would forbid. The robot really can drive it — the
    /// continuous check said so — so the cells are the approximation here, not the motion.
    fn to_cells(
        world: &GridWorldManager<Cell>,
        states: &[UnicycleState],
    ) -> Vec<NodeId> {
        let mut route: Vec<NodeId> = Vec::new();

        for state in states {
            let [x, y] = point_to_cell(state.x, state.y);
            let Some(id) = world.try_id(x as isize, y as isize) else {
                continue; // numerically off the edge; the boundary wall makes this vanishing
            };
            if route.last() != Some(&id) {
                route.push(id);
            }
        }

        route
    }
}

impl Planner for Sst {
    fn plan(
        &mut self,
        world: &GridWorldManager<Cell>,
        src: NodeId,
        dest: NodeId,
    ) -> Result<Vec<NodeId>, PlanError> {
        let (src_x, src_y) = world.xy(src);
        let (dest_x, dest_y) = world.xy(dest);

        // The robot starts at the centre of its cell, stopped and facing along +x.
        //
        // A resting start is what the grid planners implicitly assume and what a fresh run
        // begins from. Replanning a robot that is already moving means threading its live
        // state through instead, which is the same change that carries the trajectory out.
        let (x, y) = cell_to_point([src_x as i32, src_y as i32]);
        let start = UnicycleState {
            x,
            y,
            theta: 0.0,
            v: 0.0,
            omega: 0.0,
        };

        let goal = cell_to_point([dest_x as i32, dest_y as i32]);
        let collision_world = PlanningWorld::from_grid(world);
        let bounds = Bounds::from_grid(world.width() as i32, world.height() as i32);

        let outcome = self.planner.plan(
            start,
            [goal.0, goal.1],
            &collision_world,
            bounds,
            &mut self.rng,
        );

        self.last_stats = Some(outcome.stats);
        let Some(solution) = outcome.solution else {
            self.last_trajectory = None;
            return Err(PlanError::Unreachable);
        };

        let mut route = Self::to_cells(world, &solution.states);

        // The trait's contract is that a route ends at `dest`, and the planner's goal region is
        // a cell-wide ball around it — so the trajectory can legitimately stop one cell short.
        // The final step is therefore the *projection's*, not the planner's: the robot really is
        // within a metre of the goal, and this walks it the last cell.
        //
        // Safe as a step because `goal_radius` is one cell, so the last state can be at most one
        // cell away in each axis. Asserted rather than assumed, since raising `goal_radius`
        // without revisiting this would start emitting routes that teleport at the end.
        if route.last() != Some(&dest) {
            if let Some(&last) = route.last() {
                let (a, b) = (world.xy(last), world.xy(dest));
                debug_assert!(
                    a.0.abs_diff(b.0) <= 1 && a.1.abs_diff(b.1) <= 1,
                    "a solution ended {a:?}, too far from dest {b:?} to close in one step",
                );
            }
            route.push(dest);
        }
        if route.first() != Some(&src) {
            route.insert(0, src);
        }

        self.last_trajectory = Some(solution);
        Ok(route)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::planners::test_support::{empty_world, world_from_ascii};

    /// A planner on a budget the tests can afford.
    ///
    /// 5_000 rather than the 30_000 default. Measured over ten seeds, every scenario below
    /// solves 10/10 at this budget in about 165ms — the default exists for maps harder than
    /// these, and paying for it six times over would make the suite crawl for no extra signal.
    fn sst() -> Sst {
        sst_with(5_000)
    }

    fn sst_with(iterations: usize) -> Sst {
        let mut planner = Sst::new(UnicycleSpec::default(), 20);
        planner.planner.params.iterations = iterations;
        planner
    }

    /// Open ground with a two-by-two pillar between the corners.
    fn pillar_world() -> GridWorldManager<Cell> {
        world_from_ascii(&[
            "............", "............", "....##......", "....##......",
            "............", "............", "............", "............",
            "............", "............", "............", "............",
        ])
    }

    #[test]
    fn a_plan_runs_from_src_to_dest_over_passable_cells() {
        // The trait's contract, which is all any caller of `find_plan` relies on. Deliberately
        // not checked against an optimal cost: SST minimizes *duration* under the dynamics,
        // and expecting it to match an 8-connected shortest path would be asking a different
        // question of it.
        let world = empty_world(12, 12);
        let route = sst()
            .plan(&world, world.id(1, 1), world.id(9, 9))
            .expect("open ground should be solvable");

        assert_eq!(route.first(), Some(&world.id(1, 1)), "plan does not start at src");
        assert_eq!(route.last(), Some(&world.id(9, 9)), "plan does not end at dest");
        for &node in &route {
            assert!(world.passable(node), "plan crosses {:?}", world.xy(node));
        }
    }

    #[test]
    fn consecutive_cells_are_neighbors() {
        // The projection drops repeats, so what is left has to be a walk. At 2 m/s a 20ms step
        // covers 4cm against a 1m cell, so a gap here would mean the trajectory itself jumped
        // — not that the sampling was too coarse. This also covers the final appended step,
        // which is the one place `plan` adds a cell the trajectory did not visit.
        let world = empty_world(12, 12);
        let route = sst().plan(&world, world.id(1, 1), world.id(9, 9)).unwrap();

        for pair in route.windows(2) {
            let (a, b) = (world.xy(pair[0]), world.xy(pair[1]));
            let (dx, dy) = (a.0.abs_diff(b.0), a.1.abs_diff(b.1));
            assert!(dx <= 1 && dy <= 1 && (dx + dy) > 0, "{a:?} -> {b:?} is not a step");
        }
    }

    #[test]
    fn the_trajectory_behind_a_plan_is_kept() {
        // The cells are a projection; this is the thing itself. Held on the planner so that
        // carrying poses to the client later needs no second search.
        let world = empty_world(12, 12);
        let mut planner = sst();
        assert!(planner.last_trajectory().is_none(), "nothing planned yet");

        let route = planner.plan(&world, world.id(1, 1), world.id(9, 9)).unwrap();
        let trajectory = planner.last_trajectory().expect("a plan leaves its trajectory");

        assert!(
            trajectory.states.len() >= route.len(),
            "a trajectory is sampled far finer than the cells it crosses",
        );
        assert!(trajectory.cost > 0.0, "a real motion takes time");
        assert_eq!(
            point_to_cell(trajectory.states[0].x, trajectory.states[0].y),
            [1, 1],
            "the trajectory is not the one for this plan",
        );
    }

    #[test]
    fn a_walled_off_goal_is_unreachable_rather_than_wrong() {
        // The same answer A* and D* Lite give, so `plan_route` handles all three identically
        // and a sealed goal stays a route with no cells rather than an error.
        let world = world_from_ascii(&[
            "..........", "..........", "##########", "..........", "..........",
        ]);
        let mut planner = sst_with(2_000);
        assert_eq!(
            planner.plan(&world, world.id(1, 1), world.id(8, 4)),
            Err(PlanError::Unreachable),
        );
        assert!(planner.last_trajectory().is_none(), "a failed plan leaves no trajectory");
    }

    #[test]
    fn a_plan_keeps_off_the_obstacles_it_was_given() {
        // Verifies the continuous collision world really was built from this grid: a route
        // that crossed the pillar would mean `from_grid` missed it.
        //
        // A pillar in open ground rather than a wall with a gap, because the two test
        // different things. This one is about the obstacles being *seen*; threading a narrow
        // gap is about the planner's search power, which is weaker — see the module docs.
        let world = pillar_world();
        let route = sst()
            .plan(&world, world.id(1, 1), world.id(9, 9))
            .expect("a pillar in open ground is not a wall");

        for &node in &route {
            assert!(world.passable(node), "plan crosses the pillar at {:?}", world.xy(node));
        }
        assert!(
            route.iter().any(|&n| world.xy(n).1 > 3),
            "the route never got past the pillar's row",
        );
    }

    #[test]
    fn a_seed_replays_the_same_plan() {
        let world = empty_world(12, 12);
        let (src, dest) = (world.id(1, 1), world.id(9, 9));
        assert_eq!(sst().plan(&world, src, dest), sst().plan(&world, src, dest));
    }

    #[test]
    fn successive_plans_from_one_planner_differ() {
        // The rng lives on the planner and advances, so a second call is a fresh search rather
        // than a replay of the first. Without that, a replanning loop would keep proposing the
        // identical route however much the world had changed.
        let world = empty_world(12, 12);
        let mut planner = sst();
        let (src, dest) = (world.id(1, 1), world.id(9, 9));

        let first = planner.plan(&world, src, dest).unwrap();
        let second = planner.plan(&world, src, dest).unwrap();
        assert_ne!(first, second, "the planner replayed its own previous search");
    }

    // --- instrumentation harness ------------------------------------------------
    //
    //   cargo test --release --lib sst_bench -- --ignored --nocapture
    //
    // Release matters: the same sweep runs about 4x slower under `cargo test` alone, which is
    // enough to point an investigation at the wrong thing.
    //
    // Emits CSV on stdout. Redirect it somewhere and plot:
    //   cargo test --release --lib sst_bench -- --ignored --nocapture | grep ^csv, > runs.csv

    /// The maps the sweep runs over. Add rows here rather than writing new harnesses.
    fn scenarios() -> Vec<(&'static str, GridWorldManager<Cell>, (usize, usize), (usize, usize))> {
        vec![
            ("open", empty_world(12, 12), (1, 1), (9, 9)),
            (
                "pillar",
                world_from_ascii(&[
                    "............", "............", "....##......", "....##......",
                    "............", "............", "............", "............",
                    "............", "............", "............", "............",
                ]),
                (1, 1),
                (9, 9),
            ),
            (
                // The case that stalls: the goal is behind a wall with a six-cell opening, and
                // more iterations barely help. If `closest` plateaus here while `nodes` keeps
                // climbing, the tree is stuck against the wall rather than short of budget.
                "wall_wide_gap",
                world_from_ascii(&[
                    "............", "............", "............",
                    "######......", "............", "............",
                ]),
                (1, 1),
                (1, 5),
            ),
            (
                "wall_narrow_gap",
                world_from_ascii(&[
                    "............", "............", "............",
                    "#########...", "............", "............",
                ]),
                (1, 1),
                (1, 5),
            ),
        ]
    }

    #[test]
    #[ignore = "benchmark: cargo test --release --lib sst_bench -- --ignored --nocapture"]
    fn sst_bench() {
        let budgets = [2_000usize, 5_000, 15_000, 30_000];
        let seeds = 0..10u64;

        println!(
            "csv,scenario,iterations,seed,solved,cost_s,wall_ms,nodes,created,pruned,witnesses,\
collisions,select_ms,propagate_ms,book_ms,failed,dominated,fallbacks,closest_m,first_hit_iter,first_hit_ms,improvements"
        );

        for (name, world, src, dest) in scenarios() {
            for &iterations in &budgets {
                for seed in seeds.clone() {
                    let mut planner = Sst::new(UnicycleSpec::default(), seed);
                    planner.planner.params.iterations = iterations;

                    let started = std::time::Instant::now();
                    let result = planner.plan(&world, world.id(src.0, src.1), world.id(dest.0, dest.1));
                    let wall = started.elapsed();

                    let stats = planner.last_stats.as_ref().expect("a plan leaves its stats");
                    let first = stats.improvements.first();
                    let ms = |d: std::time::Duration| d.as_secs_f64() * 1e3;

                    println!(
                        "csv,{name},{iterations},{seed},{},{:.3},{:.1},{},{},{},{},{},{:.1},{:.1},{:.1},{},{},{},{:.3},{},{:.1},{}",
                        u8::from(result.is_ok()),
                        planner.last_trajectory().map_or(0.0, |t| t.cost),
                        ms(wall),
                        stats.nodes,
                        stats.nodes_created,
                        stats.nodes_pruned,
                        stats.witnesses,
                        stats.collision_queries,
                        ms(stats.selection),
                        ms(stats.propagation),
                        ms(stats.bookkeeping),
                        stats.extensions_failed,
                        stats.dominated,
                        stats.selection_fallbacks,
                        stats.closest_approach,
                        first.map_or(-1, |i| i.iteration as i64),
                        first.map_or(0.0, |i| ms(i.elapsed)),
                        stats.improvements.len(),
                    );
                }
            }
        }
    }

    /// The anytime curve on one scenario: how cost falls as the search runs.
    ///
    /// Separate from the sweep because it answers a different question — not "how long does a
    /// budget take" but "how much of that budget was worth spending". The gap between the last
    /// improvement and the end of the search is what a deadline-based planner would reclaim.
    #[test]
    #[ignore = "benchmark: cargo test --release --lib sst_anytime -- --ignored --nocapture"]
    fn sst_anytime() {
        println!("csv,scenario,seed,iteration,elapsed_ms,cost_s");
        for (name, world, src, dest) in scenarios() {
            for seed in 0..5u64 {
                let mut planner = Sst::new(UnicycleSpec::default(), seed);
                planner.planner.params.iterations = 30_000;
                planner.planner.params.progress_every = 500;
                let _ = planner.plan(&world, world.id(src.0, src.1), world.id(dest.0, dest.1));

                let stats = planner.last_stats.as_ref().unwrap();
                for improvement in &stats.improvements {
                    println!(
                        "csv,{name},{seed},{},{:.1},{:.3}",
                        improvement.iteration,
                        improvement.elapsed.as_secs_f64() * 1e3,
                        improvement.cost,
                    );
                }
                let unused = stats.iterations - stats.improvements.last().map_or(0, |i| i.iteration);
                println!(
                    "# {name} seed={seed}: {} improvements, {unused} iterations after the last one, closest {:.2}m",
                    stats.improvements.len(),
                    stats.closest_approach,
                );
            }
        }
    }

    /// Whether the tree stalls or merely runs out of budget, per scenario.
    #[test]
    #[ignore = "benchmark: cargo test --release --lib sst_progress -- --ignored --nocapture"]
    fn sst_progress() {
        println!("csv,scenario,seed,iteration,closest_m");
        for (name, world, src, dest) in scenarios() {
            for seed in 0..3u64 {
                let mut planner = Sst::new(UnicycleSpec::default(), seed);
                planner.planner.params.iterations = 30_000;
                planner.planner.params.progress_every = 1_000;
                let _ = planner.plan(&world, world.id(src.0, src.1), world.id(dest.0, dest.1));

                for (iteration, closest) in &planner.last_stats.as_ref().unwrap().progress {
                    println!("csv,{name},{seed},{iteration},{closest:.3}");
                }
            }
        }
    }
}
