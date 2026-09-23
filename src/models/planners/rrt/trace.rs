//! Running one SST search purely to watch it, rather than to drive a robot.
//!
//! Separate from [`grid_adapter`](super::grid_adapter) because the two want opposite things.
//! The adapter exists to hand a route to the rest of the system and throws the trajectory away
//! at the boundary; this exists to keep everything — every node, every prune, every pose — and
//! hands back no route at all.
//!
//! Deliberately offline and one-shot. A search is a few hundred milliseconds and a few
//! megabytes of trace, so it is a request-response, not a stream: the caller gets a fixed
//! timeline it can scrub back and forth over, and a seed replays it exactly.

use crate::models::cell::Cell;
use crate::models::grid_world_manager::GridWorldManager;
use crate::models::obstacle::ObstaclePoly;
use crate::models::planners::rrt::kino_dynamic_edge_generator::KinodynamicEdgeGenerator;
use crate::models::planners::rrt::planning_world::PlanningWorld;
use crate::models::planners::rrt::sst::{
    Bounds, SearchEvent, SearchStats, SstParams, SstPlanner,
};
use crate::models::robots::unicycle_spec::UnicycleSpec;
use crate::models::robots::unicycle_state::{SecondOrderUnicycle, UnicycleState};
use crate::models::scale::cell_to_point;
use rand::SeedableRng;
use rand::rngs::StdRng;

/// How many iterations a traced search runs when the caller does not say.
///
/// Lower than the planner's own default. A trace is for looking at, and the interesting part —
/// the tree spreading and branches being culled — is all in the first few thousand iterations;
/// past that the picture stops changing while the file keeps growing.
pub(crate) const DEFAULT_TRACE_ITERATIONS: usize = 5_000;

/// The ceiling a request cannot raise past, so one call cannot ask for a gigabyte.
pub(crate) const MAX_TRACE_ITERATIONS: usize = 50_000;

/// Everything one traced search produced.
pub(crate) struct TracedSearch {
    pub events: Vec<SearchEvent>,
    pub stats: SearchStats,
    /// The best trajectory found, as `[x, y, theta]` poses. Empty when none was.
    pub solution: Vec<[f64; 3]>,
    /// Duration of that trajectory, in seconds.
    pub solution_cost: f64,
    /// Where the search started and aimed, in metres — so a viewer can draw them without
    /// redoing the cell-to-metre conversion and risking a different answer.
    pub start: [f64; 3],
    pub goal: [f64; 2],
}

/// Runs one SST search over `obstacles` and records how it went.
///
/// CPU-bound and typically a few hundred milliseconds, so callers on an async runtime must put
/// this behind `spawn_blocking`. It takes no database handle and no runtime of its own,
/// precisely so that is possible.
pub(crate) fn trace_search(
    width: i32,
    height: i32,
    obstacles: &[ObstaclePoly],
    src: [i32; 2],
    dest: [i32; 2],
    robot: UnicycleSpec,
    seed: u64,
    iterations: usize,
) -> TracedSearch {
    // Rasterized exactly as `plan_route` would, so the shapes a viewer sees blocked are the
    // ones the search actually ran against. Not inflated: SST sweeps the real body, and
    // growing the obstacles as well would clear the same radius twice.
    let mut grid = GridWorldManager::<Cell>::new(width.max(0) as usize, height.max(0) as usize);
    let polygons: Vec<Vec<[i32; 2]>> = obstacles.iter().map(ObstaclePoly::cells).collect();
    grid.rasterize_polygons(&polygons, |cell| cell.blocked = true);

    let world = PlanningWorld::from_grid(&grid);
    let (start_x, start_y) = cell_to_point(src);
    let (goal_x, goal_y) = cell_to_point(dest);
    let start = UnicycleState {
        x: start_x,
        y: start_y,
        theta: 0.0,
        v: 0.0,
        omega: 0.0,
    };

    let planner = SstPlanner::new(
        {
            let mut generator =
                KinodynamicEdgeGenerator::new(SecondOrderUnicycle { spec: robot }, 0.02);
            generator.num_control_samples = 3;
            generator
        },
        SstParams {
            iterations: iterations.clamp(1, MAX_TRACE_ITERATIONS),
            trace: true,
            goal_radius: crate::models::scale::METERS_PER_CELL,
            ..SstParams::default()
        },
    );

    let outcome = planner.plan(
        start,
        [goal_x, goal_y],
        &world,
        Bounds::from_grid(width, height),
        &mut StdRng::seed_from_u64(seed),
    );

    let (solution, solution_cost) = match &outcome.solution {
        Some(found) => (
            found.states.iter().map(|s| [s.x, s.y, s.theta]).collect(),
            found.cost,
        ),
        None => (Vec::new(), 0.0),
    };

    TracedSearch {
        events: outcome.trace,
        stats: outcome.stats,
        solution,
        solution_cost,
        start: [start_x, start_y, 0.0],
        goal: [goal_x, goal_y],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::obstacle::CellVertex;

    fn pillar() -> ObstaclePoly {
        ObstaclePoly {
            id: 1,
            dynamic: false,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 4, y: 4 },
                CellVertex { x: 6, y: 4 },
                CellVertex { x: 6, y: 6 },
                CellVertex { x: 4, y: 6 },
                CellVertex { x: 4, y: 4 },
            ],
        }
    }

    #[test]
    fn a_traced_search_reports_a_tree_and_a_solution() {
        let traced = trace_search(
            16,
            16,
            &[pillar()],
            [1, 1],
            [13, 13],
            UnicycleSpec::default(),
            4,
            5_000,
        );

        assert!(!traced.events.is_empty(), "nothing was recorded");
        assert!(!traced.solution.is_empty(), "open ground past a pillar should solve");
        assert!(traced.solution_cost > 0.0);
        assert_eq!(traced.start, [1.5, 1.5, 0.0], "the start is the centre of its cell");
        assert_eq!(traced.goal, [13.5, 13.5]);
    }

    #[test]
    fn a_seed_replays_the_same_trace() {
        // What makes an offline trace worth having over a stream: the same request draws the
        // same picture, so a run worth studying can be handed to someone else as a seed.
        let run = || {
            trace_search(16, 16, &[pillar()], [1, 1], [13, 13], UnicycleSpec::default(), 11, 2_000)
        };
        let (a, b) = (run(), run());

        assert_eq!(a.events.len(), b.events.len());
        assert_eq!(a.solution.len(), b.solution.len());
        assert_eq!(a.solution_cost, b.solution_cost);
        assert_eq!(a.stats.nodes, b.stats.nodes);
    }

    #[test]
    fn an_outsized_request_is_clamped_rather_than_honored() {
        // One request must not be able to ask for a gigabyte of trace.
        let traced = trace_search(
            8, 8, &[], [1, 1], [6, 6], UnicycleSpec::default(), 2,
            MAX_TRACE_ITERATIONS * 100,
        );
        assert_eq!(traced.stats.iterations, MAX_TRACE_ITERATIONS);
    }

    #[test]
    fn the_search_sees_the_obstacles_it_was_given() {
        // A traced search has to run against the same world a real plan would, or the picture
        // explains a search nobody ran. Checked by the solution keeping clear of the pillar.
        let traced = trace_search(
            16, 16, &[pillar()], [1, 1], [13, 13], UnicycleSpec::default(), 4, 5_000,
        );

        for pose in &traced.solution {
            let inside = pose[0] >= 4.0 && pose[0] <= 7.0 && pose[1] >= 4.0 && pose[1] <= 7.0;
            assert!(!inside, "the solution crosses the pillar at ({}, {})", pose[0], pose[1]);
        }
    }
}
