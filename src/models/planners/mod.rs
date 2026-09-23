//! Path planners, and the movement model they share.

pub(crate) mod a_star;
pub(crate) mod d_star_lite;
pub(crate) mod movement_model;

pub(crate) mod rrt;

use crate::models::cell::Cell;
use crate::models::robots::unicycle_spec::UnicycleSpec;
use crate::models::grid_world_manager::{GridWorldManager, NodeId};
use std::fmt;

/// Distinguish errors rather than collapse to None.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PlanError {
    /// The start coordinate is outside the grid.
    SrcOffGrid,
    /// The goal coordinate is outside the grid.
    DestOffGrid,
    /// The start cell is inside an obstacle.
    SrcBlocked,
    /// The goal cell is inside an obstacle.
    DestBlocked,
    /// Both endpoints are legal, but no route connects them.
    Unreachable,
}

impl fmt::Display for PlanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            PlanError::SrcOffGrid => "start coordinate is outside the grid",
            PlanError::DestOffGrid => "goal coordinate is outside the grid",
            PlanError::SrcBlocked => "start coordinate is inside an obstacle",
            PlanError::DestBlocked => "goal coordinate is inside an obstacle",
            PlanError::Unreachable => "no route connects the start to the goal",
        };
        f.write_str(message)
    }
}

/// A* is stateless and implements this on a unit struct — it could just as well have stayed
/// a bare `fn`. D* Lite could not: it keeps `g`, `rhs`, its priority queue and `k_m` between
/// calls so that a replan touches only what the world changed, and running it from scratch
/// each time throws away the one property it exists for. `&mut self` is where that state
/// lives, and is the reason this is a trait.
///
/// Implementations are reached through [`find_plan`](GridWorldManager::find_plan), which
/// resolves coordinates and rejects illegal endpoints first — so `src` and `dest` here are
/// already known to be in bounds and passable, and the only failure an implementation reports
/// is [`PlanError::Unreachable`].
pub(crate) trait Planner {
    fn plan(
        &mut self,
        world: &GridWorldManager<Cell>,
        src: NodeId,
        dest: NodeId,
    ) -> Result<Vec<NodeId>, PlanError>;
}

/// Which planner a request asked for.
///
/// Worth having alongside the trait because the handler needs a *name* as well as an
/// implementation: a plan's `meta` records which planner produced it. Choosing the two
/// separately is how they drift, so both come from one value here. Adding a planner is a
/// variant plus an arm in each `match`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum PlannerKind {
    AStar,
    /// Used by `POST /grids/{id}/replan`, where replanning a just-changed world is the whole
    /// point. `POST /grids/{id}/plans` still hardcodes `AStar`; letting a client choose means
    /// giving `PlanInput` a `planner` field.
    DStarLite,
    /// Stable Sparse RRT over the second-order unicycle.
    ///
    /// The odd one out, and worth knowing how. The other two search the grid for the cheapest
    /// 8-connected path and are exact about it; this one samples trajectories that obey the
    /// robot's acceleration and wheel-speed limits, and minimizes *duration* rather than grid
    /// cost. Its route is therefore usually dearer by `path_cost` and is not meant to be
    /// compared on that number — the comparison it exists for is whether the machine can
    /// actually drive the answer.
    ///
    /// `allow(dead_code)`: dispatch is complete and the tests select it, but no endpoint
    /// offers it yet. Two things are needed before one can, and the second is the reason this
    /// is not simply switched on — a `planner` field on `PlanInput`, and a `spawn_blocking`
    /// around the search in `generate_grid_plan`, which today runs inline. A* returns in
    /// microseconds so inline is fine; SST takes on the order of a second and would hold a
    /// runtime worker for all of it.
    #[allow(dead_code)]
    Sst,
}

/// What a planner needs beyond the grid itself.
///
/// A struct rather than more arguments because only one planner reads either field, and a
/// signature that grew a parameter per planner would make adding the next one a change to
/// every call site.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PlannerContext {
    /// The machine being planned for, where one is known.
    ///
    /// `None` on the manual replan path, which the browser drives by hand with no robot in it.
    /// A kinodynamic planner asked to run without one falls back to the default machine rather
    /// than refusing, since some body has to be integrated and the default is a real robot.
    pub robot: Option<UnicycleSpec>,
    /// Seed for planners that sample; ignored by the deterministic ones.
    pub seed: u64,
}

impl PlannerKind {
    /// The name recorded in a plan's `meta`. Spelled after the module, so a name in a stored
    /// plan leads straight to the code that produced it.
    pub(crate) fn name(self) -> &'static str {
        match self {
            PlannerKind::AStar => "a_star",
            PlannerKind::DStarLite => "d_star_lite",
            PlannerKind::Sst => "sst",
        }
    }

    /// Whether the obstacles should be grown by the robot's radius before this planner sees
    /// them.
    ///
    /// True for the grid planners, which search as a point and need the clearance baked into
    /// the world to be a correct model of a body. False for [`Sst`](PlannerKind::Sst), which
    /// sweeps the real shape through continuous space and would be clearing the same radius
    /// twice — walling off gaps the robot fits through — if it were handed an inflated grid
    /// as well.
    pub(crate) fn inflates_obstacles(self) -> bool {
        match self {
            PlannerKind::AStar | PlannerKind::DStarLite => true,
            PlannerKind::Sst => false,
        }
    }

    /// `+ Send` because the only caller is an async handler that holds the planner across a
    /// database `await`, and a future holding a non-`Send` value is not `Send` itself. Axum
    /// reports that as "`generate_grid_plan` does not implement `Handler`", which points
    /// nowhere near the cause — hence the bound here rather than a puzzle later.
    pub(crate) fn planner(self, context: PlannerContext) -> Box<dyn Planner + Send> {
        match self {
            PlannerKind::AStar => Box::new(a_star::AStar),
            PlannerKind::DStarLite => Box::new(d_star_lite::DStarLite),
            PlannerKind::Sst => Box::new(rrt::grid_adapter::Sst::new(
                context.robot.unwrap_or_default(),
                context.seed,
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every kind there is. Listed once so a new variant is one edit, and the tests below
    /// cannot silently stop covering it.
    const ALL: [PlannerKind; 3] = [PlannerKind::AStar, PlannerKind::DStarLite, PlannerKind::Sst];

    /// Every variant dispatches under a name no other variant shares.
    ///
    /// The name is what a stored plan carries, so a collision would make two planners
    /// indistinguishable after the fact — and a variant wired to the wrong arm of `planner()`
    /// would record one planner's name against another's route.
    #[test]
    fn every_planner_kind_dispatches_under_a_distinct_name() {
        let mut names: Vec<&str> = ALL.iter().map(|kind| kind.name()).collect();
        let before = names.len();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), before, "two planners share a meta name");
    }

    /// Every variant produces a walkable route between two open cells.
    ///
    /// The weakest property they all share, and deliberately weak: it says a plan connects its
    /// endpoints over passable ground, not that it is the cheapest such plan. Optimality is
    /// asserted separately below, for the planners that actually claim it.
    #[test]
    fn every_planner_kind_returns_a_route_between_open_cells() {
        for kind in ALL {
            let world = test_support::empty_world(12, 12);
            let mut planner = kind.planner(PlannerContext {
                robot: None,
                seed: 3,
            });
            let route = world
                .find_plan([1, 1], [9, 9], planner.as_mut())
                .unwrap_or_else(|err| panic!("{} could not cross an open grid: {err}", kind.name()));

            assert_eq!(route.first(), Some(&world.id(1, 1)), "{}", kind.name());
            assert_eq!(route.last(), Some(&world.id(9, 9)), "{}", kind.name());
            for &node in &route {
                assert!(world.passable(node), "{} crossed a blocked cell", kind.name());
            }
        }
    }

    /// The grid planners find the *cheapest* 8-connected route, and agree on its price.
    ///
    /// Scoped to those two on purpose. [`PlannerKind::Sst`] is not in this list because it is
    /// not solving this problem: it minimizes the time a robot with real acceleration and
    /// wheel-speed limits needs, over a continuous space, and its route is a curve that
    /// happens to be projected onto cells afterwards. Holding it to an octile optimum would
    /// assert that a car should corner like a chess knight.
    #[test]
    fn the_grid_planners_find_the_cheapest_route() {
        for kind in [PlannerKind::AStar, PlannerKind::DStarLite] {
            let world = test_support::empty_world(4, 4);
            let mut planner = kind.planner(PlannerContext {
                robot: None,
                seed: 0,
            });
            let route = world.find_plan([0, 0], [3, 3], planner.as_mut());
            assert_eq!(
                route.map(|route| world.path_cost(&route)),
                Ok(42),
                "{} did not find the 3 diagonal steps across an open grid",
                kind.name(),
            );
        }
    }

    /// Only the planners that search as a point ask for the obstacles to be grown.
    ///
    /// Pinned because getting it backwards is invisible: an inflated grid handed to the
    /// kinodynamic planner clears the robot's radius twice and quietly seals gaps it fits
    /// through, while an uninflated one handed to a grid planner lets a body clip corners.
    #[test]
    fn only_the_point_searchers_inflate_their_obstacles() {
        assert!(PlannerKind::AStar.inflates_obstacles());
        assert!(PlannerKind::DStarLite.inflates_obstacles());
        assert!(
            !PlannerKind::Sst.inflates_obstacles(),
            "SST sweeps a real body and must not also be given inflated obstacles",
        );
    }
}

/// Fixtures shared by the planner test modules.
///
/// These live here rather than in whichever planner was written first because none of them
/// belongs to a planner: a map *is* the test input for anything that searches, and an
/// independent optimal-cost oracle is what *every* planner has to be held against. Copying
/// the oracle per planner would let two planners be consistently wrong together.
#[cfg(test)]
pub(crate) mod test_support {
    use crate::models::cell::Cell;
    use crate::models::grid_world_manager::{GridWorldManager, NodeId};
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;

    pub(crate) fn empty_world(width: usize, height: usize) -> GridWorldManager<Cell> {
        GridWorldManager::new(width, height)
    }

    /// Builds a world from an ASCII map: `#` blocked, `.` plain ground, a digit that cell's
    /// terrain cost.
    ///
    /// The shape belongs in the test body as a picture, not as a run of
    /// `world[id].blocked = true` lines.
    pub(crate) fn world_from_ascii(rows: &[&str]) -> GridWorldManager<Cell> {
        let width = rows[0].len();
        assert!(rows.iter().all(|row| row.len() == width), "ragged map");

        GridWorldManager::from_fn(width, rows.len(), |x, y| match rows[y].as_bytes()[x] {
            b'#' => Cell {
                blocked: true,
                terrain_cost: 0,
            },
            b'.' => Cell::default(),
            digit @ b'0'..=b'9' => Cell {
                blocked: false,
                terrain_cost: (digit - b'0') as u16,
            },
            other => panic!("unknown map character {:?}", other as char),
        })
    }

    pub(crate) fn coords(world: &GridWorldManager<Cell>, path: &[NodeId]) -> Vec<(usize, usize)> {
        path.iter().map(|&node| world.xy(node)).collect()
    }

    /// The properties a plan must have whichever route it picked: it runs from src to
    /// dest, never stands on a blocked cell, and only ever moves between legal steps.
    ///
    /// Asserted separately from cost because a path can be the right *price* and still
    /// be nonsense — a broken parent chain yields a cheap sequence that teleports.
    pub(crate) fn assert_walkable(
        world: &GridWorldManager<Cell>,
        path: &[NodeId],
        src: NodeId,
        dest: NodeId,
    ) {
        assert_eq!(path.first(), Some(&src), "plan does not start at src");
        assert_eq!(path.last(), Some(&dest), "plan does not end at dest");

        for &node in path {
            assert!(
                world.passable(node),
                "plan crosses the blocked cell {:?}",
                world.xy(node),
            );
        }
        for step in path.windows(2) {
            let (from, to) = (step[0], step[1]);
            assert!(
                world.passable_neighbors(from).any(|n| n == to),
                "{:?} -> {:?} is not a legal step",
                world.xy(from),
                world.xy(to),
            );
        }
    }

    /// Uniform-cost search over the same edges — A* with `h == 0`, which is optimal on
    /// any graph without depending on a heuristic at all.
    ///
    /// That is the point: it shares the movement model with the code under test, so it does
    /// not check `movement_model`, but it does pin what a heuristic or an incremental update
    /// rule can break. A planner that overestimates shows up as a *dearer* path, not a
    /// visibly wrong one, and no hand-written expected route would catch it.
    ///
    /// Directional on purpose: [`step_cost`](GridWorldManager::step_cost) charges the terrain
    /// of the cell being *entered*, so the cheapest route from `src` to `dest` need not cost
    /// what the reverse does. A backwards search that reads an edge the wrong way round is
    /// only caught by an oracle that keeps the directions straight.
    pub(crate) fn dijkstra_cost(
        world: &GridWorldManager<Cell>,
        src: NodeId,
        dest: NodeId,
    ) -> Option<u32> {
        if !world.passable(src) || !world.passable(dest) {
            return None;
        }

        let mut best = vec![u32::MAX; world.len()];
        best[src.0] = 0;
        let mut frontier = BinaryHeap::new();
        frontier.push(Reverse((0u32, src)));

        while let Some(Reverse((cost, node))) = frontier.pop() {
            if node == dest {
                return Some(cost);
            }
            if cost > best[node.0] {
                continue;
            }
            for next in world.passable_neighbors(node) {
                let tentative = cost + world.step_cost(node, next);
                if tentative < best[next.0] {
                    best[next.0] = tentative;
                    frontier.push(Reverse((tentative, next)));
                }
            }
        }
        None
    }

    /// Deterministic xorshift, so the sweeps below need no `rand` dependency and a failure
    /// is reproducible from the seed alone.
    pub(crate) fn xorshift(seed: u64) -> impl FnMut() -> u64 {
        let mut state = seed;
        move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        }
    }
}
