//! Path planners, and the movement model they share.

pub(crate) mod a_star;
pub(crate) mod d_star_lite;
pub(crate) mod movement_model;

use crate::models::cell::Cell;
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
    /// `allow(dead_code)` only because nothing *selects* a planner yet — the handler hardcodes
    /// `AStar`. Giving `PlanInput` a `planner` field is what retires this attribute.
    #[allow(dead_code)]
    DStarLite,
}

impl PlannerKind {
    /// The name recorded in a plan's `meta`. Spelled after the module, so a name in a stored
    /// plan leads straight to the code that produced it.
    pub(crate) fn name(self) -> &'static str {
        match self {
            PlannerKind::AStar => "a_star",
            PlannerKind::DStarLite => "d_star_lite",
        }
    }

    /// `+ Send` because the only caller is an async handler that holds the planner across a
    /// database `await`, and a future holding a non-`Send` value is not `Send` itself. Axum
    /// reports that as "`generate_grid_plan` does not implement `Handler`", which points
    /// nowhere near the cause — hence the bound here rather than a puzzle later.
    pub(crate) fn planner(self) -> Box<dyn Planner + Send> {
        match self {
            PlannerKind::AStar => Box::new(a_star::AStar),
            PlannerKind::DStarLite => Box::new(d_star_lite::DStarLite),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every variant dispatches to something that plans, under a name no other variant shares.
    ///
    /// The name is what a stored plan carries, so a collision would make two planners
    /// indistinguishable after the fact — and a variant wired to the wrong arm of `planner()`
    /// would record one planner's name against another's route.
    #[test]
    fn every_planner_kind_dispatches_under_a_distinct_name() {
        let kinds = [PlannerKind::AStar, PlannerKind::DStarLite];

        let mut names: Vec<&str> = kinds.iter().map(|kind| kind.name()).collect();
        let before = names.len();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), before, "two planners share a meta name");

        let world = test_support::empty_world(4, 4);
        for kind in kinds {
            let mut planner = kind.planner();
            let route = world.find_plan([0, 0], [3, 3], planner.as_mut());
            assert_eq!(
                route.map(|route| world.path_cost(&route)),
                Ok(42),
                "{} did not find the 3 diagonal steps across an open grid",
                kind.name(),
            );
        }
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
