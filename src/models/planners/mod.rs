//! Path planners, and the movement model they share.

pub(crate) mod a_star;
pub(crate) mod d_lite;
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
}

impl PlannerKind {
    /// The name recorded in a plan's `meta`.
    pub(crate) fn name(self) -> &'static str {
        match self {
            PlannerKind::AStar => "a_star",
        }
    }

    /// `+ Send` because the only caller is an async handler that holds the planner across a
    /// database `await`, and a future holding a non-`Send` value is not `Send` itself. Axum
    /// reports that as "`generate_grid_plan` does not implement `Handler`", which points
    /// nowhere near the cause — hence the bound here rather than a puzzle later.
    pub(crate) fn planner(self) -> Box<dyn Planner + Send> {
        match self {
            PlannerKind::AStar => Box::new(a_star::AStar),
        }
    }
}

/// Builds a world from an ASCII map: `#` blocked, `.` plain ground, a digit that cell's
/// terrain cost.
///
/// Lives here rather than in one planner's test module because the map *is* the test
/// input for anything that searches — the shape belongs in the test body as a picture,
/// not as a run of `world[id].blocked = true` lines.
#[cfg(test)]
pub(crate) fn world_from_ascii(
    rows: &[&str],
) -> crate::models::grid_world_manager::GridWorldManager<crate::models::cell::Cell> {
    use crate::models::cell::Cell;
    use crate::models::grid_world_manager::GridWorldManager;

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
