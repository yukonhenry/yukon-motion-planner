//! Path planners, and the movement model they share.

pub(crate) mod a_star;
pub(crate) mod grid_cost;

use std::fmt;

/// Why a plan could not be produced.
///
/// Worth distinguishing rather than collapsing to `None` because the caller answers
/// them differently: a bad endpoint is the *request's* fault and deserves a 400, while
/// an unreachable goal is a truthful answer about the world — the grid and both
/// endpoints were fine, there simply is no route.
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

/// Builds a world from an ASCII map: `#` blocked, `.` plain ground, a digit that cell's
/// terrain cost.
///
/// Lives here rather than in one planner's test module because the map *is* the test
/// input for anything that searches — the shape belongs in the test body as a picture,
/// not as a run of `world[id].blocked = true` lines.
#[cfg(test)]
pub(crate) fn world_from_ascii(rows: &[&str]) -> crate::models::grid_world_manager::GridWorldManager<crate::models::cell::Cell> {
    use crate::models::cell::Cell;
    use crate::models::grid_world_manager::GridWorldManager;

    let width = rows[0].len();
    assert!(rows.iter().all(|row| row.len() == width), "ragged map");

    GridWorldManager::from_fn(width, rows.len(), |x, y| match rows[y].as_bytes()[x] {
        b'#' => Cell { blocked: true, terrain_cost: 0 },
        b'.' => Cell::default(),
        digit @ b'0'..=b'9' => Cell { blocked: false, terrain_cost: (digit - b'0') as u16 },
        other => panic!("unknown map character {:?}", other as char),
    })
}