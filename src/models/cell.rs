//! The static contents of one grid cell.

/// What a cell *is*, as distinct from what a search has learned about it.
///
/// Deliberately holds no search bookkeeping — no `g`, no parent, no closed flag. Those
/// live in [`Search`](super::planners::a_star::Search), so one world can be searched
/// repeatedly, and by different algorithms, with no reset pass in between and with the
/// search taking `&self` rather than `&mut self`.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct Cell {
    /// Impassable — rasterized from an obstacle polygon.
    pub blocked: bool,
    /// Extra cost to *enter* this cell, added to the step cost. `0` is plain ground.
    ///
    /// Kept `u16` so a cell stays small; the search widens it to `u32` when
    /// accumulating.
    pub terrain_cost: u16,
}

impl Cell {
    pub fn passable(&self) -> bool {
        !self.blocked
    }
}
