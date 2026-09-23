//! The one place cells and metres meet.
//!
//! Two coordinate systems have grown up side by side here. The grid world is integer cell
//! indices all the way down — [`ObstaclePoly`](super::obstacle::ObstaclePoly) vertices,
//! [`RobotBody::position`](super::robot::RobotBody::position), every planner's `NodeId`. The
//! kinodynamic model is continuous metres: `track_width` is 0.45 **m**, `max_linear_vel` is
//! 2.0 **m/s**. Neither is wrong, but nothing said what one cell was worth, so the two could
//! not be compared and a trajectory could not be drawn on a grid.
//!
//! This module is that exchange rate, and it is deliberately the *only* copy of it. A second
//! conversion written inline somewhere is how the planner and the renderer end up disagreeing
//! about where the robot is.

/// How many metres one cell is across.
///
/// One metre, which is a choice rather than a discovery, and worth stating plainly: a 30×30
/// grid is a 30 m × 30 m room, which is a plausible size for the warehouse-ish scenarios this
/// simulates. The reason it is *this* number rather than something finer is
/// [`inflation_cells`] — at one metre a 0.45 m robot fits comfortably inside a single cell, so
/// the point-robot assumption the grid planners have always made is actually true rather than
/// merely convenient.
///
/// A constant rather than a column on `grid_worlds` because nothing yet varies it. Every
/// conversion goes through the functions below, so promoting it to per-world data later means
/// giving them a parameter, not hunting for arithmetic.
pub const METERS_PER_CELL: f64 = 1.0;

/// The centre of a cell, in metres.
///
/// The *centre* rather than the corner: a cell is where the robot stands, and a robot standing
/// at the corner of its cell would be half inside the neighbouring one. This is also what
/// makes the round trip through [`meters_to_cell`] stable — a converted-and-converted-back
/// cell index is always itself, which a corner convention only manages up to floating-point
/// luck.
pub fn cell_to_meters(cell: i32) -> f64 {
    (cell as f64 + 0.5) * METERS_PER_CELL
}

/// Which cell a metric coordinate falls in.
///
/// Floor rather than round, so cell `n` spans exactly `[n, n+1)` metres and no coordinate
/// belongs to two cells. Negative coordinates floor away from zero, which keeps that true off
/// the left and top edges of the grid — `as i32` alone would truncate toward zero and make
/// `-0.5` land in cell 0 alongside `+0.5`.
pub fn meters_to_cell(meters: f64) -> i32 {
    (meters / METERS_PER_CELL).floor() as i32
}

/// A metric pose's cell, as the planners spell a position.
pub fn point_to_cell(x: f64, y: f64) -> [i32; 2] {
    [meters_to_cell(x), meters_to_cell(y)]
}

/// The centre of a cell as a metric point, for starting a trajectory from a planned route.
pub fn cell_to_point(cell: [i32; 2]) -> (f64, f64) {
    (cell_to_meters(cell[0]), cell_to_meters(cell[1]))
}

/// How many cells of obstacle dilation a body of `radius_meters` needs before a *point* robot
/// planning over the result is a correct model of it.
///
/// This is the configuration-space trick, and the reason the grid planners and a kinodynamic
/// planner can be compared at all: inflate the obstacles by the robot's radius once, and
/// "where can this body go" becomes "where can this point go" for every planner that searches
/// the result. Doing it here rather than per planner is what keeps their costs measuring the
/// same thing.
///
/// The arithmetic is not `radius / cell`, which is the tempting form and is wrong. The robot's
/// reference point sits at a cell *centre*, so it already has half a cell of room in every
/// direction before it can reach into the neighbouring cell at all. Only the radius beyond
/// that half-cell has to be paid for in dilation:
///
/// ```text
///     |<- cell ->|
///     |  .--r--. |          r <= cell/2  ->  0 cells: the body never leaves its own cell
///     |  (  o  ) |
///     |  `-----' |
/// ```
///
/// At the current [`METERS_PER_CELL`] that means a 0.45 m-wide robot dilates by nothing, and
/// the grid keeps behaving exactly as it always has — which is the point, not an accident. The
/// naive form would have inflated it by a full metre and walled off corridors the robot can
/// drive down. At a finer scale the same call starts returning cells, and every planner picks
/// the change up together.
///
/// Conservative in shape: dilation is Chebyshev (a square of cells), not Euclidean, so corners
/// are inflated slightly more than a disc strictly requires. That matches the 8-connected
/// movement model the planners already use, and erring toward *more* clearance cannot produce
/// a route the robot could not drive.
pub fn inflation_cells(radius_meters: f64) -> i32 {
    let free = METERS_PER_CELL / 2.0;
    if !radius_meters.is_finite() || radius_meters <= free {
        return 0;
    }
    ((radius_meters - free) / METERS_PER_CELL).ceil() as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cell_maps_to_its_own_centre_and_back() {
        // The round trip is the property worth pinning: every conversion in the codebase is
        // one of these two, so a cell that comes back as a different cell would move the
        // robot by drawing it.
        for cell in -5..50 {
            assert_eq!(meters_to_cell(cell_to_meters(cell)), cell, "cell {cell}");
        }
    }

    #[test]
    fn a_cell_owns_a_half_open_span_of_metres() {
        // Cell n is [n, n+1) metres. Both ends matter: a closed span would put the boundary
        // in two cells at once, and an obstacle edge sits exactly on a boundary.
        assert_eq!(meters_to_cell(0.0), 0);
        assert_eq!(meters_to_cell(0.999), 0);
        assert_eq!(meters_to_cell(1.0), 1, "the boundary belongs to the cell above");

        // Off the left edge, where truncation and flooring disagree. `as i32` would truncate
        // -0.5 toward zero and hand back cell 0, putting an off-grid coordinate on the grid.
        assert_eq!(meters_to_cell(-0.5), -1);
        assert_eq!(meters_to_cell(-1.0), -1);
        assert_eq!(meters_to_cell(-1.5), -2);
    }

    #[test]
    fn a_body_that_fits_its_cell_needs_no_inflation() {
        // The claim METERS_PER_CELL was chosen for. If this ever stops holding, the grid
        // planners' point robot has quietly become a lie.
        let robot_radius = 0.45 / 2.0;
        assert!(
            robot_radius <= METERS_PER_CELL / 2.0,
            "the default robot no longer fits inside one cell",
        );
        assert_eq!(inflation_cells(robot_radius), 0);
    }

    #[test]
    fn inflation_pays_only_for_the_radius_beyond_the_half_cell() {
        // The naive `ceil(radius / cell)` would return 1 for every one of these, inflating a
        // 5 cm robot by a whole metre and sealing corridors it could drive straight down.
        assert_eq!(inflation_cells(0.0), 0);
        assert_eq!(inflation_cells(0.05), 0);
        assert_eq!(inflation_cells(METERS_PER_CELL / 2.0), 0, "exactly touching is clear");

        // Past the half-cell, dilation starts — and grows by a cell per cell of radius.
        assert_eq!(inflation_cells(0.51), 1);
        assert_eq!(inflation_cells(1.5), 1);
        assert_eq!(inflation_cells(1.51), 2);
        assert_eq!(inflation_cells(2.5), 2);
    }

    #[test]
    fn a_dilated_obstacle_always_clears_the_body_that_dilated_it() {
        // The property the formula exists to guarantee, asserted rather than re-derived: the
        // clearance granted must cover the radius, or the "point robot" the planners search
        // with is smaller than the machine and will clip something.
        for tenths in 0..60 {
            let radius = tenths as f64 / 10.0;
            let granted = inflation_cells(radius) as f64 * METERS_PER_CELL + METERS_PER_CELL / 2.0;
            assert!(
                granted >= radius - 1e-12,
                "radius {radius} got only {granted} m of clearance",
            );
        }
    }

    #[test]
    fn a_nonsense_radius_inflates_by_nothing_rather_than_panicking() {
        // `as i32` on a non-finite float is a saturating cast, so an inflation of NaN would
        // silently become i32::MAX cells and block the entire grid. Caught here instead.
        assert_eq!(inflation_cells(f64::NAN), 0);
        assert_eq!(inflation_cells(f64::NEG_INFINITY), 0);
        assert_eq!(inflation_cells(-1.0), 0);
    }
}
