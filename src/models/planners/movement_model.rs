//! The movement model: Definition of [`GridWorldManager`] actions and definitions when grid is defined with [`Cell`] structs.
//!
//! ['Cell'](crate::models::cell::Cell) primitive costs and planner-independent rules are defined.

use crate::models::cell::Cell;
use crate::models::grid_world_manager::{GridWorldManager, NodeId};
use crate::models::planners::{PlanError, Planner};

/// Step costs, scaled by 10 so a diagonal stays an integer.
///
/// A diagonal step is √2 ≈ 1.414 times an orthogonal one. Scaling by 10 gives 10 and
/// 14, which keeps every cost a `u32`: `BinaryHeap<Reverse<(u32, ..)>>` then orders
/// correctly with no `f32` wrapper, and `g` never accumulates floating-point drift.
pub const ORTHOGONAL_COST: u32 = 10;
pub const DIAGONAL_COST: u32 = 14;

impl GridWorldManager<Cell> {
    pub fn passable(&self, id: NodeId) -> bool {
        self[id].passable()
    }

    /// The steps actually available from `id`: in-bounds, passable, and not cutting the
    /// corner of an obstacle.
    ///
    /// [`neighbors8`](GridWorldManager::neighbors8) leaves the corner rule to its caller, and
    /// this is where it gets applied. A diagonal move between two cells that share two
    /// *blocked* orthogonal cells passes through the seam of a wall — geometrically it
    /// clips the obstacle's corner, and no agent with any width can make it.
    ///
    /// The rule here is the strict one: **both** shared orthogonal cells must be clear.
    /// The permissive variant, which allows brushing a single corner, is this same test
    /// with `&&` relaxed to `||`.
    ///
    /// Forbidding steps never disturbs [`octile_heuristic`](Self::octile_heuristic_h):
    /// removing edges can only make the true cost higher, so a heuristic that already
    /// never overestimated still doesn't.
    pub fn passable_neighbors(&self, id: NodeId) -> impl Iterator<Item=NodeId> + '_ {
        let (x, y) = self.xy(id);

        self.neighbors8(id).filter(move |&next| {
            if !self.passable(next) {
                return false;
            }
            let (next_x, next_y) = self.xy(next);
            if next_x == x || next_y == y {
                return true; // orthogonal: no corner to cut
            }
            // The two cells the diagonal squeezes between. Both coordinates come from
            // in-bounds cells — `next_x` from `next`, `y` from `id` — so each pairing is
            // itself in bounds and `id` cannot panic.
            self.passable(self.id(next_x, y)) && self.passable(self.id(x, next_y))
        })
    }

    /// Cost of stepping between two *adjacent* nodes, including the terrain cost of the
    /// cell being entered.
    ///
    /// Diagonality is derived from the coordinates, so this is correct for any adjacent
    /// pair however the neighbor was produced.
    pub fn step_cost(&self, from: NodeId, to: NodeId) -> u32 {
        let (x0, y0) = self.xy(from);
        let (x1, y1) = self.xy(to);
        let base = if x0 != x1 && y0 != y1 {
            DIAGONAL_COST
        } else {
            ORTHOGONAL_COST
        };
        base + self[to].terrain_cost as u32
    }

    /// What a whole path costs: the sum of its steps.
    ///
    /// A one-cell path costs nothing — the start is never *entered*, so its terrain is
    /// never charged, matching [`step_cost`](Self::step_cost).
    pub fn path_cost(&self, path: &[NodeId]) -> u32 {
        path.windows(2)
            .map(|step| self.step_cost(step[0], step[1]))
            .sum()
    }

    /// Octile distance — the exact cost of the cheapest *unobstructed* 8-connected path.
    ///
    /// Take `min(dx, dy)` diagonal steps, then `|dx - dy|` orthogonal ones:
    /// `14 * min + 10 * (max - min)`, which rearranges to the form below.
    ///
    /// Admissible (never exceeds the true cost, since obstacles and terrain can only
    /// make the real path dearer) and consistent (`h(a) <= step_cost(a, n) + h(n)` for
    /// every neighbor `n`). Consistency is what lets A* close a node on first
    /// expansion and never reopen it.
    pub fn octile_heuristic_h(&self, from: NodeId, to: NodeId) -> u32 {
        let (x0, y0) = self.xy(from);
        let (x1, y1) = self.xy(to);
        let dx = x0.abs_diff(x1) as u32;
        let dy = y0.abs_diff(y1) as u32;

        ORTHOGONAL_COST * (dx + dy) - (2 * ORTHOGONAL_COST - DIAGONAL_COST) * dx.min(dy)
    }

    /// Entry point from the handler: plans between two `[x, y]` cell coordinates using
    /// `planner`.
    /// Specific error conditions are reported as `PlanError` so the handler can return a 400 with a
    /// message that explains what went wrong.
    pub(crate) fn find_plan(
        &self,
        src: [i32; 2],
        dest: [i32; 2],
        planner: &mut dyn Planner,
    ) -> Result<Vec<NodeId>, PlanError> {
        let src_id = self
            .try_id(src[0] as isize, src[1] as isize)
            .ok_or(PlanError::SrcOffGrid)?;
        let dest_id = self
            .try_id(dest[0] as isize, dest[1] as isize)
            .ok_or(PlanError::DestOffGrid)?;

        if !self.passable(src_id) {
            return Err(PlanError::SrcBlocked);
        }
        if !self.passable(dest_id) {
            return Err(PlanError::DestBlocked);
        }

        planner.plan(self, src_id, dest_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::planners::a_star::AStar;
    use crate::models::planners::test_support::*;

    fn neighbor_coords(world: &GridWorldManager<Cell>, id: NodeId) -> Vec<(usize, usize)> {
        let mut coords: Vec<_> = world.passable_neighbors(id).map(|n| world.xy(n)).collect();
        coords.sort();
        coords
    }

    #[test]
    fn octile_matches_hand_computed_distances() {
        let world = empty_world(10, 10);
        let h = |a: (usize, usize), b: (usize, usize)| {
            world.octile_heuristic_h(world.id(a.0, a.1), world.id(b.0, b.1))
        };

        assert_eq!(h((0, 0), (0, 0)), 0, "no distance to itself");
        assert_eq!(h((0, 0), (3, 0)), 30, "3 orthogonal steps");
        assert_eq!(h((0, 0), (0, 3)), 30);
        assert_eq!(h((0, 0), (3, 3)), 42, "3 diagonal steps");
        // 3 diagonals then 2 straight: 3*14 + 2*10.
        assert_eq!(h((0, 0), (5, 3)), 62);
    }

    #[test]
    fn octile_is_symmetric() {
        let world = empty_world(7, 5);
        for (a, _) in world.iter() {
            for (b, _) in world.iter() {
                assert_eq!(
                    world.octile_heuristic_h(a, b),
                    world.octile_heuristic_h(b, a),
                    "asymmetric between {:?} and {:?}",
                    world.xy(a),
                    world.xy(b),
                );
            }
        }
    }

    #[test]
    fn octile_is_consistent() {
        // The property A* actually depends on: h(a) <= step_cost(a, n) + h(n) for every
        // neighbor. Together with h(goal) == 0 this implies admissibility, so it is the
        // one worth pinning. Terrain only raises step_cost, so testing at 0 is the
        // tightest case.
        //
        // Deliberately over `neighbors8` rather than `passable_neighbors`: every step
        // the corner rule removes is one constraint fewer, so holding on the full edge
        // set is the strictly stronger claim.
        let world = empty_world(8, 6);
        let goal = world.id(7, 5);

        for (node, _) in world.iter() {
            let h_node = world.octile_heuristic_h(node, goal);
            for next in world.neighbors8(node) {
                let bound = world.step_cost(node, next) + world.octile_heuristic_h(next, goal);
                assert!(
                    h_node <= bound,
                    "inconsistent at {:?} -> {:?}: {h_node} > {bound}",
                    world.xy(node),
                    world.xy(next),
                );
            }
        }
    }

    #[test]
    fn octile_is_zero_only_at_the_goal() {
        let world = empty_world(6, 6);
        let goal = world.id(3, 4);
        for (node, _) in world.iter() {
            let h = world.octile_heuristic_h(node, goal);
            assert_eq!(
                h == 0,
                node == goal,
                "h == 0 away from the goal breaks admissibility"
            );
        }
    }

    #[test]
    fn step_cost_separates_orthogonal_from_diagonal() {
        let world = empty_world(4, 4);
        let center = world.id(1, 1);

        assert_eq!(world.step_cost(center, world.id(2, 1)), ORTHOGONAL_COST);
        assert_eq!(world.step_cost(center, world.id(1, 0)), ORTHOGONAL_COST);
        assert_eq!(world.step_cost(center, world.id(2, 2)), DIAGONAL_COST);
        assert_eq!(world.step_cost(center, world.id(0, 2)), DIAGONAL_COST);
    }

    #[test]
    fn step_cost_adds_the_terrain_of_the_cell_entered() {
        let mut world = empty_world(3, 3);
        let costly = world.id(2, 1);
        world[costly].terrain_cost = 7;

        let from = world.id(1, 1);
        assert_eq!(world.step_cost(from, costly), ORTHOGONAL_COST + 7);
        // Leaving a costly cell is free; the charge is for entering.
        assert_eq!(world.step_cost(costly, from), ORTHOGONAL_COST);
    }

    #[test]
    fn path_cost_sums_the_steps() {
        let world = world_from_ascii(&["..3..", ".....", "....."]);
        let path: Vec<NodeId> = [(0, 0), (1, 0), (2, 0), (3, 1)]
            .iter()
            .map(|&(x, y)| world.id(x, y))
            .collect();
        // 10 + (10 + 3) + 14.
        assert_eq!(world.path_cost(&path), 37);
    }

    #[test]
    fn a_single_cell_path_costs_nothing() {
        let world = world_from_ascii(&["9.."]);
        // Even standing on dear ground: terrain is charged on entry, and nothing is
        // entered. An empty path is the same, and neither may underflow.
        assert_eq!(world.path_cost(&[world.id(0, 0)]), 0);
        assert_eq!(world.path_cost(&[]), 0);
    }

    #[test]
    fn diagonal_cost_sits_between_one_and_two_orthogonal_steps() {
        // Sanity on the 10/14 scaling: a diagonal must beat going around the corner,
        // or it is never worth taking...
        assert!(DIAGONAL_COST < 2 * ORTHOGONAL_COST);
        // ...and must cost more than a straight step, or it is always worth taking.
        assert!(DIAGONAL_COST > ORTHOGONAL_COST);
    }

    #[test]
    fn passable_follows_the_blocked_flag() {
        let mut world = empty_world(3, 3);
        let wall = world.id(1, 1);
        assert!(world.passable(wall));
        world[wall].blocked = true;
        assert!(!world.passable(wall));
    }

    // --- the corner rule -------------------------------------------------

    #[test]
    fn an_open_cell_keeps_all_eight_neighbors() {
        let world = world_from_ascii(&["...", "...", "..."]);
        assert_eq!(
            neighbor_coords(&world, world.id(1, 1)),
            vec![
                (0, 0),
                (0, 1),
                (0, 2),
                (1, 0),
                (1, 2),
                (2, 0),
                (2, 1),
                (2, 2)
            ],
        );
    }

    #[test]
    fn a_diagonal_between_two_blocked_cells_is_not_a_step() {
        // The seam of a diagonal wall: (1,0) and (0,1) are blocked, so (0,0) -> (1,1)
        // clips the corner where they meet.
        let world = world_from_ascii(&[".#.", "#..", "..."]);
        assert!(
            !world
                .passable_neighbors(world.id(0, 0))
                .any(|n| n == world.id(1, 1)),
            "the diagonal squeezed between two blocked cells",
        );
        // ...and with the corner sealed, (0,0) has no step at all.
        assert_eq!(neighbor_coords(&world, world.id(0, 0)), Vec::new());
    }

    #[test]
    fn a_diagonal_past_a_single_blocked_cell_is_not_a_step_either() {
        // The strict rule: one blocked cell in the pair is enough to forbid it. Under
        // the permissive variant this diagonal would survive, so it is the test that
        // pins which of the two is in force.
        let world = world_from_ascii(&["...", "#..", "..."]);
        let from = world.id(0, 0);
        assert!(
            !world.passable_neighbors(from).any(|n| n == world.id(1, 1)),
            "a diagonal brushed the corner of a single blocked cell",
        );
        // The orthogonal step alongside that same wall is untouched.
        assert!(world.passable_neighbors(from).any(|n| n == world.id(1, 0)));
    }

    #[test]
    fn the_corner_rule_is_symmetric() {
        // Steps have to be forbidden in both directions, or the planner can find a route
        // that no reversed plan could retrace.
        let world = world_from_ascii(&[".#.", "#..", "..."]);
        let (corner, inner) = (world.id(0, 0), world.id(1, 1));
        assert!(!world.passable_neighbors(corner).any(|n| n == inner));
        assert!(!world.passable_neighbors(inner).any(|n| n == corner));
    }

    #[test]
    fn blocked_cells_are_never_neighbors_however_they_are_reached() {
        let world = world_from_ascii(&["...", ".#.", "..."]);
        let wall = world.id(1, 1);
        for (node, _) in world.iter() {
            assert!(
                !world.passable_neighbors(node).any(|n| n == wall),
                "{:?} offers a step into the wall",
                world.xy(node),
            );
        }
    }

    // --- the endpoint checks ---------------------------------------------
    //
    // These belong to the entry point rather than to A*: each one rejects before the planner
    // is called at all, so they hold for whichever planner is passed. A* is used below only
    // because some planner has to be.

    #[test]
    fn an_off_grid_endpoint_is_reported_as_such() {
        let world = world_from_ascii(&[".....", "..#..", "....."]);
        let plan = |src, dest| world.find_plan(src, dest, &mut AStar);

        assert_eq!(plan([0, 0], [5, 0]), Err(PlanError::DestOffGrid));
        assert_eq!(plan([0, 0], [0, 3]), Err(PlanError::DestOffGrid));
        assert_eq!(plan([-1, 0], [4, 2]), Err(PlanError::SrcOffGrid));
        // Src is checked first: a request with both wrong names the start, and fixing
        // that surfaces the goal next.
        assert_eq!(plan([9, 9], [9, 9]), Err(PlanError::SrcOffGrid));
    }

    #[test]
    fn a_blocked_endpoint_is_reported_as_such() {
        let world = world_from_ascii(&[".....", "..#..", "....."]);
        let plan = |src, dest| world.find_plan(src, dest, &mut AStar);

        assert_eq!(plan([2, 1], [4, 2]), Err(PlanError::SrcBlocked));
        assert_eq!(plan([0, 0], [2, 1]), Err(PlanError::DestBlocked));
        // Both endpoints clear, so the same map does plan when asked something sane.
        assert!(plan([0, 0], [4, 2]).is_ok());
    }

    #[test]
    fn an_off_grid_endpoint_outranks_a_blocked_one() {
        // Ordering worth pinning now that both checks live in one function: a coordinate
        // that isn't on the grid has no cell to call blocked, so bounds must come first.
        let world = world_from_ascii(&["..#", "...", "..."]);
        assert_eq!(
            world.find_plan([9, 9], [2, 0], &mut AStar),
            Err(PlanError::SrcOffGrid),
        );
    }

    #[test]
    fn the_corner_rule_never_looks_outside_the_grid() {
        // Every diagonal at an edge or corner has both of its shared cells in bounds by
        // construction; this pins that the lookup can't be made to panic.
        let world = world_from_ascii(&["....", "....", "...."]);
        for (node, _) in world.iter() {
            let _ = world.passable_neighbors(node).count();
        }
    }
}
