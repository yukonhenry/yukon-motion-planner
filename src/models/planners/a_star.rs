//! A* over a [`GridWorldManager<Cell>`].
//!
//! The world stays immutable during a search: everything the search learns lives in
//! [`Search`], whose arrays are indexed by `NodeId.0`. That is what the flat node id
//! buys — no hashing in the inner loop, and no per-run cleanup of the world.
//!
//! Which steps exist and what they cost is not decided here — that is
//! [`movement_model`](super::movement_model), so every other planner shares the same answer
//! rather than importing it from A*.

use crate::models::cell::Cell;
use crate::models::grid_world_manager::{GridWorldManager, NodeId};
use crate::models::planners::{PlanError, Planner};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Per-search bookkeeping. Every array is `world.len()` long and indexed by `NodeId.0`.
pub struct Search {
    /// Best known cost from the start. [`Search::UNREACHED`] means "not yet seen".
    pub cost_to_node: Vec<u32>,
    /// The node each node was best reached from, for path reconstruction.
    ///
    /// One parent per node — the edge is implied by *(parent → node)*, so storing a
    /// pair would just repeat the node's own id.
    pub parent: Vec<Option<NodeId>>,
    /// Already expanded. With a consistent heuristic its `g` is final, so a later,
    /// staler heap entry for the same node can be dropped on sight.
    pub closed: Vec<bool>,
}

impl Search {
    /// Sentinel for "no path to here yet". Using `u32::MAX` rather than an `Option`
    /// keeps `g` a flat array of plain integers.
    pub const UNREACHED: u32 = u32::MAX;

    pub fn new(len: usize) -> Self {
        Self {
            cost_to_node: vec![Self::UNREACHED; len],
            parent: vec![None; len],
            closed: vec![false; len],
        }
    }

    pub fn reached(&self, NodeId(i): NodeId) -> bool {
        self.cost_to_node[i] != Self::UNREACHED
    }

    /// Walks `parent` back from `dest`, returning the path start-first.
    ///
    /// Returns `None` if `dest` was never reached, or if the parent chain fails to
    /// terminate.
    pub fn reconstruct(&self, dest: NodeId) -> Option<Vec<NodeId>> {
        if !self.reached(dest) {
            return None;
        }

        let mut path = vec![dest];
        let mut current = dest;

        // A correct search cannot build a cycle in `parent`, but a bug that did would
        // spin here forever — and this runs inside a request handler, where an infinite
        // loop wedges a worker instead of failing visibly. No path visits a node twice,
        // so the node count is a hard bound on the hops: falling out of this loop means
        // the chain is corrupt, and `None` reports that as "no path" rather than hanging.
        for _ in 0..self.cost_to_node.len() {
            match self.parent[current.0] {
                Some(parent) => {
                    path.push(parent);
                    current = parent;
                }
                None => {
                    path.reverse();
                    return Some(path);
                }
            }
        }
        None
    }
}

/// A* itself holds nothing between runs — every array it touches lives in [`Search`], which
/// is built per call. The unit struct exists to satisfy [`Planner`], not to carry state.
pub(crate) struct AStar;

impl Planner for AStar {
    fn plan(
        &mut self,
        world: &GridWorldManager<Cell>,
        src_id: NodeId,
        dest_id: NodeId,
    ) -> Result<Vec<NodeId>, PlanError> {
        let mut search = Search::new(world.len());
        search.cost_to_node[src_id.0] = 0;

        let mut nodes_to_expand = BinaryHeap::new();
        let start_f = world.octile_heuristic_h(src_id, dest_id);
        nodes_to_expand.push(Reverse((start_f, Reverse(0u32), src_id)));

        while let Some(Reverse((_f, _g, node))) = nodes_to_expand.pop() {
            if node == dest_id {
                // Reached, so this only fails on a corrupt parent chain — in which case
                // "no route" is the honest answer and beats hanging the request.
                return search.reconstruct(dest_id).ok_or(PlanError::Unreachable);
            }
            if search.closed[node.0] {
                continue;
            } // stale duplicate entry
            search.closed[node.0] = true;

            for next in world.passable_neighbors(node) {
                if search.closed[next.0] {
                    continue;
                }
                let tentative_g = search.cost_to_node[node.0] + world.step_cost(node, next);
                // `cost_to_node` starts at UNREACHED, so `<` doubles as the "first time
                // seen" case — no separate `reached` flag needed.
                if tentative_g < search.cost_to_node[next.0] {
                    search.cost_to_node[next.0] = tentative_g;
                    search.parent[next.0] = Some(node);
                    let f = tentative_g + world.octile_heuristic_h(next, dest_id);
                    // Ordered by `f`, then by *larger* `g` — equivalently smaller `h`,
                    // since `f = g + h`. Among nodes that look equally good, the one
                    // further along is likelier to reach the goal, and on open ground
                    // whole plateaus of equal `f` would otherwise be expanded breadth
                    // first. Tie order never affects optimality, only how much is
                    // touched getting there.
                    nodes_to_expand.push(Reverse((f, Reverse(tentative_g), next)));
                    // Pushing a duplicate beats a decrease-key: the stale entry is
                    // dropped by the `closed` check when it surfaces.
                }
            }
        }
        Err(PlanError::Unreachable)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::planners::world_from_ascii;

    fn empty_world(width: usize, height: usize) -> GridWorldManager<Cell> {
        GridWorldManager::new(width, height)
    }

    /// A* reached the way the handler reaches it. Naming the planner once here keeps the
    /// assertions below about the route rather than about the dispatch.
    fn plan(
        world: &GridWorldManager<Cell>,
        src: [i32; 2],
        dest: [i32; 2],
    ) -> Result<Vec<NodeId>, PlanError> {
        world.find_plan(src, dest, &mut AStar)
    }

    #[test]
    fn search_starts_with_every_node_unreached() {
        let search = Search::new(12);
        assert_eq!(search.cost_to_node.len(), 12);
        assert_eq!(search.parent.len(), 12);
        assert_eq!(search.closed.len(), 12);
        assert!(search.cost_to_node.iter().all(|&g| g == Search::UNREACHED));
        assert!(search.parent.iter().all(Option::is_none));
        assert!(!search.closed.iter().any(|&c| c));
        assert!(!search.reached(NodeId(0)));
    }

    #[test]
    fn reconstruct_walks_the_parent_chain_start_first() {
        let mut search = Search::new(9);
        // A hand-built chain 0 -> 4 -> 8, as a finished search would leave it.
        for (node, parent) in [(0, None), (4, Some(NodeId(0))), (8, Some(NodeId(4)))] {
            search.cost_to_node[node] = 0;
            search.parent[node] = parent;
        }

        assert_eq!(
            search.reconstruct(NodeId(8)),
            Some(vec![NodeId(0), NodeId(4), NodeId(8)]),
            "path must read start-first",
        );
        assert_eq!(
            search.reconstruct(NodeId(0)),
            Some(vec![NodeId(0)]),
            "start is its own path"
        );
    }

    #[test]
    fn reconstruct_returns_none_for_an_unreached_node() {
        let search = Search::new(4);
        assert_eq!(search.reconstruct(NodeId(3)), None);
    }

    #[test]
    fn reconstruct_gives_up_on_a_parent_chain_that_never_ends() {
        // Only reachable through a bug, but the whole point of the bound is that it
        // returns instead of spinning — so the cycle has to actually be exercised.
        let mut search = Search::new(4);
        for node in 0..4 {
            search.cost_to_node[node] = 0;
            search.parent[node] = Some(NodeId((node + 1) % 4));
        }
        assert_eq!(
            search.reconstruct(NodeId(0)),
            None,
            "a cycle must terminate as None"
        );
    }

    #[test]
    fn reconstruct_handles_a_chain_as_long_as_the_grid() {
        // The bound is the node count, and a path may legitimately visit every node.
        // One short and this would reject a valid path.
        let mut search = Search::new(5);
        for node in 0..5 {
            search.cost_to_node[node] = 0;
            search.parent[node] = node.checked_sub(1).map(NodeId);
        }
        assert_eq!(
            search.reconstruct(NodeId(4)),
            Some((0..5).map(NodeId).collect::<Vec<_>>()),
            "a path through every node must survive the cycle guard",
        );
    }

    #[test]
    fn optimal_path_without_obstacle_is_straight_line() {
        let world = empty_world(5, 5);
        let route = plan(&world, [0, 0], [3, 3]).expect("open grid");
        assert_eq!(route.len(), 4, "3 steps from (0,0) to (3,3)");
        assert_eq!(route[0], world.id(0, 0));
        assert_eq!(route[1], world.id(1, 1));
        assert_eq!(route[2], world.id(2, 2));
        assert_eq!(route[3], world.id(3, 3));
    }

    // --- searching around obstacles --------------------------------------

    fn coords(world: &GridWorldManager<Cell>, path: &[NodeId]) -> Vec<(usize, usize)> {
        path.iter().map(|&node| world.xy(node)).collect()
    }

    /// The properties a plan must have whichever route it picked: it runs from src to
    /// dest, never stands on a blocked cell, and only ever moves between legal steps.
    ///
    /// Asserted separately from cost because a path can be the right *price* and still
    /// be nonsense — a broken parent chain yields a cheap sequence that teleports.
    fn assert_walkable(world: &GridWorldManager<Cell>, path: &[NodeId], src: NodeId, dest: NodeId) {
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
    /// any graph without depending on the heuristic at all.
    ///
    /// That is the point: it shares the movement model with the code under test, so it does
    /// not check `movement_model`, but it does pin the two things a heuristic can break.
    /// An overestimating `h` shows up as a *dearer* path, not a visibly wrong one, and
    /// no hand-written expected route would catch it.
    fn dijkstra_cost(world: &GridWorldManager<Cell>, src: NodeId, dest: NodeId) -> Option<u32> {
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

    #[test]
    fn plan_routes_around_a_wall_through_its_gap() {
        let world = world_from_ascii(&[
            "....#....",
            "....#....",
            "....#....",
            "....#....",
            ".........",
        ]);
        let (src, dest) = (world.id(0, 0), world.id(8, 0));

        let route = plan(&world, [0, 0], [8, 0]).expect("the wall has a gap");
        assert_walkable(&world, &route, src, dest);
        assert!(
            route.iter().any(|&node| world.xy(node).1 == 4),
            "the only way past the wall is the bottom row, but the plan stayed above it: {:?}",
            coords(&world, &route),
        );
        assert_eq!(
            world.path_cost(&route),
            dijkstra_cost(&world, src, dest).unwrap()
        );
    }

    #[test]
    fn a_wall_with_no_gap_is_unreachable() {
        let world = world_from_ascii(&["..#..", "..#..", "..#.."]);
        assert_eq!(plan(&world, [0, 1], [4, 1]), Err(PlanError::Unreachable));
        // The two halves are each still internally reachable — the error above is the
        // wall, not a search that gives up on the first blocked neighbor.
        assert!(plan(&world, [0, 0], [1, 2]).is_ok());
    }

    #[test]
    fn a_diagonal_wall_cannot_be_squeezed_through() {
        // The corner rule at plan scale. Without it the search slips through the seam
        // where the blocked cells meet, and the plan clips straight across the wall.
        let world = world_from_ascii(&["...#", "..#.", ".#..", "#..."]);
        assert_eq!(
            plan(&world, [0, 0], [3, 3]),
            Err(PlanError::Unreachable),
            "a diagonal wall must be a wall",
        );
    }

    #[test]
    fn a_cell_walled_in_on_every_side_is_unreachable() {
        let world = world_from_ascii(&[".....", ".###.", ".#.#.", ".###.", "....."]);
        assert_eq!(
            plan(&world, [0, 0], [2, 2]),
            Err(PlanError::Unreachable),
            "into the pocket",
        );
        assert_eq!(
            plan(&world, [2, 2], [0, 0]),
            Err(PlanError::Unreachable),
            "out of the pocket",
        );
    }

    #[test]
    fn src_equal_to_dest_is_a_single_cell_plan() {
        let world = world_from_ascii(&["...", ".#.", "..."]);
        assert_eq!(plan(&world, [2, 0], [2, 0]), Ok(vec![world.id(2, 0)]));
    }

    #[test]
    fn plan_detours_around_expensive_terrain() {
        // Nothing is blocked here — the obstacle is purely the price of crossing row 0,
        // which the straight line would otherwise take.
        let world = world_from_ascii(&["..999..", ".......", "......."]);
        let (src, dest) = (world.id(0, 0), world.id(6, 0));

        let route = plan(&world, [0, 0], [6, 0]).expect("open map");
        assert_walkable(&world, &route, src, dest);
        assert!(
            route.iter().all(|&node| world[node].terrain_cost == 0),
            "the plan paid for costly ground it could have gone around: {:?}",
            coords(&world, &route),
        );
        // Down onto row 1, across, and back up: 14 + 4 * 10 + 14.
        assert_eq!(world.path_cost(&route), 68);
        // The detour has to actually be cheaper than the straight line for this test to
        // mean anything — priced from the map rather than asserted as a constant, so
        // retuning the step costs can't leave a tautology behind.
        let straight: Vec<NodeId> = (0..=6).map(|x| world.id(x, 0)).collect();
        assert!(world.path_cost(&route) < world.path_cost(&straight));
    }

    #[test]
    fn terrain_too_cheap_to_avoid_is_crossed() {
        // The mirror of the test above: the detour is only right when it is cheaper.
        // A cost of 1 does not justify two diagonals, so the plan should go straight.
        let world = world_from_ascii(&["..111..", ".......", "......."]);
        let route = plan(&world, [0, 0], [6, 0]).expect("open map");
        assert_eq!(
            coords(&world, &route),
            (0..=6).map(|x| (x, 0)).collect::<Vec<_>>(),
            "3 units of terrain is cheaper than leaving the row and coming back",
        );
    }

    #[test]
    fn plan_matches_dijkstra_on_random_obstacle_maps() {
        // The real regression net. Hand-drawn maps pin routes we already understand;
        // this sweeps mazes nobody looked at and checks the two claims that matter —
        // a plan exists exactly when one exists, and it costs exactly the optimum.
        const N: usize = 10;

        // Same xorshift as the rasterization sweep, to stay deterministic without rand.
        let mut seed: u64 = 0x2026_0804;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };

        for trial in 0..400 {
            let mut world: GridWorldManager<Cell> = GridWorldManager::new(N, N);
            for id in 0..world.len() {
                let cell = &mut world[NodeId(id)];
                // ~30% blocked: dense enough that many pairs are genuinely unreachable,
                // sparse enough that most are not.
                cell.blocked = next() % 10 < 3;
                cell.terrain_cost = (next() % 5) as u16;
            }

            let src = NodeId((next() as usize) % world.len());
            let dest = NodeId((next() as usize) % world.len());
            world[src].blocked = false;
            world[dest].blocked = false;

            let expected = dijkstra_cost(&world, src, dest);
            // Straight at the trait impl rather than through `find_plan`: both
            // endpoints are forced clear just above, so the endpoint checks have nothing to
            // say here and the search is what's under test.
            let found = AStar.plan(&world, src, dest);

            match (found, expected) {
                (Ok(route), Some(expected)) => {
                    assert_walkable(&world, &route, src, dest);
                    assert_eq!(
                        world.path_cost(&route),
                        expected,
                        "trial {trial}: {:?} -> {:?} is not optimal",
                        world.xy(src),
                        world.xy(dest),
                    );
                }
                (Err(PlanError::Unreachable), None) => {}
                (found, _) => panic!(
                    "trial {trial}: A* returned {found:?} from {:?} to {:?}, but Dijkstra {}",
                    world.xy(src),
                    world.xy(dest),
                    match expected {
                        Some(cost) => format!("found a path costing {cost}"),
                        None => "found none".to_string(),
                    },
                ),
            }
        }
    }
}
