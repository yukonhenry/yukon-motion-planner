//! D* Lite over a [`GridWorldManager<Cell>`], static-world first cut.
//!
//! [Koenig & Likhachev's](https://www.aaai.org/Papers/AAAI/2002/AAAI02-068.pdf) algorithm,
//! and the reason [`Planner`] takes `&mut self`. Two things
//! make it different from [`a_star`](super::a_star).
//!
//! **It searches backwards.** `g[s]` estimates the cost from `s` *to the goal*, the queue is
//! seeded with the goal, and the path is read off afterwards by walking downhill from the
//! start. That inversion is what makes the state reusable: when the world changes near the
//! robot, costs *to the goal* mostly don't, so most of `g` survives.
//!
//! **It is consistency-driven rather than closed-set-driven.** A node is *locally consistent*
//! when `g[s] == rhs[s]`, where `rhs` is a one-step lookahead over successors. The search does
//! not close nodes; it repairs inconsistent ones until the start is consistent and nothing
//! cheaper than the start is left in the queue. Repair is exactly the operation an edge-cost
//! change needs, which is why the incremental version is a small addition rather than a
//! rewrite.
//!
//! What is *not* here yet: `update_edge_costs` and the moving-robot loop. With a static world
//! and a fixed start there is nothing to reuse between calls, so [`DStarLite::plan`] builds its
//! state, runs one repair pass, and drops it — see the note on [`DStarLite`]. `k_m` is carried
//! through the key arithmetic anyway, pinned at zero, because leaving it out and adding it
//! later means re-deriving every key expression.
//!
//! One consequence worth knowing: with `k_m` pinned, the key-refile branch in
//! [`Repair::compute_shortest_path`] cannot be reached, and no test here covers it. Everything
//! else is exercised, the underconsistent branch included — a cost *rise* drives that one, and
//! `a_risen_cost_is_repaired_rather_than_believed` drives it by hand.

use crate::models::cell::Cell;
use crate::models::grid_world_manager::{GridWorldManager, NodeId};
use crate::models::planners::{PlanError, Planner};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// "No known route to the goal". A sentinel rather than `Option<u32>` so `g` and `rhs` stay flat
/// arrays of plain integers, matching A*'s `Search::UNREACHED`.
///
/// Every arithmetic use is saturating: `INF + anything` must stay `INF` rather than wrap to a
/// small number, which would read as a suspiciously good route.
const INF: u32 = u32::MAX;

/// A queue priority, compared lexicographically.
///
/// `k1` is the f-value — the best estimate through this node, offset by `k_m` — and `k2` is the
/// g-value that ties are broken on. A plain tuple gets the ordering for free.
type Key = (u32, u32);

/// D* Lite.
///
/// A unit struct *for now*, deliberately: with a static world and a fixed start, state kept
/// between [`plan`](Planner::plan) calls could never be reused, and a field nothing reads is
/// worse than no field. The incremental version is what fills this in — it holds [`Repair`]
/// across calls, and `plan` becomes "reuse if the goal is unchanged, else rebuild". The trait's
/// `&mut self` is already the right shape for that.
pub(crate) struct DStarLite;

/// The state one repair pass works over. Every array is `world.len()` long, indexed by
/// `NodeId.0`.
struct Repair {
    /// Cost from each node *to the goal*, as currently believed.
    g: Vec<u32>,
    /// One step of lookahead: `min` over successors of `step_cost + g[successor]`. `g` and `rhs`
    /// agreeing is what "locally consistent" means.
    rhs: Vec<u32>,
    /// Inconsistent nodes waiting to be repaired, cheapest key first.
    ///
    /// Lazily deleted, like A*'s heap here: a node's key changes as its `rhs` moves, and rather
    /// than sift an existing entry we push a new one and let the old surface and be discarded.
    /// `queue_key` is the arbiter of which entry is live.
    queue: BinaryHeap<Reverse<(Key, usize)>>,
    /// The key each node is *currently* queued under, or `None` if it is not queued. A popped
    /// entry whose key doesn't match this is stale and skipped.
    queue_key: Vec<Option<Key>>,
    /// Heuristic offset accumulated as the start moves. Pinned at zero here; it exists so the
    /// key arithmetic is already correct when the robot starts moving.
    k_m: u32,
    start: NodeId,
    goal: NodeId,
}

impl Repair {
    fn new(world: &GridWorldManager<Cell>, start: NodeId, goal: NodeId) -> Self {
        let mut repair = Self {
            g: vec![INF; world.len()],
            rhs: vec![INF; world.len()],
            queue: BinaryHeap::new(),
            queue_key: vec![None; world.len()],
            k_m: 0,
            start,
            goal,
        };

        // The goal costs nothing to reach from itself, which makes it the one inconsistent node
        // to begin with — `g` is INF everywhere, so the whole search grows out of this seed.
        repair.rhs[goal.0] = 0;
        let key = repair.key(world, goal);
        repair.enqueue(goal, key);
        repair
    }

    /// `[min(g, rhs) + h(start, node) + k_m, min(g, rhs)]`.
    ///
    /// `h` is measured from the *start*, not the goal: the search runs backwards, so this is
    /// what keeps it focused on the region between the two rather than expanding out from the
    /// goal in every direction. [`octile_heuristic_h`](GridWorldManager::octile_heuristic_h) is
    /// symmetric and admissible in both directions, so reading it backwards like this is sound.
    fn key(&self, world: &GridWorldManager<Cell>, node: NodeId) -> Key {
        let best = self.g[node.0].min(self.rhs[node.0]);
        (
            best.saturating_add(world.octile_heuristic_h(self.start, node))
                .saturating_add(self.k_m),
            best,
        )
    }

    fn enqueue(&mut self, node: NodeId, key: Key) {
        self.queue_key[node.0] = Some(key);
        self.queue.push(Reverse((key, node.0)));
    }

    /// Marks `node` as no longer queued. The heap entry itself is left to be discarded when it
    /// surfaces, which is what makes this O(1).
    fn dequeue(&mut self, node: NodeId) {
        self.queue_key[node.0] = None;
    }

    /// The cheapest *live* entry, discarding stale ones off the top on the way.
    fn top(&mut self) -> Option<(Key, NodeId)> {
        while let Some(&Reverse((key, index))) = self.queue.peek() {
            if self.queue_key[index] == Some(key) {
                return Some((key, NodeId(index)));
            }
            self.queue.pop();
        }
        None
    }

    /// Recomputes `rhs[node]` from its successors and re-files the node by whether that left it
    /// consistent.
    ///
    /// The goal is exempt: its `rhs` is 0 by definition, and recomputing it from successors
    /// would talk it out of being the goal.
    fn update_vertex(&mut self, world: &GridWorldManager<Cell>, node: NodeId) {
        if node != self.goal {
            // `step_cost(node, next)` and not the reverse: cost is charged for *entering* a
            // cell, so the direction is load-bearing even though the edges themselves are
            // symmetric. Reading it backwards is invisible on a uniform map, and even on a
            // costly one it leaves the chosen *route* unchanged while mispricing it — see
            // `a_backwards_search_prices_the_direction_it_travels`.
            self.rhs[node.0] = world
                .passable_neighbors(node)
                .map(|next| self.g[next.0].saturating_add(world.step_cost(node, next)))
                .min()
                .unwrap_or(INF);
        }

        self.dequeue(node);
        if self.g[node.0] != self.rhs[node.0] {
            let key = self.key(world, node);
            self.enqueue(node, key);
        }
    }

    /// Repairs inconsistent nodes until the start is consistent and nothing in the queue could
    /// still improve it.
    fn compute_shortest_path(&mut self, world: &GridWorldManager<Cell>) {
        while let Some((k_old, node)) = self.top() {
            // Stop once the start is consistent *and* no queued node outranks it. Both halves
            // are needed: the first alone would stop before the start's value is settled, and
            // the second alone would keep repairing a region that can no longer matter.
            let start_key = self.key(world, self.start);
            let start_settled = self.g[self.start.0] == self.rhs[self.start.0];
            if k_old >= start_key && start_settled {
                break;
            }

            let k_new = self.key(world, node);
            if k_old > k_new {
                // Queued under a stale `k_m`: re-file at the current key and take it again
                // later.
                //
                // Cannot fire yet, and not merely because `k_m` is zero — `update_vertex`
                // re-files eagerly, so a live entry's key is always current, and `k_m` is the
                // only thing that can shift a key without touching the node. It is kept because
                // it is what makes the queue's ordering survive a moving start, and deriving it
                // again later is harder than leaving it here. The one branch in this file no
                // test reaches; see the module note.
                self.enqueue(node, k_new);
            } else if self.g[node.0] > self.rhs[node.0] {
                // Overconsistent: the lookahead found something better than `g` claims, so `g`
                // can simply take it. This is the ordinary case, and the one that does the work
                // on a first run — it is A*'s expansion wearing different clothes.
                self.g[node.0] = self.rhs[node.0];
                self.dequeue(node);
                for pred in world.passable_neighbors(node) {
                    self.update_vertex(world, pred);
                }
            } else {
                // Underconsistent: `g` is stale-optimistic, promising a route that no longer
                // prices out. Invalidate it and let this node and its predecessors re-derive
                // their own values. Unreachable on a first run over a static world — nothing
                // has yet made `g` too good — but it is the branch a cost *increase* takes, so
                // it is written now rather than bolted on later.
                self.g[node.0] = INF;
                for pred in world.passable_neighbors(node) {
                    self.update_vertex(world, pred);
                }
                self.update_vertex(world, node);
            }
        }
        // Falling out because the queue emptied is not a special case: an unreachable start
        // keeps `g == rhs == INF`, which is *consistent*, so the loop would have broken anyway
        // on the next iteration. `extract_path` reads the INF and reports it.
    }

    /// Walks downhill from the start, taking the cheapest `step_cost + g` at each node.
    ///
    /// The path is not recorded during the search — with correct `g` values this reads it back
    /// exactly, and an array of parents that a repair would have to keep in step is an array of
    /// parents that can disagree with `g`.
    fn extract_path(&self, world: &GridWorldManager<Cell>) -> Result<Vec<NodeId>, PlanError> {
        if self.g[self.start.0] == INF {
            return Err(PlanError::Unreachable);
        }

        let mut path = vec![self.start];
        let mut node = self.start;

        // Each hop strictly decreases `g` by at least one step's cost, so this cannot cycle and
        // no path can exceed the node count. The bound guards against a corrupt `g` spinning
        // inside a request handler; it is not an expected exit — same reasoning as A*'s
        // `reconstruct`.
        for _ in 0..world.len() {
            if node == self.goal {
                return Ok(path);
            }
            let next = world
                .passable_neighbors(node)
                .filter(|next| self.g[next.0] != INF)
                .min_by_key(|&next| self.g[next.0].saturating_add(world.step_cost(node, next)))
                .ok_or(PlanError::Unreachable)?;
            path.push(next);
            node = next;
        }
        Err(PlanError::Unreachable)
    }
}

impl Planner for DStarLite {
    fn plan(
        &mut self,
        world: &GridWorldManager<Cell>,
        src: NodeId,
        dest: NodeId,
    ) -> Result<Vec<NodeId>, PlanError> {
        let mut repair = Repair::new(world, src, dest);
        repair.compute_shortest_path(world);
        repair.extract_path(world)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::planners::a_star::AStar;
    use crate::models::planners::test_support::*;

    /// D* Lite reached the way the handler reaches it.
    fn plan(
        world: &GridWorldManager<Cell>,
        src: [i32; 2],
        dest: [i32; 2],
    ) -> Result<Vec<NodeId>, PlanError> {
        world.find_plan(src, dest, &mut DStarLite)
    }

    #[test]
    fn optimal_path_without_obstacle_is_straight_line() {
        let world = empty_world(5, 5);
        let route = plan(&world, [0, 0], [3, 3]).expect("open grid");
        assert_eq!(
            coords(&world, &route),
            vec![(0, 0), (1, 1), (2, 2), (3, 3)],
            "3 diagonal steps and nothing else",
        );
    }

    #[test]
    fn src_equal_to_dest_is_a_single_cell_plan() {
        // The degenerate case a backwards search is likeliest to fumble: the start *is* the
        // seed, so it is already consistent before the loop does anything.
        let world = world_from_ascii(&["...", ".#.", "..."]);
        assert_eq!(plan(&world, [2, 0], [2, 0]), Ok(vec![world.id(2, 0)]));
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
            dijkstra_cost(&world, src, dest).unwrap(),
        );
    }

    #[test]
    fn plan_detours_around_expensive_terrain() {
        let world = world_from_ascii(&["..999..", ".......", "......."]);
        let (src, dest) = (world.id(0, 0), world.id(6, 0));

        let route = plan(&world, [0, 0], [6, 0]).expect("open map");
        assert_walkable(&world, &route, src, dest);
        assert!(
            route.iter().all(|&node| world[node].terrain_cost == 0),
            "the plan paid for costly ground it could have gone around: {:?}",
            coords(&world, &route),
        );
        assert_eq!(world.path_cost(&route), 68, "down, across, and back up");
    }

    // --- what a backwards search can get wrong ---------------------------

    #[test]
    fn a_backwards_search_prices_the_direction_it_travels() {
        // The bug D* Lite invites here: an `rhs` that reads `step_cost(next, node)` instead of
        // `step_cost(node, next)`. `step_cost` charges the terrain of the cell being *entered*,
        // so the two are not the same edge, and the mistake is easy to make in a search that
        // runs goal-to-start while its costs run start-to-goal.
        //
        // Asserted against `g[start]` — the cost the search *computed* — rather than against the
        // returned route, and that distinction is the whole test. Two routes between the same
        // endpoints differ in cost only by their interiors: the endpoints' terrain appears in
        // both, so it cancels, and the cheapest route has the same shape whichever way the
        // edges are read. A reversed reading therefore never changes the path — it only
        // misprices it, and only a cost assertion can see that.
        let world = world_from_ascii(&["9..0"]);
        let (left, right) = (world.id(0, 0), world.id(3, 0));

        for (src, dest) in [(left, right), (right, left)] {
            let mut repair = Repair::new(&world, src, dest);
            repair.compute_shortest_path(&world);

            assert_eq!(
                repair.g[src.0],
                dijkstra_cost(&world, src, dest).unwrap(),
                "{:?} -> {:?} was priced as though it ran the other way",
                world.xy(src),
                world.xy(dest),
            );
        }

        // And the two directions really do differ, or the check above proves nothing: rightward
        // leaves the 9 behind and pays for a 0, leftward the reverse.
        assert_eq!(dijkstra_cost(&world, left, right), Some(30));
        assert_eq!(dijkstra_cost(&world, right, left), Some(39));
    }

    // --- unreachable goals -----------------------------------------------

    #[test]
    fn a_wall_with_no_gap_is_unreachable() {
        let world = world_from_ascii(&["..#..", "..#..", "..#.."]);
        assert_eq!(plan(&world, [0, 1], [4, 1]), Err(PlanError::Unreachable));
        // Each half is still internally reachable — the error above is the wall, not a search
        // that gave up the moment it met a blocked neighbor.
        assert!(plan(&world, [0, 0], [1, 2]).is_ok());
    }

    #[test]
    fn a_diagonal_wall_cannot_be_squeezed_through() {
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

    // --- against the oracles ---------------------------------------------

    #[test]
    fn plan_matches_dijkstra_on_random_obstacle_maps() {
        // The real regression net, and the same sweep A* faces. Hand-drawn maps pin routes we
        // already understand; this sweeps mazes nobody looked at and checks the two claims that
        // matter — a plan exists exactly when one exists, and it costs exactly the optimum.
        const N: usize = 10;
        let mut next = xorshift(0x2026_0811);

        for trial in 0..400 {
            let mut world: GridWorldManager<Cell> = GridWorldManager::new(N, N);
            for id in 0..world.len() {
                let cell = &mut world[NodeId(id)];
                cell.blocked = next() % 10 < 3;
                cell.terrain_cost = (next() % 5) as u16;
            }

            let src = NodeId((next() as usize) % world.len());
            let dest = NodeId((next() as usize) % world.len());
            world[src].blocked = false;
            world[dest].blocked = false;

            let expected = dijkstra_cost(&world, src, dest);
            let found = DStarLite.plan(&world, src, dest);

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
                    "trial {trial}: D* Lite returned {found:?} from {:?} to {:?}, but Dijkstra {}",
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

    #[test]
    fn d_lite_and_a_star_agree_on_price_if_not_on_route() {
        // Two optimal planners must agree on cost everywhere, and need not agree on which of
        // several equally cheap routes to take. Asserting the routes match would pin tie-break
        // order that neither planner promises; asserting the costs match is the real contract.
        //
        // Every pair on one awkward map rather than random pairs across many: this is about the
        // two planners agreeing, and exhausting a single map finds more disagreement per trial
        // than sampling would.
        let world = world_from_ascii(&[
            ".2..#....",
            "..#..3...",
            ".#...#...",
            "...9#..#.",
            "..#....#.",
            "1...#..2.",
        ]);

        for (src, _) in world.iter() {
            if !world.passable(src) {
                continue;
            }
            for (dest, _) in world.iter() {
                if !world.passable(dest) {
                    continue;
                }
                let by_a_star = AStar.plan(&world, src, dest);
                let by_d_lite = DStarLite.plan(&world, src, dest);

                match (by_a_star, by_d_lite) {
                    (Ok(a), Ok(d)) => {
                        assert_walkable(&world, &d, src, dest);
                        assert_eq!(
                            world.path_cost(&a),
                            world.path_cost(&d),
                            "{:?} -> {:?}: A* priced {:?}, D* Lite {:?}",
                            world.xy(src),
                            world.xy(dest),
                            coords(&world, &a),
                            coords(&world, &d),
                        );
                    }
                    (Err(a), Err(d)) => assert_eq!(a, d),
                    (a, d) => panic!(
                        "{:?} -> {:?}: A* said {a:?}, D* Lite said {d:?}",
                        world.xy(src),
                        world.xy(dest),
                    ),
                }
            }
        }
    }

    /// A world the size the sweeps below use, built from `next`.
    ///
    /// Opposite corners rather than random endpoints: the point of a large grid is a long search
    /// that crosses the whole map, and a random pair is usually neighbours.
    fn wide_open_maze(
        n: usize,
        next: &mut impl FnMut() -> u64,
    ) -> (GridWorldManager<Cell>, NodeId, NodeId) {
        let mut world: GridWorldManager<Cell> = GridWorldManager::new(n, n);
        for id in 0..world.len() {
            let cell = &mut world[NodeId(id)];
            // Sparser than the 10x10 sweeps: at 30% blocked a 60x60 grid is almost always
            // disconnected, and every trial would come back `Unreachable` having proved little.
            cell.blocked = next() % 10 < 2;
            cell.terrain_cost = (next() % 5) as u16;
        }
        let (src, dest) = (NodeId(0), NodeId(world.len() - 1));
        world[src].blocked = false;
        world[dest].blocked = false;
        (world, src, dest)
    }

    #[test]
    fn the_planners_agree_on_price_across_large_random_maps() {
        // The same claim as `d_lite_and_a_star_agree_on_price_if_not_on_route`, at a scale where
        // the two planners genuinely diverge. On a 9x6 map their frontiers overlap almost
        // entirely; over 60x60 they explore substantially different regions — A* outward from the
        // start, D* Lite backwards from the goal — so agreeing here is a much stronger statement
        // than agreeing there.
        //
        // Costs only, again: the routes differ on most of these maps, and legitimately so.
        let mut next = xorshift(0xBEEF);

        for trial in 0..40 {
            let (world, src, dest) = wide_open_maze(60, &mut next);

            let by_a_star = AStar.plan(&world, src, dest);
            let by_d_lite = DStarLite.plan(&world, src, dest);

            match (by_a_star, by_d_lite) {
                (Ok(a), Ok(d)) => {
                    assert_walkable(&world, &d, src, dest);
                    assert_eq!(
                        world.path_cost(&a),
                        world.path_cost(&d),
                        "trial {trial}: A* priced {} and D* Lite {}",
                        world.path_cost(&a),
                        world.path_cost(&d),
                    );
                }
                (Err(a), Err(d)) => assert_eq!(a, d, "trial {trial}"),
                (a, d) => panic!("trial {trial}: A* said {a:?}, D* Lite said {d:?}"),
            }
        }
    }

    /// What the two planners cost when run from scratch, printed rather than asserted.
    ///
    /// `#[ignore]`d on purpose: a wall-clock assertion is a test that fails on a loaded machine
    /// and tells you nothing about correctness. This exists to be *read* —
    /// `cargo test --release -- --ignored --nocapture` — and the release profile matters, since a
    /// debug build's bounds checking flatters neither planner evenly.
    ///
    /// What it is for: D* Lite run statelessly is a backwards A* carrying machinery it cannot yet
    /// use. Expanding a node calls `update_vertex` on each of ~8 predecessors, and each of those
    /// rescans its own ~8 neighbours to recompute `rhs` — so it pays roughly 64 neighbour
    /// inspections per expansion where A* pays 8, and A* additionally stops the moment it pops the
    /// goal. Measured at 60x60 that came out around 4x slower.
    ///
    /// That ratio is the number incremental replanning has to beat. One jittered corner
    /// invalidates a handful of cells, so a repair that reuses its state should be far cheaper
    /// than either figure here — and if it is not, the repair is touching too much.
    #[test]
    #[ignore = "a benchmark, not a check — run with --release --ignored --nocapture"]
    fn bench_stateless_replanning_costs_more_than_a_star() {
        use std::time::Instant;

        let mut next = xorshift(0xBEEF);
        let worlds: Vec<_> = (0..40).map(|_| wide_open_maze(60, &mut next)).collect();

        // Planned first so neither timing pays for the allocation, and so a wrong answer is
        // reported as wrong rather than as fast.
        let a_costs: Vec<_> = worlds
            .iter()
            .map(|(w, s, d)| AStar.plan(w, *s, *d).map(|r| w.path_cost(&r)))
            .collect();
        let d_costs: Vec<_> = worlds
            .iter()
            .map(|(w, s, d)| DStarLite.plan(w, *s, *d).map(|r| w.path_cost(&r)))
            .collect();
        assert_eq!(
            a_costs, d_costs,
            "the planners disagreed — timings are moot"
        );

        let start = Instant::now();
        for (world, src, dest) in &worlds {
            let _ = AStar.plan(world, *src, *dest);
        }
        let a_star = start.elapsed();

        let start = Instant::now();
        for (world, src, dest) in &worlds {
            let _ = DStarLite.plan(world, *src, *dest);
        }
        let d_star_lite = start.elapsed();

        let differing = worlds
            .iter()
            .filter(|(w, s, d)| AStar.plan(w, *s, *d) != DStarLite.plan(w, *s, *d))
            .count();

        println!(
            "\n{} maps at 60x60, from scratch:\n  \
             a_star       {a_star:>12.2?}\n  \
             d_star_lite  {d_star_lite:>12.2?}   ({:.2}x)\n  \
             same cost every time, different route on {differing}/{}\n",
            worlds.len(),
            d_star_lite.as_secs_f64() / a_star.as_secs_f64(),
            worlds.len(),
        );
    }

    // --- the machinery ---------------------------------------------------

    #[test]
    fn every_reached_node_ends_locally_consistent() {
        // The invariant the algorithm is built on, and one a wrong `update_vertex` can break
        // while still returning a plausible path: once the pass settles, no node the search
        // reached may still disagree with its own lookahead.
        //
        // Nodes the search never needed are exempt — leaving those inconsistent is precisely how
        // D* Lite avoids doing A*'s whole job.
        let world = world_from_ascii(&["....", ".#3.", ".#..", "...."]);
        let (src, dest) = (world.id(0, 0), world.id(3, 3));

        let mut repair = Repair::new(&world, src, dest);
        repair.compute_shortest_path(&world);

        for (node, _) in world.iter() {
            if !world.passable(node) || repair.g[node.0] == INF {
                continue;
            }
            let lookahead = if node == dest {
                0
            } else {
                world
                    .passable_neighbors(node)
                    .map(|next| repair.g[next.0].saturating_add(world.step_cost(node, next)))
                    .min()
                    .unwrap_or(INF)
            };
            assert_eq!(
                repair.g[node.0],
                lookahead,
                "{:?} settled inconsistent",
                world.xy(node),
            );
        }
    }

    #[test]
    fn the_goal_is_the_only_node_that_costs_nothing() {
        // `g` is cost-to-goal, so a zero anywhere else means the backwards search leaked its
        // seed — the failure that would make every route look free.
        let world = world_from_ascii(&["....", "..#.", "...."]);
        let (src, dest) = (world.id(0, 0), world.id(3, 2));

        let mut repair = Repair::new(&world, src, dest);
        repair.compute_shortest_path(&world);

        for (node, _) in world.iter() {
            assert_eq!(
                repair.g[node.0] == 0,
                node == dest,
                "{:?} costs nothing to reach the goal from",
                world.xy(node),
            );
        }
    }

    #[test]
    fn a_risen_cost_is_repaired_rather_than_believed() {
        // Covers the underconsistent branch, which nothing else here reaches: on a first pass
        // over a static world no `g` is ever too *good*, so only a cost that goes up can drive
        // it. Reached by hand rather than through a public API because `update_edge_costs` is
        // not written yet — this pins the machinery that one will be built on, and is the test
        // that fails first if the branch is wrong.
        //
        // Raising terrain on the direct diagonal should push the route onto the clear row.
        let mut world = world_from_ascii(&["....", "....", "...."]);
        let (src, dest) = (world.id(0, 0), world.id(3, 0));

        let mut repair = Repair::new(&world, src, dest);
        repair.compute_shortest_path(&world);
        assert_eq!(repair.g[src.0], 30, "3 orthogonal steps along the top row");

        // Make the top row dear. `step_cost` charges the terrain of the cell *entered*, so the
        // edges whose cost changed are the ones leading into each mutated cell — meaning it is
        // those cells' *neighbors* whose lookahead is now stale.
        let risen = [world.id(1, 0), world.id(2, 0)];
        for &cell in &risen {
            world[cell].terrain_cost = 9;
        }
        for &cell in &risen {
            for neighbor in world.passable_neighbors(cell).collect::<Vec<_>>() {
                repair.update_vertex(&world, neighbor);
            }
            repair.update_vertex(&world, cell);
        }
        repair.compute_shortest_path(&world);

        let expected = dijkstra_cost(&world, src, dest).expect("still reachable");
        assert_eq!(
            repair.g[src.0], expected,
            "the repair kept believing the route it had already priced",
        );

        let route = repair.extract_path(&world).expect("still reachable");
        assert_walkable(&world, &route, src, dest);
        assert_eq!(world.path_cost(&route), expected);
        assert!(
            route.iter().any(|&node| world.xy(node).1 == 1),
            "the risen row should have pushed the route down: {:?}",
            coords(&world, &route),
        );
    }

    #[test]
    fn a_stale_queue_entry_is_discarded_rather_than_repaired_twice() {
        // Lazy deletion's one contract: `queue_key` decides which heap entry is live, whichever
        // order the heap hands them back.
        let world = empty_world(3, 3);
        let (src, dest) = (world.id(0, 0), world.id(2, 2));
        let mut repair = Repair::new(&world, src, dest);
        // Drop the goal seed so the queue holds only what this test puts in it.
        repair.dequeue(dest);

        let node = world.id(1, 1);

        // A key that improved: the cheap entry is the live one, and the heap offers it first.
        repair.enqueue(node, (500, 500));
        repair.enqueue(node, (100, 100));
        assert_eq!(
            repair.top(),
            Some(((100, 100), node)),
            "the newer key must win",
        );

        // A key that got *worse* — what an underconsistent repair does. Here the heap offers
        // the stale entry first, so it is only skipped if `queue_key` is actually consulted.
        repair.enqueue(node, (700, 700));
        assert_eq!(
            repair.top(),
            Some(((700, 700), node)),
            "a raised key must not still be found at its old cheap position",
        );

        repair.dequeue(node);
        assert_eq!(
            repair.top(),
            None,
            "every entry for a dequeued node must be gone",
        );
    }
}
