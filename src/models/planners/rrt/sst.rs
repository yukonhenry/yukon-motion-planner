//! Stable Sparse RRT — asymptotically near-optimal kinodynamic planning without a steering
//! function.
//!
//! The problem this solves is the one [`KinodynamicEdgeGenerator`] creates. Forward
//! propagation gives exactly feasible edges but cannot land on a *prescribed* state, so RRT\*'s
//! rewire step — reconnect a node to a chosen parent — has nothing to attach to and never
//! fires. SST (Li, Littlefield & Bekris, IJRR 2016) reaches near-optimality a different way,
//! with no boundary value problem anywhere:
//!
//! * **Best-first selection.** Rather than extending from the node *nearest* a sample, extend
//!   from the cheapest-to-reach node within [`selection_radius`](SstParams::selection_radius)
//!   of it. Growth is biased toward branches that are actually cheap, which is where RRT
//!   throws its optimality away.
//! * **Witness-based pruning.** The space is covered by witnesses of radius
//!   [`witness_radius`](SstParams::witness_radius), and each witness keeps only the single
//!   cheapest node near it. A node beaten at its own witness is deactivated, and pruned
//!   outright once it has no children left.
//!
//! The second mechanism is what makes the tree *sparse*, and it is the property that matters
//! most here: node count is bounded by the witness set rather than by the iteration count, so
//! a long search does not degenerate the way a plain kinodynamic RRT's does. For replanning
//! under moving obstacles — which is what this simulator is for — a tree that stays small is
//! worth more than a marginally shorter path.
//!
//! What is given up: this is near-optimal with respect to δ-robust solutions, not optimal, and
//! the two radii are real tuning parameters rather than incidental constants.

use crate::models::planners::rrt::kino_dynamic_edge_generator::KinodynamicEdgeGenerator;
use crate::models::planners::rrt::planning_world::PlanningWorld;
use crate::models::robots::unicycle_state::{UnicycleControl, UnicycleState, wrap_angle};
use crate::models::scale::METERS_PER_CELL;
use rand::Rng;

/// How far apart two states are, for selection and for sparsification.
///
/// Weighted rather than Euclidean-over-position, because position alone is wrong for a
/// second-order system: a node pointed the wrong way, or carrying speed it cannot shed, is not
/// "near" anything it cannot turn or decelerate to reach. A metric that ignores that picks
/// parents the dynamics cannot extend from, and the tree wastes its samples rediscovering it.
///
/// The weights are squared-space, applied before the square root, and mix units on purpose —
/// they are the exchange rate between a metre, a radian, and a metre per second. They are the
/// main thing to tune alongside the two radii.
#[derive(Debug, Clone, Copy)]
pub(crate) struct StateMetric {
    pub position: f64,
    pub heading: f64,
    pub linear_velocity: f64,
    pub angular_velocity: f64,
}

impl Default for StateMetric {
    fn default() -> Self {
        // Position dominant, heading worth roughly half a metre per radian, and the velocity
        // terms present but small — enough to stop a fast node and a stopped one at the same
        // place being treated as the same state, without letting speed swamp geometry.
        Self {
            position: 1.0,
            heading: 0.3,
            linear_velocity: 0.2,
            angular_velocity: 0.1,
        }
    }
}

impl StateMetric {
    pub fn distance(&self, a: &UnicycleState, b: &UnicycleState) -> f64 {
        let (dx, dy) = (a.x - b.x, a.y - b.y);
        // Wrapped, so a heading of +pi and one of -pi are the same heading rather than the
        // furthest apart two headings can be.
        let dtheta = wrap_angle(a.theta - b.theta);
        let dv = a.v - b.v;
        let domega = a.omega - b.omega;

        (self.position * (dx * dx + dy * dy)
            + self.heading * dtheta * dtheta
            + self.linear_velocity * dv * dv
            + self.angular_velocity * domega * domega)
            .sqrt()
    }
}

/// The metric extent the planner samples within.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Bounds {
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
}

impl Bounds {
    /// The metric box a grid of this size occupies.
    pub fn from_grid(width: i32, height: i32) -> Self {
        Self {
            min_x: 0.0,
            max_x: width as f64 * METERS_PER_CELL,
            min_y: 0.0,
            max_y: height as f64 * METERS_PER_CELL,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct SstParams {
    /// δ_BN — the ball around a sample within which the *cheapest* node is selected.
    ///
    /// Larger biases harder toward low-cost branches at the price of exploration; smaller
    /// degenerates toward plain nearest-neighbor RRT. Must exceed `witness_radius` for
    /// selection to have anything to choose between.
    pub selection_radius: f64,
    /// δ_s — the sparsification radius. One node survives per witness ball, so this sets the
    /// resolution of the tree and, with it, how large the tree can grow at all.
    pub witness_radius: f64,
    /// How close, in metres, counts as reaching the goal.
    ///
    /// Position only: arrival speed and heading are unconstrained, which is a baseline
    /// simplification and a visible one — the robot may arrive at full tilt. Constraining it
    /// means adding the velocity terms to this test, and makes the problem markedly harder.
    pub goal_radius: f64,
    /// How often the sample is taken at the goal rather than at random.
    pub goal_bias: f64,
    pub iterations: usize,
    pub metric: StateMetric,
    /// Sample `closest_approach` into [`SearchStats::progress`] every this many iterations.
    ///
    /// `0` disables it, which is the default: the sampling itself is trivial, but the vector
    /// it fills is pure diagnostics and a planner in a replanning loop should not accumulate
    /// one per tick.
    pub progress_every: usize,
    /// Record a replayable [`SearchEvent`] trace. Off by default: a trace is a few megabytes
    /// and nothing in a running simulation reads one.
    pub trace: bool,
}

impl Default for SstParams {
    fn default() -> Self {
        Self {
            selection_radius: 1.5,
            witness_radius: 0.6,
            goal_radius: 1.0,
            goal_bias: 0.05,
            iterations: 5_000,
            metric: StateMetric::default(),
            progress_every: 0,
            trace: false,
        }
    }
}

/// A trajectory from the start state to the goal region.
///
/// `states` and `controls` are the concatenation of the tree edges along the solution branch,
/// with the duplicated join states removed — so `controls[i]` applied to `states[i]` for `dt`
/// yields `states[i + 1]`, all the way along. That property is what makes this a trajectory
/// rather than a list of waypoints.
#[derive(Debug, Clone)]
pub(crate) struct SstSolution {
    pub states: Vec<UnicycleState>,
    pub controls: Vec<UnicycleControl>,
    /// Duration in seconds — the cost SST minimizes.
    pub cost: f64,
}

/// At most this many poses are kept per edge in a trace.
///
/// An edge is up to `max_extension_steps` states — 25 at 20ms — and at a metre per cell that
/// whole edge spans under a fifth of a cell on screen. Keeping every state would multiply the
/// trace's size for detail no viewer can resolve, so it is thinned to endpoints plus enough
/// in between to keep a curve looking like one.
const TRACE_POSES_PER_EDGE: usize = 6;

/// One thing that happened to the tree, in the order it happened.
///
/// Enough to replay the search exactly as it ran: every node arrives after its parent, and
/// every prune names a node that was added earlier. A viewer can therefore apply these in
/// order and never hold a dangling reference — which is why nothing here is sampled or
/// skipped, however long the search. Size is controlled by thinning the poses *within* an
/// edge ([`TRACE_POSES_PER_EDGE`]) rather than by dropping events, because a dropped
/// `NodeAdded` would orphan everything descended from it.
#[derive(Debug, Clone, serde::Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub(crate) enum SearchEvent {
    /// A node joined the tree, reached by `path` from its parent.
    NodeAdded {
        iteration: usize,
        id: usize,
        parent: usize,
        /// `[x, y, theta]` in metres and radians, thinned — always including both ends.
        path: Vec<[f64; 3]>,
        /// Cost-to-come in seconds.
        cost: f64,
    },
    /// Sparsification removed a node: something cheaper took over its witness, and it had no
    /// descendants left to justify keeping it. This is the event that distinguishes SST from a
    /// plain kinodynamic RRT, and the one worth watching.
    NodePruned { iteration: usize, id: usize },
    /// A new witness ball was opened, at the state that founded it.
    WitnessAdded { iteration: usize, x: f64, y: f64 },
    /// The best solution got cheaper.
    SolutionImproved { iteration: usize, cost: f64 },
}

/// One moment the best solution got cheaper.
///
/// The anytime curve, which is the measurement that says whether a *time budget* would be a
/// better interface than an iteration count — where the knee is, and how much of the search
/// was spent after the last improvement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Improvement {
    pub iteration: usize,
    pub elapsed: std::time::Duration,
    pub cost: f64,
}

/// Where a search spent itself.
///
/// Not debug scaffolding. Node count against iteration count *is* the sparsity claim that
/// distinguishes SST from plain kinodynamic RRT, and the phase split is the only way to tell
/// a planner that is slow because it checks too many poses from one that is slow because it
/// scans too many nodes — which call for opposite fixes.
///
/// Counters are unconditional; they are integer increments against work measured in
/// microseconds. The three phase timers take one `Instant::now()` each per iteration, which at
/// 30_000 iterations is well under a millisecond in total.
#[derive(Debug, Clone, Default)]
pub(crate) struct SearchStats {
    pub iterations: usize,
    /// Nodes still in the tree when the search ended.
    pub nodes: usize,
    /// Nodes created over the whole search, pruned ones included.
    pub nodes_created: usize,
    /// Nodes removed by the sparsification cascade.
    pub nodes_pruned: usize,
    pub witnesses: usize,
    /// Extensions where every sampled rollout collided at once, so no edge was produced.
    pub extensions_failed: usize,
    /// Candidates discarded because a cheaper node already held their witness. High here means
    /// the tree is resampling ground it has already covered well.
    pub dominated: usize,
    /// Selections that found nothing inside `selection_radius` and fell back to the nearest
    /// node. High here means the radius is small relative to how spread the tree is.
    pub selection_fallbacks: usize,
    /// Shape queries asked of the collision world — normally the dominant cost.
    pub collision_queries: u64,
    /// Sampling plus nearest/best-near scanning.
    pub selection: std::time::Duration,
    /// Forward propagation, which includes every collision query.
    pub propagation: std::time::Duration,
    /// Witness lookup, insertion, pruning and the goal test.
    pub bookkeeping: std::time::Duration,
    pub total: std::time::Duration,
    /// Each time the best solution got cheaper.
    pub improvements: Vec<Improvement>,
    /// The closest any node ever came to the goal, in metres.
    ///
    /// The diagnostic for a search that never arrives: a value that sits well above
    /// `goal_radius` says the tree stalled rather than merely ran out of budget.
    pub closest_approach: f64,
    /// `closest_approach` sampled as the search ran, as `(iteration, metres)`.
    ///
    /// A plateau here is the signature of the tree getting stuck against an obstacle — the
    /// case where more iterations do not help and the fix belongs in selection instead.
    /// Empty unless [`SstParams::progress_every`] is set.
    pub progress: Vec<(usize, f64)>,
}

/// What a search produced, and what it took.
#[derive(Debug, Clone)]
pub(crate) struct SstOutcome {
    pub solution: Option<SstSolution>,
    pub stats: SearchStats,
    /// Empty unless [`SstParams::trace`] was set.
    pub trace: Vec<SearchEvent>,
}

impl SstOutcome {
    /// Nodes still in the tree. Kept as a method because it reads better at call sites than
    /// reaching through `stats` for the one number almost everything wants.
    pub fn nodes(&self) -> usize {
        self.stats.nodes
    }
}

struct Node {
    state: UnicycleState,
    parent: Option<usize>,
    /// The edge from `parent` to here. `None` for the root, and dropped when a node is pruned
    /// so a long search does not hold on to the states of trajectories it abandoned.
    edge: Option<Vec<UnicycleState>>,
    controls: Option<Vec<UnicycleControl>>,
    /// Cost-to-come, in seconds.
    cost: f64,
    /// Whether this node may still be selected from. A node beaten at its own witness is
    /// deactivated but stays in the tree while its descendants need it as an ancestor.
    active: bool,
    children: usize,
    /// Tombstone: pruned entirely. Indices stay valid, so parents never dangle.
    alive: bool,
}

/// The search's own state, kept whole so its invariants can be asserted.
struct Tree {
    nodes: Vec<Node>,
    witnesses: Vec<Witness>,
    best: Option<SstSolution>,
    stats: SearchStats,
    trace: Vec<SearchEvent>,
}

struct Witness {
    state: UnicycleState,
    /// The cheapest node yet seen within `witness_radius` of this witness.
    rep: Option<usize>,
}

/// Thins an edge's states to at most [`TRACE_POSES_PER_EDGE`] poses, ends always included.
///
/// Both ends matter: the first is where the parent left off and the last is the node itself,
/// so dropping either would make the drawn tree disconnected or put nodes in the wrong place.
fn thin_poses(states: &[UnicycleState]) -> Vec<[f64; 3]> {
    let pose = |s: &UnicycleState| [s.x, s.y, s.theta];
    if states.len() <= TRACE_POSES_PER_EDGE {
        return states.iter().map(pose).collect();
    }

    let last = states.len() - 1;
    let step = last as f64 / (TRACE_POSES_PER_EDGE - 1) as f64;
    (0..TRACE_POSES_PER_EDGE)
        .map(|i| pose(&states[((i as f64 * step).round() as usize).min(last)]))
        .collect()
}

pub(crate) struct SstPlanner {
    pub generator: KinodynamicEdgeGenerator,
    pub params: SstParams,
}

impl SstPlanner {
    pub fn new(generator: KinodynamicEdgeGenerator, params: SstParams) -> Self {
        Self { generator, params }
    }

    pub fn plan(
        &self,
        start: UnicycleState,
        goal: [f64; 2],
        world: &PlanningWorld,
        bounds: Bounds,
        rng: &mut impl Rng,
    ) -> SstOutcome {
        let tree = self.grow(start, goal, world, bounds, rng);
        SstOutcome {
            solution: tree.best,
            stats: tree.stats,
            trace: tree.trace,
        }
    }

    /// The search itself, handing back the whole tree.
    ///
    /// Separate from [`plan`](Self::plan) so the tests can assert the structural invariants —
    /// that pruning never orphans a node, and that the active set stays bounded by the witness
    /// set — against the real thing rather than against a summary of it.
    fn grow(
        &self,
        start: UnicycleState,
        goal: [f64; 2],
        world: &PlanningWorld,
        bounds: Bounds,
        rng: &mut impl Rng,
    ) -> Tree {
        let mut nodes = vec![Node {
            state: start.clone(),
            parent: None,
            edge: None,
            controls: None,
            cost: 0.0,
            active: true,
            children: 0,
            alive: true,
        }];
        let mut witnesses = vec![Witness {
            state: start,
            rep: Some(0),
        }];

        // The best solution is extracted the moment it is found rather than held as a node
        // index, because the node it came from is prunable like any other: a goal node that
        // gets beaten at its witness and has no children is removed, and an index kept across
        // that would name a tombstone.
        let mut best: Option<SstSolution> = None;
        let mut stats = SearchStats {
            nodes_created: 1,
            closest_approach: f64::INFINITY,
            ..SearchStats::default()
        };
        let started = std::time::Instant::now();
        let mut trace: Vec<SearchEvent> = Vec::new();
        // Reused across iterations rather than allocated per prune.
        let mut pruned: Vec<usize> = Vec::new();

        for iteration in 0..self.params.iterations {
            let tick = std::time::Instant::now();
            let sample = self.sample(goal, &bounds, rng);
            let selected = self.best_near(&nodes, &sample, &mut stats);
            let selected_at = std::time::Instant::now();
            stats.selection += selected_at - tick;

            let edge = self
                .generator
                .generate_edge(&nodes[selected].state, &sample, world, rng);
            let propagated_at = std::time::Instant::now();
            stats.propagation += propagated_at - selected_at;

            if self.params.progress_every > 0 && iteration % self.params.progress_every == 0 {
                stats.progress.push((iteration, stats.closest_approach));
            }

            let Some(edge) = edge else {
                stats.extensions_failed += 1;
                stats.bookkeeping += propagated_at.elapsed();
                continue;
            };

            let new_state = edge.states.last().expect("an edge has states").clone();
            let new_cost = nodes[selected].cost + edge.cost;

            // --- is this node locally the best? --------------------------------------
            let witness = self.nearest_witness(&witnesses, &new_state).filter(|&(_, d)| {
                d <= self.params.witness_radius
            });
            let witness_index = match witness {
                Some((index, _)) => index,
                None => {
                    // Nothing covers this state, so it founds its own witness ball and is
                    // trivially the best node in it.
                    if self.params.trace {
                        trace.push(SearchEvent::WitnessAdded {
                            iteration,
                            x: new_state.x,
                            y: new_state.y,
                        });
                    }
                    witnesses.push(Witness {
                        state: new_state.clone(),
                        rep: None,
                    });
                    witnesses.len() - 1
                }
            };

            let peer = witnesses[witness_index].rep;
            let dominates = match peer {
                None => true,
                Some(peer) => new_cost < nodes[peer].cost,
            };
            if !dominates {
                stats.dominated += 1;
                stats.bookkeeping += propagated_at.elapsed();
                continue;
            }

            // --- admit it ------------------------------------------------------------
            let new_index = nodes.len();
            if self.params.trace {
                trace.push(SearchEvent::NodeAdded {
                    iteration,
                    id: new_index,
                    parent: selected,
                    path: thin_poses(&edge.states),
                    cost: new_cost,
                });
            }
            nodes.push(Node {
                state: new_state,
                parent: Some(selected),
                edge: Some(edge.states),
                controls: Some(edge.controls),
                cost: new_cost,
                active: true,
                children: 0,
                alive: true,
            });
            nodes[selected].children += 1;
            stats.nodes_created += 1;
            witnesses[witness_index].rep = Some(new_index);

            if let Some(peer) = peer {
                self.prune(&mut nodes, peer, &mut pruned);
                stats.nodes_pruned += pruned.len();
                if self.params.trace {
                    trace.extend(
                        pruned
                            .iter()
                            .map(|&id| SearchEvent::NodePruned { iteration, id }),
                    );
                }
            }

            // How close the tree has come, whether or not it counts as arrival. A search that
            // never solves is diagnosed by this number sitting flat, not by the empty result.
            let reach = {
                let state = &nodes[new_index].state;
                ((state.x - goal[0]).powi(2) + (state.y - goal[1]).powi(2)).sqrt()
            };
            stats.closest_approach = stats.closest_approach.min(reach);

            // --- has it arrived? -----------------------------------------------------
            if self.reached(&nodes[new_index].state, goal)
                && best.as_ref().is_none_or(|b| new_cost < b.cost)
            {
                best = Some(self.extract(&nodes, new_index));
                stats.improvements.push(Improvement {
                    iteration,
                    elapsed: started.elapsed(),
                    cost: new_cost,
                });
                if self.params.trace {
                    trace.push(SearchEvent::SolutionImproved {
                        iteration,
                        cost: new_cost,
                    });
                }
            }
            stats.bookkeeping += propagated_at.elapsed();
        }

        stats.iterations = self.params.iterations;
        stats.total = started.elapsed();
        stats.nodes = nodes.iter().filter(|n| n.alive).count();
        stats.witnesses = witnesses.len();
        stats.collision_queries = world.queries();

        Tree {
            nodes,
            witnesses,
            best,
            stats,
            trace,
        }
    }

    /// Deactivates a dominated node, then removes it and any ancestors it was the last reason
    /// to keep.
    ///
    /// The cascade is what stops the tree filling with inactive nodes that exist only to be
    /// somebody's parent. It stops at the first node that still has a child, and never touches
    /// the root — which is unprunable anyway, since cost-to-come is a duration and nothing can
    /// reach the start more cheaply than starting there.
    /// `removed` is cleared and filled with the ids taken out, so a caller recording a trace
    /// learns them without rescanning the tree — which would turn the whole search quadratic.
    fn prune(&self, nodes: &mut [Node], peer: usize, removed: &mut Vec<usize>) {
        removed.clear();
        nodes[peer].active = false;

        let mut cursor = peer;
        while nodes[cursor].alive && !nodes[cursor].active && nodes[cursor].children == 0 {
            let Some(parent) = nodes[cursor].parent else {
                break; // the root stays, whatever else happens
            };
            nodes[cursor].alive = false;
            // Drop the trajectory with the node: on a long search these are the bulk of the
            // memory, and an abandoned branch's states are never read again.
            nodes[cursor].edge = None;
            nodes[cursor].controls = None;
            nodes[parent].children -= 1;
            removed.push(cursor);
            cursor = parent;
        }
    }

    /// Best-first selection: the cheapest active node within `selection_radius` of the sample,
    /// falling back to the nearest active node when the ball is empty.
    ///
    /// The fallback is what keeps the planner probabilistically complete — without it a sample
    /// in unexplored space would select nothing and the iteration would be wasted, which is
    /// exactly the region the tree most needs to reach.
    fn best_near(&self, nodes: &[Node], sample: &UnicycleState, stats: &mut SearchStats) -> usize {
        let mut cheapest: Option<usize> = None;
        let mut nearest = (f64::INFINITY, 0usize);

        // Linear, which is honest for SST in a way it would not be for RRT: the witness set
        // bounds how many nodes there are to scan. A k-d tree over the same metric is the
        // obvious next step once a profile says this is the cost.
        for (index, node) in nodes.iter().enumerate() {
            if !node.alive || !node.active {
                continue;
            }
            let distance = self.params.metric.distance(&node.state, sample);
            if distance < nearest.0 {
                nearest = (distance, index);
            }
            if distance <= self.params.selection_radius
                && cheapest.is_none_or(|best| node.cost < nodes[best].cost)
            {
                cheapest = Some(index);
            }
        }

        cheapest.unwrap_or_else(|| {
            stats.selection_fallbacks += 1;
            nearest.1
        })
    }

    fn nearest_witness(&self, witnesses: &[Witness], state: &UnicycleState) -> Option<(usize, f64)> {
        witnesses
            .iter()
            .enumerate()
            .map(|(index, witness)| (index, self.params.metric.distance(&witness.state, state)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }

    fn sample(&self, goal: [f64; 2], bounds: &Bounds, rng: &mut impl Rng) -> UnicycleState {
        let spec = &self.generator.model.spec;

        // Goal biasing fixes only the position. Heading and velocity stay random so the tree
        // is invited to reach the goal from any approach, rather than only the one arbitrary
        // pose a fully-specified goal sample would name.
        let (x, y) = if rng.r#gen::<f64>() < self.params.goal_bias {
            (goal[0], goal[1])
        } else {
            (
                rng.gen_range(bounds.min_x..=bounds.max_x),
                rng.gen_range(bounds.min_y..=bounds.max_y),
            )
        };

        UnicycleState {
            x,
            y,
            theta: rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI),
            v: rng.gen_range(spec.min_linear_vel..=spec.max_linear_vel),
            omega: rng.gen_range(-spec.max_angular_vel..=spec.max_angular_vel),
        }
    }

    fn reached(&self, state: &UnicycleState, goal: [f64; 2]) -> bool {
        let (dx, dy) = (state.x - goal[0], state.y - goal[1]);
        (dx * dx + dy * dy).sqrt() <= self.params.goal_radius
    }

    /// Walks the branch back to the root and concatenates it into one trajectory.
    ///
    /// Each edge repeats its parent's state as its own first entry, so the join is dropped on
    /// the way in — keeping it would put a duplicated state in the middle of the path and a
    /// zero-length step in anything that integrates or draws it.
    fn extract(&self, nodes: &[Node], goal_node: usize) -> SstSolution {
        let mut branch = Vec::new();
        let mut cursor = goal_node;
        while let Some(parent) = nodes[cursor].parent {
            branch.push(cursor);
            cursor = parent;
        }
        branch.reverse();

        let mut states = vec![nodes[cursor].state.clone()];
        let mut controls = Vec::new();
        for index in branch {
            let edge = nodes[index].edge.as_ref().expect("a live branch keeps its edge");
            states.extend(edge[1..].iter().cloned());
            controls.extend(
                nodes[index]
                    .controls
                    .as_ref()
                    .expect("a live branch keeps its controls")
                    .iter()
                    .cloned(),
            );
        }

        SstSolution {
            cost: nodes[goal_node].cost,
            states,
            controls,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::obstacle::{CellVertex, ObstaclePoly};
    use crate::models::robots::unicycle_spec::UnicycleSpec;
    use crate::models::robots::unicycle_state::SecondOrderUnicycle;
    use crate::simulators::simulator_engine::SimulatorEngine;
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rapier2d::prelude::{Ball, Pose, Vector};

    const DT: f64 = 0.02;

    fn planner(params: SstParams) -> SstPlanner {
        SstPlanner::new(
            KinodynamicEdgeGenerator::new(
                SecondOrderUnicycle {
                    spec: UnicycleSpec::default(),
                },
                DT,
            ),
            params,
        )
    }

    fn at_rest(x: f64, y: f64, theta: f64) -> UnicycleState {
        UnicycleState {
            x,
            y,
            theta,
            v: 0.0,
            omega: 0.0,
        }
    }

    /// A wall two cells thick spanning the full width at y = 4..5.
    fn full_width_wall(width: i32) -> ObstaclePoly {
        ObstaclePoly {
            id: 1,
            dynamic: false,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 0, y: 4 },
                CellVertex { x: width - 1, y: 4 },
                CellVertex { x: width - 1, y: 5 },
                CellVertex { x: 0, y: 5 },
                CellVertex { x: 0, y: 4 },
            ],
        }
    }

    #[test]
    fn a_solution_is_an_actual_trajectory_not_a_list_of_waypoints() {
        // The property that separates a kinodynamic plan from a path: re-integrating the
        // returned controls from the returned start state has to reproduce the returned
        // states, exactly, all the way along.
        //
        // This is also the only thing that pins the edge concatenation in `extract`. Each edge
        // repeats its parent's state as its own first entry, and keeping that duplicate — or
        // dropping one too many — puts a zero-length or a teleporting step in the middle of
        // the path, which no amount of looking at the endpoints would reveal.
        let planner = planner(SstParams {
            iterations: 3_000,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[], 20, 20);
        let outcome = planner.plan(
            at_rest(2.0, 2.0, 0.0),
            [16.0, 16.0],
            &world,
            Bounds::from_grid(20, 20),
            &mut StdRng::seed_from_u64(9),
        );

        let solution = outcome.solution.expect("open ground should be solvable");
        assert_eq!(
            solution.states.len(),
            solution.controls.len() + 1,
            "states must fence the controls",
        );

        let mut state = solution.states[0];
        for (step, control) in solution.controls.iter().enumerate() {
            state = SimulatorEngine::step(&planner.generator.model, &state, control, DT);
            let expected = solution.states[step + 1];
            assert!(
                (state.x - expected.x).abs() < 1e-12
                    && (state.y - expected.y).abs() < 1e-12
                    && (state.theta - expected.theta).abs() < 1e-12
                    && (state.v - expected.v).abs() < 1e-12,
                "step {step} diverges: integrated ({}, {}) but the path says ({}, {})",
                state.x,
                state.y,
                expected.x,
                expected.y,
            );
        }

        // The cost is the duration, and the duration is the number of steps flown.
        assert!((solution.cost - solution.controls.len() as f64 * DT).abs() < 1e-9);
    }

    #[test]
    fn a_solution_starts_where_it_was_told_and_reaches_the_goal() {
        let params = SstParams {
            iterations: 3_000,
            ..SstParams::default()
        };
        let planner = planner(params);
        let world = PlanningWorld::from_obstacles(&[], 20, 20);
        let start = at_rest(2.0, 2.0, 0.3);
        let outcome = planner.plan(
            start,
            [16.0, 16.0],
            &world,
            Bounds::from_grid(20, 20),
            &mut StdRng::seed_from_u64(4),
        );

        let solution = outcome.solution.expect("open ground should be solvable");
        assert_eq!(solution.states[0], start, "a plan must start from the given state");

        let last = solution.states.last().unwrap();
        let reach = ((last.x - 16.0).powi(2) + (last.y - 16.0).powi(2)).sqrt();
        assert!(reach <= params.goal_radius, "ended {reach} m from the goal");
    }

    #[test]
    fn no_state_on_a_solution_touches_an_obstacle() {
        // A wall with a doorway at the right-hand end, so the only route is around it and the
        // planner cannot succeed by accident.
        // 10_000 rather than the ~5_000 that happens to work for this seed, for margin:
        // measured over twelve seeds this doorway solves 12/12 at 6_000, 10_000 and 20_000
        // iterations, with mean cost falling 12.86 -> 12.31 -> 11.34 s as the budget grows.
        //
        // That last trend is the near-optimality showing up empirically, and it is the number
        // worth watching if the metric or the two radii are ever retuned.
        let planner = planner(SstParams {
            iterations: 10_000,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[full_width_wall(9)], 12, 12);
        let outcome = planner.plan(
            at_rest(2.0, 1.5, 0.0),
            [2.0, 9.0],
            &world,
            Bounds::from_grid(12, 12),
            &mut StdRng::seed_from_u64(21),
        );

        let solution = outcome.solution.expect("a doorway at x >= 9 leaves a way round");
        for state in &solution.states {
            assert!(
                !world.intersects(
                    Pose::new(Vector::new(state.x as f32, state.y as f32), state.theta as f32),
                    &Ball::new(planner.generator.model.spec.radius() as f32),
                ),
                "solution passes through an obstacle at ({}, {})",
                state.x,
                state.y,
            );
        }
        // It really did have to go round: a straight run would cross y = 5.
        assert!(
            solution.states.iter().any(|s| s.x > 9.0),
            "the path never used the doorway, so the wall was not respected",
        );
    }

    #[test]
    fn a_sealed_goal_yields_no_solution() {
        // The honest failure. A wall clean across the world with no doorway leaves the goal
        // genuinely unreachable, and the planner has to say so rather than return a path that
        // drives through it.
        let planner = planner(SstParams {
            iterations: 3_000,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[full_width_wall(11)], 10, 10);
        let outcome = planner.plan(
            at_rest(5.0, 1.5, 0.0),
            [5.0, 8.5],
            &world,
            Bounds::from_grid(10, 10),
            &mut StdRng::seed_from_u64(13),
        );

        assert!(outcome.solution.is_none(), "found a way through a sealed wall");
        assert!(outcome.nodes() > 1, "the tree should still have explored the near side");
    }

    #[test]
    fn pruning_never_orphans_a_node() {
        // The invariant the prune cascade has to maintain. A node whose parent was removed
        // would make `extract` walk into a tombstone, and the failure would surface as a
        // corrupt path rather than as a panic anywhere near the cause.
        let planner = planner(SstParams {
            iterations: 4_000,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[], 10, 10);
        let tree = planner.grow(
            at_rest(5.0, 5.0, 0.0),
            [9.0, 9.0],
            &world,
            Bounds::from_grid(10, 10),
            &mut StdRng::seed_from_u64(77),
        );

        for (index, node) in tree.nodes.iter().enumerate() {
            if !node.alive {
                continue;
            }
            match node.parent {
                None => assert_eq!(index, 0, "only the root may have no parent"),
                Some(parent) => assert!(
                    tree.nodes[parent].alive,
                    "node {index} survived its own parent {parent}",
                ),
            }
            assert!(
                node.edge.is_some() || index == 0,
                "live node {index} lost the edge that reaches it",
            );
        }

        // A live node's child count has to match the children it actually has, or the cascade
        // either stops early and leaks or keeps going and orphans.
        let mut counted = vec![0usize; tree.nodes.len()];
        for node in tree.nodes.iter().filter(|n| n.alive) {
            if let Some(parent) = node.parent {
                counted[parent] += 1;
            }
        }
        for (index, node) in tree.nodes.iter().enumerate() {
            if node.alive {
                assert_eq!(node.children, counted[index], "node {index} miscounts its children");
            }
        }
    }

    #[test]
    fn the_active_set_stays_bounded_by_the_witness_set() {
        // SST's defining structural claim: each witness keeps exactly one representative, so
        // the number of nodes still available for selection cannot exceed the number of
        // witnesses however long the search runs. This is what bounds the tree, and with it
        // the cost of the nearest-neighbor scan.
        let planner = planner(SstParams {
            iterations: 4_000,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[], 10, 10);
        let tree = planner.grow(
            at_rest(5.0, 5.0, 0.0),
            [9.0, 9.0],
            &world,
            Bounds::from_grid(10, 10),
            &mut StdRng::seed_from_u64(31),
        );

        let active = tree.nodes.iter().filter(|n| n.alive && n.active).count();
        assert!(
            active <= tree.witnesses.len(),
            "{active} active nodes against {} witnesses",
            tree.witnesses.len(),
        );

        // And each witness's representative must itself be a live node — a rep pointing at a
        // pruned node would resurrect it as a selection candidate.
        for witness in &tree.witnesses {
            if let Some(rep) = witness.rep {
                assert!(tree.nodes[rep].alive, "a witness represents a pruned node");
            }
        }
    }

    #[test]
    fn sparsification_actually_discards_nodes() {
        // The other half of the claim: pruning has to *fire*. A correct-looking implementation
        // whose cascade never triggers would pass every invariant above while growing exactly
        // like plain RRT, which is the thing SST exists not to do.
        let planner = planner(SstParams {
            iterations: 4_000,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[], 10, 10);
        let outcome = planner.plan(
            at_rest(5.0, 5.0, 0.0),
            [9.0, 9.0],
            &world,
            Bounds::from_grid(10, 10),
            &mut StdRng::seed_from_u64(31),
        );

        assert!(
            outcome.nodes() < outcome.stats.nodes_created,
            "no node was ever pruned: {} created, {} kept",
            outcome.stats.nodes_created,
            outcome.nodes(),
        );
        assert!(
            outcome.nodes() < outcome.stats.iterations / 2,
            "the tree is not sparse: {} nodes over {} iterations",
            outcome.nodes(),
            outcome.stats.iterations,
        );
    }

    #[test]
    fn a_longer_search_does_not_return_a_worse_path() {
        // Near-optimality is asymptotic, so this asserts the direction rather than a rate:
        // with the same seed and more iterations, the best solution found cannot get worse,
        // because it is only ever replaced by a strictly cheaper one.
        let world = PlanningWorld::from_obstacles(&[], 20, 20);
        let cost_at = |iterations| {
            planner(SstParams {
                iterations,
                ..SstParams::default()
            })
            .plan(
                at_rest(2.0, 2.0, 0.0),
                [16.0, 16.0],
                &world,
                Bounds::from_grid(20, 20),
                &mut StdRng::seed_from_u64(5),
            )
            .solution
            .map(|s| s.cost)
        };

        let (short, long) = (cost_at(2_000), cost_at(8_000));
        let short = short.expect("2000 iterations should already solve open ground");
        let long = long.expect("a longer search cannot lose a solution it already had");
        assert!(long <= short, "more search made the path worse: {short} -> {long}");
    }

    #[test]
    fn a_seed_replays_the_same_search() {
        let world = PlanningWorld::from_obstacles(&[], 20, 20);
        let run = |seed| {
            planner(SstParams {
                iterations: 1_500,
                ..SstParams::default()
            })
            .plan(
                at_rest(2.0, 2.0, 0.0),
                [16.0, 16.0],
                &world,
                Bounds::from_grid(20, 20),
                &mut StdRng::seed_from_u64(seed),
            )
        };

        let (a, b) = (run(101), run(101));
        assert_eq!(a.nodes(), b.nodes());
        assert_eq!(a.stats.nodes_created, b.stats.nodes_created);
        assert_eq!(a.stats.witnesses, b.stats.witnesses);
        assert_eq!(
            a.solution.map(|s| (s.cost, s.states.len())),
            b.solution.map(|s| (s.cost, s.states.len())),
        );
    }

    /// Builds a node with a known state and cost, with no tree around it.
    fn node(state: UnicycleState, cost: f64, active: bool) -> Node {
        Node {
            state,
            parent: Some(0),
            edge: Some(vec![state]),
            controls: Some(Vec::new()),
            cost,
            active,
            children: 0,
            alive: true,
        }
    }

    #[test]
    fn selection_takes_the_cheapest_node_near_the_sample_not_the_nearest() {
        // The mechanism SST is named for, and the one thing that separates it from plain
        // kinodynamic RRT once the pruning is in place. Selecting by proximity is what throws
        // optimality away, and the failure is silent: the planner still returns paths, just
        // consistently worse ones. Asserted on a hand-built tree so nothing is left to chance.
        let planner = planner(SstParams {
            selection_radius: 2.0,
            ..SstParams::default()
        });
        let sample = at_rest(10.0, 10.0, 0.0);

        let nodes = vec![
            node(at_rest(0.0, 0.0, 0.0), 0.0, true),      // 0: the root, far away
            node(at_rest(10.5, 10.0, 0.0), 9.0, true),    // 1: nearest to the sample, dear
            node(at_rest(11.5, 10.0, 0.0), 2.0, true),    // 2: further but cheap — the pick
        ];

        assert_eq!(
            planner.best_near(&nodes, &sample, &mut SearchStats::default()),
            2,
            "selection went to the nearest node rather than the cheapest one near it",
        );
    }

    #[test]
    fn selection_falls_back_to_the_nearest_node_when_nothing_is_near() {
        // Without this the planner would stall: a sample out in unexplored space has no node
        // within the radius, and returning nothing would waste precisely the iterations aimed
        // at the frontier the tree most needs to reach.
        let planner = planner(SstParams {
            selection_radius: 1.0,
            ..SstParams::default()
        });
        let nodes = vec![
            node(at_rest(0.0, 0.0, 0.0), 0.0, true),
            node(at_rest(5.0, 5.0, 0.0), 8.0, true),
        ];

        // (30, 30) is far from both; node 1 is nearer despite costing more.
        assert_eq!(planner.best_near(&nodes, &at_rest(30.0, 30.0, 0.0), &mut SearchStats::default()), 1);
    }

    #[test]
    fn selection_ignores_pruned_and_deactivated_nodes() {
        // A deactivated node is one already beaten at its own witness; selecting from it again
        // would undo the pruning, and selecting from a tombstone would extend a branch that is
        // no longer in the tree.
        let planner = planner(SstParams {
            selection_radius: 5.0,
            ..SstParams::default()
        });
        let sample = at_rest(10.0, 10.0, 0.0);

        let mut nodes = vec![
            node(at_rest(0.0, 0.0, 0.0), 0.0, true),
            node(at_rest(10.0, 10.0, 0.0), 1.0, false), // cheapest and nearest, but inactive
            node(at_rest(12.0, 10.0, 0.0), 5.0, true),
        ];
        assert_eq!(planner.best_near(&nodes, &sample, &mut SearchStats::default()), 2, "an inactive node was selected");

        nodes[2].alive = false;
        assert_eq!(planner.best_near(&nodes, &sample, &mut SearchStats::default()), 0, "a pruned node was selected");
    }

    fn traced(iterations: usize) -> SstOutcome {
        let planner = planner(SstParams {
            iterations,
            trace: true,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[], 16, 16);
        planner.plan(
            at_rest(2.0, 2.0, 0.0),
            [13.0, 13.0],
            &world,
            Bounds::from_grid(16, 16),
            &mut StdRng::seed_from_u64(8),
        )
    }

    #[test]
    fn a_trace_replays_without_ever_dangling() {
        // The property a viewer depends on and the only one that really matters: applying the
        // events in order must never reference a node that does not exist yet, or one already
        // removed. Anything else in a trace is cosmetic; this decides whether it can be drawn
        // at all.
        let outcome = traced(3_000);
        assert!(!outcome.trace.is_empty(), "tracing was on but nothing was recorded");

        let mut live: std::collections::HashSet<usize> = std::collections::HashSet::new();
        live.insert(0); // the root is never an event; it is where a viewer starts
        let mut seen_prunes = 0;

        for event in &outcome.trace {
            match event {
                SearchEvent::NodeAdded { id, parent, path, .. } => {
                    assert!(live.contains(parent), "node {id} arrived before its parent {parent}");
                    assert!(live.insert(*id), "node {id} was added twice");
                    assert!(path.len() >= 2, "an edge needs both ends");
                }
                SearchEvent::NodePruned { id, .. } => {
                    assert!(live.remove(id), "node {id} was pruned while not in the tree");
                    seen_prunes += 1;
                }
                SearchEvent::WitnessAdded { .. } | SearchEvent::SolutionImproved { .. } => {}
            }
        }

        assert!(seen_prunes > 0, "a trace with no prunes cannot show what SST does");
        assert_eq!(
            live.len(),
            outcome.stats.nodes,
            "replaying the trace does not reproduce the tree the search ended with",
        );
    }

    #[test]
    fn a_trace_is_ordered_and_accounted_for() {
        let outcome = traced(3_000);

        let iterations: Vec<usize> = outcome
            .trace
            .iter()
            .map(|e| match e {
                SearchEvent::NodeAdded { iteration, .. }
                | SearchEvent::NodePruned { iteration, .. }
                | SearchEvent::WitnessAdded { iteration, .. }
                | SearchEvent::SolutionImproved { iteration, .. } => *iteration,
            })
            .collect();
        assert!(
            iterations.windows(2).all(|w| w[0] <= w[1]),
            "a trace must be in the order the search ran",
        );

        // Every counter the stats report has to match what the trace shows, or one of the two
        // is lying about the same search.
        let added = outcome.trace.iter().filter(|e| matches!(e, SearchEvent::NodeAdded { .. })).count();
        let pruned = outcome.trace.iter().filter(|e| matches!(e, SearchEvent::NodePruned { .. })).count();
        let witnesses = outcome.trace.iter().filter(|e| matches!(e, SearchEvent::WitnessAdded { .. })).count();
        let improved = outcome.trace.iter().filter(|e| matches!(e, SearchEvent::SolutionImproved { .. })).count();

        assert_eq!(added + 1, outcome.stats.nodes_created, "the root is the one node with no event");
        assert_eq!(pruned, outcome.stats.nodes_pruned);
        assert_eq!(witnesses + 1, outcome.stats.witnesses, "the root founds the first witness");
        assert_eq!(improved, outcome.stats.improvements.len());
    }

    #[test]
    fn tracing_is_off_unless_asked_for() {
        // A trace is megabytes, and a replanning loop would accumulate one per tick.
        let planner = planner(SstParams {
            iterations: 500,
            ..SstParams::default()
        });
        let world = PlanningWorld::from_obstacles(&[], 16, 16);
        let outcome = planner.plan(
            at_rest(2.0, 2.0, 0.0),
            [13.0, 13.0],
            &world,
            Bounds::from_grid(16, 16),
            &mut StdRng::seed_from_u64(8),
        );
        assert!(outcome.trace.is_empty(), "a trace was recorded without being asked for");
    }

    #[test]
    fn an_edge_is_thinned_but_keeps_both_ends() {
        // Thinning is what keeps a trace to a sane size. Losing an end instead would draw the
        // tree disconnected, or put a node somewhere it never was.
        let states: Vec<UnicycleState> = (0..25)
            .map(|i| at_rest(i as f64, i as f64 * 2.0, 0.0))
            .collect();
        let thinned = thin_poses(&states);

        assert!(thinned.len() <= TRACE_POSES_PER_EDGE, "{} poses", thinned.len());
        assert_eq!(thinned.first().unwrap()[0], 0.0, "lost the start of the edge");
        assert_eq!(thinned.last().unwrap()[0], 24.0, "lost the end of the edge");

        // A short edge is passed through whole; there is nothing to gain by thinning it.
        let short: Vec<UnicycleState> = (0..3).map(|i| at_rest(i as f64, 0.0, 0.0)).collect();
        assert_eq!(thin_poses(&short).len(), 3);
    }

    #[test]
    fn the_metric_reads_a_wrapped_heading_as_near() {
        // Heading is on a circle. A metric that subtracted raw angles would call +pi and -pi
        // the two furthest-apart headings, and SST would then keep both as separate witnesses
        // and select between them as though they were different robots.
        let metric = StateMetric::default();
        let pi = std::f64::consts::PI;
        let pose = |theta| UnicycleState {
            x: 0.0,
            y: 0.0,
            theta,
            v: 0.0,
            omega: 0.0,
        };

        assert!(metric.distance(&pose(pi - 0.01), &pose(-pi + 0.01)) < 0.05);
        assert!(metric.distance(&pose(0.0), &pose(0.0)).abs() < 1e-12);
        // And it still separates headings that really are opposed.
        assert!(metric.distance(&pose(0.0), &pose(pi)) > 0.5);
    }
}
