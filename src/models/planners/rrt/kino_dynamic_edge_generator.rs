use crate::models::planners::rrt::planning_world::PlanningWorld;
use crate::models::planners::rrt::trajector_edge::TrajectoryEdge;
use crate::models::robots::unicycle_state::{SecondOrderUnicycle, UnicycleControl, UnicycleState};
use crate::simulators::simulator_engine::SimulatorEngine;
use rand::Rng;
use rapier2d::prelude::{Ball, Pose, Vector};

/// Extends a search tree by rolling the dynamics forward under sampled controls.
///
/// Worth being precise about what this primitive does and does not give you. Every edge it
/// produces is *exactly* dynamically feasible: it comes out of integrating the real ODE under
/// a real, saturated control, with no linearization anywhere. What it cannot do is land on a
/// *prescribed* state — it answers "where do these controls take me", never "which controls
/// take me there". That second question is the two-point boundary value problem, and nothing
/// here solves it.
///
/// Which is why an algorithm over this primitive can be RRT but not RRT*: rewiring means
/// reconnecting a node to a *chosen* parent, and "somewhere near" is not a connection. Getting
/// optimality out of forward propagation alone needs the sparsity-and-pruning machinery of SST
/// rather than a rewire step.
pub struct KinodynamicEdgeGenerator {
    pub model: SecondOrderUnicycle,
    pub dt: f64,                        // Integration time step (e.g., 0.02s)
    pub num_control_samples: usize,     // Number of random controls to trial per extension
    pub min_extension_steps: usize,     // Minimum duration of an edge execution
    pub max_extension_steps: usize,     // Maximum duration of an edge execution
    /// The robot's body, as the shape swept through the world.
    ///
    /// A disc of the spec's own radius — the same radius the grid planners inflate their
    /// obstacles by, so the two are clearing one machine rather than two differently-sized
    /// ones sharing a name.
    body: Ball,
}

impl KinodynamicEdgeGenerator {
    pub fn new(model: SecondOrderUnicycle, dt: f64) -> Self {
        let body = Ball::new(model.spec.radius() as f32);
        Self {
            model,
            dt,
            num_control_samples: 15,    // Branching test factor
            min_extension_steps: 5,     // 100ms minimum path length
            max_extension_steps: 25,    // 500ms maximum path length
            body,
        }
    }

    /// Generates a valid trajectory edge from a starting node toward a target state vector.
    ///
    /// `rng` is threaded in rather than reached for globally so a search replays from a seed.
    /// The rest of this codebase already treats that as non-negotiable — see
    /// [`Xorshift`](crate::models::rng::Xorshift) and the run-replay tests — and a planner
    /// whose failures cannot be re-run is a planner that cannot be debugged.
    pub fn generate_edge(
        &self,
        start_state: &UnicycleState,
        target_state: &UnicycleState,
        world: &PlanningWorld,
        rng: &mut impl Rng,
    ) -> Option<TrajectoryEdge> {
        let mut best_edge: Option<TrajectoryEdge> = None;
        let mut closest_distance = f64::MAX;

        // 1. Roll out multiple random forward control options
        for _ in 0..self.num_control_samples {
            // Sample random inputs bounded within our physical specs
            let spec = &self.model.spec;
            let control_sample = UnicycleControl {
                accel: rng.gen_range(-spec.max_linear_accel..=spec.max_linear_accel),
                angular_accel: rng.gen_range(-spec.max_angular_accel..=spec.max_angular_accel),
            };

            // Sample a random time horizon for this action sequence. Inclusive of the maximum:
            // the previous `min + rand % (max - min)` could never produce `max`, so the
            // longest extension the generator advertised was unreachable.
            let steps = rng.gen_range(self.min_extension_steps..=self.max_extension_steps);

            let mut current_state = start_state.clone();
            let mut edge_states = vec![current_state.clone()];
            let mut edge_controls = Vec::new();

            // 2. Propagate forward step-by-step using our RK4 integrator engine
            for _ in 0..steps {
                let next_state =
                    SimulatorEngine::step(&self.model, &current_state, &control_sample, self.dt);

                // 3. Collision validation, against the pose actually being tested
                //
                // A rollout that hits something is *truncated*, not discarded: the part before
                // the collision is a perfectly good edge, and throwing all of it away is what
                // left the tree unable to grow in exactly the cluttered places worth planning
                // through. Whatever survives the break is collision-free by construction.
                if world.intersects(self.pose_of(&next_state), &self.body) {
                    break;
                }

                current_state = next_state;
                edge_states.push(current_state.clone());
                edge_controls.push(control_sample.clone());
            }

            // An extension that collided on its first step went nowhere, and a single state is
            // not a motion.
            if edge_controls.is_empty() {
                continue;
            }

            // 4. How close did this segment bring us to the sampled target?
            //
            // Position only, which is a known weakness rather than an oversight: for a
            // second-order system a node pointed the wrong way, or carrying the wrong speed,
            // is not really "near" anything it cannot turn or decelerate to reach. Replacing
            // this with a weighted metric over heading and velocity is the single change most
            // likely to improve tree quality.
            let dx = current_state.x - target_state.x;
            let dy = current_state.y - target_state.y;
            let dist = (dx * dx + dy * dy).sqrt(); // standard Euclidean metric for positioning

            if dist < closest_distance {
                closest_distance = dist;
                best_edge = Some(TrajectoryEdge {
                    // Priced by what was actually flown, not by what was sampled: charging a
                    // truncated rollout for its full horizon would price an edge by the
                    // obstacle it ran into.
                    cost: edge_controls.len() as f64 * self.dt,
                    states: edge_states,
                    controls: edge_controls,
                });
            }
        }

        best_edge
    }

    /// The robot's pose, as the collision world spells one.
    ///
    /// Rapier 2D is single-precision and glam-backed, so the planner's `f64` state is narrowed
    /// here, at the boundary, rather than the integrator being dragged down to `f32` to match.
    fn pose_of(&self, state: &UnicycleState) -> Pose {
        Pose::new(
            Vector::new(state.x as f32, state.y as f32),
            state.theta as f32,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::robots::unicycle_spec::UnicycleSpec;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    fn generator() -> KinodynamicEdgeGenerator {
        KinodynamicEdgeGenerator::new(
            SecondOrderUnicycle {
                spec: UnicycleSpec::default(),
            },
            0.02,
        )
    }

    fn state(x: f64, y: f64, theta: f64, v: f64) -> UnicycleState {
        UnicycleState { x, y, theta, v, omega: 0.0 }
    }

    #[test]
    fn every_state_on_an_edge_is_collision_free() {
        // The invariant the whole primitive rests on, asserted by re-checking the returned
        // states against the same world — so a truncation that kept one state too many, the
        // classic off-by-one when a loop breaks on a hit, cannot slip through.
        let generator = generator();
        let world = PlanningWorld::from_obstacles(&[], 20, 20);
        let mut rng = StdRng::seed_from_u64(7);

        for _ in 0..40 {
            let edge = generator
                .generate_edge(
                    &state(4.0, 4.0, 0.5, 1.0),
                    &state(16.0, 16.0, 0.0, 0.0),
                    &world,
                    &mut rng,
                )
                .expect("open ground should always extend");

            for s in &edge.states {
                assert!(
                    !world.intersects(generator.pose_of(s), &generator.body),
                    "edge passes through ({}, {})",
                    s.x,
                    s.y,
                );
            }
        }
    }

    #[test]
    fn an_edge_is_priced_by_what_it_actually_flew() {
        // cost, states and controls must describe one motion. Charging a truncated rollout for
        // its sampled horizon would over-price it, and any cost-aware search built on this
        // would then prefer edges purely for having been cut short.
        let generator = generator();
        let world = PlanningWorld::from_obstacles(&[], 20, 20);
        let mut rng = StdRng::seed_from_u64(11);

        for _ in 0..40 {
            let edge = generator
                .generate_edge(&state(4.0, 4.0, 0.0, 1.0), &state(16.0, 10.0, 0.0, 0.0), &world, &mut rng)
                .unwrap();

            assert_eq!(edge.states.len(), edge.controls.len() + 1, "states fence the controls");
            assert!((edge.cost - edge.controls.len() as f64 * generator.dt).abs() < 1e-12);
            assert!(edge.controls.len() <= generator.max_extension_steps);
        }
    }

    #[test]
    fn a_rollout_into_an_obstacle_is_truncated_rather_than_thrown_away() {
        // Nose-to-the-wall and driving into it, so every rollout collides within a step or
        // two — far sooner than `min_extension_steps`. That is what makes this test able to
        // tell truncation from discarding at all: an edge shorter than the minimum sampled
        // horizon *cannot* have come from anywhere but a truncated rollout, and under the
        // original discard-the-whole-thing behavior there would be no edge here at all.
        //
        // One control sample per call, so a lucky non-colliding rollout cannot be selected in
        // place of the truncated ones and hide the behavior under test.
        let mut generator = generator();
        generator.num_control_samples = 1;

        // The top wall's inner face is at y = 12.0, and the body is 0.225 m in radius, so the
        // robot collides at y >= 11.775. Starting at 11.70 leaves 0.075 m — under two steps at
        // 2 m/s, whichever acceleration is sampled.
        let world = PlanningWorld::from_obstacles(&[], 6, 12);
        let mut rng = StdRng::seed_from_u64(3);

        let mut truncated = 0;
        for _ in 0..100 {
            let Some(edge) = generator.generate_edge(
                &state(3.0, 11.70, std::f64::consts::FRAC_PI_2, 2.0),
                &state(3.0, 18.0, std::f64::consts::FRAC_PI_2, 0.0),
                &world,
                &mut rng,
            ) else {
                continue;
            };

            assert!(
                edge.controls.len() < generator.min_extension_steps,
                "nothing was truncated: {} steps against a wall 0.075 m away",
                edge.controls.len(),
            );
            assert!(!edge.controls.is_empty(), "a zero-length edge is not a motion");

            // The truncated edge has to be priced by what it flew. This is the assertion that
            // catches a cost charged for the *sampled* horizon, which is indistinguishable
            // from the honest one until a rollout is actually cut short.
            assert!(
                (edge.cost - edge.controls.len() as f64 * generator.dt).abs() < 1e-12,
                "edge of {} steps priced at {}",
                edge.controls.len(),
                edge.cost,
            );
            assert_eq!(edge.states.len(), edge.controls.len() + 1);

            for s in &edge.states {
                assert!(
                    !world.intersects(generator.pose_of(s), &generator.body),
                    "truncation kept a colliding state at ({}, {})",
                    s.x,
                    s.y,
                );
            }
            truncated += 1;
        }

        assert!(
            truncated > 0,
            "every rollout into the wall was thrown away, so the tree cannot grow in clutter",
        );
    }

    #[test]
    fn a_pose_with_no_room_at_all_yields_no_edge() {
        // The honest failure: a start already wedged in geometry has no collision-free
        // extension, and the generator must say so rather than hand back a zero-length edge a
        // caller would add to the tree.
        let generator = generator();
        let world = PlanningWorld::from_obstacles(&[], 1, 1);
        let mut rng = StdRng::seed_from_u64(5);

        assert!(
            generator
                .generate_edge(&state(0.0, 0.0, 0.0, 0.0), &state(1.0, 1.0, 0.0, 0.0), &world, &mut rng)
                .is_none(),
        );
    }

    #[test]
    fn a_seed_replays_the_same_edge() {
        // Why the rng is a parameter. Without it a failing search cannot be re-run, which is
        // the whole reason `Xorshift` exists elsewhere in this codebase.
        let generator = generator();
        let world = PlanningWorld::from_obstacles(&[], 20, 20);
        let (start, target) = (state(4.0, 4.0, 0.3, 1.0), state(15.0, 15.0, 0.0, 0.0));

        let once = generator
            .generate_edge(&start, &target, &world, &mut StdRng::seed_from_u64(42))
            .unwrap();
        let twice = generator
            .generate_edge(&start, &target, &world, &mut StdRng::seed_from_u64(42))
            .unwrap();

        assert_eq!(once.cost, twice.cost);
        assert_eq!(once.states.len(), twice.states.len());
        for (a, b) in once.states.iter().zip(&twice.states) {
            assert_eq!((a.x, a.y, a.theta), (b.x, b.y, b.theta));
        }
    }

    #[test]
    fn the_whole_sampled_horizon_is_reachable() {
        // `min + rand % (max - min)` could never produce `max`, so the longest stride the
        // generator advertised never happened. Pinned because it is silent — the planner still
        // works, just never extends as far as it says it can.
        // One sample per call, so the returned edge *is* the sampled horizon rather than the
        // best of fifteen — otherwise selection pressure, not the sampler, decides the length
        // and the test measures the wrong thing.
        let mut generator = generator();
        generator.num_control_samples = 1;

        // Open ground in the middle of a large world at low speed: nothing can truncate, so
        // every edge runs its full sampled horizon.
        let world = PlanningWorld::from_obstacles(&[], 60, 60);
        let mut rng = StdRng::seed_from_u64(1);

        let (mut longest, mut shortest) = (0, usize::MAX);
        for _ in 0..200 {
            let edge = generator
                .generate_edge(&state(30.0, 30.0, 0.0, 0.5), &state(31.0, 31.0, 0.0, 0.0), &world, &mut rng)
                .unwrap();
            longest = longest.max(edge.controls.len());
            shortest = shortest.min(edge.controls.len());
        }
        assert_eq!(longest, generator.max_extension_steps, "the longest extension never occurs");
        assert_eq!(shortest, generator.min_extension_steps, "the shortest extension never occurs");
    }
}
