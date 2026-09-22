//! The obstacle geometry a kinodynamic edge is checked against.
//!
//! Deliberately *not* a physics simulation. A planner asks one question — "would the robot,
//! placed exactly here, be touching anything?" — thousands of times per edge, about poses the
//! robot never actually visits. Rapier's rigid-body machinery answers a different question,
//! about bodies that move under forces over time, and driving it for this is both wrong and
//! slow:
//!
//! * `RigidBody::set_next_kinematic_translation` does not move anything. It records a target
//!   for the next `PhysicsPipeline::step`, from which a kinematic velocity is derived. Nothing
//!   moves until that step runs.
//! * `NarrowPhase::contact_pairs_with` reports contacts computed *during* the last step. A
//!   planner steps nothing, so it reads whatever the world looked like before planning began.
//!
//! Together those mean the obvious-looking teleport-then-ask-the-narrow-phase check silently
//! answers about a stale pose. What this module uses instead is the scene-query path —
//! [`QueryPipeline::intersect_shape`] against an explicit [`Pose`] — which takes `&self`, needs
//! no stepping, and tests the pose it is handed.

use crate::models::cell::Cell;
use crate::models::grid_world_manager::GridWorldManager;
use crate::models::obstacle::{ObstaclePoly, footprint};
use crate::models::scale::METERS_PER_CELL;
use rapier2d::parry::query::DefaultQueryDispatcher;
use rapier2d::prelude::*;

/// Static collision geometry, built once per replan and queried thereafter.
///
/// Holds a `RigidBodySet` only because the query filter's signature wants one; every collider
/// here is free-standing and attached to no body, which is the right model for scenery that a
/// planner treats as immovable for the duration of a search.
pub struct PlanningWorld {
    /// How many shape queries have been asked of this world.
    ///
    /// A `Cell` so [`intersects`](Self::intersects) keeps taking `&self` — the planner holds
    /// this world immutably from its innermost loop, and threading a `&mut` up through
    /// `generate_edge` to count would reshape the call graph for a diagnostic. Not `Sync`, but
    /// a world belongs to one search on one thread.
    queries: std::cell::Cell<u64>,
    colliders: ColliderSet,
    bodies: RigidBodySet,
    broad_phase: BroadPhaseBvh,
    dispatcher: DefaultQueryDispatcher,
}

impl PlanningWorld {
    /// Builds the world an RRT search runs against, from the same obstacles the grid planners
    /// are given.
    ///
    /// The obstacles are rasterized by [`footprint`] and each blocked cell becomes a square,
    /// rather than the polygon outlines being converted to metric geometry directly. That is a
    /// choice, and the reason is comparability: `footprint` is *the* definition of "blocked" in
    /// this codebase — the grid planners search it, obstacle collision uses it, and the
    /// frontend overlay mirrors it. A second, subtly different notion of the same obstacle
    /// would let one planner route through a corner another considers solid, and the two
    /// planners' costs would stop measuring the same world.
    ///
    /// The cost is that a diagonal obstacle edge becomes a staircase for the continuous
    /// planner too. That is the price of the grid being the shared reference, and it is paid
    /// on purpose rather than by accident.
    ///
    /// One collider per blocked cell. The BVH makes the query cost logarithmic in that count,
    /// so the obvious optimization — merging runs of cells in a row into one box — is a
    /// constant-factor improvement on build time and is left until something measures it.
    pub fn from_obstacles(obstacles: &[ObstaclePoly], width: i32, height: i32) -> Self {
        let mut colliders = ColliderSet::new();
        let half = (METERS_PER_CELL / 2.0) as f32;

        for obstacle in obstacles {
            for (x, y) in footprint(obstacle, width, height) {
                colliders.insert(
                    ColliderBuilder::cuboid(half, half)
                        .translation(Self::center(x, y))
                        .build(),
                );
            }
        }

        Self::finish(colliders, width, height)
    }

    /// Builds the world from an already-rasterized grid.
    ///
    /// The entry point a [`Planner`](crate::models::planners::Planner) can reach, since the
    /// trait hands over a `GridWorldManager` rather than the polygons it came from. The result
    /// is identical to [`from_obstacles`](Self::from_obstacles) on the same shapes — both box
    /// the blocked cells — so which route a planner arrives by makes no difference to what it
    /// finds solid.
    pub fn from_grid(grid: &GridWorldManager<Cell>) -> Self {
        let mut colliders = ColliderSet::new();
        let half = (METERS_PER_CELL / 2.0) as f32;

        for (id, cell) in grid.iter() {
            if !cell.blocked {
                continue;
            }
            let (x, y) = grid.xy(id);
            colliders.insert(
                ColliderBuilder::cuboid(half, half)
                    .translation(Self::center(x, y))
                    .build(),
            );
        }

        Self::finish(colliders, grid.width() as i32, grid.height() as i32)
    }

    /// The metric centre of a cell, as a rapier vector.
    fn center(x: usize, y: usize) -> Vector {
        Vector::new(
            (x as f64 as f32 + 0.5) * METERS_PER_CELL as f32,
            (y as f64 as f32 + 0.5) * METERS_PER_CELL as f32,
        )
    }

    /// Adds the boundary and indexes everything.
    fn finish(mut colliders: ColliderSet, width: i32, height: i32) -> Self {
        Self::add_boundary(&mut colliders, width, height);
        let mut world = Self {
            queries: std::cell::Cell::new(0),
            colliders,
            bodies: RigidBodySet::new(),
            broad_phase: BroadPhaseBvh::new(),
            dispatcher: DefaultQueryDispatcher,
        };
        world.rebuild_broad_phase();
        world
    }

    /// Four walls just outside the grid, each one cell thick.
    ///
    /// Outside rather than on the boundary cells, so the outermost row and column stay usable:
    /// the grid planners can route along them, and a continuous planner that could not would
    /// be solving a smaller problem.
    fn add_boundary(colliders: &mut ColliderSet, width: i32, height: i32) {
        let (w, h) = (
            width as f32 * METERS_PER_CELL as f32,
            height as f32 * METERS_PER_CELL as f32,
        );
        let t = METERS_PER_CELL as f32 / 2.0;

        for (hx, hy, x, y) in [
            (w / 2.0 + t, t, w / 2.0, -t),     // below
            (w / 2.0 + t, t, w / 2.0, h + t),  // above
            (t, h / 2.0 + t, -t, h / 2.0),     // left
            (t, h / 2.0 + t, w + t, h / 2.0),  // right
        ] {
            colliders.insert(
                ColliderBuilder::cuboid(hx, hy)
                    .translation(Vector::new(x, y))
                    .build(),
            );
        }
    }

    /// Indexes every collider for querying.
    ///
    /// The broad phase is normally driven by `PhysicsPipeline::step`; `update` is public
    /// precisely so it can be driven without one, which is what a query-only world needs. Each
    /// collider is reported as modified because this runs once, on a world that was just
    /// built, so all of them are new.
    fn rebuild_broad_phase(&mut self) {
        let modified: Vec<ColliderHandle> = self.colliders.iter().map(|(handle, _)| handle).collect();
        self.broad_phase.update(
            &IntegrationParameters::default(),
            &self.colliders,
            &self.bodies,
            &modified,
            &[],
            &mut Vec::new(),
        );
    }

    /// Whether `shape`, placed at `pose`, touches any obstacle.
    ///
    /// Short-circuits on the first hit: a planner only ever needs to know *whether* a pose is
    /// blocked, never by what, and collecting every overlapping collider to then discard all
    /// but one is wasted work in the innermost loop of a search.
    pub fn intersects(&self, pose: Pose, shape: &dyn Shape) -> bool {
        self.queries.set(self.queries.get() + 1);
        self.broad_phase
            .as_query_pipeline(
                &self.dispatcher,
                &self.bodies,
                &self.colliders,
                QueryFilter::default(),
            )
            .intersect_shape(pose, shape)
            .next()
            .is_some()
    }

    /// How many shape queries have been run against this world.
    ///
    /// The planner's dominant cost, so the number to watch first when a search is slow: every
    /// integration step of every sampled rollout asks exactly one.
    pub fn queries(&self) -> u64 {
        self.queries.get()
    }

    /// How many colliders the world holds. Test support, and a cheap sanity check that a
    /// rebuild actually indexed something.
    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::obstacle::CellVertex;

    /// A solid block covering cells 3..=5 in both axes.
    fn block() -> ObstaclePoly {
        ObstaclePoly {
            id: 1,
            dynamic: false,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 3, y: 3 },
                CellVertex { x: 5, y: 3 },
                CellVertex { x: 5, y: 5 },
                CellVertex { x: 3, y: 5 },
                CellVertex { x: 3, y: 3 },
            ],
        }
    }

    fn at(x: f64, y: f64) -> Pose {
        Pose::new(Vector::new(x as f32, y as f32), 0.0)
    }

    /// A point-sized probe, so these tests are about the *world* rather than about any
    /// particular robot's girth.
    fn probe() -> Ball {
        Ball::new(1.0e-3)
    }

    #[test]
    fn a_pose_inside_an_obstacle_collides() {
        // The case the old narrow-phase check got wrong in the most damaging way: a robot
        // fully swallowed by a shape has no *contact manifold* with it, so a contact-pair
        // query reports nothing and the planner routes straight through the middle.
        let world = PlanningWorld::from_obstacles(&[block()], 10, 10);
        assert!(world.intersects(at(4.5, 4.5), &probe()), "dead centre of a solid block");
    }

    #[test]
    fn a_pose_in_open_ground_does_not() {
        let world = PlanningWorld::from_obstacles(&[block()], 10, 10);
        assert!(!world.intersects(at(0.5, 0.5), &probe()));
        assert!(!world.intersects(at(8.5, 8.5), &probe()));
    }

    #[test]
    fn the_metric_world_blocks_exactly_what_the_grid_does() {
        // The property the whole rasterize-then-box approach exists for. If these two ever
        // disagree, a route one planner calls legal is one the other calls a collision, and
        // comparing their costs stops meaning anything.
        let (width, height) = (10, 10);
        let obstacle = block();
        let world = PlanningWorld::from_obstacles(&[obstacle.clone()], width, height);
        let blocked = footprint(&obstacle, width, height);

        for y in 0..height {
            for x in 0..width {
                let center = at(
                    (x as f64 + 0.5) * METERS_PER_CELL,
                    (y as f64 + 0.5) * METERS_PER_CELL,
                );
                assert_eq!(
                    world.intersects(center, &probe()),
                    blocked.contains(&(x as usize, y as usize)),
                    "cell ({x}, {y}) disagrees with the rasterizer",
                );
            }
        }
    }

    #[test]
    fn the_grid_edge_is_solid_but_the_outermost_cells_are_not() {
        // Off-grid is impassable to `find_plan`, so it must be impassable here too — but the
        // boundary row itself is legal ground, and walling it off would quietly shrink the
        // world the continuous planner searches.
        let world = PlanningWorld::from_obstacles(&[], 10, 10);

        assert!(!world.intersects(at(0.5, 0.5), &probe()), "cell (0,0) is usable ground");
        assert!(!world.intersects(at(9.5, 9.5), &probe()), "cell (9,9) is usable ground");

        assert!(world.intersects(at(-0.5, 5.0), &probe()), "left of the grid");
        assert!(world.intersects(at(10.5, 5.0), &probe()), "right of the grid");
        assert!(world.intersects(at(5.0, -0.5), &probe()), "below the grid");
        assert!(world.intersects(at(5.0, 10.5), &probe()), "above the grid");
    }

    #[test]
    fn a_body_with_width_collides_before_its_centre_does() {
        // What makes this worth having over a cell lookup: the robot's *extent* is tested, so
        // a pose whose centre is clear but whose body overlaps is still a collision.
        let world = PlanningWorld::from_obstacles(&[block()], 10, 10);

        // Cell (2, 4) is clear, and its centre at x = 2.5 is 0.5 m from the block's face at
        // x = 3.0. A point fits; a half-metre-radius body does not.
        assert!(!world.intersects(at(2.5, 4.5), &probe()));
        assert!(
            world.intersects(at(2.5, 4.5), &Ball::new(0.6)),
            "a 0.6 m body should overlap a face 0.5 m away",
        );
    }

    #[test]
    fn an_empty_world_still_indexes_its_boundary() {
        let world = PlanningWorld::from_obstacles(&[], 6, 6);
        assert_eq!(world.collider_count(), 4, "four walls and nothing else");
    }
}
