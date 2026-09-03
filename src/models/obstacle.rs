//! How an obstacle is written to, and read back from, `grid_worlds.obs_polygons`.
//!
//! The column is `jsonb`, so the shape lives here rather than in the schema. Both sides of
//! the API depend on it — `grid_crud` validates and writes it, `planner_crud` reads it back
//! and rasterizes — so it is defined once rather than per handler.
//!
//! **Attributes belong to the obstacle, not to its vertices.** A polygon is an object with
//! `id` and `dynamic` beside its vertex list, not a bare array of vertices each repeating
//! them. Repeating them would allow a shape whose first vertex claims `dynamic` and whose
//! fourth denies it, with nothing to say which wins.
//!
//! Geometry stops at [`ObstaclePoly::cells`]. Everything below the API boundary — the
//! rasterizer, the planners — works in plain `[i32; 2]` cell pairs and knows nothing about
//! ids or motion, which is what keeps this change out of
//! [`grid_world_manager`](crate::models::grid_world_manager).

use crate::models::grid_world_manager::GridWorldManager;
use crate::models::rng::Xorshift;
use std::collections::HashSet;
use serde::{Deserialize, Serialize};

/// One vertex, as a pair of cell indices.
///
/// An object rather than a two-element array so the field names travel with the data;
/// `{"x": 4, "y": 8}` survives being read by something that doesn't already know the
/// convention, where `[4, 8]` does not.
///
/// Cell *indices*, not continuous coordinates — sub-cell motion is not expressible here, and
/// giving obstacles smooth movement means revisiting this type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct CellVertex {
    pub x: i32,
    pub y: i32,
}

impl CellVertex {
    pub(crate) fn to_cell(self) -> [i32; 2] {
        [self.x, self.y]
    }
}

/// One obstacle: a closed polygon, an identity, and whether it is allowed to move.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct ObstaclePoly {
    /// Stable across saves, so a moving obstacle can be followed from one snapshot to the
    /// next. Assigned by the client, which is the only place that knows a shape's history;
    /// the server checks the ids in a payload are distinct and otherwise leaves them alone.
    pub id: i32,
    /// Whether a simulation may perturb this obstacle. `false` for anything the user drew
    /// as fixed scenery.
    ///
    /// A JSON boolean rather than `0`/`1`: the field is a flag, and a typed struct makes the
    /// wire format match the meaning at no cost.
    #[serde(default)]
    pub dynamic: bool,
    /// Cells translated per tick, as `[dx, dy]`. `[0, 0]` — the default, and what every
    /// obstacle stored before this field existed reads as — keeps the original behavior:
    /// a dynamic obstacle with no velocity jitters one corner at random.
    ///
    /// Whole cells rather than a rate. Obstacles round-trip through JSON on the manual
    /// `POST /grids/{id}/replan` path every tick, so a fractional velocity would need its
    /// unspent remainder carried on the wire as well — otherwise the stepped and scheduled
    /// simulations would disagree about where a slow obstacle had got to, which is exactly
    /// the divergence [`crate::models::simulation::plan_route`] exists to prevent.
    #[serde(default)]
    pub velocity: [i32; 2],
    pub vertices: Vec<CellVertex>,
}

/// The eight directions a jittered vertex may take, as `(dx, dy)`.
///
/// Matches the planner's 8-connected movement model, so a shape can deform along any direction
/// the agent could itself have travelled.
const DIRECTIONS: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

impl ObstaclePoly {
    /// The geometry alone, in the form the rasterizer takes.
    pub(crate) fn cells(&self) -> Vec<[i32; 2]> {
        self.vertices
            .iter()
            .copied()
            .map(CellVertex::to_cell)
            .collect()
    }

    /// Whether the vertex list repeats its first corner at the end.
    ///
    /// The client stores rings closed, which makes "corners" and "entries" differ by exactly
    /// one. It matters here because the repeated entry is not a corner of its own — jittering
    /// it independently of the first would tear the ring open into a different shape.
    fn is_closed_ring(&self) -> bool {
        self.vertices.len() > 1 && self.vertices.first() == self.vertices.last()
    }

    /// How many distinct corners the ring has.
    fn corner_count(&self) -> usize {
        if self.is_closed_ring() {
            self.vertices.len() - 1
        } else {
            self.vertices.len()
        }
    }

    /// This obstacle shifted by `(dx, dy)`, or `None` if that would put any corner off-grid.
    ///
    /// All-or-nothing because a translation is rigid: clamping the corners that would leave
    /// the grid while letting the rest travel would silently reshape the obstacle into
    /// something the user never drew.
    pub(crate) fn translated(&self, dx: i32, dy: i32, width: i32, height: i32) -> Option<Self> {
        let mut moved = self.clone();
        for vertex in &mut moved.vertices {
            let x = vertex.x + dx;
            let y = vertex.y + dy;
            if !(0..width).contains(&x) || !(0..height).contains(&y) {
                return None;
            }
            *vertex = CellVertex { x, y };
        }
        Some(moved)
    }

    /// Moves one randomly chosen corner one cell in one of eight directions, clamped to the
    /// grid. Returns whether the geometry actually changed.
    ///
    /// One corner rather than the whole shape, because deforming is the interesting case: a
    /// translation keeps every edge cost the planner already knows and merely shifts where they
    /// are, while a single moved corner adds cells on one side and frees them on another, which
    /// is what an incremental replan has to cope with. It also stands in for *uncertainty* about
    /// an obstacle's true extent rather than about its position.
    ///
    /// Clamping can leave the corner where it was — at a wall, a step into the wall is simply
    /// not taken. That is reported as `false` rather than retried: a tick in which nothing moved
    /// is a legitimate outcome, and looping until something does would quietly bias the walk
    /// away from the edges of the grid.
    pub(crate) fn jitter_one_vertex(
        &mut self,
        rng: &mut Xorshift,
        width: i32,
        height: i32,
    ) -> bool {
        let corners = self.corner_count();
        if corners == 0 || width <= 0 || height <= 0 {
            return false;
        }

        // Read before the move: once the first entry changes, `first == last` no longer holds
        // and the ring can no longer be recognized as closed.
        let closed = self.is_closed_ring();
        let corner = rng.below(corners);
        let (dx, dy) = DIRECTIONS[rng.below(DIRECTIONS.len())];

        let before = self.vertices[corner];
        let after = CellVertex {
            x: (before.x + dx).clamp(0, width - 1),
            y: (before.y + dy).clamp(0, height - 1),
        };
        if after == before {
            return false;
        }

        self.vertices[corner] = after;
        // The repeated entry is the same corner, so it moves with it or the ring tears.
        if closed && corner == 0 {
            let last = self.vertices.len() - 1;
            self.vertices[last] = after;
        }
        true
    }
}

/// Advances every `dynamic` obstacle by one tick, and reports how many actually moved.
///
/// Static obstacles are left exactly as they are — that is the whole meaning of the flag — so
/// the returned count is also the answer to "did this tick change the world at all", which is
/// what tells a caller whether a replan has anything to do.
/// What one environment tick did to the obstacles.
#[derive(Debug, Default, PartialEq, Eq)]
pub(crate) struct MotionReport {
    /// How many obstacles actually changed shape or position.
    pub moved: usize,
    /// Obstacles whose move was refused because it would have overlapped another obstacle or
    /// left the grid. Ids, so a client can point at the shape that is stuck.
    pub blocked: Vec<i32>,
    /// Obstacles whose move was refused because it would have run over the robot.
    ///
    /// Separate from [`blocked`](Self::blocked) because it means something different to a
    /// user: two obstacles jostling is scenery, a machine being driven at is a warning.
    pub robot_hits: Vec<i32>,
}

/// Every cell one obstacle covers, boundary included.
///
/// Rasterized by [`GridWorldManager`] rather than by a polygon test of its own, and that is
/// the whole design: this is the *same* fill the planner uses to decide which cells are
/// blocked, so "these two obstacles overlap" and "the robot cannot stand there" are one
/// predicate rather than two that might disagree. Concave shapes come free with it — the fill
/// is even-odd scanline plus a Bresenham edge pass, which has no convexity assumption.
pub(crate) fn footprint(
    obstacle: &ObstaclePoly,
    width: i32,
    height: i32,
) -> HashSet<(usize, usize)> {
    let mut grid = GridWorldManager::<bool>::new(width.max(0) as usize, height.max(0) as usize);
    grid.rasterize_polygon(&obstacle.cells(), |cell| *cell = true);
    grid.iter()
        .filter(|(_, covered)| **covered)
        .map(|(id, _)| grid.xy(id))
        .collect()
}

/// Advances every dynamic obstacle one tick, refusing any move that would collide.
///
/// Obstacles are considered in the order they arrive, each tested against the *committed*
/// state of the others — so an obstacle that has already moved this tick is checked at its new
/// position, and one that has not is checked where it still stands. That makes the outcome a
/// function of the list order rather than of iteration luck, and the list order is the
/// client's, which is stable across a run.
///
/// A refused move is simply not taken: the obstacle stays exactly where it was for this tick
/// and tries again on the next one. Nothing is latched, so two obstacles that drift apart
/// resume moving without anything having to clear a flag.
///
/// `robot` is where the machine is standing, or `None` on the manual replan path, which has no
/// robot to run over.
pub(crate) fn advance_one_tick(
    obstacles: &mut [ObstaclePoly],
    rng: &mut Xorshift,
    width: i32,
    height: i32,
    robot: Option<[i32; 2]>,
) -> MotionReport {
    let mut report = MotionReport::default();

    // Built once and patched as obstacles commit, rather than rebuilt per candidate: only the
    // shape that just moved can have changed.
    let mut footprints: Vec<HashSet<(usize, usize)>> = obstacles
        .iter()
        .map(|o| footprint(o, width, height))
        .collect();

    for index in 0..obstacles.len() {
        if !obstacles[index].dynamic {
            continue;
        }

        let [dx, dy] = obstacles[index].velocity;
        if dx == 0 && dy == 0 {
            // No velocity means the original behavior: deform rather than translate. Jitter
            // is unchecked against collisions on purpose — it is a statement about
            // *uncertainty* in an obstacle's extent, not about it driving somewhere.
            if obstacles[index].jitter_one_vertex(rng, width, height) {
                report.moved += 1;
                footprints[index] = footprint(&obstacles[index], width, height);
            }
            continue;
        }

        let Some(candidate) = obstacles[index].translated(dx, dy, width, height) else {
            // Off the grid. Rejected whole rather than clamped per vertex, which would
            // deform a shape that is supposed to be rigid — a wall blocks like anything else.
            report.blocked.push(obstacles[index].id);
            continue;
        };

        let cells = footprint(&candidate, width, height);

        if let Some([rx, ry]) = robot
            && rx >= 0
            && ry >= 0
            && cells.contains(&(rx as usize, ry as usize))
        {
            report.robot_hits.push(obstacles[index].id);
            continue;
        }

        let hits_obstacle = footprints
            .iter()
            .enumerate()
            .any(|(other, occupied)| other != index && !occupied.is_disjoint(&cells));
        if hits_obstacle {
            report.blocked.push(obstacles[index].id);
            continue;
        }

        obstacles[index] = candidate;
        footprints[index] = cells;
        report.moved += 1;
    }

    report
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The wire format, pinned. A silent rename here would be a silent data migration, since
    /// every stored row is read back through these names.
    #[test]
    fn an_obstacle_round_trips_through_its_json_shape() {
        let json = serde_json::json!({
            "id": 7,
            "dynamic": true,
            "velocity": [1, -2],
            "vertices": [{"x": 4, "y": 8}, {"x": 5, "y": 11}, {"x": 17, "y": 6}],
        });

        let parsed: ObstaclePoly = serde_json::from_value(json.clone()).expect("valid shape");
        assert_eq!(parsed.id, 7);
        assert!(parsed.dynamic);
        assert_eq!(parsed.velocity, [1, -2]);
        assert_eq!(parsed.cells(), vec![[4, 8], [5, 11], [17, 6]]);
        assert_eq!(
            serde_json::to_value(&parsed).unwrap(),
            json,
            "not symmetric"
        );
    }

    #[test]
    fn an_obstacle_stored_before_velocity_existed_reads_as_stationary() {
        // Every `grid_world_states` row written before this field predates it, and a plan's
        // world is read back through this type. Defaulting to `[0, 0]` is what lets those
        // rows keep their original meaning — dynamic, and jittering — rather than failing to
        // parse or silently acquiring a direction nobody chose.
        let json = serde_json::json!({
            "id": 3,
            "dynamic": true,
            "vertices": [{"x": 0, "y": 0}, {"x": 2, "y": 0}, {"x": 2, "y": 2}],
        });

        let parsed: ObstaclePoly = serde_json::from_value(json).expect("valid shape");
        assert_eq!(parsed.velocity, [0, 0]);
    }

    #[test]
    fn dynamic_defaults_to_false_when_absent() {
        // Scenery is the common case, so omitting the flag must mean "does not move" rather
        // than being rejected.
        let parsed: ObstaclePoly = serde_json::from_value(serde_json::json!({
            "id": 1,
            "vertices": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}],
        }))
        .expect("dynamic is optional");
        assert!(!parsed.dynamic);
    }

    #[test]
    fn a_vertex_missing_a_coordinate_is_rejected() {
        // The point of the typed struct over a map: this fails here, at the boundary, rather
        // than as an absent key noticed somewhere downstream.
        let result: Result<ObstaclePoly, _> = serde_json::from_value(serde_json::json!({
            "id": 1,
            "vertices": [{"x": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}],
        }));
        assert!(result.is_err(), "a vertex without `y` is not a vertex");
    }

    // --- translation and collision ----------------------------------------

    /// A closed rectangle from `(x, y)` to `(x + w, y + h)`, travelling at `velocity`.
    fn box_at(id: i32, x: i32, y: i32, w: i32, h: i32, velocity: [i32; 2]) -> ObstaclePoly {
        ObstaclePoly {
            id,
            dynamic: true,
            velocity,
            vertices: vec![
                CellVertex { x, y },
                CellVertex { x: x + w, y },
                CellVertex { x: x + w, y: y + h },
                CellVertex { x, y: y + h },
                CellVertex { x, y },
            ],
        }
    }

    /// A `U` opening upward: the gap between its arms is inside the bounding box but outside
    /// the shape, which is the whole point of testing concave collision.
    fn u_shape(id: i32, velocity: [i32; 2]) -> ObstaclePoly {
        ObstaclePoly {
            id,
            dynamic: true,
            velocity,
            vertices: vec![
                CellVertex { x: 2, y: 2 },
                CellVertex { x: 8, y: 2 },
                CellVertex { x: 8, y: 8 },
                CellVertex { x: 7, y: 8 },
                CellVertex { x: 7, y: 3 },
                CellVertex { x: 3, y: 3 },
                CellVertex { x: 3, y: 8 },
                CellVertex { x: 2, y: 8 },
                CellVertex { x: 2, y: 2 },
            ],
        }
    }

    fn tick(obstacles: &mut [ObstaclePoly], robot: Option<[i32; 2]>) -> MotionReport {
        let mut rng = Xorshift::new(1);
        advance_one_tick(obstacles, &mut rng, 20, 20, robot)
    }

    #[test]
    fn a_velocity_translates_the_whole_shape() {
        let mut obstacles = vec![box_at(1, 0, 0, 2, 2, [3, -0])];
        let report = tick(&mut obstacles, None);

        assert_eq!(report.moved, 1);
        assert!(report.blocked.is_empty());
        // Every corner moved by the same amount: a translation is rigid.
        assert_eq!(obstacles[0].cells(), vec![[3, 0], [5, 0], [5, 2], [3, 2], [3, 0]]);
    }

    #[test]
    fn a_translation_that_would_leave_the_grid_is_refused_whole() {
        // Clamping the corners that fall off while letting the rest travel would reshape the
        // obstacle into something nobody drew, so the wall blocks like any other obstruction.
        let mut obstacles = vec![box_at(1, 17, 0, 2, 2, [3, 0])];
        let before = obstacles[0].clone();
        let report = tick(&mut obstacles, None);

        assert_eq!(report.moved, 0);
        assert_eq!(report.blocked, vec![1]);
        assert_eq!(obstacles[0], before, "a refused move must change nothing");
    }

    #[test]
    fn an_obstacle_stops_rather_than_overlap_another() {
        let mut obstacles = vec![box_at(1, 0, 0, 2, 2, [2, 0]), box_at(2, 3, 0, 2, 2, [0, 0])];
        // The second is dynamic with no velocity, so it jitters; freeze it to keep this test
        // about the first one's translation.
        obstacles[1].dynamic = false;

        let report = tick(&mut obstacles, None);
        assert_eq!(report.moved, 0);
        assert_eq!(report.blocked, vec![1]);
        assert_eq!(obstacles[0].cells()[0], [0, 0], "it should not have moved");
    }

    #[test]
    fn collision_follows_the_shape_rather_than_its_bounding_box() {
        // A box driving into the mouth of a `U`. Their bounding boxes overlap immediately, so
        // an AABB test would refuse the move — but the cells do not, and the box should slide
        // cleanly into the gap. This is what makes the check concave-correct.
        let mut obstacles = vec![u_shape(1, [0, 0]), box_at(2, 4, 10, 2, 2, [0, -4])];
        obstacles[0].dynamic = false;

        let report = tick(&mut obstacles, None);
        assert_eq!(report.moved, 1, "the gap in the U is free space");
        assert!(report.blocked.is_empty());
        assert_eq!(obstacles[1].cells()[0], [4, 6]);

        // One more step drives it into the closed end, which must be refused.
        obstacles[1].velocity = [0, -3];
        let report = tick(&mut obstacles, None);
        assert_eq!(report.moved, 0);
        assert_eq!(report.blocked, vec![2]);
    }

    #[test]
    fn an_obstacle_stops_rather_than_run_the_robot_over() {
        let mut obstacles = vec![box_at(1, 0, 0, 2, 2, [3, 0])];
        let report = tick(&mut obstacles, Some([4, 1]));

        // Reported apart from `blocked`: two obstacles jostling is scenery, a machine being
        // driven at is a warning.
        assert_eq!(report.robot_hits, vec![1]);
        assert!(report.blocked.is_empty());
        assert_eq!(report.moved, 0);
        assert_eq!(obstacles[0].cells()[0], [0, 0]);
    }

    #[test]
    fn a_blocked_obstacle_moves_again_once_the_way_clears() {
        // Nothing is latched: the refusal lasts one tick, so obstacles that drift apart
        // resume on their own rather than needing a flag cleared.
        let mut obstacles = vec![box_at(1, 0, 0, 2, 2, [3, 0])];
        assert_eq!(tick(&mut obstacles, Some([4, 1])).robot_hits, vec![1]);

        let report = tick(&mut obstacles, Some([15, 15]));
        assert_eq!(report.moved, 1);
        assert!(report.robot_hits.is_empty());
    }

    #[test]
    fn a_static_obstacle_never_translates_however_fast_it_claims_to_be() {
        // `dynamic` stays the master switch: velocity on scenery is inert, not a back door.
        let mut obstacles = vec![box_at(1, 0, 0, 2, 2, [3, 3])];
        obstacles[0].dynamic = false;
        let before = obstacles[0].clone();

        assert_eq!(tick(&mut obstacles, None), MotionReport::default());
        assert_eq!(obstacles[0], before);
    }

    // --- jitter -----------------------------------------------------------

    /// A closed square ring, as the client stores one: first corner repeated at the end.
    fn square(dynamic: bool) -> ObstaclePoly {
        ObstaclePoly {
            id: 1,
            dynamic,
            velocity: [0, 0],
            vertices: vec![
                CellVertex { x: 3, y: 3 },
                CellVertex { x: 6, y: 3 },
                CellVertex { x: 6, y: 6 },
                CellVertex { x: 3, y: 6 },
                CellVertex { x: 3, y: 3 },
            ],
        }
    }

    #[test]
    fn jitter_moves_exactly_one_corner_by_at_most_one_cell() {
        // The defining property: a *deformation*, not a translation. Swept over many seeds
        // because which corner and which direction are both random, and a bug that occasionally
        // moves two corners would hide behind a single lucky draw.
        for seed in 1..200u64 {
            let before = square(true);
            let mut after = before.clone();
            let mut rng = Xorshift::new(seed);
            if !after.jitter_one_vertex(&mut rng, 10, 10) {
                continue; // clamped against an edge; nothing moved this draw
            }

            assert_eq!(
                after.vertices.len(),
                before.vertices.len(),
                "seed {seed}: the vertex count changed",
            );

            // Compared over corners rather than entries, so the closed ring's repeated first
            // entry is not counted as a second moved corner.
            let corners = before.corner_count();
            let differing: Vec<usize> = (0..corners)
                .filter(|&i| after.vertices[i] != before.vertices[i])
                .collect();
            assert_eq!(differing.len(), 1, "seed {seed}: moved {differing:?}");

            let i = differing[0];
            let (dx, dy) = (
                after.vertices[i].x - before.vertices[i].x,
                after.vertices[i].y - before.vertices[i].y,
            );
            assert!(
                dx.abs() <= 1 && dy.abs() <= 1,
                "seed {seed}: jumped by ({dx}, {dy})",
            );
        }
    }

    #[test]
    fn jitter_keeps_a_closed_ring_closed() {
        // The repeated entry is the same corner as the first. Moving one without the other tears
        // the ring into a different shape, which the rasterizer would happily fill.
        for seed in 1..200u64 {
            let mut obstacle = square(true);
            let mut rng = Xorshift::new(seed);
            obstacle.jitter_one_vertex(&mut rng, 10, 10);
            assert_eq!(
                obstacle.vertices.first(),
                obstacle.vertices.last(),
                "seed {seed}: the ring came open",
            );
        }
    }

    #[test]
    fn jitter_keeps_every_vertex_on_the_grid() {
        // A shape pinned into the top-left corner, where four of the eight directions lead off
        // the grid. Staying in bounds is what keeps the result re-validatable and rasterizable.
        for seed in 1..300u64 {
            let mut obstacle = ObstaclePoly {
                id: 1,
                dynamic: true,
                velocity: [0, 0],
                vertices: vec![
                    CellVertex { x: 0, y: 0 },
                    CellVertex { x: 1, y: 0 },
                    CellVertex { x: 0, y: 1 },
                    CellVertex { x: 0, y: 0 },
                ],
            };
            let mut rng = Xorshift::new(seed);
            obstacle.jitter_one_vertex(&mut rng, 4, 4);

            for vertex in &obstacle.vertices {
                assert!(
                    (0..4).contains(&vertex.x) && (0..4).contains(&vertex.y),
                    "seed {seed}: {vertex:?} left the 4x4 grid",
                );
            }
        }
    }

    #[test]
    fn jitter_is_reproducible_from_its_seed() {
        // What makes a run that exposes a planner bug worth reporting.
        let run = |seed: u64| {
            let mut obstacle = square(true);
            let mut rng = Xorshift::new(seed);
            for _ in 0..25 {
                obstacle.jitter_one_vertex(&mut rng, 10, 10);
            }
            obstacle
        };
        assert_eq!(run(4242), run(4242));
        assert_ne!(run(4242), run(4243), "the seed made no difference");
    }

    #[test]
    fn a_tick_moves_dynamic_obstacles_and_leaves_the_rest_alone() {
        let mut obstacles = vec![square(false), square(true)];
        let unchanged = obstacles[0].clone();
        let mut rng = Xorshift::new(0x2026_0811);

        // Ticked until something moves, so the assertion is about the flag rather than about
        // whichever draw the first tick happened to make.
        let mut moved = 0;
        for _ in 0..20 {
            moved += advance_one_tick(&mut obstacles, &mut rng, 10, 10, None).moved;
            if moved > 0 {
                break;
            }
        }

        assert!(moved > 0, "20 ticks moved nothing");
        assert_eq!(obstacles[0], unchanged, "a static obstacle moved");
        assert_ne!(obstacles[1], unchanged, "the dynamic obstacle did not move");
    }

    #[test]
    fn a_tick_over_only_static_obstacles_reports_no_movement() {
        // What tells the caller a replan has nothing to do.
        let mut obstacles = vec![square(false), square(false)];
        let before = obstacles.clone();
        let mut rng = Xorshift::new(5);
        assert_eq!(advance_one_tick(&mut obstacles, &mut rng, 10, 10, None).moved, 0);
        assert_eq!(obstacles, before);
    }

    #[test]
    fn a_jittered_obstacle_still_satisfies_what_the_api_requires_of_one() {
        // Jitter feeds straight back into a payload the server will re-validate, so it must not
        // be able to produce a shape its own validator rejects: three corners minimum, all in
        // bounds. Ids and the flag ride along untouched.
        let mut obstacle = square(true);
        let mut rng = Xorshift::new(31337);
        for _ in 0..200 {
            obstacle.jitter_one_vertex(&mut rng, 8, 8);
            assert!(obstacle.vertices.len() >= 3);
            assert!(
                obstacle
                    .vertices
                    .iter()
                    .all(|v| (0..8).contains(&v.x) && (0..8).contains(&v.y))
            );
            assert_eq!(obstacle.id, 1);
            assert!(obstacle.dynamic);
        }
    }

    #[test]
    fn the_old_bare_array_format_is_rejected() {
        // Rows written before obstacles had attributes look like `[[4, 8], ...]`. Nothing
        // reads both formats, so this is the error a stale row produces — worth pinning so it
        // is recognizable rather than mysterious.
        let result: Result<ObstaclePoly, _> =
            serde_json::from_value(serde_json::json!([[4, 8], [5, 11], [17, 6]]));
        assert!(result.is_err());
    }
}
