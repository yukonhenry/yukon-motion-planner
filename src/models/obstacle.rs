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

use crate::models::rng::Xorshift;
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
pub(crate) fn advance_one_tick(
    obstacles: &mut [ObstaclePoly],
    rng: &mut Xorshift,
    width: i32,
    height: i32,
) -> usize {
    let mut moved = 0;
    for obstacle in obstacles.iter_mut().filter(|o| o.dynamic) {
        if obstacle.jitter_one_vertex(rng, width, height) {
            moved += 1;
        }
    }
    moved
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
            "vertices": [{"x": 4, "y": 8}, {"x": 5, "y": 11}, {"x": 17, "y": 6}],
        });

        let parsed: ObstaclePoly = serde_json::from_value(json.clone()).expect("valid shape");
        assert_eq!(parsed.id, 7);
        assert!(parsed.dynamic);
        assert_eq!(parsed.cells(), vec![[4, 8], [5, 11], [17, 6]]);
        assert_eq!(
            serde_json::to_value(&parsed).unwrap(),
            json,
            "not symmetric"
        );
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

    // --- jitter -----------------------------------------------------------

    /// A closed square ring, as the client stores one: first corner repeated at the end.
    fn square(dynamic: bool) -> ObstaclePoly {
        ObstaclePoly {
            id: 1,
            dynamic,
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
            moved += advance_one_tick(&mut obstacles, &mut rng, 10, 10);
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
        assert_eq!(advance_one_tick(&mut obstacles, &mut rng, 10, 10), 0);
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
