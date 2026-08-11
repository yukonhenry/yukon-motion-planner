//! Ground-truth footprints for the TypeScript mirror of the rasterizer.
//!
//! `frontend/src/geometry.ts` reimplements `rasterize_polygon` so the UI can shade the
//! cells a route is really avoiding, and the two drifting apart would draw a confidently
//! wrong picture. The fixtures in `frontend/src/geometry.raster.test.ts` are the output
//! of *this* file, not of anyone's reading of the algorithm.
//!
//! Ignored by default because it asserts nothing — it prints. Regenerate with:
//!
//! ```text
//! cargo test --test raster_fixtures -- --ignored --nocapture
//! ```
//!
//! then paste the `FIXTURE` lines into the TypeScript test.

use yukon_motion_planner::models::grid_world_manager::GridWorldManager;

fn footprint(width: usize, height: usize, polygon: &[[i32; 2]]) -> Vec<[usize; 2]> {
    let mut grid = GridWorldManager::<u8>::filled(width, height, 0u8);
    grid.rasterize_polygon(polygon, |c| *c = 1);

    let mut cells = Vec::new();
    for y in 0..height {
        for x in 0..width {
            if grid[grid.try_id(x as isize, y as isize).unwrap()] == 1 {
                cells.push([x, y]);
            }
        }
    }
    cells
}

/// Cases chosen for where the two implementations could plausibly diverge: integer
/// division that truncates toward zero rather than flooring, the half-open scanline rule
/// at shared corners, Bresenham's tie-breaking, and the bounds check that has to reject
/// negatives rather than wrap them.
#[test]
#[ignore = "prints fixtures for the TypeScript mirror; asserts nothing"]
fn dump() {
    let cases: Vec<(&str, usize, usize, Vec<[i32; 2]>)> = vec![
        ("triangle", 12, 10, vec![[2, 1], [9, 4], [3, 8]]),
        // Same shape closed, since the UI now stores the first corner twice.
        (
            "triangle_closed",
            12,
            10,
            vec![[2, 1], [9, 4], [3, 8], [2, 1]],
        ),
        ("square", 10, 10, vec![[2, 2], [6, 2], [6, 6], [2, 6]]),
        // Left-leaning edges are where truncation and flooring disagree.
        ("left_leaning", 12, 12, vec![[9, 1], [2, 5], [8, 10]]),
        (
            "concave_v",
            14,
            12,
            vec![[1, 1], [6, 8], [11, 1], [11, 10], [1, 10]],
        ),
        (
            "horizontal_edge",
            10,
            8,
            vec![[1, 2], [8, 2], [8, 5], [1, 5]],
        ),
        (
            "collinear",
            12,
            8,
            vec![[1, 1], [5, 1], [9, 1], [9, 5], [1, 5]],
        ),
        ("thin_diagonal", 10, 10, vec![[0, 0], [9, 9], [8, 9]]),
        ("degenerate_line", 10, 10, vec![[1, 1], [7, 4], [1, 1]]),
        ("partly_offgrid", 8, 8, vec![[-3, 2], [5, -1], [6, 6]]),
    ];

    for (name, w, h, polygon) in cases {
        println!(
            "FIXTURE {name} {w} {h} {} {}",
            serde_json::to_string(&polygon).unwrap(),
            serde_json::to_string(&footprint(w, h, &polygon)).unwrap()
        );
    }
}
