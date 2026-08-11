//! A dense 2D grid world, stored row-major in a flat `Vec`.
//!
//! The design goal is search-friendliness rather than array math: a cell's identity is
//! a flat [`NodeId`], so path-planning bookkeeping (`g_score`, `came_from`, `closed`)
//! can be plain `Vec`s indexed directly instead of `HashMap`s keyed by coordinate pairs.
//! That removes hashing from the hot loop and keeps the accesses cache-friendly.
//!
//! Extending to 3D means changing only the index arithmetic —
//! `(z * height + y) * width + x` — and the neighbor offsets.
//!
//! ```
//! # use yukon_motion_planner::models::grid_world_manager::GridWorldManager;
//! let mut grid = GridWorldManager::filled(4, 3, 0u8);
//! let id = grid.id(2, 1);
//! grid[id] = 7;
//! assert_eq!(grid.xy(id), (2, 1));
//! assert_eq!(grid[id], 7);
//! ```

use std::ops::{Index, IndexMut};

/// A cell's flat index.
///
/// Deliberately a bare `usize`: it doubles as the index into any per-node array a
/// search keeps alongside the grid.
///
/// Ids are only meaningful for the grid that produced them — one grid's [`NodeId`]
/// used against a differently-shaped grid will silently address the wrong cell.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct NodeId(pub usize);

/// Orthogonal steps, in N/S/W/E order.
const ORTHOGONAL: &[(i8, i8)] = &[(0, -1), (0, 1), (-1, 0), (1, 0)];

/// Orthogonal steps first, then diagonals — callers that treat the two differently
/// (√2 step cost, corner-cutting rules) can rely on that grouping.
const ORTHOGONAL_AND_DIAGONAL: &[(i8, i8)] = &[
    (0, -1),
    (0, 1),
    (-1, 0),
    (1, 0),
    (-1, -1),
    (1, -1),
    (-1, 1),
    (1, 1),
];

/// A `width` × `height` grid of `T`, stored row-major.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct GridWorldManager<T> {
    width: usize,
    height: usize,
    cells: Vec<T>,
}

impl<T: Clone> GridWorldManager<T> {
    /// Builds a grid with every cell set to `cell`.
    pub fn filled(width: usize, height: usize, cell: T) -> Self {
        Self {
            width,
            height,
            cells: vec![cell; width * height],
        }
    }
}

impl<T: Default> GridWorldManager<T> {
    /// Builds a grid of `T::default()` — the usual starting point before filling obstacles.
    ///
    /// Goes through [`from_fn`](Self::from_fn) rather than [`filled`](Self::filled) so
    /// the bound stays at `Default`: cell types that are `Default` but not `Clone` — one
    /// holding a `File` or a `JoinHandle`, say — are still constructible.
    pub fn new(width: usize, height: usize) -> Self {
        Self::from_fn(width, height, |_, _| T::default())
    }
}

impl<T> GridWorldManager<T> {
    /// Builds a grid by calling `f(x, y)` for each cell, in row-major order.
    pub fn from_fn(width: usize, height: usize, mut f: impl FnMut(usize, usize) -> T) -> Self {
        let mut cells = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                cells.push(f(x, y));
            }
        }
        Self {
            width,
            height,
            cells,
        }
    }

    /// Wraps existing row-major cells.
    ///
    /// Returns `None` unless `cells.len() == width * height`, so a mismatch surfaces
    /// here rather than as scrambled rows later.
    pub fn from_vec(width: usize, height: usize, cells: Vec<T>) -> Option<Self> {
        (cells.len() == width * height).then_some(Self {
            width,
            height,
            cells,
        })
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Total cell count, and the length any per-node array should have.
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    pub fn contains(&self, x: usize, y: usize) -> bool {
        x < self.width && y < self.height
    }

    /// The id of cell `(x, y)`.
    ///
    /// # Panics
    /// If the coordinates are outside the grid. Use [`try_id`](Self::try_id) for
    /// coordinates that may be out of range — notably anything derived by offsetting,
    /// where the result can also go negative.
    #[inline]
    pub fn id(&self, x: usize, y: usize) -> NodeId {
        assert!(
            self.contains(x, y),
            "({x}, {y}) is outside a {}x{} grid",
            self.width,
            self.height
        );
        NodeId(y * self.width + x)
    }

    /// The id of cell `(x, y)`, or `None` if it falls outside the grid.
    ///
    /// Takes signed coordinates so that negative results from offset arithmetic are
    /// rejected rather than wrapping — the reason neighbor lookups can't just use
    /// `usize`.
    #[inline]
    pub fn try_id(&self, x: isize, y: isize) -> Option<NodeId> {
        (x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height)
            .then(|| NodeId(y as usize * self.width + x as usize))
    }

    /// The coordinates of `id`.
    ///
    /// # Panics
    /// If `id` is not from a grid of this shape.
    #[inline]
    pub fn xy(&self, NodeId(i): NodeId) -> (usize, usize) {
        assert!(
            i < self.cells.len(),
            "node {i} is outside a grid of {} cells",
            self.cells.len()
        );
        (i % self.width, i / self.width)
    }

    pub fn get(&self, id: NodeId) -> Option<&T> {
        self.cells.get(id.0)
    }

    pub fn get_mut(&mut self, id: NodeId) -> Option<&mut T> {
        self.cells.get_mut(id.0)
    }

    /// The backing storage, row-major.
    pub fn cells(&self) -> &[T] {
        &self.cells
    }

    pub fn cells_mut(&mut self) -> &mut [T] {
        &mut self.cells
    }

    /// Every cell paired with its id, in row-major order.
    pub fn iter(&self) -> impl Iterator<Item = (NodeId, &T)> {
        self.cells
            .iter()
            .enumerate()
            .map(|(i, cell)| (NodeId(i), cell))
    }

    /// The in-bounds orthogonal (4-connected) neighbors of `id`.
    /// The returned iterator borrows nothing, so the grid stays mutable while it is
    /// held — flood fills and rasterization both want that.
    pub fn neighbors4(&self, id: NodeId) -> impl Iterator<Item = NodeId> + use<T> {
        self.neighbors(id, ORTHOGONAL)
    }

    /// The in-bounds 8-connected neighbors of `id`, orthogonal directions first.
    ///
    /// Diagonal steps cost √2 rather than 1; that's the caller's to apply, as is any
    /// rule about cutting corners between two blocked orthogonal cells.
    pub fn neighbors8(&self, id: NodeId) -> impl Iterator<Item = NodeId> + use<T> {
        self.neighbors(id, ORTHOGONAL_AND_DIAGONAL)
    }

    fn neighbors(
        &self,
        id: NodeId,
        offsets: &'static [(i8, i8)],
    ) -> impl Iterator<Item = NodeId> + use<T> {
        // Copied out so the iterator captures no borrow of `self`.
        let (x, y) = self.xy(id);
        let (width, height) = (self.width, self.height);

        offsets.iter().filter_map(move |&(dx, dy)| {
            let nx = x as isize + dx as isize;
            let ny = y as isize + dy as isize;
            (nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height)
                .then(|| NodeId(ny as usize * width + nx as usize))
        })
    }

    /// Marks every cell covered by each polygon, boundary included.
    ///
    /// `mark` decides what "covered" means for the cell type — `|c| c.blocked = true`
    /// for a struct cell, `|c| *c = 1` for a plain integer. It mutates in place rather
    /// than replacing, so unrelated fields (terrain cost, elevation) survive.
    pub fn rasterize_polygons<M: Fn(&mut T)>(&mut self, polygons: &[Vec<[i32; 2]>], mark: M) {
        for polygon in polygons {
            self.rasterize_polygon(polygon, &mark);
        }
    }

    /// Marks every cell covered by one polygon, boundary included.
    pub fn rasterize_polygon<M: Fn(&mut T)>(&mut self, vertices: &[[i32; 2]], mark: M) {
        self.fill_polygon_interior(vertices, &mark);
        for i in 0..vertices.len() {
            let start = vertices[i];
            let end = vertices[(i + 1) % vertices.len()];
            self.draw_bresenham_edge(start, end, &mark);
        }
    }

    /// Scanline fill of the polygon interior.
    ///
    /// Boundary cells are the edge pass's job — `x_intersect` truncates toward zero
    /// rather than flooring, so spans can start a cell late on left-leaning edges.
    fn fill_polygon_interior<M: Fn(&mut T)>(&mut self, vertices: &[[i32; 2]], mark: &M) {
        // Bounding box limits the scanlines; clamping to the grid keeps a polygon
        // sitting far off-grid from iterating millions of empty rows.
        let (min_y, max_y) = vertices
            .iter()
            .fold((i32::MAX, i32::MIN), |(lo, hi), &[_, y]| {
                (lo.min(y), hi.max(y))
            });
        let min_y = min_y.max(0);
        let max_y = max_y.min(self.height as i32 - 1);

        for y in min_y..=max_y {
            let mut intersections = Vec::new();
            for i in 0..vertices.len() {
                let (x1, y1) = (vertices[i][0], vertices[i][1]);
                let (x2, y2) = (
                    vertices[(i + 1) % vertices.len()][0],
                    vertices[(i + 1) % vertices.len()][1],
                );

                if (y1 <= y && y < y2) || (y2 <= y && y < y1) {
                    let x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
                    intersections.push(x_intersect);
                }
            }

            intersections.sort_unstable();

            for pair in intersections.chunks(2) {
                if let [start_x, end_x] = pair {
                    for x in *start_x..=*end_x {
                        if let Some(id) = self.try_id(x as isize, y as isize) {
                            mark(&mut self[id]);
                        }
                    }
                }
            }
        }
    }

    fn draw_bresenham_edge<M: Fn(&mut T)>(&mut self, start: [i32; 2], end: [i32; 2], mark: &M) {
        let (x0, y0) = (start[0], start[1]);
        let (x1, y1) = (end[0], end[1]);
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;
        let mut x = x0;
        let mut y = y0;

        loop {
            if let Some(id) = self.try_id(x as isize, y as isize) {
                mark(&mut self[id]);
            }
            if x == x1 && y == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x += sx;
            }
            if e2 < dx {
                err += dx;
                y += sy;
            }
        }
    }
}

impl<T> Index<NodeId> for GridWorldManager<T> {
    type Output = T;

    #[inline]
    fn index(&self, id: NodeId) -> &T {
        &self.cells[id.0]
    }
}

impl<T> IndexMut<NodeId> for GridWorldManager<T> {
    #[inline]
    fn index_mut(&mut self, id: NodeId) -> &mut T {
        &mut self.cells[id.0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ids(mut v: Vec<NodeId>) -> Vec<usize> {
        v.sort();
        v.into_iter().map(|NodeId(i)| i).collect()
    }

    #[test]
    fn id_and_xy_round_trip() {
        let grid = GridWorldManager::filled(5, 3, ());
        for y in 0..3 {
            for x in 0..5 {
                assert_eq!(grid.xy(grid.id(x, y)), (x, y));
            }
        }
    }

    #[test]
    fn ids_are_row_major() {
        let grid = GridWorldManager::filled(4, 3, ());
        assert_eq!(grid.id(0, 0), NodeId(0));
        assert_eq!(grid.id(3, 0), NodeId(3));
        assert_eq!(grid.id(0, 1), NodeId(4));
        assert_eq!(grid.id(3, 2), NodeId(11));
        assert_eq!(grid.len(), 12);
    }

    #[test]
    fn from_fn_fills_in_row_major_order() {
        let grid = GridWorldManager::from_fn(3, 2, |x, y| (x, y));
        assert_eq!(
            grid.cells(),
            &[(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)]
        );
    }

    #[test]
    fn from_vec_rejects_a_length_mismatch() {
        assert!(GridWorldManager::from_vec(2, 2, vec![1, 2, 3, 4]).is_some());
        assert!(GridWorldManager::from_vec(2, 2, vec![1, 2, 3]).is_none());
        assert!(GridWorldManager::from_vec(2, 2, vec![1, 2, 3, 4, 5]).is_none());
    }

    #[test]
    fn try_id_rejects_out_of_bounds_and_negatives() {
        let grid = GridWorldManager::filled(4, 3, ());
        assert_eq!(grid.try_id(3, 2), Some(NodeId(11)));
        assert_eq!(
            grid.try_id(4, 0),
            None,
            "x == width is one past the last column"
        );
        assert_eq!(
            grid.try_id(0, 3),
            None,
            "y == height is one past the last row"
        );
        // Signed input is the point: as usize these would wrap to a huge in-range-looking value.
        assert_eq!(grid.try_id(-1, 0), None);
        assert_eq!(grid.try_id(0, -1), None);
    }

    #[test]
    #[should_panic]
    fn id_panics_out_of_bounds() {
        GridWorldManager::filled(2, 2, ()).id(2, 0);
    }

    #[test]
    fn indexing_reads_and_writes_cells() {
        let mut grid = GridWorldManager::filled(3, 3, 0);
        let id = grid.id(1, 2);
        grid[id] = 42;
        assert_eq!(grid[id], 42);
        assert_eq!(grid.get(id), Some(&42));
        assert_eq!(grid.cells().iter().filter(|&&c| c == 42).count(), 1);
    }

    #[test]
    fn neighbors4_is_clipped_at_the_edges() {
        let grid = GridWorldManager::filled(3, 3, ());
        // Interior cell has all four.
        assert_eq!(
            ids(grid.neighbors4(grid.id(1, 1)).collect()),
            vec![1, 3, 5, 7]
        );
        // Corner has two.
        assert_eq!(ids(grid.neighbors4(grid.id(0, 0)).collect()), vec![1, 3]);
        // Edge has three.
        assert_eq!(ids(grid.neighbors4(grid.id(1, 0)).collect()), vec![0, 2, 4]);
    }

    #[test]
    fn neighbors8_includes_diagonals() {
        let grid = GridWorldManager::filled(3, 3, ());
        assert_eq!(
            ids(grid.neighbors8(grid.id(1, 1)).collect()),
            vec![0, 1, 2, 3, 5, 6, 7, 8]
        );
        assert_eq!(ids(grid.neighbors8(grid.id(0, 0)).collect()), vec![1, 3, 4]);
    }

    #[test]
    fn neighbors_are_orthogonal_first() {
        let grid = GridWorldManager::filled(3, 3, ());
        let got: Vec<_> = grid.neighbors8(grid.id(1, 1)).map(|NodeId(i)| i).collect();
        assert_eq!(&got[..4], &[1, 7, 3, 5], "orthogonal neighbors come first");
    }

    #[test]
    fn neighbors_never_wrap_across_a_row() {
        // The classic flat-index bug: x - 1 at column 0 landing on the previous row.
        let grid = GridWorldManager::filled(4, 3, ());
        let left_edge = grid.id(0, 1);
        for n in grid.neighbors8(left_edge) {
            let (x, _) = grid.xy(n);
            assert!(x <= 1, "neighbor of a left-edge cell wrapped to x = {x}");
        }
    }

    #[test]
    fn single_row_and_single_column_grids_behave() {
        let row = GridWorldManager::filled(4, 1, ());
        assert_eq!(ids(row.neighbors4(row.id(0, 0)).collect()), vec![1]);
        assert_eq!(ids(row.neighbors4(row.id(1, 0)).collect()), vec![0, 2]);

        let column = GridWorldManager::filled(1, 4, ());
        assert_eq!(ids(column.neighbors4(column.id(0, 0)).collect()), vec![1]);
        assert_eq!(
            ids(column.neighbors4(column.id(0, 1)).collect()),
            vec![0, 2]
        );
    }

    #[test]
    fn neighbor_iterator_does_not_borrow_the_grid() {
        // The reason `neighbors*` uses precise capturing: a flood fill needs to write
        // cells while walking the neighbors it just produced.
        let mut grid = GridWorldManager::filled(3, 3, 0u8);
        let start = grid.id(1, 1);
        for n in grid.neighbors4(start) {
            grid[n] = 1;
        }
        assert_eq!(grid.cells(), &[0, 1, 0, 1, 0, 1, 0, 1, 0]);
    }

    #[test]
    fn works_with_struct_cells() {
        #[derive(Clone, Default, PartialEq, Eq, Debug)]
        struct Cell {
            blocked: bool,
            cost: u16,
        }

        let mut grid: GridWorldManager<Cell> = GridWorldManager::new(3, 2);
        let id = grid.id(2, 1);
        grid[id] = Cell {
            blocked: true,
            cost: 5,
        };
        assert_eq!(grid[id].cost, 5);
        assert_eq!(grid.iter().filter(|(_, c)| c.blocked).count(), 1);
    }

    #[test]
    fn new_accepts_cells_that_are_default_but_not_clone() {
        // `Option<File>` is `Default` but not `Clone`, so this type would fail to build
        // via `filled` — which is exactly why `new` routes through `from_fn` instead.
        #[derive(Default, Debug)]
        struct Cell {
            blocked: bool,
            log: Option<std::fs::File>,
        }

        let grid: GridWorldManager<Cell> = GridWorldManager::new(3, 2);
        assert_eq!(grid.len(), 6);
        assert!(grid.cells().iter().all(|c| !c.blocked && c.log.is_none()));
    }

    #[test]
    fn per_node_arrays_line_up_with_len() {
        // The payoff: search bookkeeping indexed by NodeId, no hashing.
        let grid = GridWorldManager::filled(4, 4, ());
        let mut g_score = vec![f32::INFINITY; grid.len()];
        let start = grid.id(0, 0);
        g_score[start.0] = 0.0;
        assert_eq!(g_score[0], 0.0);
        assert_eq!(g_score.len(), 16);
    }

    // --- rasterization ---------------------------------------------------

    /// Renders occupancy as ASCII so the expected shape is legible in the assertion.
    fn render(grid: &GridWorldManager<i32>) -> Vec<String> {
        (0..grid.height())
            .map(|y| {
                (0..grid.width())
                    .map(|x| if grid[grid.id(x, y)] != 0 { '#' } else { '.' })
                    .collect()
            })
            .collect()
    }

    fn rasterized(width: usize, height: usize, vertices: &[[i32; 2]]) -> GridWorldManager<i32> {
        let mut grid = GridWorldManager::filled(width, height, 0);
        grid.rasterize_polygon(vertices, |cell| *cell = 1);
        grid
    }

    /// Ground truth for "is this cell inside the polygon": even-odd ray cast.
    fn inside(vertices: &[[i32; 2]], px: i32, py: i32) -> bool {
        let mut hit = false;
        let n = vertices.len();
        for i in 0..n {
            let (xi, yi) = (vertices[i][0] as f64, vertices[i][1] as f64);
            let (xj, yj) = (
                vertices[(i + 1) % n][0] as f64,
                vertices[(i + 1) % n][1] as f64,
            );
            let (px, py) = (px as f64, py as f64);
            if (yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi {
                hit = !hit;
            }
        }
        hit
    }

    #[test]
    fn rasterizes_an_axis_aligned_square() {
        let grid = rasterized(8, 6, &[[1, 1], [4, 1], [4, 4], [1, 4]]);
        assert_eq!(
            render(&grid),
            [
                "........", ".####...", ".####...", ".####...", ".####...", "........"
            ]
        );
    }

    #[test]
    fn rasterizes_a_right_triangle() {
        let grid = rasterized(8, 7, &[[0, 0], [5, 0], [0, 5]]);
        assert_eq!(
            render(&grid),
            [
                "######..", "#####...", "####....", "###.....", "##......", "#.......", "........"
            ]
        );
    }

    #[test]
    fn rasterizes_a_concave_polygon() {
        // An L. The full row at y = 2 is correct: the (6,2)-(2,2) segment is boundary,
        // and boundary cells are part of the obstacle.
        let grid = rasterized(8, 8, &[[0, 0], [6, 0], [6, 2], [2, 2], [2, 6], [0, 6]]);
        assert_eq!(
            render(&grid),
            [
                "#######.", "#######.", "#######.", "###.....", "###.....", "###.....", "###.....",
                "........",
            ]
        );
    }

    #[test]
    fn rasterizes_a_diamond_symmetrically() {
        let grid = rasterized(7, 7, &[[3, 0], [6, 3], [3, 6], [0, 3]]);
        let rows = render(&grid);
        assert_eq!(
            rows,
            [
                "...#...", "..###..", ".#####.", "#######", ".#####.", "..###..", "...#..."
            ]
        );
        // Mirror symmetry is a cheap check that the scanline spans aren't skewed.
        for row in &rows {
            let mirrored: String = row.chars().rev().collect();
            assert_eq!(*row, mirrored, "row is not left-right symmetric: {row}");
        }
    }

    #[test]
    fn rasterizing_clips_shapes_that_leave_the_grid() {
        // Negative and past-the-edge vertices must clip, not panic or wrap.
        let grid = rasterized(8, 8, &[[-4, -4], [4, 1], [3, 12]]);
        assert!(grid.cells().iter().any(|&c| c != 0), "nothing was drawn");

        // A cell filled on one row must never be an artefact of wrapping from the row
        // above, so no row may be entirely filled by a shape this narrow.
        for row in render(&grid) {
            assert!(row.contains('.'), "row {row} looks wrapped");
        }
    }

    #[test]
    fn rasterizing_a_polygon_entirely_off_grid_draws_nothing() {
        let grid = rasterized(6, 6, &[[20, 20], [25, 20], [25, 25]]);
        assert!(grid.cells().iter().all(|&c| c == 0));
    }

    #[test]
    fn degenerate_polygons_do_not_panic() {
        // Two vertices: no interior, just the segment drawn twice.
        let line = rasterized(8, 8, &[[1, 1], [5, 5]]);
        assert_eq!(line.cells().iter().filter(|&&c| c != 0).count(), 5);

        // A single vertex, and none at all — both must be no-ops rather than
        // `% 0` panics in the edge-wrapping arithmetic.
        assert_eq!(
            rasterized(4, 4, &[[2, 2]])
                .cells()
                .iter()
                .filter(|&&c| c != 0)
                .count(),
            1
        );
        assert!(rasterized(4, 4, &[]).cells().iter().all(|&c| c == 0));
    }

    #[test]
    fn rasterizing_marks_struct_cells_without_clobbering_them() {
        #[derive(Clone, Default, Debug, PartialEq, Eq)]
        struct Cell {
            blocked: bool,
            terrain_cost: u16,
        }

        // Terrain that already exists before obstacles are stamped in.
        let mut grid = GridWorldManager::from_fn(6, 6, |x, y| Cell {
            blocked: false,
            terrain_cost: (x + y) as u16,
        });

        grid.rasterize_polygon(&[[1, 1], [3, 1], [3, 3]], |cell| cell.blocked = true);

        assert!(
            grid.iter().filter(|(_, c)| c.blocked).count() > 0,
            "nothing was marked"
        );

        // Marking in place must leave every other field intact — assigning a whole
        // cell would have wiped the terrain costs.
        for (id, cell) in grid.iter() {
            let (x, y) = grid.xy(id);
            assert_eq!(
                cell.terrain_cost,
                (x + y) as u16,
                "rasterizing overwrote terrain at ({x}, {y})",
            );
        }
    }

    #[test]
    fn rasterizes_several_polygons_in_one_call() {
        let mut grid = GridWorldManager::filled(8, 8, 0);
        let polygons = vec![
            vec![[0, 0], [2, 0], [2, 2], [0, 2]],
            vec![[5, 5], [7, 5], [7, 7], [5, 7]],
        ];
        grid.rasterize_polygons(&polygons, |cell| *cell = 1);

        assert_eq!(grid[grid.id(1, 1)], 1);
        assert_eq!(grid[grid.id(6, 6)], 1);
        assert_eq!(grid[grid.id(4, 4)], 0, "the gap between them stays clear");
        assert_eq!(grid.cells().iter().filter(|&&c| c != 0).count(), 18);
    }

    #[test]
    fn rasterized_polygons_have_no_interior_holes() {
        // The invariant that matters for planning: over-filling a boundary cell is
        // safe, but a hole lets a path slip through an obstacle.
        //
        // This pins the scanline parity rule specifically — the shape tests above
        // cover the boundary drawn by the edge pass, which this does not depend on.
        const N: usize = 12;

        // xorshift keeps the sweep deterministic without pulling in a rand dependency.
        let mut seed: u64 = 0x2026_0803;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };

        for trial in 0..500 {
            let count = 3 + (next() % 4) as usize;
            let vertices: Vec<[i32; 2]> = (0..count)
                .map(|_| [(next() % N as u64) as i32, (next() % N as u64) as i32])
                .collect();

            let grid = rasterized(N, N, &vertices);

            for y in 0..N as i32 {
                for x in 0..N as i32 {
                    if inside(&vertices, x, y) {
                        assert!(
                            grid[grid.id(x as usize, y as usize)] != 0,
                            "trial {trial}: ({x}, {y}) is inside {vertices:?} but was left empty",
                        );
                    }
                }
            }
        }
    }
}
