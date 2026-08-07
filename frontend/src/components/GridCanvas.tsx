import { useMemo, useRef, useState } from 'react';
import {
  CELL_SIZE,
  cellToPixel,
  clampCell,
  gridPixelHeight,
  gridPixelWidth,
  isClosedRing,
  obstacleHue,
  pixelToCell,
  rasterizePolygon,
  replaceVertex,
  sameCell,
  translatePolygon,
  verticesToPixels,
} from '../geometry';
import type { Endpoint, GridSize, Obstacle, Vertex } from '../types';

/**
 * The only module in the app that knows SVG exists.
 *
 * Everything about *where* shapes go lives in ../geometry; this file is purely about
 * drawing them and turning pointer events into cell coordinates. Swapping in a canvas
 * library later means rewriting this file and nothing else.
 */

interface Props {
  /** Dimensions only: a grid being composed has these before it has an id. */
  grid: GridSize;
  obstacles: Obstacle[];
  selectedId: number | null;
  onSelect: (id: number | null) => void;
  /** Vertices of the in-progress polygon, or `null` when not drawing. */
  draft: Vertex[] | null;
  onDraftAppend: (cell: Vertex) => void;
  onUpdate: (obstacleId: number, vertices: Vertex[]) => void;
  /** Which route endpoint the next click places, or `null` when not picking. */
  picking: Endpoint | null;
  onPickCell: (cell: Vertex) => void;
  src: Vertex | null;
  dest: Vertex | null;
  /**
   * Cells of the route being shown, start first; `null` when none is.
   *
   * It can never disagree with the obstacles drawn around it: a grid with a route is
   * frozen against edits, and a grid with unsaved edits refuses to plan.
   */
  route: Vertex[] | null;
  /** Shade the cells the planner actually blocks, not just the drawn outline. */
  showFootprint: boolean;
}

/** An in-flight drag. Held in state, but it only changes when the *cell* changes. */
type Drag =
  | { kind: 'vertex'; obstacleId: number; index: number; vertices: Vertex[] }
  | { kind: 'move'; obstacleId: number; origin: Vertex; from: Vertex[]; vertices: Vertex[] };

/**
 * A run of cells as one <path> of square subpaths, for the same reason gridPath is one
 * element: a footprint can be hundreds of cells, and that many <rect>s is that many DOM
 * nodes to lay out on every drag step.
 */
const cellsToPath = (cells: Vertex[]) =>
  cells
    .map(
      ([x, y]) =>
        `M${x * CELL_SIZE} ${y * CELL_SIZE}h${CELL_SIZE}v${CELL_SIZE}h${-CELL_SIZE}z`,
    )
    .join('');

const toPoints = (vertices: Vertex[]) =>
  verticesToPixels(vertices)
    .map(([x, y]) => `${x},${y}`)
    .join(' ');

export function GridCanvas({
  grid,
  obstacles,
  selectedId,
  onSelect,
  draft,
  onDraftAppend,
  onUpdate,
  picking,
  onPickCell,
  src,
  dest,
  route,
  showFootprint,
}: Props) {
  const svgRef = useRef<SVGSVGElement>(null);
  const [drag, setDrag] = useState<Drag | null>(null);
  const [hover, setHover] = useState<Vertex | null>(null);

  const width = gridPixelWidth(grid);
  const height = gridPixelHeight(grid);
  const drawing = draft !== null;
  /** Drawing and picking differ in what a click *means*, but both address a cell. */
  const placing = drawing || picking !== null;

  // One <path> for the whole lattice: a 200x200 grid is 400 line segments, and as
  // separate elements that is 400 DOM nodes to lay out on every render.
  const gridPath = useMemo(() => {
    const segments: string[] = [];
    for (let x = 0; x <= grid.width; x++) {
      segments.push(`M${x * CELL_SIZE} 0V${height}`);
    }
    for (let y = 0; y <= grid.height; y++) {
      segments.push(`M0 ${y * CELL_SIZE}H${width}`);
    }
    return segments.join('');
  }, [grid.width, grid.height, width, height]);

  const eventToCell = (e: React.PointerEvent | React.MouseEvent): Vertex => {
    const rect = svgRef.current!.getBoundingClientRect();
    return clampCell(
      [pixelToCell(e.clientX - rect.left), pixelToCell(e.clientY - rect.top)],
      grid,
    );
  };

  /** What to draw for an obstacle: the drag preview if it's being dragged, else its stored shape. */
  const shapeOf = (obstacle: Obstacle): Vertex[] =>
    drag?.obstacleId === obstacle.id ? drag.vertices : obstacle.vertices;

  /** The distinct corners of a shape — indices still address the full array. */
  const cornersOf = (vertices: Vertex[]): Vertex[] =>
    isClosedRing(vertices) ? vertices.slice(0, -1) : vertices;

  // Recomputed mid-drag on purpose: watching the footprint grow and shrink as a vertex
  // moves is how you learn which cells a shape really costs. `drag` is a dependency
  // because shapeOf reads it.
  const footprints = useMemo(
    () =>
      showFootprint
        ? obstacles.map((obstacle) => ({
            id: obstacle.id,
            d: cellsToPath(rasterizePolygon(shapeOf(obstacle), grid)),
          }))
        : [],
    [showFootprint, obstacles, grid, drag],
  );

  const startVertexDrag =
    (obstacle: Obstacle, index: number) => (e: React.PointerEvent) => {
      e.stopPropagation();
      e.currentTarget.setPointerCapture(e.pointerId);
      setDrag({ kind: 'vertex', obstacleId: obstacle.id, index, vertices: obstacle.vertices });
    };

  const startMoveDrag = (obstacle: Obstacle) => (e: React.PointerEvent) => {
    e.stopPropagation();
    onSelect(obstacle.id);
    e.currentTarget.setPointerCapture(e.pointerId);
    setDrag({
      kind: 'move',
      obstacleId: obstacle.id,
      origin: eventToCell(e),
      from: obstacle.vertices,
      vertices: obstacle.vertices,
    });
  };

  const onPointerMove = (e: React.PointerEvent) => {
    const cell = eventToCell(e);
    setHover(cell);

    if (!drag) return;

    if (drag.kind === 'vertex') {
      const next = replaceVertex(drag.vertices, drag.index, cell, grid);
      // Re-render only when the snapped cell actually changed.
      if (!sameCell(next[drag.index], drag.vertices[drag.index])) {
        setDrag({ ...drag, vertices: next });
      }
    } else {
      const next = translatePolygon(
        drag.from,
        cell[0] - drag.origin[0],
        cell[1] - drag.origin[1],
        grid,
      );
      if (!sameCell(next[0], drag.vertices[0])) {
        setDrag({ ...drag, vertices: next });
      }
    }
  };

  const endDrag = () => {
    if (!drag) return;
    const original = obstacles.find((o) => o.id === drag.obstacleId)?.vertices;
    const moved =
      original && original.some((v, i) => !sameCell(v, drag.vertices[i]));
    if (moved) {
      onUpdate(drag.obstacleId, drag.vertices);
    }
    setDrag(null);
  };

  /**
   * Keeps a click that landed on a shape from reaching the background.
   *
   * Pointer events and click are separate streams: the `stopPropagation` in the
   * pointerdown handlers stops neither the click that follows nor, without this, the
   * `onSelect(null)` below — so selecting an obstacle by clicking it would be undone by
   * the same gesture, and a finished vertex drag would clear the handles it was using.
   */
  const keepSelection = (e: React.MouseEvent) => e.stopPropagation();

  const onBackgroundClick = (e: React.MouseEvent) => {
    if (picking) {
      // An endpoint may legitimately be placed on top of an obstacle — the server is
      // the authority on whether that cell is blocked, and its refusal is the message
      // worth showing. So this deliberately does not filter by what is underneath.
      onPickCell(eventToCell(e));
    } else if (drawing) {
      onDraftAppend(eventToCell(e));
    } else {
      onSelect(null);
    }
  };

  return (
    <svg
      ref={svgRef}
      className={`canvas ${placing ? 'canvas--placing' : ''}`}
      width={width}
      height={height}
      onClick={onBackgroundClick}
      onPointerMove={onPointerMove}
      onPointerUp={endDrag}
      onPointerCancel={endDrag}
      onPointerLeave={() => setHover(null)}
    >
      <rect width={width} height={height} className="canvas__bg" />
      <path d={gridPath} className="canvas__grid" />

      {/* Under the outlines, never taking a click: this is the shape the planner routes
          around, and seeing it sit proud of the polygon on top is the whole point —
          vertices are cell centers, so rasterization claims cells the stroke only
          clips. */}
      {footprints.length > 0 && (
        <g className="footprint" style={{ pointerEvents: 'none' }}>
          {footprints.map(({ id, d }) => (
            <path key={id} d={d} fill={`hsl(${obstacleHue(id)} 70% 55% / 0.55)`} />
          ))}
        </g>
      )}

      {/* While placing a cell, shapes must not swallow the click meant for underneath. */}
      <g style={{ pointerEvents: placing ? 'none' : undefined }}>
        {obstacles.map((obstacle) => {
          const vertices = shapeOf(obstacle);
          const selected = obstacle.id === selectedId;
          const hue = obstacleHue(obstacle.id);
          return (
            <g key={obstacle.id}>
              <polygon
                points={toPoints(vertices)}
                className={`obstacle ${selected ? 'obstacle--selected' : ''}`}
                style={{
                  fill: `hsl(${hue} 70% 55% / ${selected ? 0.5 : 0.32})`,
                  stroke: `hsl(${hue} 65% 45%)`,
                }}
                onPointerDown={startMoveDrag(obstacle)}
                onClick={keepSelection}
              />
              {/* One handle per corner: the repeated closing entry sits exactly under
                  the first, so drawing it would stack two circles on one cell. Dragging
                  either index moves both anyway — see replaceVertex. */}
              {selected &&
                cornersOf(vertices).map(([x, y], index) => (
                  <circle
                    key={index}
                    cx={cellToPixel(x)}
                    cy={cellToPixel(y)}
                    r={6}
                    className="handle"
                    onPointerDown={startVertexDrag(obstacle, index)}
                    onClick={keepSelection}
                  />
                ))}
            </g>
          );
        })}
      </g>

      {/* Above the obstacles: a route that ducks behind one is the case you most want
          to see clearly. Cells are filled as well as joined, because which cells the
          plan occupies is the actual output — the polyline is just legibility. */}
      {route && route.length > 0 && (
        <g className="route">
          {route.map(([x, y], index) => (
            <rect
              key={index}
              x={x * CELL_SIZE}
              y={y * CELL_SIZE}
              width={CELL_SIZE}
              height={CELL_SIZE}
              className="route__cell"
            />
          ))}
          <polyline points={toPoints(route)} className="route__line" />
        </g>
      )}

      {([['src', src], ['dest', dest]] as const).map(([kind, cell]) =>
        cell ? (
          <g key={kind} className={`endpoint-marker endpoint-marker--${kind}`}>
            <circle cx={cellToPixel(cell[0])} cy={cellToPixel(cell[1])} r={9} />
            <text x={cellToPixel(cell[0])} y={cellToPixel(cell[1])} dy="0.35em">
              {kind === 'src' ? 'S' : 'G'}
            </text>
          </g>
        ) : null,
      )}

      {draft && draft.length > 0 && (
        <g className="draft">
          <polyline
            points={
              // Rubber-band the last placed vertex to the cursor.
              toPoints(hover ? [...draft, hover] : draft)
            }
            className="draft__line"
          />
          {draft.map(([x, y], index) => (
            <circle key={index} cx={cellToPixel(x)} cy={cellToPixel(y)} r={5} className="draft__dot" />
          ))}
        </g>
      )}

      {placing && hover && (
        <rect
          x={hover[0] * CELL_SIZE}
          y={hover[1] * CELL_SIZE}
          width={CELL_SIZE}
          height={CELL_SIZE}
          className="hover-cell"
        />
      )}
    </svg>
  );
}
