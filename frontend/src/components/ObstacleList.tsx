import { obstacleHue } from "../geometry";
import type { Obstacle, Vertex } from "../types";

interface Props {
  obstacles: Obstacle[];
  selectedId: number | null;
  onSelect: (id: number | null) => void;
  onDelete: (id: number) => void;
  /** Marks a shape as one a simulation may move or reshape. */
  onSetDynamic: (id: number, dynamic: boolean) => void;
  /** Sets how many cells a shape travels per tick, as `[dx, dy]`. */
  onSetVelocity: (id: number, velocity: Vertex) => void;
  /** Frozen grids are read-only until an edit forks a version, this flag included. */
  readOnly: boolean;
}

export function ObstacleList({
  obstacles,
  selectedId,
  onSelect,
  onDelete,
  onSetDynamic,
  onSetVelocity,
  readOnly,
}: Props) {
  return (
    <section className="panel">
      <h2>Obstacles ({obstacles.length})</h2>

      {obstacles.length === 0 ? (
        <p className="muted">None yet — use “Draw obstacle” and click cells.</p>
      ) : (
        <ul className="obstacle-list">
          {obstacles.map((obstacle, index) => (
            <li
              key={obstacle.id}
              className={obstacle.id === selectedId ? "selected" : ""}
              onClick={() => onSelect(obstacle.id)}
            >
              <span
                className="swatch"
                style={{
                  background: `hsl(${obstacleHue(obstacle.id)} 70% 55%)`,
                }}
              />
              {/* Numbered by position, matching how the API names a bad
                                one ("obstacle 1 has vertex …"). The id beside it is
                                stored, but position is what the error messages use. */}
              <span className="obstacle-list__id">#{index}</span>
              <span className="muted">{obstacle.vertices.length - 1} vertices</span>
              {/* The click here must not also select the row, or the
                                label's own click would toggle twice. */}
              <label
                className="obstacle-list__dynamic"
                title="A simulation may move or reshape this obstacle"
                onClick={(e) => e.stopPropagation()}
              >
                <input
                  type="checkbox"
                  checked={obstacle.dynamic}
                  disabled={readOnly}
                  onChange={(e) => onSetDynamic(obstacle.id, e.target.checked)}
                />
                dynamic
              </label>
              {/* Only meaningful once the shape may move, and hidden otherwise so a static
                  obstacle's row stays as short as it was. On its own line rather than beside
                  the name: two number boxes on a sidebar row leave nothing legible.

                  `[0, 0]` is not "no motion" but "the original behavior" — a dynamic obstacle
                  with no velocity jitters one corner at random rather than travelling — which
                  the hint says out loud, because an empty pair of boxes otherwise reads as
                  "stationary". */}
              {obstacle.dynamic && (
                <div
                  className="obstacle-list__velocity"
                  title="Cells travelled per tick. 0, 0 jitters a random corner instead."
                  onClick={(e) => e.stopPropagation()}
                >
                  <label>
                    <span>x</span>
                    <input
                      type="number"
                      value={obstacle.velocity[0]}
                      disabled={readOnly}
                      onChange={(e) =>
                        onSetVelocity(obstacle.id, [
                          Math.trunc(Number(e.target.value) || 0),
                          obstacle.velocity[1],
                        ])
                      }
                    />
                  </label>
                  <label>
                    <span>y</span>
                    <input
                      type="number"
                      value={obstacle.velocity[1]}
                      disabled={readOnly}
                      onChange={(e) =>
                        onSetVelocity(obstacle.id, [
                          obstacle.velocity[0],
                          Math.trunc(Number(e.target.value) || 0),
                        ])
                      }
                    />
                  </label>
                  <span className="obstacle-list__velocity-hint">cells/tick</span>
                </div>
              )}
              <button
                type="button"
                className="danger subtle"
                onClick={(e) => {
                  e.stopPropagation();
                  onDelete(obstacle.id);
                }}
              >
                Delete
              </button>
            </li>
          ))}
        </ul>
      )}

      {/* Said once for the panel rather than on every row: a per-row note would change each
          row's width with the value it describes, and identical controls that sit in
          different places are harder to read than one line of explanation. */}
      {obstacles.some((o) => o.dynamic && o.velocity[0] === 0 && o.velocity[1] === 0) && (
        <p className="muted hint">
          A dynamic obstacle with velocity 0, 0 jitters one random corner each tick instead of
          travelling.
        </p>
      )}
    </section>
  );
}
