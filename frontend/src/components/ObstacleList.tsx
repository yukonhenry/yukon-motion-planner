import {obstacleHue} from '../geometry';
import type {Obstacle} from '../types';

interface Props {
    obstacles: Obstacle[];
    selectedId: number | null;
    onSelect: (id: number | null) => void;
    onDelete: (id: number) => void;
    /** Marks a shape as one a simulation may move or reshape. */
    onSetDynamic: (id: number, dynamic: boolean) => void;
    /** Frozen grids are read-only until an edit forks a version, this flag included. */
    readOnly: boolean;
}

export function ObstacleList({
                                 obstacles,
                                 selectedId,
                                 onSelect,
                                 onDelete,
                                 onSetDynamic,
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
                            className={obstacle.id === selectedId ? 'selected' : ''}
                            onClick={() => onSelect(obstacle.id)}
                        >
              <span
                  className="swatch"
                  style={{background: `hsl(${obstacleHue(obstacle.id)} 70% 55%)`}}
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
        </section>
    );
}