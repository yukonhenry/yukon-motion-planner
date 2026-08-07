import {obstacleHue} from '../geometry';
import type {Obstacle} from '../types';

interface Props {
    obstacles: Obstacle[];
    selectedId: number | null;
    onSelect: (id: number | null) => void;
    onDelete: (id: number) => void;
}

export function ObstacleList({obstacles, selectedId, onSelect, onDelete}: Props) {
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
                                one ("obstacle 1 has vertex …") — the id beside it is a
                                local handle the server never sees. */}
                            <span className="obstacle-list__id">#{index}</span>
                            <span className="muted">{obstacle.vertices.length - 1} vertices</span>
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