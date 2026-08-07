import type { Endpoint, Plan, Vertex } from '../types';

interface Props {
  src: Vertex | null;
  dest: Vertex | null;
  /** Which endpoint the next canvas click places, or `null` when not picking. */
  picking: Endpoint | null;
  onPick: (endpoint: Endpoint | null) => void;
  onGenerate: () => void;
  onClear: () => void;
  pending: boolean;
  /** Every route stored against this grid, oldest first. */
  plans: Plan[];
  /** The one currently drawn, or `null`. */
  active: Plan | null;
  onShow: (planId: number) => void;
  onDelete: (planId: number) => void;
  /**
   * Unsaved obstacle edits are on the canvas. The server plans against the grid *as
   * stored*, so a route generated now would not match what the user is looking at.
   */
  blocked: boolean;
  /** The grid was never saved, so there is nothing to plan against yet. */
  unsaved: boolean;
  /** Saving would fork a new version rather than rewrite this one. */
  frozen: boolean;
}

const ENDPOINTS: { key: Endpoint; label: string }[] = [
  { key: 'src', label: 'Start' },
  { key: 'dest', label: 'Goal' },
];

const cellLabel = (cell: Vertex | null) => (cell ? `[${cell[0]}, ${cell[1]}]` : 'not set');

export function RoutePanel({
  src,
  dest,
  picking,
  onPick,
  onGenerate,
  onClear,
  pending,
  plans,
  active,
  onShow,
  onDelete,
  blocked,
  unsaved,
  frozen,
}: Props) {
  const cells = { src, dest };
  const ready = src !== null && dest !== null && !blocked && !unsaved;

  return (
    <section className="panel">
      <h2>Route</h2>

      <div className="stack">
        {ENDPOINTS.map(({ key, label }) => (
          <div className="row endpoint" key={key}>
            <span className={`endpoint__dot endpoint__dot--${key}`} />
            <span className="endpoint__name">{label}</span>
            <span className="endpoint__cell muted">{cellLabel(cells[key])}</span>
            <button
              type="button"
              className="subtle"
              // Clicking the active one backs out, so picking is never a mode you are
              // stuck in without reaching for Escape.
              aria-pressed={picking === key}
              onClick={() => onPick(picking === key ? null : key)}
            >
              {picking === key ? 'Picking…' : 'Set'}
            </button>
          </div>
        ))}

        {picking && (
          <p className="muted hint">
            Click a cell to place the {picking === 'src' ? 'start' : 'goal'}.
          </p>
        )}

        {/* Planning is refused rather than warned about, because the route would be
            computed against stored obstacles and drawn over the edited ones — a picture
            that is wrong without looking wrong. */}
        {(blocked || unsaved) && (
          <p className="muted hint">
            {unsaved
              ? 'Create the grid before planning a route across it.'
              : frozen
                ? 'These edits aren’t saved yet. Save them as a new version to plan against them — the routes below belong to the version you started from.'
                : 'Save your changes first — routes are planned against the saved grid.'}
          </p>
        )}

        <div className="row">
          <button type="button" onClick={onGenerate} disabled={!ready || pending}>
            {pending ? 'Planning…' : 'Generate route'}
          </button>
          <button
            type="button"
            className="subtle"
            onClick={onClear}
            disabled={!src && !dest && !active}
          >
            Clear
          </button>
        </div>

        {active &&
          (active.vertices.length > 0 ? (
            <p className="muted">
              {active.vertices.length} cells,{' '}
              <span title="Scaled by 10: an orthogonal step costs 10, a diagonal 14.">
                cost {active.meta.cost}
              </span>{' '}
              (plan #{active.id}).
            </p>
          ) : (
            // The API saves this as a plan rather than refusing it, so the UI has to
            // distinguish "planned, no route" from "not planned yet".
            <p className="muted">No route — the goal is unreachable from the start.</p>
          ))}

        {plans.length > 0 && (
          <>
            <h3 className="panel__subhead">Saved routes ({plans.length})</h3>
            <ul className="plan-list">
              {plans.map((plan) => (
                <li key={plan.id} className={plan.id === active?.id ? 'selected' : ''}>
                  <button type="button" className="subtle plan-list__pick" onClick={() => onShow(plan.id)}>
                    #{plan.id}{' '}
                    <span className="muted">
                      [{plan.meta.src_vertex.join(', ')}] → [{plan.meta.dest_vertex.join(', ')}]
                    </span>
                  </button>
                  <button
                    type="button"
                    className="danger subtle"
                    // Deleting the last one unfreezes the grid, which is the only way
                    // back to editing it in place.
                    onClick={() => onDelete(plan.id)}
                  >
                    Delete
                  </button>
                </li>
              ))}
            </ul>
            <p className="muted hint">
              Deleting every route unfreezes this grid for editing.
            </p>
          </>
        )}
      </div>
    </section>
  );
}