import type { SearchTracePlayer } from "../hooks/useSearchTrace";

/**
 * Controls for replaying one SST search.
 *
 * Deliberately separate from the canvas: this decides *when* in the search we are looking, and
 * GridCanvas decides how that looks. The reduction from events to a drawable tree is in
 * ../searchTrace.ts, where it can be tested without either.
 *
 * The numbers under the scrubber are the point as much as the picture is. Nodes-against-created
 * is the sparsity claim that separates SST from a plain kinodynamic RRT, and the phase split
 * says whether a slow search is checking too many poses or scanning too many nodes — which want
 * opposite fixes.
 */

interface Props {
  player: SearchTracePlayer;
  /** Runs a search with the endpoints currently on the canvas. Disabled when they are unset. */
  onRun: () => void;
  canRun: boolean;
  /** How many events have been applied, and what the tree looks like there. */
  live: number;
  pruned: number;
  iteration: number;
  bestCost: number | null;
}

const SPEEDS = [0.25, 1, 4, 16];

export function SearchTracePanel({
  player,
  onRun,
  canRun,
  live,
  pruned,
  iteration,
  bestCost,
}: Props) {
  const { trace, loading, error, frame, playing, speed } = player;
  const total = trace?.events.length ?? 0;

  return (
    <section className="panel">
      <h2>Search trace</h2>
      <p className="muted">
        Runs one SST search over the obstacles on screen and replays how the tree grew. Nothing
        is stored and no robot moves.
      </p>

      <div className="row">
        <button type="button" onClick={onRun} disabled={!canRun || loading}>
          {loading ? "Searching…" : "Run search"}
        </button>
        {trace && (
          <button type="button" onClick={player.clear} disabled={loading}>
            Clear
          </button>
        )}
      </div>

      {!canRun && <p className="muted">Place a start and a goal first.</p>}
      {error && <div className="status status--error">{error}</div>}

      {trace && (
        <>
          <div className="row">
            <button type="button" onClick={player.toggle} disabled={total === 0}>
              {playing ? "Pause" : frame >= total ? "Replay" : "Play"}
            </button>
            <input
              type="range"
              min={0}
              max={total}
              value={frame}
              onChange={(event) => player.seek(Number(event.target.value))}
              aria-label="Search progress"
              style={{ flex: 1 }}
            />
          </div>

          <div className="row">
            {SPEEDS.map((option) => (
              <button
                key={option}
                type="button"
                onClick={() => player.setSpeed(option)}
                aria-pressed={speed === option}
                className={speed === option ? "is-active" : undefined}
              >
                {option}×
              </button>
            ))}
          </div>

          <div className="row">
            <label>
              <input
                type="checkbox"
                checked={player.showGhosts}
                onChange={(event) => player.setShowGhosts(event.target.checked)}
              />{" "}
              Pruned branches
            </label>
            <label>
              <input
                type="checkbox"
                checked={player.showWitnesses}
                onChange={(event) => player.setShowWitnesses(event.target.checked)}
              />{" "}
              Witnesses
            </label>
          </div>

          <dl className="stats">
            <dt>Iteration</dt>
            <dd>
              {iteration} / {trace.stats.iterations}
            </dd>
            <dt>Tree</dt>
            <dd>
              {live} live, {pruned} culled
            </dd>
            <dt>Best route</dt>
            <dd>{bestCost === null ? "none yet" : `${bestCost.toFixed(2)} s`}</dd>
          </dl>

          {/* The finished search, which does not change as the playhead moves. */}
          <dl className="stats">
            <dt>Final tree</dt>
            <dd>
              {trace.stats.nodes} of {trace.stats.nodes_created} kept
              {trace.stats.nodes_pruned > 0 && ` · ${trace.stats.nodes_pruned} culled`}
            </dd>
            <dt>Witnesses</dt>
            <dd>{trace.stats.witnesses}</dd>
            <dt>Collision checks</dt>
            <dd>{trace.stats.collision_queries.toLocaleString()}</dd>
            <dt>Time</dt>
            <dd>
              {trace.stats.total_ms.toFixed(0)} ms — {phase(trace.stats.selection_ms, trace.stats)}{" "}
              select, {phase(trace.stats.propagation_ms, trace.stats)} propagate
            </dd>
            <dt>Outcome</dt>
            <dd>
              {trace.reachable
                ? `reached in ${trace.solution_cost.toFixed(2)} s`
                : `never arrived — closest ${trace.stats.closest_approach.toFixed(2)} m`}
            </dd>
            <dt>Seed</dt>
            <dd>
              <code>{trace.seed}</code>
            </dd>
          </dl>
        </>
      )}
    </section>
  );
}

/** A phase as a share of the three measured phases, which is the comparison that matters. */
function phase(ms: number, stats: { selection_ms: number; propagation_ms: number; bookkeeping_ms: number }) {
  const total = stats.selection_ms + stats.propagation_ms + stats.bookkeeping_ms;
  return total > 0 ? `${((100 * ms) / total).toFixed(0)}%` : "—";
}
