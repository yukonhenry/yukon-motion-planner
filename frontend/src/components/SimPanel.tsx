interface Props {
    /** How many ticks have run, or 0 before the first. */
    tick: number;
    /** Result of the last tick, or `null` before one has run. */
    last: { reachable: boolean; cost: number; moved: number; planner: string } | null;
    /** How many obstacles on the canvas are flagged dynamic. */
    dynamicCount: number;
    /** Why stepping is unavailable, or `null` when it is available. */
    blocked: string | null;
    pending: boolean;
    onStep: () => void;
    onReset: () => void;
}

/**
 * Drives the moving-obstacle simulation, one tick per click.
 *
 * A run lives entirely here and on the server's desk for the length of one request: each tick
 * sends the obstacles as they stand, the server jitters one vertex of each dynamic shape and
 * replans, and the result comes back. Nothing is written to the database, so a run neither
 * freezes the grid nor leaves a saved route per frame — which also means closing the tab loses
 * it, and Reset simply forgets it.
 *
 * Explicitly a manual clock for now: the frontend is the thing advancing time.
 */
export function SimPanel({
                             tick,
                             last,
                             dynamicCount,
                             blocked,
                             pending,
                             onStep,
                             onReset,
                         }: Props) {
    return (
        <section className="panel">
            <h2>Simulation</h2>

            <div className="stack">
                {dynamicCount === 0 ? (
                    <p className="muted">
                        No obstacle is marked <strong>dynamic</strong> — stepping would change
                        nothing. Tick one in the obstacle list.
                    </p>
                ) : (
                    <p className="muted hint">
                        Each step jitters one corner of {dynamicCount === 1 ? 'the' : 'each of the'}{' '}
                        {dynamicCount} dynamic {dynamicCount === 1 ? 'obstacle' : 'obstacles'} and
                        replans. Nothing is saved.
                    </p>
                )}

                {blocked && <p className="status status--error">{blocked}</p>}

                {tick > 0 && last && (
                    <p className="muted">
                        Tick {tick} · {last.moved} moved ·{' '}
                        {last.reachable ? `cost ${last.cost}` : 'no route'} · {last.planner}
                    </p>
                )}

                <div className="row">
                    <button
                        type="button"
                        onClick={onStep}
                        disabled={pending || blocked !== null || dynamicCount === 0}
                    >
                        {pending ? 'Replanning…' : tick === 0 ? 'Start stepping' : 'Step'}
                    </button>
                    <button type="button" className="subtle" onClick={onReset} disabled={tick === 0}>
                        Reset
                    </button>
                </div>
            </div>
        </section>
    );
}
