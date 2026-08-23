import type { SimRun } from '../hooks/useSimRun';

interface Props {
    /** How many manual ticks have run, or 0 before the first. */
    tick: number;
    /** Result of the last manual tick, or `null` before one has run. */
    last: { reachable: boolean; cost: number; moved: number; planner: string } | null;
    /** How many obstacles on the canvas are flagged dynamic. */
    dynamicCount: number;
    /** Why stepping is unavailable, or `null` when it is available. */
    blocked: string | null;
    pending: boolean;
    onStep: () => void;
    onReset: () => void;

    /** The backend-scheduled run, or `null` when none is going. */
    run: SimRun | null;
    /** Why a run cannot be started, or `null` when it can. */
    runBlocked: string | null;
    runPending: boolean;
    onStart: () => void;
    onStop: () => void;
}

/** Seconds as a frequency, which is how a schedule is easier to read. */
function rate(seconds: number): string {
    return `${seconds}s (${(1 / seconds).toFixed(seconds < 1 ? 0 : 2)} Hz)`;
}

/**
 * The two ways to run the moving-obstacle simulation.
 *
 * **Step** is the manual clock: one request per click, the browser holding the run's state,
 * nothing stored. It is still here because it is the only way to inspect a single tick.
 *
 * **Run** hands the whole thing to the backend. Two tasks start on two independent schedules —
 * the environment at the grid's `sim_interval`, the replanner at the frequency the start
 * request asked for — and push what they do over a server-sent event stream. The clock lives
 * on the server from that point: closing the tab does not stop the run, and reopening it
 * rejoins.
 *
 * A run is anchored to a *saved plan*, which is where its endpoints come from. That is why
 * building the grid and computing the first plan stay ordinary synchronous steps — pressing
 * Run is what sets the finished situation in motion.
 */
export function SimPanel({
                             tick,
                             last,
                             dynamicCount,
                             blocked,
                             pending,
                             onStep,
                             onReset,
                             run,
                             runBlocked,
                             runPending,
                             onStart,
                             onStop,
                         }: Props) {
    /**
     * How far the planner is behind the world. Not a fault — it is the thing the two
     * frequencies exist to make visible — so it is reported rather than warned about.
     */
    const lag = run && run.route !== null ? run.envTick - run.routeEnvTick : 0;

    return (
        <section className="panel">
            <h2>Simulation</h2>

            <div className="stack">
                {dynamicCount === 0 ? (
                    <p className="muted">
                        No obstacle is marked <strong>dynamic</strong> — neither stepping nor
                        running would change anything. Tick one in the obstacle list.
                    </p>
                ) : (
                    <p className="muted hint">
                        {dynamicCount === 1 ? 'One obstacle' : `${dynamicCount} obstacles`} can move.
                        Each environment tick jitters one corner of{' '}
                        {dynamicCount === 1 ? 'it' : 'each'}; the planner replans on its own clock.
                    </p>
                )}

                {/* --- backend-scheduled run --- */}

                {run ? (
                    <div className="stack">
                        <p className="muted">
                            <strong>Running</strong> · world {rate(run.envInterval)} · planner{' '}
                            {rate(run.replanInterval)}
                        </p>
                        <p className="muted">
                            Env tick {run.envTick} · {run.moved} moved
                        </p>
                        <p className="muted">
                            {run.route === null ? (
                                'Waiting for the first replan…'
                            ) : (
                                <>
                                    Replan {run.planTick} ·{' '}
                                    {run.reachable ? `cost ${run.cost}` : 'no route'} ·{' '}
                                    {run.elapsedMs}ms · {run.planner}
                                    {lag > 0 && ` · ${lag} tick${lag === 1 ? '' : 's'} behind`}
                                </>
                            )}
                        </p>
                        <div className="row">
                            <button type="button" onClick={onStop} disabled={runPending}>
                                {runPending ? 'Stopping…' : 'Stop'}
                            </button>
                        </div>
                        <p className="muted hint">
                            The clock is on the server. Closing this tab leaves the run going —
                            reopening the grid rejoins it. Nothing is saved either way.
                        </p>
                    </div>
                ) : (
                    <div className="stack">
                        {runBlocked && <p className="status status--error">{runBlocked}</p>}
                        <div className="row">
                            <button
                                type="button"
                                onClick={onStart}
                                disabled={runPending || runBlocked !== null}
                            >
                                {runPending ? 'Starting…' : 'Run simulation'}
                            </button>
                        </div>
                        <p className="muted hint">
                            Runs the environment and the replanner as two backend tasks, each on
                            its own frequency — the grid's <code>sim_interval</code> for the
                            world, the start request's <code>replan_interval</code> for the
                            planner.
                        </p>
                    </div>
                )}

                {/* --- manual stepping --- */}

                {run === null && (
                    <>
                        <hr />
                        {blocked && <p className="status status--error">{blocked}</p>}

                        {tick > 0 && last && (
                            <p className="muted">
                                Step {tick} · {last.moved} moved ·{' '}
                                {last.reachable ? `cost ${last.cost}` : 'no route'} · {last.planner}
                            </p>
                        )}

                        <div className="row">
                            <button
                                type="button"
                                className="subtle"
                                onClick={onStep}
                                disabled={pending || blocked !== null || dynamicCount === 0}
                            >
                                {pending ? 'Replanning…' : tick === 0 ? 'Step once' : 'Step'}
                            </button>
                            <button
                                type="button"
                                className="subtle"
                                onClick={onReset}
                                disabled={tick === 0}
                            >
                                Reset
                            </button>
                        </div>
                    </>
                )}
            </div>
        </section>
    );
}