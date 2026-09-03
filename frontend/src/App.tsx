import {useCallback, useEffect, useMemo, useState} from 'react';
import * as api from './api';
import {GridCanvas} from './components/GridCanvas';
import {GridPicker} from './components/GridPicker';
import {ObstacleList} from './components/ObstacleList';
import {RobotPanel} from './components/RobotPanel';
import {RoutePanel} from './components/RoutePanel';
import {SavePanel} from './components/SavePanel';
import {SimPanel} from './components/SimPanel';
import {MIN_VERTICES, validateObstacles, validateVertices} from './geometry';
import {draftToInput, fromWire, toWire, useGridDraft} from './hooks/useGridDraft';
import {useGrids} from './hooks/useGrids';
import {useRobots} from './hooks/useRobots';
import {usePlans} from './hooks/usePlans';
import {useSimRun} from './hooks/useSimRun';
import type {Endpoint, Obstacle, RobotInput, Vertex} from './types';

const NO_ENDPOINTS: Record<Endpoint, Vertex | null> = {src: null, dest: null};

/** One in-progress simulation run. Ephemeral — the server keeps nothing between ticks. */
interface Sim {
    /** Where the obstacles are now, which is also what the next tick sends. */
    obstacles: Obstacle[];
    /** Chained from the last response, so the run replays from its first seed. */
    seed: number;
    route: Vertex[];
    reachable: boolean;
    cost: number;
    moved: number;
    planner: string;
    tick: number;
}

/**
 * The shape of the app follows one rule from the API: a grid row freezes as soon as a
 * plan is computed against it.
 *
 * So obstacles are edited locally and written once, on confirm — see `useGridDraft`.
 * Before any route exists, confirming rewrites the grid in place. After, it writes the
 * next version as a new row, because the stored routes describe the old one.
 */
export default function App() {
    const grids = useGrids();
    const [gridId, setGridId] = useState<number | null>(null);
    const [selectedId, setSelectedId] = useState<number | null>(null);
    const [pencil, setPencil] = useState<Vertex[] | null>(null);
    const [endpoints, setEndpoints] = useState(NO_ENDPOINTS);
    const [picking, setPicking] = useState<Endpoint | null>(null);
    /**
     * Which robot the next plan is for, or `null` before one has been chosen.
     *
     * Held here rather than in the route panel because it outlives one plan: a user comparing
     * two routes for the same machine should not have to reselect it each time. Planning is
     * refused until it is set — a route is computed *for* a machine.
     */
    const [driverId, setDriverId] = useState<number | null>(null);
    const [saving, setSaving] = useState(false);
    const [saveError, setSaveError] = useState<string | null>(null);
    /** Off by default: it is a debugging view, and it hides the grid lines underneath. */
    const [showFootprint, setShowFootprint] = useState(false);
    /**
     * The moving-obstacle run, or `null` when none is in progress.
     *
     * Held apart from the draft on purpose. The perturbed geometry is *not* an edit the user
     * made, so writing it into the draft would mark the grid dirty and offer to save tick 37's
     * shapes as if they were the intent. Keeping it here means Reset is free and the saved grid
     * is never at risk.
     */
    const [sim, setSim] = useState<Sim | null>(null);
    const [stepping, setStepping] = useState(false);

    const plans = usePlans(gridId);
    // Unscoped: the fleet is the same list whatever grid is on screen.
    const robots = useRobots();
    /**
     * The backend-scheduled run, which is a different thing from `sim` above.
     *
     * `sim` is the manual clock — the browser advancing time one request per click. This is
     * the server advancing it on two schedules of its own and pushing the results. Only one
     * can be going at a time, and the live run wins the canvas when it is.
     */
    const live = useSimRun(gridId);
    const {
        draft,
        dirty,
        unsaved,
        startNew,
        close,
        revert,
        adoptSaved,
        addObstacle,
        updateObstacle,
        setObstacleDynamic,
        removeObstacle,
        ...draftState
    } = useGridDraft(gridId);

    /** A saved, plan-bearing grid is read-only until an edit forks it. */
    const frozen = plans.frozen && !unsaved;
    const savedGrid = useMemo(
        () => grids.grids.find((g) => g.id === gridId) ?? null,
        [grids.grids, gridId],
    );

    /**
     * The obstacles a request is planned against: a manual run's if one is going, else the
     * working copy's. `null` while there is no world yet — no grid chosen, or one still loading.
     *
     * Deliberately not defaulted to `[]`, unlike {@link shownObstacles}. Drawing nothing is
     * harmless; *sending* nothing is a claim the server takes at its word — a plan records the
     * obstacles it was computed against, so falling through to "no obstacles" would store a
     * route running straight through a wall as evidence the wall was never there. "Not loaded
     * yet" and "nothing in the way" are different answers, and only one is safe to send.
     */
    const world = useMemo(() => sim?.obstacles ?? draft?.obstacles ?? null, [sim, draft]);

    const resetInteraction = useCallback(() => {
        setSelectedId(null);
        setPencil(null);
        setEndpoints(NO_ENDPOINTS);
        setPicking(null);
        setSaveError(null);
    }, []);

    // Land on a grid as soon as one exists, so the canvas is never pointlessly blank —
    // unless a new grid is being composed, which must not be thrown away.
    useEffect(() => {
        if (gridId === null && !draft && grids.grids.length > 0) {
            setGridId(grids.grids[0].id);
        }
    }, [grids.grids, gridId, draft]);

    // Cell coordinates only mean something on the grid they were picked on, so switching
    // grids drops the endpoints along with everything else.
    const selectGrid = useCallback(
        (id: number) => {
            setGridId(id);
            resetInteraction();
        },
        [resetInteraction],
    );

    const startNewGrid = useCallback(
        (name: string, width: number, height: number) => {
            // Nothing is posted yet: this only opens a canvas. `gridId` going null is what
            // makes the working copy unsaved rather than a copy of some stored row.
            setGridId(null);
            startNew(name, width, height);
            resetInteraction();
        },
        [startNew, resetInteraction],
    );

    const deleteGrid = useCallback(
        async (id: number) => {
            try {
                await grids.remove(id);
                setGridId(null);
                close();
                resetInteraction();
            } catch (e) {
                setSaveError(e instanceof Error ? e.message : String(e));
            }
        },
        [grids, close, resetInteraction],
    );

    // --- saving ------------------------------------------------------------

    /**
     * Local mirror of `validate_polygons`, so an impossible save is a disabled button
     * rather than a round trip. Shrinking a grid is what usually trips it.
     */
    const problem = draft
        ? validateObstacles(
            draft.obstacles.map((o) => o.vertices),
            draft,
        )
        : null;

    const save = useCallback(async () => {
        if (!draft || problem) return;
        setSaving(true);
        setSaveError(null);
        const input = draftToInput(draft);
        try {
            if (gridId === null) {
                const created = await grids.create(input);
                setGridId(created.id);
                adoptSaved(created);
            } else if (frozen) {
                // The routes below belong to the row we came from, so this leaves it alone and
                // moves the user onto the new snapshot.
                const next = await grids.createVersion(gridId, input);
                setGridId(next.id);
                adoptSaved(next);
                resetInteraction();
            } else {
                adoptSaved(await grids.update(gridId, input));
            }
        } catch (e) {
            setSaveError(e instanceof Error ? e.message : String(e));
        } finally {
            setSaving(false);
        }
    }, [draft, problem, gridId, frozen, grids, adoptSaved, resetInteraction]);

    const discard = useCallback(() => {
        if (gridId === null) {
            // Never saved, so there is nothing to go back to — the canvas closes instead.
            close();
        } else {
            revert();
            // The cells picked so far were picked against the edits just thrown away.
            resetInteraction();
        }
    }, [gridId, close, revert, resetInteraction]);

    // --- obstacles ---------------------------------------------------------

    const pencilError = draft && pencil ? validateVertices(pencil, draft) : null;
    const canFinish = pencil !== null && pencil.length >= MIN_VERTICES && !pencilError;

    const finishPencil = useCallback(() => {
        if (!pencil || !canFinish) return;
        addObstacle(pencil);
        setPencil(null);
    }, [pencil, canFinish, addObstacle]);

    /** Drawing and endpoint-picking both own the canvas click, so only one may be live. */
    const startPicking = useCallback((endpoint: Endpoint | null) => {
        setPicking(endpoint);
        if (endpoint) {
            setPencil(null);
            setSelectedId(null);
        }
    }, []);

    const placeEndpoint = useCallback(
        (cell: Vertex) => {
            if (!picking) return;
            setEndpoints((current) => ({...current, [picking]: cell}));
            // On the first pass, placing the start moves straight on to the goal — two
            // clicks to a plannable route instead of four.
            setPicking(picking === 'src' && endpoints.dest === null ? 'dest' : null);
        },
        [picking, endpoints.dest],
    );

    /**
     * The start and goal on show: freshly picked cells, else the ones the displayed
     * route was computed from.
     *
     * A plan carries its own endpoints — `meta.src_vertex` / `dest_vertex`, written by
     * generate_grid_plan — so selecting a saved route can mark its S and G without the
     * user re-picking them. Reading them from `meta` rather than from the route's first
     * and last cell is what makes an *unreachable* plan legible too: it is saved with no
     * cells at all, and the two markers with no line between them are the whole story.
     *
     * Both fall away with the route once the grid is dirty, since neither describes the
     * obstacles now on the canvas.
     */
    const shown = useMemo(() => {
        const planned = !dirty && plans.active ? plans.active : null;
        return {
            src: endpoints.src ?? planned?.src_vertex ?? null,
            dest: endpoints.dest ?? planned?.dest_vertex ?? null,
        };
    }, [endpoints, dirty, plans.active]);

    const generateRoute = useCallback(() => {
        if (!shown.src || !shown.dest || !world || driverId === null) return;
        void plans.generate(shown.src, shown.dest, world.map(toWire), driverId);
    }, [shown, plans, world, driverId]);

    const clearRoute = useCallback(() => {
        setEndpoints(NO_ENDPOINTS);
        setPicking(null);
        plans.hide();
    }, [plans]);

    // --- simulation --------------------------------------------------------

    /**
     * What the canvas is showing: the backend run's obstacles if one is going, else a manual
     * run's, else the working copy.
     *
     * The draft is what a *save* would write, so it stays the source of truth for the shapes the
     * user drew; this is only what is on screen.
     */
    const shownObstacles = live.run?.obstacles ?? sim?.obstacles ?? draft?.obstacles ?? [];
    const dynamicCount = shownObstacles.filter((o) => o.dynamic).length;

    /** The plan a backend run is anchored to, which is where its endpoints come from. */
    const anchorPlan = useMemo(
        () =>
            live.run
                ? (plans.plans.find((p) => p.id === live.run?.planId) ?? null)
                : (plans.active ?? null),
        [live.run, plans.plans, plans.active],
    );

    /**
     * Why a backend run cannot start, or `null`.
     *
     * Every reason is about the *stored* rows, because that is all a run reads: it takes the
     * grid's obstacles and the plan's endpoints from the database once, at start. Pending edits
     * are therefore not merely ignored — they would be invisible, which is worth refusing over.
     */
    const runBlocked =
        savedGrid === null
            ? 'Save the grid first — a run simulates the stored grid.'
            : dirty
              ? 'Save or revert the pending edits first — a run reads the stored obstacles.'
              : anchorPlan === null
                ? 'Generate or select a saved route first — a run replans between its endpoints.'
                : dynamicCount === 0
                  ? 'Mark at least one obstacle dynamic — the server refuses a run that cannot change.'
                  : null;

    /**
     * The start and goal to draw. A live run replans between the endpoints stored in *its*
     * plan's meta, which need not be whichever route the user last selected — so during a run
     * the markers follow the run rather than the selection.
     */
    const marks =
        live.run && anchorPlan
            ? {src: anchorPlan.src_vertex, dest: anchorPlan.dest_vertex}
            : shown;

    const startRun = useCallback(() => {
        if (!anchorPlan) return;
        // Manual stepping and a scheduled run are two clocks over one world; letting both go
        // would put two sets of obstacles on one canvas.
        setSim(null);
        void live.start(anchorPlan.id);
    }, [anchorPlan, live]);

    /**
     * Why a tick cannot run, or `null`. Every reason is about the *server's* view: replan plans
     * against the geometry it is sent, but the grid it needs for dimensions has to exist.
     */
    const stepBlocked =
        savedGrid === null
            ? 'Save the grid first — a run needs a stored grid to size itself against.'
            : world === null
              ? 'The grid is still loading.'
              : !shown.src || !shown.dest
                ? 'Place a start and a goal first.'
                : dirty && sim === null
                  ? 'Save or revert the pending edits first.'
                  : null;

    const step = useCallback(async () => {
        if (!savedGrid || !shown.src || !shown.dest || !world) return;
        setStepping(true);
        setSaveError(null);
        try {
            const result = await api.replan(savedGrid.id, {
                src_vertex: shown.src,
                dest_vertex: shown.dest,
                obs_polygons: world.map(toWire),
                // Absent on the first tick, so the server picks a starting seed; chained after.
                seed: sim?.seed,
            });
            setSim((current) => ({
                obstacles: result.obs_polygons.map(fromWire),
                seed: result.next_seed,
                route: result.vertices,
                reachable: result.reachable,
                cost: result.cost,
                moved: result.moved,
                planner: result.planner,
                tick: (current?.tick ?? 0) + 1,
            }));
        } catch (e) {
            setSaveError(e instanceof Error ? e.message : String(e));
        } finally {
            setStepping(false);
        }
    }, [savedGrid, shown, sim, world]);

    /** Drops the run. The draft was never touched, so the canvas snaps back to the saved shapes. */
    const resetSim = useCallback(() => {
        setSim(null);
        setSaveError(null);
    }, []);

    // A run describes one grid and one pair of endpoints; changing either leaves its obstacle
    // positions and its route describing something that is no longer on screen.
    useEffect(() => {
        setSim(null);
    }, [gridId]);

    // Esc cancels a drawing, Delete removes the selection — both only when not typing.
    useEffect(() => {
        const onKey = (e: KeyboardEvent) => {
            const target = e.target as HTMLElement | null;
            if (target && /^(INPUT|SELECT|TEXTAREA)$/.test(target.tagName)) return;

            if (e.key === 'Escape') {
                setPencil(null);
                setSelectedId(null);
                setPicking(null);
            } else if ((e.key === 'Delete' || e.key === 'Backspace') && selectedId !== null) {
                removeObstacle(selectedId);
                setSelectedId(null);
            }
        };
        window.addEventListener('keydown', onKey);
        return () => window.removeEventListener('keydown', onKey);
    }, [selectedId, removeObstacle]);

    // Saving is the most recent thing the user asked for, so its failure wins the slot.
    const saveRobot = useCallback(
        (input: RobotInput, editing: number | null) => {
            void (editing === null ? robots.create(input) : robots.update(editing, input));
        },
        [robots],
    );

    const status =
        saveError ?? live.error ?? plans.error ?? robots.error ?? draftState.error ?? grids.error;

    return (
        <div className="app">
            <aside className="sidebar">
                <h1>Yukon Motion Planner</h1>

                <GridPicker
                    grids={grids.grids}
                    selected={savedGrid}
                    onSelect={selectGrid}
                    onStartNew={startNewGrid}
                    onDelete={deleteGrid}
                    composing={unsaved}
                    frozen={frozen}
                />

                <RobotPanel
                    robots={robots.robots}
                    pending={robots.pending}
                    onSave={saveRobot}
                    onDelete={(id) => void robots.remove(id)}
                />

                {draft && (
                    <SavePanel
                        unsaved={unsaved}
                        dirty={dirty}
                        frozen={frozen}
                        version={savedGrid?.version ?? 0}
                        problem={problem}
                        pending={saving}
                        onSave={save}
                        onDiscard={discard}
                    />
                )}

                {draft && (
                    <section className="panel">
                        <h2>Draw</h2>
                        {pencil === null ? (
                            <button
                                type="button"
                                onClick={() => {
                                    setPencil([]);
                                    setSelectedId(null);
                                    setPicking(null);
                                }}
                            >
                                Draw obstacle
                            </button>
                        ) : (
                            <div className="stack">
                                <p className="muted">
                                    Click cells to place vertices ({pencil.length}/{MIN_VERTICES} minimum).
                                </p>
                                <div className="row">
                                    <button type="button" onClick={finishPencil} disabled={!canFinish}>
                                        Add
                                    </button>
                                    <button
                                        type="button"
                                        onClick={() => setPencil(pencil.slice(0, -1))}
                                        disabled={pencil.length === 0}
                                    >
                                        Undo point
                                    </button>
                                    <button type="button" onClick={() => setPencil(null)}>
                                        Cancel
                                    </button>
                                </div>
                            </div>
                        )}
                    </section>
                )}

                {draft && (
                    <RoutePanel
                        src={shown.src}
                        dest={shown.dest}
                        picking={picking}
                        onPick={startPicking}
                        onGenerate={generateRoute}
                        onClear={clearRoute}
                        pending={plans.pending}
                        plans={plans.plans}
                        active={plans.active}
                        onShow={plans.show}
                        onDelete={(id) => void plans.remove(id)}
                        robots={robots.robots}
                        driverId={driverId}
                        onPickDriver={setDriverId}
                        blocked={dirty && !unsaved}
                        frozen={frozen}
                        unsaved={unsaved}
                    />
                )}

                {draft && (
                    <section className="panel">
                        <h2>View</h2>
                        <label className="toggle">
                            <input
                                type="checkbox"
                                checked={showFootprint}
                                onChange={(e) => setShowFootprint(e.target.checked)}
                            />
                            Planner footprint
                        </label>
                        <p className="muted hint">
                            The cells the planner blocks. Vertices sit at cell centers, so this is
                            wider than the outline — and it is what a route is really avoiding.
                        </p>
                    </section>
                )}

                {draft && (
                    <ObstacleList
                        obstacles={draft.obstacles}
                        selectedId={selectedId}
                        onSelect={setSelectedId}
                        onDelete={(id) => {
                            removeObstacle(id);
                            if (id === selectedId) setSelectedId(null);
                        }}
                        onSetDynamic={setObstacleDynamic}
                        readOnly={frozen || sim !== null || live.run !== null}
                    />
                )}

                {draft && (
                    <SimPanel
                        tick={sim?.tick ?? 0}
                        last={sim}
                        dynamicCount={dynamicCount}
                        blocked={stepBlocked}
                        pending={stepping}
                        onStep={() => void step()}
                        onReset={resetSim}
                        run={live.run}
                        runBlocked={runBlocked}
                        runPending={live.pending}
                        onStart={startRun}
                        onStop={() => void live.stop()}
                    />
                )}

                {selectedId !== null && (
                    <p className="muted hint">
                        Drag the shape to move it, or drag a handle to reshape it. Delete removes it.
                    </p>
                )}
            </aside>

            <main className="stage">
                {status && <div className="status status--error">{status}</div>}

                {grids.loading && <p className="muted">Loading grids…</p>}

                {!grids.loading && !draft && <p className="muted">Create a grid to get started.</p>}

                {draft && (
                    <GridCanvas
                        grid={draft}
                        obstacles={shownObstacles}
                        selectedId={selectedId}
                        onSelect={setSelectedId}
                        draft={pencil}
                        onDraftAppend={(cell) => setPencil((d) => (d ? [...d, cell] : [cell]))}
                        // Dragging a shape during a run would edit the *draft* while the canvas
                        // shows the run's geometry — two different sets of shapes, one of them
                        // invisible. Reset or stop first.
                        onUpdate={sim || live.run ? () => {} : updateObstacle}
                        picking={picking}
                        onPickCell={placeEndpoint}
                        src={marks.src}
                        dest={marks.dest}
                        // A live run's route until its first replan lands, and the anchoring
                        // plan's route in the meantime — the run started from it, so it is the
                        // right thing to show against tick 0 rather than a blank canvas.
                        route={
                            live.run
                                ? (live.run.route ?? anchorPlan?.route_vertices ?? null)
                                : sim
                                  ? sim.route
                                  : dirty
                                    ? null
                                    : (plans.active?.route_vertices ?? null)
                        }
                        // Only a backend run has a robot: the manual Step clock moves
                        // obstacles, not machines.
                        robot={live.run?.robotPosition ?? null}
                        showFootprint={showFootprint}
                    />
                )}
            </main>
        </div>
    );
}