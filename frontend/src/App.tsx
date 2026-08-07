import {useCallback, useEffect, useMemo, useState} from 'react';
import {GridCanvas} from './components/GridCanvas';
import {GridPicker} from './components/GridPicker';
import {ObstacleList} from './components/ObstacleList';
import {RoutePanel} from './components/RoutePanel';
import {SavePanel} from './components/SavePanel';
import {MIN_VERTICES, validateObstacles, validateVertices} from './geometry';
import {draftToInput, useGridDraft} from './hooks/useGridDraft';
import {useGrids} from './hooks/useGrids';
import {usePlans} from './hooks/usePlans';
import type {Endpoint, Vertex} from './types';

const NO_ENDPOINTS: Record<Endpoint, Vertex | null> = {src: null, dest: null};

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
    const [saving, setSaving] = useState(false);
    const [saveError, setSaveError] = useState<string | null>(null);
    /** Off by default: it is a debugging view, and it hides the grid lines underneath. */
    const [showFootprint, setShowFootprint] = useState(false);

    const plans = usePlans(gridId);
    const {
        draft,
        dirty,
        unsaved,
        startNew,
        close,
        adoptSaved,
        addObstacle,
        updateObstacle,
        removeObstacle,
        ...draftState
    } = useGridDraft(gridId);

    /** A saved, plan-bearing grid is read-only until an edit forks it. */
    const frozen = plans.frozen && !unsaved;
    const savedGrid = useMemo(
        () => grids.grids.find((g) => g.id === gridId) ?? null,
        [grids.grids, gridId],
    );

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
            close();
        } else {
            // Re-selecting reloads the stored geometry, which is the definition of reverting.
            selectGrid(gridId);
        }
    }, [gridId, close, selectGrid]);

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

    const generateRoute = useCallback(() => {
        if (!endpoints.src || !endpoints.dest) return;
        void plans.generate(endpoints.src, endpoints.dest);
    }, [endpoints, plans]);

    const clearRoute = useCallback(() => {
        setEndpoints(NO_ENDPOINTS);
        setPicking(null);
        plans.hide();
    }, [plans]);

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
    const status = saveError ?? plans.error ?? draftState.error ?? grids.error;

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
                        src={endpoints.src}
                        dest={endpoints.dest}
                        picking={picking}
                        onPick={startPicking}
                        onGenerate={generateRoute}
                        onClear={clearRoute}
                        pending={plans.pending}
                        plans={plans.plans}
                        active={plans.active}
                        onShow={plans.show}
                        onDelete={(id) => void plans.remove(id)}
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
                        obstacles={draft.obstacles}
                        selectedId={selectedId}
                        onSelect={setSelectedId}
                        draft={pencil}
                        onDraftAppend={(cell) => setPencil((d) => (d ? [...d, cell] : [cell]))}
                        onUpdate={updateObstacle}
                        picking={picking}
                        onPickCell={placeEndpoint}
                        src={endpoints.src}
                        dest={endpoints.dest}
                        route={dirty ? null : (plans.active?.vertices ?? null)}
                        showFootprint={showFootprint}
                    />
                )}
            </main>
        </div>
    );
}