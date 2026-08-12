import {useCallback, useEffect, useRef, useState} from 'react';
import * as api from '../api';
import {closeRing} from '../geometry';
import type {GridDetail, GridInput, Obstacle, Vertex, WireObstacle} from '../types';

/**
 * The working copy of a grid: what is on the canvas, which is not what is in the
 * database until the user confirms.
 *
 * This is the whole reason obstacles stopped having routes of their own. A grid row is
 * immutable once a plan references it, so writing every dragged vertex straight to the
 * server would either burn a version per nudge or refuse the nudge outright. Edits
 * accumulate here instead, and one confirm writes the finished grid.
 */
export interface Draft {
    name: string;
    width: number;
    height: number;
    obstacles: Obstacle[];
}

/**
 * What actually distinguishes two saves.
 *
 * `id` is excluded — it identifies a shape rather than describing it, so renumbering is not
 * an edit. `dynamic` is included: toggling it is a change the user expects to be able to
 * save, and leaving it out would leave the Revert button greyed out over a real difference.
 */
const signature = (draft: Draft) =>
    JSON.stringify([
        draft.name,
        draft.width,
        draft.height,
        draft.obstacles.map((o) => [o.vertices, o.dynamic]),
    ]);

/**
 * The working copy's tuple vertices, in the `{x, y}` form the API reads.
 *
 * Exported because the simulation needs it too: a replan tick sends the obstacles as they stand
 * and gets them back moved, so both directions of this conversion are used outside saving.
 */
export const toWire = (obstacle: Obstacle): WireObstacle => ({
    id: obstacle.id,
    dynamic: obstacle.dynamic,
    vertices: obstacle.vertices.map(([x, y]) => ({x, y})),
});

/** The inverse. Ids and `dynamic` come from the server unchanged. */
export const fromWire = (polygon: WireObstacle): Obstacle => ({
    id: polygon.id,
    dynamic: polygon.dynamic,
    vertices: polygon.vertices.map(({x, y}): Vertex => [x, y]),
});

export const draftToInput = (draft: Draft): GridInput => ({
    name: draft.name,
    width: draft.width,
    height: draft.height,
    obs_polygons: draft.obstacles.map(toWire),
});

export function useGridDraft(gridId: number | null) {
    const [draft, setDraft] = useState<Draft | null>(null);
    /**
     * The last saved state, kept whole rather than as a signature so that reverting is a
     * restore rather than a re-fetch. `null` while the grid has never been saved.
     *
     * Safe to share structure with `draft`: every edit below rebuilds the objects it touches,
     * so nothing here is ever mutated out from under us.
     */
    const [baseline, setBaseline] = useState<Draft | null>(null);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);

    // Ids have to be unique within one grid.  Incremental counter is sufficient.
    const nextId = useRef(1);
    const adopt = useCallback((polygons: WireObstacle[]): Obstacle[] => {
        const obstacles = polygons.map(fromWire);
        nextId.current = Math.max(nextId.current, ...obstacles.map((o) => o.id + 1));
        return obstacles;
    }, []);

    /** Replaces the working copy with a saved grid, making it the new clean state. */
    const adoptSaved = useCallback(
        (grid: GridDetail) => {
            const next: Draft = {
                name: grid.name,
                width: grid.width,
                height: grid.height,
                obstacles: adopt(grid.obs_polygons),
            };
            setDraft(next);
            setBaseline(next);
            setError(null);
        },
        [adopt],
    );

    /**
     * Throws away local edits, putting the canvas back to the last saved state.
     *
     * A no-op on a grid that was never saved — there is no state to go back to, and
     * discarding it is `close`'s job.
     */
    const revert = useCallback(() => {
        if (baseline === null) return;
        setDraft(baseline);
        setError(null);
    }, [baseline]);

    // Selecting a grid loads its geometry; `GET /grids` deliberately omits it.
    useEffect(() => {
        if (gridId === null) return;

        let current = true;
        setLoading(true);
        api
            .showGrid(gridId)
            .then((grid) => {
                if (current) adoptSaved(grid);
            })
            .catch((e: unknown) => {
                if (current) setError(e instanceof Error ? e.message : String(e));
            })
            .finally(() => {
                if (current) setLoading(false);
            });

        // A slow response for a grid the user has already navigated away from must not
        // overwrite the working copy they are now looking at.
        return () => {
            current = false;
        };
    }, [gridId, adoptSaved]);

    /** Begins composing a grid that does not exist on the server yet. */
    const startNew = useCallback((name: string, width: number, height: number) => {
        setDraft({name, width, height, obstacles: []});
        setBaseline(null);
        setError(null);
    }, []);

    const close = useCallback(() => {
        setDraft(null);
        setBaseline(null);
        setError(null);
    }, []);

    const edit = useCallback((change: (draft: Draft) => Draft) => {
        setDraft((current) => (current ? change(current) : current));
    }, []);

    /**
     * The pencil hands over the cells that were clicked; storing the polygon closed is
     * what makes "corners" and "entries" differ by exactly one everywhere downstream.
     */
    const addObstacle = useCallback(
        (vertices: Vertex[]) =>
            edit((d) => ({
                ...d,
                obstacles: [
                    ...d.obstacles,
                    // Drawn as scenery: a new shape is static until the user says otherwise.
                    {id: nextId.current++, dynamic: false, vertices: closeRing(vertices)},
                ],
            })),
        [edit],
    );

    const setObstacleDynamic = useCallback(
        (id: number, dynamic: boolean) =>
            edit((d) => ({
                ...d,
                obstacles: d.obstacles.map((o) => (o.id === id ? {...o, dynamic} : o)),
            })),
        [edit],
    );

    const updateObstacle = useCallback(
        (id: number, vertices: Vertex[]) =>
            edit((d) => ({
                ...d,
                obstacles: d.obstacles.map((o) => (o.id === id ? {...o, vertices} : o)),
            })),
        [edit],
    );

    const removeObstacle = useCallback(
        (id: number) => edit((d) => ({...d, obstacles: d.obstacles.filter((o) => o.id !== id)})),
        [edit],
    );

    const rename = useCallback((name: string) => edit((d) => ({...d, name})), [edit]);

    const resize = useCallback(
        (width: number, height: number) => edit((d) => ({...d, width, height})),
        [edit],
    );

    /**
     * Unsaved edits are the only thing standing between the canvas and the database, so
     * "dirty" drives both the confirm button and the refusal to plan. A grid that has
     * never been saved is always dirty — there is nothing to compare it against.
     */
    const dirty =
        draft !== null && (baseline === null || signature(draft) !== signature(baseline));
    const unsaved = draft !== null && baseline === null;

    return {
        draft,
        dirty,
        unsaved,
        loading,
        error,
        setError,
        startNew,
        close,
        revert,
        adoptSaved,
        addObstacle,
        updateObstacle,
        setObstacleDynamic,
        removeObstacle,
        rename,
        resize,
    };
}