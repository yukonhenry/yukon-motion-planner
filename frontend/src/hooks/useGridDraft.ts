import { useCallback, useEffect, useRef, useState } from 'react';
import * as api from '../api';
import { closeRing } from '../geometry';
import type { GridDetail, GridInput, Obstacle, Vertex } from '../types';

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

/** What actually distinguishes two saves — ids are local, so they are left out. */
const signature = (draft: Draft) =>
  JSON.stringify([draft.name, draft.width, draft.height, draft.obstacles.map((o) => o.vertices)]);

export const draftToInput = (draft: Draft): GridInput => ({
  name: draft.name,
  width: draft.width,
  height: draft.height,
  obs_polygons: draft.obstacles.map((o) => o.vertices),
});

export function useGridDraft(gridId: number | null) {
  const [draft, setDraft] = useState<Draft | null>(null);
  /** Signature of the last saved state; `null` while the grid has never been saved. */
  const [baseline, setBaseline] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Obstacle ids are handed out here and never leave the client, so a plain counter is
  // enough — it only has to be unique within this working copy.
  const nextId = useRef(1);
  const adopt = useCallback((polygons: Vertex[][]): Obstacle[] => {
    return polygons.map((vertices) => ({ id: nextId.current++, vertices }));
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
      setBaseline(signature(next));
      setError(null);
    },
    [adopt],
  );

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
    setDraft({ name, width, height, obstacles: [] });
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
        obstacles: [...d.obstacles, { id: nextId.current++, vertices: closeRing(vertices) }],
      })),
    [edit],
  );

  const updateObstacle = useCallback(
    (id: number, vertices: Vertex[]) =>
      edit((d) => ({
        ...d,
        obstacles: d.obstacles.map((o) => (o.id === id ? { ...o, vertices } : o)),
      })),
    [edit],
  );

  const removeObstacle = useCallback(
    (id: number) => edit((d) => ({ ...d, obstacles: d.obstacles.filter((o) => o.id !== id) })),
    [edit],
  );

  const rename = useCallback((name: string) => edit((d) => ({ ...d, name })), [edit]);

  const resize = useCallback(
    (width: number, height: number) => edit((d) => ({ ...d, width, height })),
    [edit],
  );

  /**
   * Unsaved edits are the only thing standing between the canvas and the database, so
   * "dirty" drives both the confirm button and the refusal to plan. A grid that has
   * never been saved is always dirty — there is nothing to compare it against.
   */
  const dirty = draft !== null && (baseline === null || signature(draft) !== baseline);
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
    adoptSaved,
    addObstacle,
    updateObstacle,
    removeObstacle,
    rename,
    resize,
  };
}