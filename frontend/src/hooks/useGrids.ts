import { useCallback, useEffect, useState } from 'react';
import * as api from '../api';
import type { Grid, GridDetail, GridInput } from '../types';

/**
 * The saved grids, and the three ways to write one.
 *
 * Which write to use is not this hook's decision — it depends on whether the grid is
 * frozen by a plan, which only the caller knows. See `save` in App.tsx.
 */
export function useGrids() {
  const [grids, setGrids] = useState<Grid[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const refresh = useCallback(async () => {
    setLoading(true);
    try {
      setGrids(await api.listGrids());
      setError(null);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    void refresh();
  }, [refresh]);

  /** Drops `obs_polygons`, so a detail response can update the lean listing. */
  const summarize = ({ id, name, width, height, version }: GridDetail): Grid => ({
    id,
    name,
    width,
    height,
    version,
  });

  const create = useCallback(async (input: GridInput) => {
    const grid = await api.createGrid(input);
    setGrids((current) => [...current, summarize(grid)]);
    return grid;
  }, []);

  const update = useCallback(async (gridId: number, input: GridInput) => {
    const grid = await api.updateGrid(gridId, input);
    setGrids((current) => current.map((g) => (g.id === gridId ? summarize(grid) : g)));
    return grid;
  }, []);

  /** Appends a *new* row; the edited one stays in the list unchanged. */
  const createVersion = useCallback(async (gridId: number, input: GridInput) => {
    const grid = await api.createGridVersion(gridId, input);
    setGrids((current) => [...current, summarize(grid)]);
    return grid;
  }, []);

  const remove = useCallback(async (gridId: number) => {
    await api.deleteGrid(gridId);
    setGrids((current) => current.filter((g) => g.id !== gridId));
  }, []);

  return { grids, loading, error, setError, refresh, create, update, createVersion, remove };
}