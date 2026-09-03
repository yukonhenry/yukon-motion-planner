import { useCallback, useEffect, useState } from "react";
import * as api from "../api";
import type { Robot, RobotInput } from "../types";

/**
 * The fleet.
 *
 * Unscoped, unlike {@link usePlans} and {@link useGridDraft}: a robot is a spec with no home
 * grid, so this loads once and stays valid however the canvas changes. Nothing here freezes
 * anything either — editing a robot cannot invalidate a stored route, because a route was
 * computed from the world rather than from the machine that will drive it.
 */
export function useRobots() {
  const [robots, setRobots] = useState<Robot[]>([]);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const report = (e: unknown) => setError(e instanceof Error ? e.message : String(e));

  useEffect(() => {
    void api.listRobots().then(setRobots).catch(report);
  }, []);

  const create = useCallback(async (input: RobotInput) => {
    setPending(true);
    try {
      const saved = await api.createRobot(input);
      setRobots((current) => [...current, saved]);
      setError(null);
      return saved;
    } catch (e) {
      report(e);
      return null;
    } finally {
      setPending(false);
    }
  }, []);

  const update = useCallback(async (id: number, input: RobotInput) => {
    setPending(true);
    try {
      const saved = await api.updateRobot(id, input);
      setRobots((current) => current.map((r) => (r.id === id ? saved : r)));
      setError(null);
      return saved;
    } catch (e) {
      report(e);
      return null;
    } finally {
      setPending(false);
    }
  }, []);

  const remove = useCallback(async (id: number) => {
    try {
      await api.deleteRobot(id);
      setRobots((current) => current.filter((r) => r.id !== id));
      setError(null);
    } catch (e) {
      report(e);
    }
  }, []);

  return { robots, pending, error, create, update, remove };
}
