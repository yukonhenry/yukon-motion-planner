import { useCallback, useEffect, useRef, useState } from 'react';
import * as api from '../api';
import type { Plan, Vertex } from '../types';

/**
 * The plans stored against one grid.
 *
 * Two jobs, because they are the same fact: the list is the route history *and* the
 * freeze signal. A grid with plans cannot be edited in place — each of those routes was
 * computed against that exact geometry — so `frozen` is derived from the list rather
 * than tracked separately, and deleting the last plan unfreezes the grid immediately.
 */
export function usePlans(gridId: number | null) {
  const [plans, setPlans] = useState<Plan[]>([]);
  const [activeId, setActiveId] = useState<number | null>(null);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Bumped per request so a response can tell whether it is still wanted.
  const ticket = useRef(0);

  useEffect(() => {
    ticket.current++;
    setPlans([]);
    setActiveId(null);
    setError(null);
    setPending(false);

    if (gridId === null) return;

    const mine = ticket.current;
    void api
      .listPlans(gridId)
      .then((found) => {
        if (mine !== ticket.current) return;
        setPlans(found);
        // Show the most recent route on arrival, so a reload doesn't look like the
        // grid was never planned.
        setActiveId(found.length > 0 ? found[found.length - 1].id : null);
      })
      .catch((e: unknown) => {
        if (mine === ticket.current) setError(e instanceof Error ? e.message : String(e));
      });
  }, [gridId]);

  const generate = useCallback(
    async (src: Vertex, dest: Vertex) => {
      if (gridId === null) return;

      const mine = ++ticket.current;
      setPending(true);
      try {
        const saved = await api.generatePlan(gridId, src, dest);
        if (mine !== ticket.current) return;
        setPlans((current) => [...current, saved]);
        setActiveId(saved.id);
        setError(null);
      } catch (e) {
        if (mine !== ticket.current) return;
        setError(e instanceof Error ? e.message : String(e));
      } finally {
        if (mine === ticket.current) setPending(false);
      }
    },
    [gridId],
  );

  const remove = useCallback(async (planId: number) => {
    try {
      await api.deletePlan(planId);
      setPlans((current) => current.filter((p) => p.id !== planId));
      setActiveId((current) => (current === planId ? null : current));
      setError(null);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    }
  }, []);

  /** Hides the drawn route without deleting it — the grid stays frozen. */
  const hide = useCallback(() => setActiveId(null), []);

  const active = plans.find((p) => p.id === activeId) ?? null;

  return {
    plans,
    active,
    frozen: plans.length > 0,
    pending,
    error,
    setError,
    generate,
    remove,
    hide,
    show: setActiveId,
  };
}