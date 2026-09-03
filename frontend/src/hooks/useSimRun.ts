import { useCallback, useEffect, useRef, useState } from "react";
import * as api from "../api";
import { fromWire } from "./useGridDraft";
import type { Obstacle, SimEvent, SimStatus, Vertex } from "../types";

/**
 * What the canvas draws while the backend is running the simulation.
 *
 * The two clocks are kept apart on purpose. `envTick` counts environment ticks and `planTick`
 * replans, and they advance at whatever rates the grid's `sim_interval` and the run's
 * requested replan interval say — so a planner falling behind a fast world is a visible fact
 * rather than something smoothed over.
 * `route` lags `obstacles` for exactly that reason: it is the answer to a world one or more
 * ticks old, and `routeEnvTick` says which.
 */
export interface SimRun {
  /** Seconds per environment tick, as the server resolved them. */
  envInterval: number;
  /** Seconds per replan. */
  /** Seconds between the robot's moves, from its own capabilities. */
  robotInterval: number;
  planId: number;
  robotId: number;
  /** Where the robot is standing. */
  robotPosition: Vertex;
  /** How many cells it covered on the last move. */
  robotMoved: number;
  /**
   * The most recent obstacle-into-robot collision, or `null` if there has not been one.
   *
   * Kept rather than counted, because the useful thing to show is *which* shape drove at the
   * machine and when — a tally would say something happened without saying what.
   */
  collision: { tick: number; obstacleIds: number[]; at: Vertex } | null;
  /** Ids of obstacles that could not move on the last environment tick. */
  blocked: number[];
  /** Where the obstacles are now. */
  obstacles: Obstacle[];
  envTick: number;
  /** How many obstacles moved on the last environment tick. */
  moved: number;
  /** The most recent route, or `null` before the first replan lands. */
  route: Vertex[] | null;
  /** Which environment tick `route` was planned against. */
  routeEnvTick: number;
  planTick: number;
  reachable: boolean;
  cost: number;
  planner: string;
  /** How long the last search took, in milliseconds. */
  elapsedMs: number;
  /** For replaying this run from the start. */
  seed: number;
}

/**
 * Subscribes to one grid's backend simulation.
 *
 * The server owns the run; this hook owns only the connection to it. So `start` posts and
 * then opens an `EventSource`, `stop` posts and lets the server's final `stopped` event close
 * the stream, and unmounting closes the connection *without* stopping the run — a user who
 * navigates away and comes back finds the simulation still going, and `adopt` rejoins it.
 *
 * The state here is always the server's, never a local guess: nothing is optimistically
 * advanced, because a client that ran its own clock alongside the backend's would drift and
 * there would be no way to tell which one was right.
 */
export function useSimRun(gridId: number | null) {
  const [run, setRun] = useState<SimRun | null>(null);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const source = useRef<EventSource | null>(null);

  const disconnect = useCallback(() => {
    source.current?.close();
    source.current = null;
  }, []);

  /**
   * Opens the stream for a run already known to exist.
   *
   * `EventSource` reconnects on its own after a dropped connection, which is the reason it
   * was chosen — but it also retries a 404, so a stream that ends because the *run* ended
   * has to be closed here rather than left to retry a run that is never coming back.
   */
  const connect = useCallback(
    (id: number) => {
      disconnect();
      const stream = new EventSource(api.simStreamUrl(id));
      source.current = stream;

      const onEvent = (raw: MessageEvent<string>) => {
        const event = JSON.parse(raw.data) as SimEvent;
        setRun((current) => {
          // An event for a run this hook has already dropped — a stop and a tick
          // crossing on the wire. Nothing to apply it to.
          if (!current) return current;
          switch (event.type) {
            case "environment":
              return {
                ...current,
                obstacles: event.obs_polygons.map(fromWire),
                envTick: event.tick,
                moved: event.moved,
                blocked: event.blocked,
              };
            case "collision":
              return {
                ...current,
                collision: {
                  tick: event.tick,
                  obstacleIds: event.obstacle_ids,
                  at: event.robot_position,
                },
              };
            case "plan":
              return {
                ...current,
                robotPosition: event.position,
                robotMoved: event.moved,
                route: event.vertices,
                routeEnvTick: event.env_tick,
                planTick: event.tick,
                reachable: event.reachable,
                cost: event.cost,
                planner: event.planner,
                elapsedMs: event.elapsed_ms,
              };
            case "stopped":
              return null;
          }
        });

        if (event.type === "stopped") disconnect();
      };

      stream.addEventListener("environment", onEvent as EventListener);
      stream.addEventListener("collision", onEvent as EventListener);
      stream.addEventListener("plan", onEvent as EventListener);
      stream.addEventListener("stopped", onEvent as EventListener);

      // Fires both for a dropped connection (which `EventSource` will retry) and for a
      // stream that never opened. Distinguished by `readyState`: CLOSED means the
      // request failed outright — the run is gone — and retrying would be a loop.
      stream.onerror = () => {
        if (stream.readyState === EventSource.CLOSED) {
          disconnect();
          setRun(null);
          setError("the simulation stream closed — the run is no longer on the server");
        }
      };
    },
    [disconnect],
  );

  /** Turns a status response into the run state, with no route until the first replan. */
  const adoptStatus = useCallback((status: SimStatus) => {
    setRun({
      envInterval: status.env_interval,
      robotInterval: status.robot_interval,
      planId: status.plan_id,
      robotId: status.robot_id,
      robotPosition: status.robot_position,
      robotMoved: 0,
      collision: null,
      blocked: [],
      obstacles: status.obs_polygons.map(fromWire),
      envTick: status.env_tick,
      moved: 0,
      route: null,
      routeEnvTick: status.env_tick,
      planTick: 0,
      reachable: false,
      cost: 0,
      planner: "",
      elapsedMs: 0,
      seed: status.seed,
    });
  }, []);

  const start = useCallback(
    async (planId: number) => {
      if (gridId === null) return;
      setPending(true);
      setError(null);
      try {
        adoptStatus(await api.startSim(gridId, { plan_id: planId }));
        connect(gridId);
      } catch (e) {
        setError(e instanceof Error ? e.message : String(e));
      } finally {
        setPending(false);
      }
    },
    [gridId, adoptStatus, connect],
  );

  const stop = useCallback(async () => {
    if (gridId === null) return;
    setPending(true);
    try {
      await api.stopSim(gridId);
      setError(null);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      // Whether or not the post succeeded, this client is done watching. A stop that
      // 404s means the run had already ended, which is the state being asked for.
      disconnect();
      setRun(null);
      setPending(false);
    }
  }, [gridId, disconnect]);

  // Rejoin a run already going on this grid — after a page reload, or after switching away
  // and back. A 404 is the ordinary answer (nothing is running) and is not an error.
  useEffect(() => {
    disconnect();
    setRun(null);
    setError(null);
    if (gridId === null) return;

    let current = true;
    void api
      .simStatus(gridId)
      .then((status) => {
        if (!current) return;
        adoptStatus(status);
        connect(gridId);
      })
      .catch(() => {
        /* nothing running on this grid */
      });

    return () => {
      current = false;
      disconnect();
    };
  }, [gridId, adoptStatus, connect, disconnect]);

  return { run, pending, error, setError, start, stop };
}
