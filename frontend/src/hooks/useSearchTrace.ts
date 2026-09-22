import { useCallback, useEffect, useRef, useState } from "react";
import { searchTrace } from "../api";
import type { SearchTrace, SearchTraceInput } from "../types";

/**
 * Fetching one SST search trace and playing it back.
 *
 * The playhead is an index into the event list rather than a wall-clock time, because the
 * events are not evenly spaced in time and nobody wants to watch the search at the speed it
 * actually ran — a few hundred milliseconds, most of it after the last interesting thing
 * happened. Stepping by events makes every one of them visible and makes the scrubber linear.
 *
 * Playback is driven by requestAnimationFrame rather than setInterval so it stays in step with
 * the compositor and pauses itself in a background tab. The frame is advanced by elapsed time
 * rather than by one per tick, so the speed means the same thing on any display.
 */

/** Events advanced per second at speed 1. Fast enough to feel live, slow enough to follow. */
const EVENTS_PER_SECOND = 400;

export interface SearchTracePlayer {
  trace: SearchTrace | null;
  loading: boolean;
  error: string | null;
  /** How many events have been applied — the playhead. */
  frame: number;
  playing: boolean;
  /** Multiplier on {@link EVENTS_PER_SECOND}. */
  speed: number;
  /** Show branches sparsification has culled, greyed out behind the live tree. */
  showGhosts: boolean;
  /** Show the witness centres that drive the pruning. */
  showWitnesses: boolean;

  run: (input: SearchTraceInput) => Promise<void>;
  clear: () => void;
  seek: (frame: number) => void;
  toggle: () => void;
  setSpeed: (speed: number) => void;
  setShowGhosts: (show: boolean) => void;
  setShowWitnesses: (show: boolean) => void;
}

export function useSearchTrace(gridId: number | null): SearchTracePlayer {
  const [trace, setTrace] = useState<SearchTrace | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [frame, setFrame] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [showGhosts, setShowGhosts] = useState(true);
  const [showWitnesses, setShowWitnesses] = useState(false);

  const total = trace?.events.length ?? 0;

  // The playhead is mirrored in a ref so the animation loop can advance it without reading
  // stale state and without putting a `setState` inside a `setState` updater — updaters are
  // re-invoked by React, so a side effect in one is an infinite render loop, not a one-off.
  const frameRef = useRef(0);
  const moveTo = useCallback(
    (next: number) => {
      frameRef.current = next;
      setFrame(next);
    },
    [],
  );

  // A trace belongs to the grid it was run against. Keeping it across a switch would draw one
  // grid's search over another's obstacles, which looks like a bug in the planner.
  useEffect(() => {
    setTrace(null);
    moveTo(0);
    setPlaying(false);
    setError(null);
  }, [gridId, moveTo]);

  const run = useCallback(
    async (input: SearchTraceInput) => {
      if (gridId === null) return;
      setLoading(true);
      setError(null);
      setPlaying(false);
      try {
        const next = await searchTrace(gridId, input);
        setTrace(next);
        // Start at the end: the finished tree is what a viewer wants to see first, and
        // scrubbing back from it is more natural than pressing play on an empty canvas.
        moveTo(next.events.length);
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err));
        setTrace(null);
      } finally {
        setLoading(false);
      }
    },
    [gridId, moveTo],
  );

  const clear = useCallback(() => {
    setTrace(null);
    moveTo(0);
    setPlaying(false);
    setError(null);
  }, [moveTo]);

  const seek = useCallback(
    (next: number) => {
      setPlaying(false);
      moveTo(Math.max(0, Math.min(next, total)));
    },
    [moveTo, total],
  );

  const toggle = useCallback(() => {
    if (total === 0) return;
    // Pressing play at the end replays from the start rather than doing nothing. Done before
    // flipping `playing`, and outside any updater, so neither setter runs inside the other.
    if (!playing && frameRef.current >= total) moveTo(0);
    setPlaying((was) => !was);
  }, [moveTo, playing, total]);

  // The rAF loop. `last` is a ref so changing speed mid-play does not restart the animation,
  // and the fractional remainder is carried so a slow speed still advances rather than
  // rounding to zero every frame.
  const carry = useRef(0);
  useEffect(() => {
    if (!playing || total === 0) return;

    let handle = 0;
    let last = performance.now();
    carry.current = 0;
    const step = (now: number) => {
      const elapsed = (now - last) / 1000;
      last = now;
      carry.current += elapsed * EVENTS_PER_SECOND * speed;
      const whole = Math.floor(carry.current);
      if (whole > 0) {
        carry.current -= whole;
        const next = frameRef.current + whole;
        moveTo(Math.min(next, total));
        if (next >= total) {
          setPlaying(false);
          return; // the effect's cleanup cancels nothing that was never scheduled
        }
      }
      handle = requestAnimationFrame(step);
    };

    handle = requestAnimationFrame(step);
    return () => cancelAnimationFrame(handle);
  }, [moveTo, playing, speed, total]);

  return {
    trace,
    loading,
    error,
    frame,
    playing,
    speed,
    showGhosts,
    showWitnesses,
    run,
    clear,
    seek,
    toggle,
    setSpeed,
    setShowGhosts,
    setShowWitnesses,
  };
}
