import { useState } from 'react';
import type { Grid } from '../types';

interface Props {
  grids: Grid[];
  selected: Grid | null;
  onSelect: (gridId: number) => void;
  /** Begins composing a grid locally. Nothing is written until the user confirms. */
  onStartNew: (name: string, width: number, height: number) => void;
  onDelete: (gridId: number) => Promise<unknown>;
  /** A new grid is being composed, so the picker's selection is not what's on screen. */
  composing: boolean;
  /** The selected grid is frozen by plans and cannot be deleted. */
  frozen: boolean;
}

/** Grid dimensions the canvas can still sensibly draw. */
const MIN_SIZE = 1;
const MAX_SIZE = 200;

/** "maze v3", so two snapshots of one grid are told apart in the list. */
const label = (grid: Grid) =>
  `${grid.name} v${grid.version} (${grid.width}×${grid.height})`;

export function GridPicker({
  grids,
  selected,
  onSelect,
  onStartNew,
  onDelete,
  composing,
  frozen,
}: Props) {
  const [creating, setCreating] = useState(false);
  const [name, setName] = useState('');
  // Held as text, not numbers, so the field can be *empty* mid-edit. Round-tripping
  // through `Number(e.target.value)` turns a cleared box into 0, which React writes
  // straight back — a leading zero that reappears every time you delete it, leaving no
  // way to type a value at all.
  const [width, setWidth] = useState('20');
  const [height, setHeight] = useState('15');
  /**
   * Which grid the delete button is currently asking about — an id rather than a flag,
   * so switching grids mid-question retracts it instead of leaving a prompt that names
   * one grid and would delete another.
   *
   * Deliberately not `window.confirm`: that blocks the page synchronously, which stops
   * far more than the app. A tab sitting on a native dialog also stops answering
   * anything attached to it, and browsers raise such a tab to the foreground to show it.
   */
  const [confirmingId, setConfirmingId] = useState<number | null>(null);

  const confirming = selected !== null && confirmingId === selected.id;

  const size = { width: Number(width), height: Number(height) };
  const valid = ([size.width, size.height] as const).every(
    (n) => width !== '' && height !== '' && Number.isInteger(n) && n >= MIN_SIZE && n <= MAX_SIZE,
  );

  const submit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!valid) return;
    // Deliberately local: the grid is not posted until its obstacles are drawn and the
    // user confirms, so this only opens a canvas to draw on.
    onStartNew(name.trim() || 'untitled', size.width, size.height);
    setName('');
    setCreating(false);
  };

  return (
    <section className="panel">
      <h2>Grid</h2>

      <div className="row">
        <select
          value={composing ? '' : (selected?.id ?? '')}
          onChange={(e) => onSelect(Number(e.target.value))}
          disabled={grids.length === 0}
        >
          {composing && <option value="">unsaved grid</option>}
          {!composing && grids.length === 0 && <option value="">no grids yet</option>}
          {grids.map((grid) => (
            <option key={grid.id} value={grid.id}>
              {label(grid)}
            </option>
          ))}
        </select>
        <button type="button" onClick={() => setCreating((c) => !c)}>
          {creating ? 'Cancel' : 'New…'}
        </button>
      </div>

      {creating && (
        <form className="stack" onSubmit={submit}>
          <label>
            Name
            <input value={name} onChange={(e) => setName(e.target.value)} placeholder="untitled" />
          </label>
          <div className="row">
            <label>
              Width
              <input
                type="number"
                min={MIN_SIZE}
                max={MAX_SIZE}
                value={width}
                onChange={(e) => setWidth(e.target.value)}
              />
            </label>
            <label>
              Height
              <input
                type="number"
                min={MIN_SIZE}
                max={MAX_SIZE}
                value={height}
                onChange={(e) => setHeight(e.target.value)}
              />
            </label>
          </div>
          {!valid && (
            <p className="muted hint">
              Width and height must be whole numbers between {MIN_SIZE} and {MAX_SIZE}.
            </p>
          )}
          <button type="submit" disabled={!valid}>
            Start drawing
          </button>
        </form>
      )}

      {selected &&
        !composing &&
        (confirming ? (
          <div className="stack">
            <p className="muted hint">
              Delete “{selected.name}” v{selected.version}? This cannot be undone.
            </p>
            <div className="row">
              <button
                type="button"
                className="danger"
                onClick={() => {
                  setConfirmingId(null);
                  void onDelete(selected.id);
                }}
              >
                Delete
              </button>
              <button type="button" onClick={() => setConfirmingId(null)}>
                Cancel
              </button>
            </div>
          </div>
        ) : (
          <button
            type="button"
            className="danger subtle"
            disabled={frozen}
            title={
              frozen
                ? 'Delete this grid’s routes first — they were planned against it'
                : undefined
            }
            onClick={() => setConfirmingId(selected.id)}
          >
            Delete this grid
          </button>
        ))}
    </section>
  );
}