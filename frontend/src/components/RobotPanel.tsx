import {useState} from 'react';
import type {Robot, RobotInput} from '../types';

interface Props {
    robots: Robot[];
    pending: boolean;
    onSave: (input: RobotInput, editing: number | null) => void;
    onDelete: (robotId: number) => void;
}

/** Empty rather than `0`, so a fresh form does not read as a robot that cannot move. */
const NO_DRAFT = {name: '', velocity: '', interval: ''};

/**
 * Defining the fleet: a name, and how fast the machine may travel.
 *
 * `max_velocity` gets a field of its own while everything else in `capabilities` stays
 * free-form JSON, because it is the one capability the simulator actually reads — a run
 * advances the robot by this many cells per environment tick. A robot without it cannot be
 * simulated, which is why the server rejects it rather than defaulting.
 *
 * The form doubles as the editor: picking a robot loads it here, so a typo is fixable without
 * deleting and re-adding the machine every route already points at.
 */
export function RobotPanel({robots, pending, onSave, onDelete}: Props) {
    const [draft, setDraft] = useState(NO_DRAFT);
    const [editing, setEditing] = useState<number | null>(null);

    // Parsed once and reused: these decide both whether Save is live and what gets sent, and
    // the two must not disagree.
    const velocity = Number(draft.velocity);
    const interval = Number(draft.interval);
    const named = draft.name.trim().length > 0;
    const positive = (raw: string, value: number) =>
        raw.trim().length > 0 && Number.isFinite(value) && value > 0;
    const movable = positive(draft.velocity, velocity);
    const clocked = positive(draft.interval, interval);

    const reset = () => {
        setDraft(NO_DRAFT);
        setEditing(null);
    };

    const edit = (robot: Robot) => {
        setDraft({
            name: robot.name,
            velocity: String(robot.capabilities.max_velocity),
            interval: String(robot.capabilities.task_interval),
        });
        setEditing(robot.id);
    };

    const submit = () => {
        if (!named || !movable || !clocked) return;
        // Spread first so editing a robot keeps any capability this form does not know about
        // rather than dropping it — `PUT` replaces the whole spec.
        const existing = robots.find((r) => r.id === editing)?.capabilities;
        onSave(
            {
                name: draft.name.trim(),
                capabilities: {...existing, max_velocity: velocity, task_interval: interval},
            },
            editing,
        );
        reset();
    };

    return (
        <section className="panel">
            <h2>Robots</h2>

            {robots.length === 0 ? (
                <p className="muted hint">No robots yet. A run needs one to drive the route.</p>
            ) : (
                <ul className="plan-list">
                    {robots.map((robot) => (
                        <li key={robot.id} className={robot.id === editing ? 'selected' : ''}>
                            <button
                                type="button"
                                className="subtle plan-list__pick"
                                onClick={() => edit(robot)}
                            >
                                {robot.name}{' '}
                                <span className="muted">
                                    {robot.capabilities.max_velocity} cells/tick ·{' '}
                                    {robot.capabilities.task_interval}s
                                </span>
                            </button>
                            <button
                                type="button"
                                className="danger subtle"
                                onClick={() => onDelete(robot.id)}
                            >
                                Delete
                            </button>
                        </li>
                    ))}
                </ul>
            )}

            <div className="stack">
                <label className="row">
                    <span>Name</span>
                    <input
                        value={draft.name}
                        placeholder="Scout"
                        onChange={(e) => setDraft({...draft, name: e.target.value})}
                    />
                </label>
                <label className="row">
                    <span>Max velocity</span>
                    <input
                        type="number"
                        min="0"
                        step="0.1"
                        value={draft.velocity}
                        placeholder="1"
                        onChange={(e) => setDraft({...draft, velocity: e.target.value})}
                    />
                </label>
                <p className="muted hint">
                    Cells travelled each time the robot moves.
                </p>

                <label className="row">
                    <span>Task interval</span>
                    <input
                        type="number"
                        min="0"
                        step="0.05"
                        value={draft.interval}
                        placeholder="0.5"
                        onChange={(e) => setDraft({...draft, interval: e.target.value})}
                    />
                </label>
                <p className="muted hint">
                    Seconds between moves — the robot's own clock, separate from how often the
                    world's obstacles move.
                </p>

                {((draft.velocity.trim().length > 0 && !movable) ||
                    (draft.interval.trim().length > 0 && !clocked)) && (
                    <p className="muted hint">Both must be numbers greater than zero.</p>
                )}

                <div className="row">
                    <button
                        type="button"
                        onClick={submit}
                        disabled={!named || !movable || !clocked || pending}
                    >
                        {pending ? 'Saving…' : editing === null ? 'Add robot' : 'Save changes'}
                    </button>
                    {editing !== null && (
                        <button type="button" className="subtle" onClick={reset}>
                            Cancel
                        </button>
                    )}
                </div>
            </div>
        </section>
    );
}
