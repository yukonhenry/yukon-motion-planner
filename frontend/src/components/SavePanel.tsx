interface Props {
    /** The grid has never been saved — confirming will create it. */
    unsaved: boolean;
    /** The working copy differs from what is stored. */
    dirty: boolean;
    /** Plans reference the saved grid, so an edit has to fork a new version. */
    frozen: boolean;
    /** Version of the saved grid, used to name the version a fork would write. */
    version: number;
    /** A local validation problem, or `null`. Blocks saving. */
    problem: string | null;
    pending: boolean;
    onSave: () => void;
    onDiscard: () => void;
}

/**
 * The confirm step.
 *
 * Nothing on the canvas reaches the database until this button is pressed — the point
 * being that a grid is saved once, whole, rather than accumulating a row per dragged
 * vertex. Which write it performs depends on the state of the grid it came from, and
 * the label says which, because "save" meaning "create v4 at a new id" is a surprise
 * worth spending a button label on.
 */
export function SavePanel({
                              unsaved,
                              dirty,
                              frozen,
                              version,
                              problem,
                              pending,
                              onSave,
                              onDiscard,
                          }: Props) {
    const action = unsaved
        ? 'Create grid'
        : frozen
            ? `Save as v${version + 1}`
            : 'Save changes';

    return (
        <section className="panel">
            <h2>{unsaved ? 'New grid' : 'Changes'}</h2>

            <div className="stack">
                {frozen && !unsaved && (
                    <p className="muted hint">
                        This grid has saved routes planned against the original set of obstacles. Saving writes
                        v{version + 1} as a
                        separate grid with a newly defined obstacle set and leaves v{version} intact with its obstacles
                        and routes.
                    </p>
                )}

                {!dirty && !unsaved && <p className="muted">No unsaved changes.</p>}

                {problem && <p className="status status--error">{problem}</p>}

                <div className="row">
                    <button
                        type="button"
                        onClick={onSave}
                        disabled={!dirty || pending || problem !== null}
                    >
                        {pending ? 'Saving…' : action}
                    </button>
                    <button type="button" className="subtle" onClick={onDiscard} disabled={!dirty}>
                        {unsaved ? 'Discard' : 'Revert'}
                    </button>
                </div>
            </div>
        </section>
    );
}