# Gemini

Canonical working agreement: [AGENTS.md](AGENTS.md).
Implementation roadmap: [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md).

The rule below is repeated here in full because it is not optional.

## Commit discipline (mandatory, applies to every model and every tool)

- **Commit as you work.** A change that is finished is committed. Do not leave
  finished work uncommitted, and do not wait to be asked — committing is part of
  doing the work, not a step after it.
- **One commit per change.** If describing the commit needs the word "and", it is
  two commits. A new type, its tests and its wiring are separate commits.
- **One line per commit message.** No body. No bullet list. No blank-line-then-
  paragraph. No trailing period.
- **Brief and imperative.** Lower case, under ~60 characters:
  `add knock voice`, `fix runner delay clamp`, `test junction reflection`.
- **Never add attribution trailers.** No `Co-Authored-By:` line, ever, for any
  model or provider. No "Generated with", no tool name, no emoji. The commit
  author is the repository owner and nobody else.
- **Never amend or rebase** a commit that is already made unless explicitly asked.
- **Never `git push`** unless explicitly asked.
- **Never `git add -A` / `git add .`** — stage the exact paths for that one change.

Good:

```
add knock resonance voice
tune knock mode from bore and gas temperature
test knock pitch scales inversely with bore
```

Bad:

```
Add knock voice and wire it into the mix, plus tests

- adds KnockVoice
- wires into EngineSynth::tick

Co-Authored-By: Some Model <noreply@example.com>
```

## Finishing a stage (mandatory, applies to every model and every tool)

A stage in [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md) is a
contract, not a suggestion. It is finished when **every commit in its Commits
list exists and every bullet in its Tests section is a real test that runs**.
Not when the new module compiles.

- **Tick the list.** Before saying a stage is done, read its Commits list and
  its Tests list back and check each line against `git log` and the test names.
  The Tests section is usually longer than the Commits list; both are required.
- **New code must have a caller.** A type that nothing in the shipping path
  constructs and calls is not finished work, it is scaffolding. Wiring it in is
  part of the same stage, and the stage is not done until the old thing it
  replaces is gone from the path.
- **No dead ports.** Do not allocate a field, a buffer or a pipe the code never
  reads, and do not pass a hardcoded `0.0` where a real signal belongs. Either
  connect it or leave it out.
- **Handle every variant.** When you match on a geometry enum, handle all of it.
  Silently dropping a variant makes an engine that has one go quiet, and nothing
  will fail to tell you.
- **Every commit builds and tests green.** Stage the files a change actually
  spans, all of them, in one commit. A commit that only builds once the *next*
  one lands is a broken commit.
- **Say what you did not do.** If you stop early, run out of room, or decide a
  bullet is wrong, say so explicitly in your final message and leave the plan
  alone. Silence reads as completion, and the next person believes it.
