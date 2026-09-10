# Kilo Code

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
