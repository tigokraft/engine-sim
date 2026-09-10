# Working agreement

Rust real-time engine simulator: a 0D thermodynamic solver whose output is
synthesised into audio. `src/physics` solves the cycle, `src/audio` turns it into
sound, `src/ui` draws it, `src/bench` holds the engine catalogue.

The implementation roadmap lives in [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md).
Read the stage you are working on before touching code; each stage carries its own
prompt, file list, formulas and required tests.

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

## Code

- SI units everywhere unless the name says otherwise: metres, kilograms, seconds,
  Kelvin, Pascals, radians. Suffix non-SI names (`_deg`, `_rpm`, `_hz`).
- Physics solves in `f64`; the synth works in `f32`. The narrowing happens once,
  at the `EngineSnapshot` boundary, and nowhere else.
- Nothing in the audio callback may allocate, lock, block, or panic. Every buffer
  a voice will ever need is allocated at construction.
- New acoustic behaviour must come from geometry and gas state, not from a tone
  knob added to `SynthConfig`. If a constant has no physical name, it is a bug
  waiting to be argued about.
- Every new element gets a test asserting an analytic result, not a golden
  waveform: a closed-open pipe resonates at `c/4L`, a junction reflects
  `(A1-A2)/(A1+A2)`, a hotter pipe raises every resonance by `sqrt(T2/T1)`.
- Match the density of the surrounding comments. This codebase explains *why*, in
  prose, at the definition. Keep doing that.
- `cargo fmt` and `cargo clippy` clean before the commit.
