---
title: Resource Coordination
audience: [human, ai]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# Resource Coordination

How concurrent agents (and agents racing a human) avoid corrupting each other's
builds, sims and measurements. The protocol every agent must follow is in
`AGENTS.md` under "Resource coordination"; this guide is the reasoning and the
recovery recipes.

## Scope

Covers the four shared resources below and the `tools/lock` module that guards
them.

Does **not** cover: correctness of the score rig itself (`ARCHITECTURE.md` 3L,
`docs/SCORE_RIG_RESULTS.md`), the knowledge model (`docs/KNOWLEDGE_MODEL.md`), or
intra-sweep parallelism, which `sweep.ps1` already owns via `-MaxWorkers`.

## Content

### The four resources

| Resource | Held during | Conflicts with | Why it must be exclusive |
|---|---|---|---|
| `gradle-build` | `compileJava` / `test` / `dumpSimLaunch` | itself | `build/test-results/*.xml` is clobbered by a concurrent run, and the "442 tests" baseline cited across `AGENTS.md`, `KNOWN_ISSUES.md` and `docs/CHANGELOG.md` is read out of those XML files. |
| `sim-gui` | `simulateJava` / SimGUI | itself, `sweep` | Owns NT4 5810, WebServer 5800, CameraServer 1181-1182 for the whole run. Hardcoded `public static final` constants with no environment override. |
| `sweep` | `tools/score/sweep.ps1` | itself, `sim-gui` | Same ports, plus a live GUI sim and a rig worker sharing one NetworkTables namespace is silent data corruption, not a bind error. |
| `deploy` | `gradlew deploy` | itself | One RoboRIO; last writer wins. |

The port analysis, and the verification that offsetting is impossible, are in the
header comment of `tools/score/sweep.ps1` (lines 17-50). Do not re-derive it here.

### Using it

```powershell
powershell -File tools/lock/status.ps1        # who holds what, and any dashboard
powershell -File tools/lock/acquire.ps1 -Resource gradle-build -Reason "full suite"
powershell -File tools/lock/release.ps1 -Resource gradle-build
```

Exit codes: `0` acquired, `3` timed out, `1` bad usage. Wrap protected work in
`try` / `finally` and release in the `finally`.

`-AnchorPid` exists because liveness is anchored on a PID. `launch-gui.ps1` is
the case that needs it: that script returns immediately while the sim keeps
running, so anchoring on its own process ID would free the lock instantly. Pass
the PID of the thing you actually care about.

### Waiting, and what "stale" means

A blocked acquire prints a line every 15 seconds, so the wait is visible rather
than a silent hang. On timeout it throws naming the holder, its PID, how long it
has held the lock, and its stated reason.

Staleness is judged **only** on dead PIDs, never on elapsed time. A heartbeat
timeout would be actively wrong here: a 4-wide sweep or a 170 second practice
match legitimately holds its lock for many minutes, and killing it on a timer
would reproduce exactly the interrupt this module exists to prevent. A dead PID
is the only trustworthy signal, because when an agent dies its PIDs are gone.
See `.agents/teamwork/sentinel/BRIEFING.md`, where an orchestrator was terminated
on quota exhaustion mid-task.

Liveness is PID **plus** process start time. A bare PID is not enough: Windows
recycles PIDs, and a dead sim whose PID had been reassigned to an unrelated
process would look alive forever and wedge the lock.

### Orchestrators

An orchestrator that fans out N workers should take `gradle-build` **once** for
the whole batch and let the workers run under it, rather than having each worker
contend for the lock. Concurrent `gradlew` invocations against one tree are not
a performance problem to be tuned; they invalidate the result. The wrapper is
`tools/lock/with-lock.ps1 -Resource <r> -Command "..."`, which holds the lock
for exactly one child command and releases it in a `finally` — probe first with
`tools/dev/check.ps1 -Resource <r>`.

Verification is one automated gate plus one human, never a parallel fan-out of
gates: `tools/dev/verify.ps1` owns the mechanical checks (roadmap, counts,
docs-contract, optional rig-schema), and a single reviewer owns judgment with a
30-minute TTL. Five parallel verifiers with no owner produced zero verdicts on
Sep 25; do not repeat that shape.

### Recovery recipes

| Symptom | Cause | Action |
|---|---|---|
| `BuildConstants.java` does not compile, truncated mid-file | before 2026-09-29, `generateBuildConstants` wrote non-atomically and `upToDateWhen { exists() }` then skipped it forever | Fixed. The task now writes via a temp file plus `ATOMIC_MOVE` and validates that the file is structurally complete, so a truncated file self-heals on the next build. Just rebuild. |
| `extractReleaseNative` fails, or undeletable directories under `build/` | a JVM holds a DLL | Kill **the specific PID that holds it**, from `status.ps1` or the reported PID. Never `taskkill /IM java.exe`; that is another agent's live run. |
| `[lock] STALE: reclaiming ...` on acquire | the previous owner died holding the lock | Expected and self-healing. The warning names the dead owner. |
| A lock looks HELD but nothing is running | the anchoring process is a still-open sim window or a `-NoExit` shell | Close it, or run `release.ps1` if you are the owner. |
| A sweep is degraded, or `NT3/NT4 server socket error` | something outside the lock protocol holds a port, usually a human's Elastic or a raw `./gradlew simulateJava` | `status.ps1` reports dashboards. Close it. The lock is advisory and a human bypasses it. |

### What the lock does not protect

It is **advisory**: it protects agent against agent only. A human running
`./gradlew test` in a terminal, or a teammate in VS Code, bypasses it entirely.
Against that case what actually holds is the dashboard check in `sweep.ps1` and
the non-destructive shutdown in `run-practice-match.ps1`. Treat a lock timeout as
"someone is busy", never as "clear the way".

## Verification

- Lock module exercised directly on 2026-09-29 (PowerShell 5.1.26100.9549):
  exclusive-create blocks a second acquirer; a timeout exits `3` naming the
  holder and does **not** steal the lock; a stale lock from a dead owner is
  reclaimed with a warning; a forged lock naming a live PID with mismatched
  start ticks reads `STALE`; `sim-gui` blocks `sweep` while `gradle-build` and
  `deploy` stay independent; a re-entrant acquire by the same process does not
  self-deadlock; `release.ps1` refuses to free another owner's lock.
- `generateBuildConstants` self-heal verified 2026-09-29: both output files were
  truncated to 120 and 1 bytes, then `compileJava --offline` regenerated them
  (583 and 191 bytes) with no stray temp file. Under the previous `exists()`
  check the task was skipped and the corruption persisted.
- `smoke-headless.ps1` run end to end under the lock: `BUILD SUCCESSFUL`, lock
  released, no leftover lock files.
- Full suite on the current binary, after all of the above: **44 files / 442
  tests / 0 failures** (`--offline --no-daemon --rerun-tasks`). The count is
  measured, never inferred: `build/test-results/test/*.xml` sums to 442 across 44
  files, and an on-disk count of `@Test` annotations agrees at 442. Note that a
  PowerShell `**` glob undercounts this badly, which is the trap
  `KNOWN_ISSUES.md` warns about — count from the XML.
- **The lock prevented a real collision, and it was worth having.** During the
  nav work a background full-suite run and a foreground targeted run overlapped
  and the second died with `Unable to delete directory
  build\test-results\test\binary\output.bin` — exactly the shared-output clobber
  this resource exists to prevent. Every test class then failed as
  "could not execute", which reads like a catastrophic regression and was not
  one: each class passed individually, and a single clean re-run was green. Two
  lessons for the next agent: hold `gradle-build` for the whole verification
  window, and if a run dies on an undeletable `build/` directory, re-run once
  before investigating.
- **The advisory gap is real and was hit in practice.** A 150 s headless match
  (pid 1528, `-Dfrc.headless=true -Dfrc.headless.durationSec=150`) was running
  with `sweep` reported **free** — it had been launched without taking the lock.
  It held the `build/jni` natives (so `extractReleaseNative` failed), bound
  ports 1181/1250, and the concurrent load crashed a test JVM with
  `-1073741819` (access violation), which surfaced as a red suite rather than
  the environmental problem it was. **The correct response was to wait, not to
  kill it** — that PID was another agent's live match, and killing it is exactly
  the interrupt this module exists to stop. It exited on its own after 167 s and
  the next run was green. If a run fails on `extractReleaseNative` or a
  `non-zero exit value -1073741819`, check for a foreign headless match *before*
  concluding anything about the code.
- **`--rerun-tasks` is not a usable verification form while a sweep is running,
  and the reason is sharper than "something holds a handle".** It forces
  `extractReleaseNative` to re-copy the JNI natives, and the actual failure is
  `build\jni\release\wpiHal.dll … being used by another process` — the running
  match has that DLL mapped. Waiting for one match to exit is **not** sufficient
  when a sweep is cycling matches: on 2026-09-29 a second match
  (`-Dfrc.headless=true`, pid 38184) was already 16 s old when the first one
  exited, so there was no window in which re-extraction was safe, and re-running
  failed identically three times. Forcing it would mean killing another agent's
  match, which is the interrupt this module exists to prevent. **Use
  `.\gradlew build --offline --no-daemon` for verification**; reserve
  `--rerun-tasks` for a genuinely idle machine, and read
  `extractReleaseNative FAILED` as a contention signal rather than a build
  break.
- Not yet verified: two agents actually racing each other in one session by
  design. The mechanism is tested directly as listed above, but the end-to-end
  multi-agent case has not been exercised, which is why the `AGENTS.md` rule is
  advisory.
- Dev-gate tooling verified 2026-10-07 (no lock needed; all read-only plus new
  files under `tools/dev/`): `check.ps1` stamped a dirty tree (29 modified + 7
  untracked), a STALE `sim-gui` lock, running AdvantageScope processes, and an
  unset `JAVA_HOME`, each with an actionable message; `with-lock.ps1`
  acquired/ran/released `deploy` and left it free; `verify.ps1` reported 2 PASS
  (roadmap 106/106, docs-contract) and 1 loud FAIL when `sync-test-counts.ps1`
  refused a 1-file `build/test-results` snapshot being written by a concurrent
  suite run instead of syncing garbage numbers; `check-docs.ps1` passed the
  working tree with the expected last_verified warning on another agent's
  in-flight change.
- Next review due: 2026-10-29.

## Related

- `AGENTS.md` "Resource coordination" - the protocol every agent must follow.
- `tools/lock/Lock.psm1` - implementation, including the conflict table.
- `KNOWN_ISSUES.md` sections A and E - the port, dashboard and loop-health
  findings that motivated the resource split.
- `ARCHITECTURE.md` section 3L - score measurement rig contract.
