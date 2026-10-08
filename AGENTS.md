---
title: Agent Rules
audience: [human, ai]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# AGENTS.md — 2026Dreams / TitanRoboticsBuildSeason

FRC Team 8334 robot code. The GradleRIO project lives in `TitanRoboticsBuildSeason/` — run all Gradle commands from there, not the repo root.

## Build / run (Windows PowerShell, from `TitanRoboticsBuildSeason/`)

Must use the WPILib 2026 JDK or builds fail (`Unsupported class file major version`):

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
.\gradlew compileJava --offline   # fast compile
.\gradlew test --offline --no-daemon   # JUnit 5 suite (64 test files, 568 tests as of 2026-10-07 — green on clean re-run, see KNOWN_ISSUES.md §A)
.\gradlew simulateJava            # desktop SimGUI + IronMaple arena
.\gradlew deploy                  # deploy to RoboRIO (same JAVA_HOME)
```

- Always use the wrapper (`gradlew`/`gradlew.bat`), never system `gradle`.
- `--offline` avoids slow/failing network fetches against `C:\Users\Public\wpilib\2026\maven`.
- GradleRIO `2026.2.1`, Java 17 (`build.gradle`). `settings.gradle` resolves plugins from the local WPILib maven, not Maven Central.
- Single test: `.\gradlew test --tests "frc.robot.Intelligence.JevDecisionEngineTest"` (`test { forkEvery = 1 }`, so full suite is slow — prefer `--tests`).
- Stale `java` processes lock `build/jni` DLLs and `build/test-results` binaries: if `extractReleaseNative` fails or the build complains about undeletable dirs, find the PID that actually holds the file and kill **that PID only**, then re-run. **Never `taskkill /IM java.exe` and never `gradlew --stop`** — on a shared tree those kill other agents' live test runs and sims, which is the failure this repo's locking exists to prevent (see §Resource coordination). Daemon-locked runs can also produce a phantom one-off test failure — clean re-run (`--rerun-tasks`) is the tiebreaker before chasing a regression.
- Botched `generateBuildConstants` cannot corrupt the tree any more: it writes via a temp file plus an atomic move, and its up-to-date check requires the output to be structurally complete. A file truncated by a killed build now regenerates on the next build instead of persisting as a permanent compile error.

## Resource coordination

Multiple agents share **one** working tree, so `build/`, `.gradle/`, and a fixed set of WPILib network ports are single-tenant even though nothing used to enforce it. Guard the four resources in `tools/lock` before touching any of them:

| Resource | Hold it while running | Conflicts with |
|---|---|---|
| `gradle-build` | `compileJava`, `test`, `dumpSimLaunch` | itself |
| `sim-gui` | `simulateJava` / SimGUI | itself, `sweep` |
| `sweep` | `tools/score/sweep.ps1` | itself, `sim-gui` |
| `deploy` | `gradlew deploy` | itself |

```powershell
powershell -File tools/lock/status.ps1                          # who holds what
powershell -File tools/lock/acquire.ps1 -Resource gradle-build -Reason "full suite"
powershell -File tools/lock/release.ps1 -Resource gradle-build  # in a finally
```

- **Wait, then fail loudly.** A blocked acquire prints progress every 15 s and exits `3` naming the holder, its PID, its age and its stated reason. That is a *wait*, not a corruption — do not clear the way by killing anything. Re-run later, or work a different resource.
- **Never kill another agent's process to make room.** Not `taskkill /IM java.exe`, not `gradlew --stop`, not a wildcard `Stop-Process` over `java.exe`. Kill a specific PID you own, or use the lock.
- **Fan-out:** an orchestrator takes `gradle-build` once for the whole batch; workers run under it rather than each contending for it. Concurrent `gradlew` runs against one tree are not a tuning problem, they invalidate the result.
- **Fast path (use the wrappers, don't hand-roll):** `powershell -File tools/dev/check.ps1 -Resource gradle-build` probes JDK + dirty stamp + dashboard + lock in one shot (exit `0` ok, `1` env problem, `3` lock held; `-ForSweep` is strict). `powershell -File tools/lock/with-lock.ps1 -Resource <r> -Command "..."` holds the lock for exactly one command (the orchestrator-takes-once pattern above). `powershell -File tools/dev/verify.ps1` is the single automated verifier (roadmap + counts + docs-contract gates) — one human reviewer on top of it, 30-minute verifier TTL, no parallel-gate fan-outs that never close.
- A lock held by a dead process is reclaimed automatically and loudly (`[lock] STALE`). `status.ps1` reports a running Elastic/AdvantageScope/SimGUI too, which is legal for a GUI sim but contaminates a rig sweep (`KNOWN_ISSUES.md` §E).
- The lock is **advisory**: it protects agent-against-agent. A human in a terminal or a teammate in VS Code bypasses it, so a timeout is a reason to wait, never a reason to assume the path is clear.

Reasoning, the conflict matrix, and recovery recipes: `TitanRoboticsBuildSeason/docs/COORDINATION.md`.

## Generated code — do not hand-edit

- `src/main/java/frc/robot/BuildConstants.java` and `src/main/deploy/git_info.json` are generated by `generateBuildConstants` (runs on every `compileJava`).
- `.apt_generated/`, `.apt_generated_tests/`, `build/` — AdvantageKit `@AutoLog` (`akit-autolog`) + WPILib annotation output. Never edit; fix the source annotation instead.
- `lib/akit-autolog-26.0.2.jar` fallback: `build.gradle` uses the local jar if present, else Maven `akit-autolog:<version from vendordeps/AdvantageKit.json>`.

## Architecture (`src/main/java/frc/robot/`)

- Entrypoints: `Main.java` → `Robot.java` (extends AdvantageKit `LoggedRobot`, not `TimedRobot`). `Teleop.java` holds driver/operator bindings; `Auto/AutoMissionExecutor.java` + `Auto/Missions/` hold autonomous.
- Subsystems are singletons behind `SubsystemManager` (`Subsystems/` + `Telemetry/Dashboard`): `SwerveBase`, `Shooter`, `Intake`, `Vision`, `Dashboard`, `LEDs`, `MatchCoach`. Custom `Interfaces/Subsystem` interface — do not convert to WPILib `Subsystem`/`Command` patterns.
- Jev knowledge tiers: `Intelligence/MatchKnowledge` is a **sealed interface**, not one record with a flag — `ClairvoyantKnowledge` (sim operator's full picture, including fuel counts per field zone) vs `ObservedKnowledge` (real-robot sensor truth: zone counts are `0`, no opponent list). Read `docs/KNOWLEDGE_MODEL.md` before touching the decision layer; the reasoning is not obvious from the code.
- `Sim/LoopHealth` is armed only by a headless match and stamps loop timing into the score-rig JSONL. It is a measurement-validity gate, not robot behaviour — do not arm it on the real robot.
- Mode handling: `Data/Constants.getMode()` returns `REAL`/`SIM`/`REPLAY` (`RobotBase.isReal()`). `GameSim` + `AIRobotSim` only init in sim (`Robot.java:85-88`). Logger writes `.wpilog` on REAL, NT-only on SIM, replay via `LogFileUtil`.
- IO abstraction: each subsystem has Spark hardware vs Sim IO (AdvantageKit pattern). Keep hardware/sim branches paired.
- Vendor libs pinned in `vendordeps/`: YAGSL swerve, Phoenix 6, REVLib, Choreo, PhotonVision, AdvantageKit. Don't bump versions without checking Sim compat.

## Conventions that differ from defaults

- **Coordinates are Blue-origin only.** All field points defined for Blue (`X=0` at Blue wall); mirror with `Utils/AllianceFlipUtil.java` (`X_red = FIELD_LENGTH - X_blue`, Y unchanged; `FIELD_LENGTH`/`FIELD_WIDTH` come from `Navigation/FieldMap.java`). Never hardcode Red coordinates or maintain a parallel Blue/Red constant pair — derive the Red value. Documented exceptions: `FieldMap.Depots`, whose two loading bays are genuinely asymmetric on the real field, and the tower-post AABBs at `FieldMap.java:506-511`, which mirror both axes deliberately.
- **No console prints for driver alerts.** Use `Telemetry/Alert.java` + `AlertManager` (Elastic banner/tables) and `Subsystems/LEDs.java` patterns. `Robot` silences joystick warnings and disables LiveWindow intentionally — don't re-enable.
- **Tunables go through `Telemetry/TunableNumber.java` + `Telemetry/Dashboard.java`** (backs the 7-tab `elastic-layout.json`). Don't add raw SmartDashboard numbers for PID/constants; `Constants.TUNING_MODE = true` gates tuning.
- **Timing quirks in `Robot.java` are intentional:** 100 Hz odometry subloop (`addPeriodic(..., 0.010, 0.005)`), `System.gc()` in `disabledInit()`, coprocessor `PortForwarder` 5801–5805, Elastic layout `WebServer` on port 5800. Don't "clean these up."
- Key runtime rules encoded in code: shooter fires only inside the Alliance Zone, which is owned solely by `FieldMap.AllianceZones` (`BLUE_ZONE_MAX_X` = `BLUE_HUB_X` = 4.6256 m, `RED_ZONE_MIN_X` = 11.9154 m) — read it from there, never hardcode a zone edge. Intake arm Standby `347°` / Ground `250°`; kicker fires after flywheels within ±150 RPM.

## Testing / sim notes

- Tests live in `src/test/java/frc/robot/` mirroring package names (`Sim/`, `Auto/`, `Subsystems/`, `Telemetry/`, `Test/`, `Hardware/`, `Navigation/`, `Utils/`, `Data/`).
- Simulation stack: IronMaple swerve physics + `GameSim` (54 Fuel pieces) + `ShooterSim` + `AIRobotSim` (1–3 Jev AI opponents) + PhotonVision sim. Reset via Elastic `Simulation & Match Info` tab, not code changes.
- `bind() to port 1181 failed` warning in sim is non-fatal — ignore it.
- MapleSim `SimulatedBattery` is one static battery shared by all sim robots; `Robot.simulationInit()` disables it via `disableBatterySim()` (its own escape hatch). Do not remove — without it, multi-bot sim browns out and spams the console every sub-tick. `Robot.simulationPeriodic()`'s `BatterySim` model stays authoritative for RoboRIO voltage.
- Docs: `ARCHITECTURE.md` (subsystem contracts), `SIMULATION_GUIDE.md` (SimGUI/Elastic/AdvantageScope setup), `OPERATORS_GUIDE.md` (controller map), `src/main/java/frc/robot/Test/README.md` (TestMode/SysId). Trust `build.gradle`/code over prose when they conflict.
- Docs index: `TitanRoboticsBuildSeason/docs/INDEX.md` is the authoritative map. `docs/RESOURCES.md` supersedes `Resources.txt` + `docs_context/useful_documentation.txt` for new links. `docs/CHANGELOG.md` is agent-maintained. `.agents/teamwork/ARCHIVE.md` marks orchestration chatter as scratch.

## Docs Contract (strict — all agents)

1. **Reference before acting:** read `docs/INDEX.md` + the one topic guide for the task + root `KNOWN_ISSUES.md` before any code change. Check `docs/RESOURCES.md` before web search. Never cite `.agents/teamwork` scratch, `reports/`, or `build/` as spec.
2. **Update in the same change:** any behavior-affecting edit (runtime rules, NT keys, controls, scoring, sim physics, build commands, test counts) must also touch docs in the same commit: bump `last_verified` frontmatter, add a `docs/CHANGELOG.md` bullet with test evidence, and update `KNOWN_ISSUES.md` status tags (`[OPEN]`/`[PARTIAL]`/`[RESOLVED]`). Mechanical help, not optional judgment: `tools/dev/check-docs.ps1` fails a change with no `CHANGELOG` touch or no `INDEX.md` row for a new guide; `tools/dev/sync-test-counts.ps1` owns the suite numbers (measured XML, never hand-synced — it refuses partial results dirs).
3. **No duplication:** link to the single owning guide; don't paste the same paragraph into two files. New guides copy `docs/_TEMPLATE.md` (frontmatter: title, audience, owner, last_verified, status; body sections: Scope, Content, Verification, Related) **and are not finished until they have a row in the `docs/INDEX.md` durable-guides table** — `docs/KNOWLEDGE_MODEL.md` shipped linked from code and from `KNOWN_ISSUES.md` but missing from that table, so the link dangled until a later review caught it.
4. **Verify stamp:** `last_verified` = date code + docs were confirmed together (green build/test). Docs older than 30 days are `status: needs-review`. If the test suite is red, say so in the `## Verification` section rather than leaving a stale "N/N green" — a "green" line that predates a regression is worse than a red one, because it is trusted.
5. **Do not let a test rewrite stand in for a fix.** When a new test fails on arrival, check the *premise* first. Two of the four failures found Sep 28 were cleared by rewriting the test to pass explicit state — which was correct for those tests and silently deleted the only evidence for a real defect. Cite tests as evidence only after re-running them on the current binary.
6. **File what you find, not just what you fix.** When you discover a defect, trap, or non-obvious behavior — even one you fix in the same change — record it where the next agent will look: a `KNOWN_ISSUES.md` entry (`[OPEN]` with repro/file refs if unfixed, `[RESOLVED]` with the fix + test evidence if fixed), not just a code comment or chat message. Dead ends count too: one line ("tried X, measured Y, not worth pursuing") saves the next agent an hour. Behavior changes also need the rule-2 `CHANGELOG` bullet; pure learnings belong in `KNOWN_ISSUES.md` history or the owning guide. The Sep 28 episode is the cautionary tale: five tests went green and the only evidence for the real defect nearly vanished with them (rule 5).
