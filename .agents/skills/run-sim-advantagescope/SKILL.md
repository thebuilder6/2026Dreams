---
name: Run Sim with AdvantageScope
description: Headless sim gate then GUI SimGUI plus Elastic and AdvantageScope for TitanRoboticsBuildSeason, with auto-launch
---

# Run Sim with AdvantageScope

Use for desktop sim runs of the `TitanRoboticsBuildSeason` GradleRIO project.
Full setup lives in `TitanRoboticsBuildSeason/SIMULATION_GUIDE.md` — this skill
is the runnable shortcut, not a second copy.

## Preconditions

- Working directory: `TitanRoboticsBuildSeason/` (Gradle root, not repo root).
- WPILib 2026 JDK only:
  ```powershell
  $env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
  $env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
  ```
- Wrapper only (`.\gradlew.bat`), always `--offline`.
- Never hand-edit generated code: `src/main/java/frc/robot/BuildConstants.java`,
  `src/main/deploy/git_info.json`, `.apt_generated*/`, `build/`.

## Shared resources — acquire before you touch one

Other agents and a human may be using this tree. Every script in this skill
takes the appropriate lock itself, so the normal path needs nothing from you. If
you invoke `gradlew` **directly** rather than through these scripts, take the lock
yourself:

```powershell
powershell -File tools/lock/status.ps1     # first: is anyone already working?
powershell -File tools/lock/acquire.ps1 -Resource gradle-build -Reason "compileJava"
# ... run the build ...
powershell -File tools/lock/release.ps1 -Resource gradle-build   # in a finally
```

`gradle-build` for `compileJava` / `test` / `dumpSimLaunch`, `sim-gui` for a GUI
sim, `sweep` for a score-rig sweep, `deploy` for a RoboRIO deploy. A blocked
acquire waits, printing who holds it, then exits `3`. **That is a wait, not a
corruption — do not kill the holder.** Full rules: `AGENTS.md`
§Resource coordination and `docs/COORDINATION.md`.

## Tier 0 — Headless gate (CI-safe, no display)

Run `scripts/smoke-headless.ps1`, or manually:

```powershell
.\gradlew compileJava --offline
.\gradlew test --offline --tests "frc.robot.Sim.TrainingMatchScenarioTest" --tests "frc.robot.Sim.TrainingMatchScenarioApplicationTest"
```

This validates scenario inputs only — it does not run a live match.
The full suite is slow (`forkEvery = 1`); prefer `--tests`. See
`KNOWN_ISSUES.md` §A for the current test count and any tracked failures.
On `build/jni` lock or phantom one-off failure: kill **the specific PID holding
the file** (from `tools/lock/status.ps1` or the error), never `taskkill /IM
java.exe` and never `gradlew --stop` — on a shared tree those kill another
agent's live run. Then re-run with `--rerun-tasks` before chasing a regression.

## Tier 1 — GUI sim with auto-launch (needs display)

1. Launch: `scripts/launch-gui.ps1 [-RealDs]`, or `.\gradlew simulateJava --offline [-PrealDs]`.
   Run in background / separate window — it blocks. `-RealDs` loads
   `halsim_ds_socket` (the VS Code "Use Real DriverStation" checkbox,
   `build.gradle` `-PrealDs` flag, verified via `release_java.json`) so the
   real DS app drives match phases; SimGUI stays for sticks/NT view.
2. Script auto-launches Elastic and AdvantageScope when found on `PATH` or in
   default install locations. If missing, it prints manual steps and continues.
3. SimGUI: map controller to Joystick 0 (Joystick 1 = operator, Joystick 2 =
   manual opponent Bot 0). Enable Teleoperated or Autonomous.
4. Reset only via Elastic `Simulation & Match Info` tab (`Reset Sim` /
   `Respawn Balls`), never code edits.

## Elastic (auto or manual)

- Connect `127.0.0.1:5810`. Layout is served by the robot on port 5800
  (`Robot.java:robotInit` starts `WebServer`): `Ctrl+D` → Load Layout From
  Robot → `elastic-layout.json` → Full Reload.
- Source file: `TitanRoboticsBuildSeason/src/main/deploy/elastic-layout.json`
  (7 tabs; see `SIMULATION_GUIDE.md` §Tab map).

## AdvantageScope (auto or manual)

- Connect NetworkTables `127.0.0.1`.
- Preferred: File → Open Layout → `TitanRoboticsBuildSeason/advantagescope-layout.json`
  (real AdvantageScope schema: `hubs[].state.tabs.tabs`, sources as
  `NT:<path>`; player pose is the Field2d robot key
  `/SmartDashboard/Field/Robot`, set every update in `SwerveBase`).
- Manual fallback (`references/topics.md`): 3D Field `2026 Rebuilt`,
  poses `/SmartDashboard/Field` + `/RealOutputs/Pose`,
  pieces `Simulation/GamePieces`, arm `/Subsystems/Intake/ArmPose3d`,
  shooter `/Subsystems/Shooter/ShooterPose3d`.
- Logger modes (`Robot.java:63-78`): REAL writes `.wpilog` + NT,
  SIM is NT-only, REPLAY via `LogFileUtil`.

## Tier 2 — Practice match loop (sim bots + log review)

Semi-automated: enabling still needs a click in SimGUI; everything else is scripted.

1. Run `scripts/run-practice-match.ps1 [-MatchSec 170] [-Drill "..."] [-RealDs]`.
   It takes the `sim-gui` lock itself, so if another agent is running a sim it
   waits and then exits `3` naming the holder — do **not** stop that sim to make
   room. `-RealDs` is required for representative bot behavior (see step 2).
   It reuses Tier 0 as a preflight expectation (compile green), starts the sim
   with console tee'd to `TitanRoboticsBuildSeason/reports/sim-console-<ts>.log`,
   prints the checklist, waits for Enter after the match, stops the sim, and
   runs the reviewer.
2. Checklist: Elastic Opponent AI ON (1–3 bots), `Simulation/DebugAI=true`
   (otherwise the Bot0 STUCK pipeline stays silent). **Use the real Driver
   Station in Practice mode, not the SimGUI DS** — hub shifts run off the sim
   clock (`HubSchedule.java:187-195`), but only a real practice match gives
   proper auto→teleop phases and shift/hub timers. SimGUI-DS runs produce a
   degenerate pickup-1-or-2-and-shoot pattern that is a DS artifact, not a
   bot regression; do not judge behavior from them. Enable from the real DS
   and play the full match.
3. Review with `scripts/review-practice-log.py --log <capture> [--drill ...]`
   (stdlib only — `ntcore` is not installed and pip is offline, so no NT polling).
   Verdicts: exceptions/crashes → FAIL; STUCK dumps or brownout lines →
   ATTENTION; `bind() 1181` is INFO-only. Report lands in `reports/`.
4. Interpret stuck clusters via `KNOWN_ISSUES.md`: Hub/ramp-footprint targets →
   `DENY_SHOOTING_LANE` staging; trench corridors → trench transit/yield;
   any brownout line → `disableBatterySim` regression (must stay 0).
5. Scoring review still needs the Elastic scoreboard eyeball or
   `jev_coach.py --report` (mock data without ntcore — do not trust its numbers).

### Post-run `.wpilog` review (no new dependencies)

`scripts/review-wpilog.ps1 <file.wpilog>` compiles and runs
`scripts/LogReview.java` against wpiutil from the local WPILib maven
(offline-safe, javap-verified API only). Reports metadata, topic inventory,
DS enabled windows, per-bot identical-pose windows ≥1 s gated on DS-enabled
*and* non-DISABLED/IDLE StateDetail (window start times included), IsStuck
runs, StateDetail transitions, last Score/Fuel, and `exception` string hits.
Legacy single-bot `AI_Telemetry/*` (no BotN segment) is attributed to
`Bot0-legacy`. NT-conduit captures name topics with `NT:`/`AdvantageKit/`
prefixes — the reviewer normalizes all three shapes.

## Traps — do not "fix"

- **Never kill another agent's process to free a resource.** No `taskkill /IM
  java.exe`, no `gradlew --stop`, no wildcard `Stop-Process` over `java.exe`. A
  lock timeout is a wait; take a specific PID you own, or wait.
- `bind() to port 1181 failed`: non-fatal CameraServer collision, ignore.
- `SimulatedBattery.disableBatterySim()` in `simulationInit` must stay:
  MapleSim uses one static battery for all bots; without it multi-bot sim
  browns out and spams every sub-tick. `BatterySim` in
  `simulationPeriodic` stays authoritative.
- Coordinates are Blue-origin only; derive Red via `AllianceFlipUtil`
  (`FieldMap.AllianceZones` owns zone edges `BLUE_ZONE_MAX_X` 4.6256 /
  `RED_ZONE_MIN_X` 11.9154 — never hardcode).

## Verification

- Tier 0 green = skill gate passed.
- Tier 1 eyeball: robot drives in SimGUI, poses + fuel move in AdvantageScope,
  scores reconcile on Elastic Match Scoreboard tab.
- Vendor truth: `docs/RESOURCES.md` (docs.wpilib.org, docs.advantagekit.org,
  maple-sim, photonvision, elastic widget ref) before web search.
