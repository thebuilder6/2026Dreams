---
title: Simulation Setup
audience: [human, ai]
owner: sim-owner
last_verified: 2026-10-06
status: authoritative
---

# 🎮 2026 Desktop Simulation Setup & User Guide

Welcome to the **Titan Robotics 2026 Simulation Environment**. Our simulation stack provides a complete, high-fidelity virtual proving ground for testing swerve drive kinematics, ballistics shooter trajectories, intake mechanisms, autonomous pathfinding, and AI opponent sparring without requiring physical robot hardware.

---

## 🏗️ Simulation Architecture & Capabilities

The simulation environment integrates several real-time physics engines and telemetry streams:

| System | Simulation Engine | Capabilities |
| :--- | :--- | :--- |
| **Swerve Drivebase** | **IronMaple 2D Physics** | Rigid-body swerve kinematics with wheel slip, realistic carpet friction, inertia, and bumper-to-bumper collision dynamics. |
| **Field & Game Pieces** | **`GameSim` + IronMaple** | 54 dynamic Fuel game pieces (12 Blue, 12 Red, 30 Neutral Midfield) with ground collection, hopper capacity limits, scoring detection, and respawning. |
| **Dual Flywheel Shooter** | **`ShooterSim`** | Physics-based projectile trajectory math incorporating dual flywheel slip efficiency ($\eta = 0.42$), launch angle, and 3D parabolic flight into the Hub goal. |
| **Ground Intake Arm** | **`ArmSim` + DC Motor Sim** | Trapezoid-profiled arm pivot with gravity feedforward, ground plane collision damping, and roller ingestion volume. |
| **AI Opponent Robot** | **`AIRobotSim` + Jev AI** | Autonomous sparring partner capable of tactical shooting lane denial, lead pursuit interception, midfield shadowing, aggressive bumper pinning, or 2-player manual control. |
| **Vision & AprilTags** | **PhotonVision Sim + Rubik Pi Sim** | Desktop simulation of multi-camera AprilTag pose estimation and YOLO neural network ball detection ("Ball Hunt"). |
| **Electrical System** | **WPILib `BatterySim` & `RoboRioSim`** | Dynamic battery voltage sag calculation based on instantaneous current draw summed across all subsystems. Note: MapleSim's own static `SimulatedBattery` is disabled in `Robot.simulationInit()` (`disableBatterySim()`) to avoid multi-bot brownout spam; the `BatterySim` model stays authoritative. |

---

## 📋 Prerequisites

Before launching the simulation, ensure you have the following installed:

1. **WPILib 2026 Suite**:
   - Contains the required Java 17 JDK (located at `C:\Users\Public\wpilib\2026\jdk` on Windows) and the WPILib Simulation GUI (`SimGUI`).
2. **Game Controller** *(Recommended)*:
   - Xbox 360 / Xbox One / Xbox Series X controller (or Logitech F310 in XInput mode).
   - Alternatively, you can use keyboard-mapped joysticks within the WPILib SimGUI.
3. **Visualization Tools**:
   - **[Elastic Dashboard](https://github.com/Gold872/elastic-dashboard)**: Driver HUD, diagnostic scorecards, SysID bench, and Tuning & PID interface.
   - **[AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)**: 3D field rendering, robot poses, arm articulation, and moving game pieces.

---

## 🚀 Launching the Simulation

### Option A: From VS Code (Recommended)

1. Open the project root folder (`TitanRoboticsBuildSeason`) in VS Code.
2. Press `Ctrl + Shift + P` (or `Cmd + Shift + P` on macOS) to open the Command Palette.
3. Type and select **`WPILib: Simulate Robot Code on Desktop`** (or press `F5`).
4. If prompted to select simulation extensions, check **`Sim GUI`** and click **OK**.

### Option B: From Terminal / PowerShell

Run the following commands in PowerShell from `TitanRoboticsBuildSeason/`:

```powershell
# Set Java environment to the WPILib 2026 JDK
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:Path = "$($env:JAVA_HOME)\bin;$($env:Path)"

# Launch simulation
.\gradlew simulateJava
```

The **WPILib Simulation GUI (SimGUI)** window will appear automatically.

---

## 🕹️ Configuring Controllers in SimGUI

To control the robot with a physical gamepad:

1. In the WPILib SimGUI window, locate the **`System Joysticks`** panel on the left.
2. Find your connected controller (e.g., `Xbox Controller (XInput...)`).
3. Drag and drop it into **`Joystick 0`** under the **`Joysticks`** panel for the **Driver Controller**.
4. *(Optional)* Drag a second controller into **`Joystick 1`** for the **Operator Controller**.
5. *(Optional)* Drag a third controller into **`Joystick 2`** to manually pilot the **Opponent AI Robot** in 2-Player sparring mode.

> [!TIP]
> **No Gamepad?** In SimGUI, you can assign keyboard keys to axes and buttons by selecting **Joysticks -> Keyboard 0** and mapping keys (e.g. WASD for Left Stick, Arrow keys for Right Stick).

---

## 📊 Connecting Elastic Dashboard

### Optional TypeSafe decisions for simulator bots

The `Features/Use TypeSafe Jev AI` toggle applies to simulator sparring bots as well as the player Co-Pilot. Set `TYPESAFE_API_KEY` in the environment that launches Gradle (or set the `typesafe.api.key` JVM property), enable the toggle, and choose `TYPESAFE_CLOUD` or `AUTO_FALLBACK` in `JevAI/DecisionMode`. With the toggle off, bots remain fully local. Each bot keeps its own asynchronous request and decision state and dispatches at most once per second. A shared FIFO dispatcher spaces requests by 150 ms and replaces queued bot snapshots with the latest state. Bot telemetry is under `JevAI/Sim/<bot path>`. Local eligibility, confidence, freshness, and safety checks still gate every cloud choice. There is no per-match request or spending cap; API use can incur charges, so leave the toggle off for offline runs.

Elastic Dashboard is pre-configured with 7 tabs (Driver Dashboard, AI Coach & Practice, Pre-Flight Diagnostics, SysID & Characterization, Simulation & Match Info, Match Scoreboard, Tuning & PID) adhering strictly to the official [Elastic Widget Reference](https://frc-elastic.gitbook.io/docs/additional-features-and-references/widgets-list-and-properties-reference).

### Instant Setup via Remote Layout Downloading (Recommended)
Our robot code serves the official layout directly over HTTP port 5800 (`edu.wpi.first.net.WebServer`):
1. Launch **Elastic Dashboard**.
2. Connect to the robot / simulation (`127.0.0.1` on port `5810`).
3. Press **`Ctrl + D`** (or go to **File -> Load Layout From Robot**).
4. Select `elastic-layout.json` and choose **Full Reload** (or **Overwrite**).
5. Elastic will pull the exact, validated layout directly from the robot deploy directory!

### Manual Setup (Alternative)
1. In Elastic Dashboard, click the layout dropdown (or File menu) and select **Open Layout File**.
2. Open `TitanRoboticsBuildSeason/elastic-layout.json` (or `src/main/deploy/elastic-layout.json`).

### Widget Features Across Tabs
- **Tab 1: Driver Dashboard**: Dedicated `Match Time` countdown clock (red at 15 s, yellow at 30 s per `elastic-layout.json:42-43`), 2D `Field` widget (`/SmartDashboard/Field`), live Hub active indicator, `Graph` widget displaying live Flywheel RPM response, held fuel `Number Bar`, and clickable `Toggle Switch` controls for Snap Turn, Auto Aim, Ball Hunt, Glide Points, and Slow Mode.
- **Tab 2: AI Coach & Practice**: Real-time driver grading ($A+$ to $D$), cycle timing bars, shooting accuracy bar, drill mode chooser, `Toggle Button` for 1-click arena reset, `Toggle Switch` for haptic collision rumble, and Jev AI coaching directives.
- **Tab 3: Pre-Flight Diagnostics**: Automated 15-second scorecard with progress bar and individual `Toggle Button` widgets to pulse each swerve steer/drive motor, intake arm, intake rollers, and flywheels.
- **Tab 4: SysID & Characterization**: `Toggle Button` for Quasistatic / Dynamic Forward / Reverse and ABORT / E-STOP, with real-time `Graph` widgets for live applied voltage and velocity response waves.
- **Tab 5: Simulation & Match Info**: Dropdown menus for Opponent Count (1, 2, or 3 bots) and per-bot Archetypes (Bot 0 Lead, Bot 1 Bully, Bot 2 Adaptive), Ally Bots Active toggle with Ally Count (0–2) and Ally Speed slider, interactive `Number Slider` for opponent speed (20-100%), interactive `Toggle Button` controls for Sim Reset and Respawn Balls, multi-bot state/score telemetry, and `Toggle Switch` for Opponent AI.
- **Tab 6: Match Scoreboard**: Live red/blue totals, Leader, auto/teleop fuel splits, per-robot balls, foul points, and climb status — see `OPERATORS_GUIDE.md` §6 for the widget map.
- **Tab 7: Tuning & PID**: Flywheel dual-RPM bars, pivot arm setpoint/goal bars, interactive `Toggle Switch` settings, and text displays with `show_submit_button: true` to edit PID constants live.

---

## 🔭 Connecting AdvantageScope (3D Visualizer)

AdvantageScope gives you a live 3D rendering of the arena, robot, articulated mechanisms, and game pieces:

1. Launch **AdvantageScope**.
2. Select **File -> Connect to NetworkTables**.
3. Set the address to `127.0.0.1` and connect.
4. **Configure 3D Field**:
   - Open a **3D Field** tab.
   - Select the field model: **2026 Rebuilt** (or 2024 Crescendo as fallback).
   - Under **Robot Poses**, add `/SmartDashboard/Field` or `/RealOutputs/Pose`.
   - Under **Game Pieces**, add `Simulation/GamePieces` (`SimDashboardKeys.java:120`; Logger `FieldSimulation/Fuel` at `GameSim.java:316`) to see Fuel balls (54 standard lightweight layout; higher `fieldFuelCount` draws a seeded subset up to `TrainingMatchScenario.MAX_FIELD_FUEL_COUNT` 384 of the full-density preplaced positions).
5. **Configure Mechanism 3D**:
   - Add `/Subsystems/Intake/ArmPose3d` to observe the intake arm rotating between standby ($347^\circ$) and ground ($250^\circ$).
   - Add `/Subsystems/Shooter/ShooterPose3d` to visualize the shooter flywheel angle and position.

---

## 🎮 How to Test & Operate in Simulation

### Training scenario input validation

The current training scenario API applies a match duration, a seeded subset of the existing preplaced depot/midfield fuel positions, and every robot's archetype, starting pose, and preload. Blue slot 0 is an independent `AIRobotInstance`; later Blue slots use the ally pool and Red slots use the opponent pool. The physical `SwerveBase` is parked off-field during the scenario and restored when it is cleared.

### 3v3 AI-vs-AI training matches

Set the seed via the `Simulation/Training/Seed` NT key (default 2026, `GameSim.java:233`) and start/stop via `Simulation/Training/Start3v3|Stop` (`SimDashboardKeys.java:109-111`). Note: the shipped `elastic-layout.json` has no widget bound to these keys — set them from code, NT, or the coaching script until a layout widget lands. This applies `TrainingMatchScenario.default3v3` (3 Blue + 3 Red on staggered lanes, 8-fuel preloads, 150 s, 54 fuel) and parks the player `SwerveBase`. `Simulation/Training/Stop` clears back to interactive defaults. Training bots only drive while the DriverStation is enabled (clean start/stop); run Autonomous 15 s then Teleoperated as usual and the scenario clock, Hub schedule seeding, scoring, and referee all follow. When the clock expires the scoreboard latches to `Training/Result/*` (Winner, Blue/RedScore, Margin, AUTO/TELEOP splits, climb).

### Headless 3v3 matches (no GUI, replayable)

`Sim/HeadlessMatchDriver.java` runs the same 3v3 through the real robot loop with a self-driving DS sequence (AUTO → disabled gap → TELEOP until the scenario clock expires), then writes a replayable `.wpilog` and a markdown report and exits. No SimGUI, gamepad, or clicks required:

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
.\gradlew simulateJavaRelease --offline -Pheadless [-Pseed=2026] [-PdurationSec=150] [-PautoSec=15]
```

Options (`-Pkey=value`): `seed` (default 2026), `durationSec` (150), `autoSec` (15), `disabledGapSec` (3), `bootWaitSec` (8), `fieldFuelCount` (108 loose field pieces + 6×8 preloads = 156 total), `logDir` (`logs/`, gitignored), `reportDir` (`reports/`), `resultJsonl` (unset = off; appends one JSON object per match), `snapshotCards` (unset = off; appends sampled decision-card TSV rows — objective transitions, stall onsets, shift edges, 30 s floor, max ~12/match; merge into `decision_cards.tsv` by hand after review), `variant` (default `baseline`), `replica` (default 0). Counts above the 54-ball lightweight layout draw a seeded subset from the full-density preplaced positions (`GameSim`). A 30 s smoke (`-PdurationSec=30 -PautoSec=5`) finishes in ~50 s wall clock. Replay: AdvantageScope → File → Open Log → `logs/headless_3v3_seed<seed>_<stamp>.wpilog` (per-bot poses/objectives, `FieldSimulation/Fuel`, `Headless/*` phase + scoreboard, existing `JevAI/*` / `Trajectory/*` / `Scoreboard/*`). Lifecycle note: `Robot.autonomousInit` uses `AIRobotSim.resetForMatchStart()` so enabling auto re-applies scenario spawns instead of wiping bots to queuing poses.

> **A 150 s scenario clock is mandatory for a valid score sample.** `HubSchedule`'s phase table is hardcoded to 130/105/80/55/30 s remaining, and `MatchScoreTracker.updateClimbEvaluation` zeroes every climb whenever `20 < matchTime <= 150`. A 30 s smoke therefore lands straight in ENDGAME, never exercises a shift, and scores zero climb — fine for "does it run", useless as a fitness value.

Every report now ends with a reconciliation line (`Blue per-bot N vs total M (residual R, unattributed U)`) and prints **`RECONCILIATION FAILED`** if either residual is non-zero. That means a scoring path is inflating an alliance total that the per-bot table cannot account for. It was non-zero in 13 of the 20 archived reports before the Sep 28 fix — see `KNOWN_ISSUES.md` §B.

### Score rig (parallel sweeps + paired comparison)

For "did this change help?" questions, run a grid instead of one match. Never parse the markdown report: the old sweep script split rows on `|` and silently produced plausible-but-wrong columns (which is how the attribution leak above was found). Read the JSONL.

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"

# 8 seeds x 2 replicas = 16 matches, 4-wide parallel by default, resumable
powershell -File tools\score\sweep.ps1 -Seeds 7,11,42,101,500,1337,2026,9999 `
    -Variants baseline -Replicas 2 -OutFile results\baseline.jsonl -Fresh

# How repeatable is a single score? (two runs of identical code)
python tools\score\compare.py --results results\baseline.jsonl --noise-floor

# Compare a variant against the baseline
powershell -File tools\score\sweep.ps1 -Variants baseline,mystery -Seeds 7,11,42,101,500,1337,2026,9999 `
    -OutFile results\grid.jsonl
python tools\score\compare.py --results results\grid.jsonl
```

- `sweep.ps1` gets its launch recipe from `gradlew dumpSimLaunch`, which reflects over the real `simulateJavaRelease` JavaExec — so the classpath, `-Djava.library.path`, and main class cannot drift from what Gradle would run. **`-Djava.library.path` alone is not enough:** WPILib's `HAL_LoadExtension` goes through the OS loader, which searches `PATH`, so the recipe also emits a `pathPrefix` the script prepends. Without it every worker dies with `Unable to find wpi driver binary` then `EXCEPTION_UNCAUGHT_CXX_EXCEPTION`, even though all natives are present.
- It uses `Start-Process`, not `Start-ThreadJob` (needs PowerShell 6+) or `Start-Job` (a per-worker process cannot share a queue).
- Rows already present for a `(variant, seed, replica)` are skipped, so an interrupted sweep is re-run rather than restarted.
- `compare.py` exits non-zero when a variant regresses or trips a guardrail, so it works as a gate. Guardrails are **role-aware**: a defender is designed not to score (`JevDecisionEngine` zeroes `scoreUtility` for `TACTICAL_DEFENDER` / `DEFENSE_BULLY`), so defenders are gated on distance travelled, not fuel share. The role comes from the sim's reported archetype, never inferred from scoring.
- **There is nothing to sweep yet.** Every Jev utility weight is a hardcoded literal in `JevDecisionEngine.evaluatePolicy`; see `KNOWN_ISSUES.md` §E for the `PolicyWeights` seam that unblocks it.
- **Parallel workers still contend for NT3 1735 / NT4 5810 / CameraServer 1181-1182.** `Robot` no longer starts the WebServer (5800) or coprocessor `PortForwarder` (5801-5805) when `frc.headless` is set, but the rest is WPILib-internal. Under load you will see `NT3/NT4 server socket error: address already in use` and `Loop time of 0.02s overrun` in `logs/sweep/*.err.log`. **Width is measured, not guessed (corrected Sep 28):** at 60 s matches, **2-wide and 4-wide are clean (0 overruns, 4-11 ms max), 6-wide is not (2-13 overruns, 37-126 ms)**, and the archived 12-wide batch had 27-53 overrun warnings per match. `-MaxWorkers` therefore defaults to **4**. Use `-MaxWorkers 1` only when 4 is also dirty on your machine — the old advice to "always use `-MaxWorkers 1`" predated the loop-health measurement and is both slower and less accurate about the actual limit. The rig now **fails the sweep** on a degraded row rather than letting you judge by eye, so a too-wide run is loud, not silent. Measured evidence and the two separate variance causes: `docs/SCORE_RIG_RESULTS.md` §4.

### Jev decision cards (no match required)

The score rig above needs a 150 s match per sample, and the headless 3v3 is currently too noisy to resolve a policy change (see `docs/SCORE_RIG_RESULTS.md`). The **decision layer** has no such problem: `JevDecisionEngine.evaluatePolicy` is a pure function of `(WorldState, MatchKnowledge, Archetype)`, so it can be exercised in milliseconds with no physics, no seeds, and no variance.

```powershell
.\gradlew jar --offline
powershell -File tools\score\run-cards.ps1        # -> results\decision_cards.md
```

- **39 cards ship** (re-run 2026-09-28; the "45" previously quoted here, in `KNOWN_ISSUES.md` §E and in `docs/CHANGELOG.md` was wrong) covering the batch threshold ladder, both documented priority inversions, inventory-full, lane-blocked, endgame climb, poach/shuttle/screen, defender behaviour, next-shift awareness, and a full timeline walk (TRANSITION → SHIFT1-4 → ENDGAME).
- **Every card is evaluated three times**: as Blue clairvoyant, as mirrored Red clairvoyant, and under the **observed** tier. The verdict is judged against the Blue pass. CO_PILOT cards are single-pass (it is the player-facing archetype, not a sparring bot).
- **Alliance symmetry holds** — every card's Blue and Red passes agree, i.e. the utility matrix is alliance-symmetric. Cards whose two passes disagree are flagged **ALLIANCE ASYMMETRY**.
- **6 of the 39 cards pick a different objective under the observed tier**, and five of those are defender cards where the clairvoyant answer is a defensive objective and the observed answer is `VACUUM_MIDFIELD`/`STAGE_STANDOFF`. That is the honest tier doing what it should *and* the defensive policy disappearing as a result — see `docs/KNOWLEDGE_MODEL.md` and `KNOWN_ISSUES.md` §A. Read the `TIER DIFFERS` rows before tuning anything defender-related.
- **`0 PASS / 0 MISMATCH / 39 UNREVIEWED`** — the `expected` column is blank for every card, so the tool is currently a report, not a gate. One card (`Z99`) is `INVALID STATE` **on purpose** (it is the canary for the schedule cross-check — do not fix it).
  - The subtlety: flipping the alliance in a *fixed* phase is **not** a mirror. The seed decides which alliance sits out, so with the default seed `'R'`, Blue is live in SHIFT1 and Red in SHIFT2. Swapping only the hub flags compares SHIFT1 against SHIFT2 and reports 18 false asymmetries — which is exactly what the first version of this tool did before it was caught.
- Each card prints the chosen objective for both passes, the rationale, the full `AIActionIntent` side by side, and a **held-fuel sweep** with a two-column Blue/Red comparison. Thresholds and inversions show up as visible steps, so you do not need to already suspect a bug to see it.
- **Every card is validated against the hub schedule.** `hubActive` / `oppHubActive` / `timeToShift` are treated as **cross-checks, not inputs** — hub state is derived from `(matchTime, phase, seed)` at evaluation time, and any card whose authored values contradict the schedule is flagged **INVALID STATE**. This caught 7 unreachable cards in the first pass. Note `matchTime` is **time remaining** (150 = match start, 0 = buzzer).
- **To review them:** edit the `expected` column in `tools\score\decision_cards.tsv` (tab-separated) and re-run. Verdicts are judged against the Blue pass.
- All cards are evaluated as the **Blue** alliance with Blue-origin coordinates, per the repo-wide Blue-only rule, and with zero chassis velocity — so they test *objective choice*, not the shoot gate.
- The report also prints a geometry table showing where `inShootingRange` (<= 4.0 m from the hub) actually holds. Read it before tuning anything: that radius covers the whole home zone *and* the whole home half out to the centerline.
- Development tool only. Not robot code, not on the roboRIO path.

Run the focused JUnit tests from `TitanRoboticsBuildSeason/` to check scenario setup:

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
.\gradlew test --offline --tests "frc.robot.Sim.TrainingMatchScenarioTest" --tests "frc.robot.Sim.TrainingMatchScenarioApplicationTest"
```

This validates scenario inputs; it does not run a training match.

## 1. Enabling the Robot
In the WPILib SimGUI:
- Click **`Teleoperated`** and then **`Enabled`** in the DriverStation control panel to start manual driving.
- Click **`Autonomous`** and then **`Enabled`** to test the auto routine selected in the Elastic Dashboard dropdown.

> [!NOTE]
> Bot0 STUCK diagnostics require `Simulation/DebugAI=true`. For representative bot behavior use a real Driver Station in Practice mode: SimGUI-DS runs show a degenerate pickup-1-or-2-and-shoot pattern that is a DS artifact, not a bot regression. The repo's guided loop lives in the `run-sim-advantagescope` skill (`.agents/skills/run-sim-advantagescope/SKILL.md`: Tier 0 headless gate, Tier 1 GUI auto-launch via `scripts/launch-gui.ps1`, Tier 2 practice-match capture + `review-practice-log.py`; opt-in real-DS via `-PrealDs` in `build.gradle`).

### 2. Driving & Handling
- **Left Stick (X/Y)**: Field-oriented translation with smooth acceleration (slew default 16, `Constants.java:130`).
- **Right Stick (X)**: Holonomic heading rotation (slew default 10, `Constants.java:131`).
- **Left Stick Click**: Toggles **Slow Mode** (35% speed) for precision positioning.
- **D-Pad (POV)**: Cardinal snap-to-heading:
  - **Up**: Face Away ($0^\circ$)
  - **Right**: Face Right ($-90^\circ$)
  - **Down**: Face Backward ($180^\circ$)
  - **Left**: Face Left ($+90^\circ$)
- **A Button (double-tap within 0.4 s)**: Zero Gyro field heading relative to current alliance (`Teleop.java:138-140,530-531`; single press does nothing).

### 3. Intaking Fuel Balls
- Drive towards any Fuel ball on the carpet.
- **Hold Left Trigger (>30%)**: The intake arm automatically deploys down to ground ($250^\circ$) and spins the rollers.
- When the bumper intersects a ball, it is ingested into the hopper. The `Held Balls` counter on the dashboard will increment.
- **Release Left Trigger**: The arm automatically retracts to the standby upright position ($347^\circ$).

### 4. Auto-Aiming & Scoring in the Hub
- Drive to any shooting position inside your Alliance Zone (Blue $X \le 4.6256\text{m}$, Red $X \ge 11.9154\text{m}$, owned by `Navigation/FieldMap.java` `AllianceZones` `:265-268`). Shooter solutions cover 1.2–6.5 m (`Shooter.java:189,242`); bots use the shared 4.20 m upper bound (`FieldMap.Hubs.SHOOTING_MAX_DISTANCE`, read by both the snipe utility and `AIRobotSim`). The lower bound is not single-owned (snipe gates at 3.6 m, `AIRobotSim` hardcodes 1.40 m, `FieldMap` says 1.60 m). Shots from Midfield are automatically inhibited.
- **Hold Right Trigger (>30%)**:
  - The robot locks heading onto the Hub center.
  - Dual flywheels spool up to the interpolated target RPM based on distance.
  - The dashboard displays **Ready to Fire** (green indicator) once aligned within $3^\circ$, flywheels reach target velocity, and the robot is verified inside the Alliance Zone.
  - The kicker feed automatically fires the balls.
  - Watch the balls arc across the field into the Hub in AdvantageScope. The **Simulation Score** counter will increment!

### 5. Glide Points & Tactical Waypoints
- **Hold Right Bumper**: The robot autonomously plans a path and navigates to the nearest tactical waypoint (Alliance Feeder, Hub perimeter, Trench auto-tunnel, or Midfield crossing).
- Stick input between 0.10 and 0.65 **blends** with the assist; exceeding 0.65 translation / 0.60 rotation (`BREAKOUT_TRANSLATION/ROTATION`, `AutonomousTeleopAgent.java:46-47`, consumed at `Teleop.java:392-393`) cancels Glide and restores full driver control.

### 6. Sparring Against Multiple Opponent AI Robots (1 to 3 Autonomous Agents)

The simulation engine supports scaling from a single sparring opponent up to **3 simultaneous AI robots** running concurrently on the field, powered by the unified System 1 (Tactical Reflex) + System 2 (Executive Strategy) Jev cognitive architecture:

- **Activating Multi-Bot Simulation**:
  - In Elastic Dashboard (`Simulation & Match Info` tab), toggle **`Opponent AI Active`** (`Features/Opponent Robot`).
  - Set the number of active opponent bots via the **`Opponent Count`** chooser (`/SmartDashboard/Simulation/OpponentCountChooser`), backed by the `Simulation/OpponentCount` number. Select `1`, `2`, or `3`.
  - Adjust sparring speed with **`Opponent Speed %`** (`Simulation/OpponentSpeedPercent`, 20% to 100%, defaults to 75%).
  - Bots spawn at staggered, non-overlapping starting coordinates on their alliance wall (Blue X=2.00 m, Red X=14.541 m = `FIELD_LENGTH − 2.00`). When the player is Blue, opponents spawn Red and allies spawn Blue (`AIRobotSim.java:1376-1423`). Interactive spawns use X=2.00/14.541; training `default3v3` uses X=2.00/14.50 with 8-fuel preloads (`TrainingMatchScenario.java:71-90`):
    - **Bot 0**: Centerline spawn ($Y=4.035\text{m}$)
    - **Bot 1**: Upper corridor spawn ($Y=5.80\text{m}$)
    - **Bot 2**: Lower corridor spawn ($Y=2.25\text{m}$)

- **Selectable AI Behavioral Archetypes**:
  Each bot can be independently configured with distinct behavioral strategies via SmartDashboard or the Elastic Dashboard:
  - **Bot 0** (`Simulation/AIModeChooser` / `Simulation/Bot0/Archetype`): Defaults to `Autonomous Fuel Cycler`.
  - **Bot 1** (`Simulation/Bot1/Archetype`): Defaults to `DEFENSE_BULLY`.
  - **Bot 2** (`Simulation/Bot2/Archetype`): Defaults to `ADAPTIVE_COMPETITOR`.

  | Archetype | Macro Strategy | Tactical Behaviors |
  | :--- | :--- | :--- |
  | **`AUTONOMOUS_CYCLER`** | High-Throughput Fuel Scoring | Evaluates Gaussian cluster density scent to target rich fuel patches. Adheres to Alliance Zone firing geofencing, standoff arcs ($2.40\text{m}$), and shoot-on-the-fly ballistics. |
  | **`DEFENSE_BULLY`** | Aggressive Physical Harassment | Pursues player bumpers, pins against walls (warn 1.8 s, max 2.4 s, `Navigation/ContactWatchdog.java:47-48`), and disrupts player intake lanes. |
  | **`ADAPTIVE_COMPETITOR`** | Hybrid Two-Way Play | Scavenges loose balls when the Hub is active; transitions to lane denial and player harassment when its Hub is inactive. |
  | **`TACTICAL_DEFENDER`** | Positional Lane & Depot Denial | Shadows player along the midfield boundary ($X = 8.27\text{m}$), blocks direct shooting corridors to the Hub, and contests neutral depots. |
  | **`LEAD_PURSUIT_INTERCEPTOR`** | Predictive Path Interception | Projects the player's instantaneous velocity vector and executes quadratic lead intercept to cut off travel routes. |
  | **`CO_PILOT` / `MANUAL_2_PLAYER`** | Human Sparring Partner | `MANUAL_2_PLAYER` is an `AIMode` (not an `Archetype`; archetype equivalent is `CO_PILOT`). Map Joystick Port 2 to drive Bot 0 directly against the primary driver using standard gamepad controls. |

- **Multi-Robot Collision Avoidance & Flocking Separation**:
  - **Soft Peer Separation**: All AI instances evaluate peer robot distances in real time. If another robot approaches within $1.10\text{m}$ (bumper-to-bumper proximity), a smooth inverse-distance repulsive force is applied, preventing multi-bot scrums or mechanical lockups.
  - **Obstacle Registration**: Each active bot registers its pose and velocity in [`DynamicRouter`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/src/main/java/frc/robot/Navigation/DynamicRouter.java), enabling player trajectory pathfinding to cleanly circumnavigate moving opponents.

- **Elastic Dashboard Multi-Bot Controls (`Simulation & Match Info` Tab)**:
  - **Arena view**: the two 2D `Field` widgets live on Tab 1 (Driver) and Tab 2 (AI Coach), not on the `Simulation & Match Info` tab — open those tabs to see the player alongside the bots with live headings and lookahead markers.
  - **Per-Bot Status Cards**:
    - **Mode & Objective**: Live displays for Bot 0, Bot 1, and Bot 2 active states (e.g. `CYCLE_SCORE_HUB`, `DENY_SHOOTING_LANE`, `STAGE_STANDOFF`).
    - **Held Fuel & Scores**: Dedicated counters tracking individual fuel counts and points scored per bot.
  - **Aggregate Telemetry**: Live indicators for `Total Opponent Score` and `Total Opponent Fuel`. (There is no layout widget bound to `MultiBotActiveCount`/`AllyActiveCount` — read those NT keys directly if needed.)

- **AdvantageScope 3D Multi-Robot Scrimmage Setup**:
  - Load the pre-configured layout: Open AdvantageScope -> **File -> Open Layout** -> select [`advantagescope-layout.json`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/advantagescope-layout.json).
  - Pre-configured views include:
    - **3D Arena Scrimmage**: Complete 3D field rendering with player (Blue) and up to 3 opponents (models per `advantagescope-layout.json:85-139`: `OpponentBot0`/`Bot1` "Crab Bot", `OpponentBot2` "Duck Bot"; no per-bot colors configured) driving with 3D projectile arcs and dynamic fuel balls.
    - **2D Tactical Field Map**: Simultaneous tracking of all robot poses, navigation waypoints, glide points, and pathfinder detours. (Ships with empty `sources` — populate manually, or use the Elastic `Field` widgets.)
    - **Multi-Bot Scrimmage Scoring**: Real-time line graphs comparing player scoring throughput against individual and aggregate AI bot scores. (The Line Graph tab ships with empty sources — add traces manually.)
    - **Fuel Inventory & Drive Dynamics**: Multi-bot hopper tracking and flywheel RPM response. (No such view ships in the layout — add traces manually.)

- **NetworkTables Telemetry Reference**:
  - *Bot 0*: `/AI_Telemetry/Bot0/ActualPose`, `/Simulation/Bot0/StateDetail`, `/Simulation/Bot0/Score`, `/Simulation/Bot0/Fuel`, `/Simulation/Bot0/Archetype`
  - *Bot 1*: `/AI_Telemetry/Bot1/ActualPose`, `/Simulation/Bot1/StateDetail`, `/Simulation/Bot1/Score`, `/Simulation/Bot1/Fuel`, `/Simulation/Bot1/Archetype`
  - *Bot 2*: `/AI_Telemetry/Bot2/ActualPose`, `/Simulation/Bot2/StateDetail`, `/Simulation/Bot2/Score`, `/Simulation/Bot2/Fuel`, `/Simulation/Bot2/Archetype`
  - *Aggregates*: `/Simulation/TotalOpponentScore`, `/Simulation/TotalOpponentFuel`, `/Simulation/MultiBotActiveCount`
  - *Field2d Objects*: `/SmartDashboard/Field/OpponentBot0`, `/SmartDashboard/Field/OpponentBot1`, `/SmartDashboard/Field/OpponentBot2`

### 7. Ally Bots & Full 3v3 FRC Match Simulation (Player + 2 Allies vs 3 Opponents)

In addition to opposing sparring robots, the simulation engine allows you to spawn **1 or 2 autonomous Ally Bots** on your own alliance team. This enables complete **3v3 FRC match simulation** with full alliance coordination:

- **Activating Ally Bots**:
  - In Elastic Dashboard (`Simulation & Match Info` tab), toggle **`Ally Bots Active`** (`Features/Ally Bots`) or select the number of allies via **`Ally Count Chooser`** (`Simulation/AllyCountChooser`).
  - Available configurations:
    - **`0 Ally Bots (Solo)`**: Standard player solo practice or 1vX sparring.
    - **`1 Ally Bot (2v3 / 2v2)`**: Spawns Ally 1 alongside the player.
    - **`2 Ally Bots (Full 3v3)`**: Spawns both Ally 1 and Ally 2, forming a full 3-robot alliance!
  - Allies line up along your alliance driver wall alongside the player robot:
    - **Player**: Center start position ($X \approx 2.00\text{m}, Y \approx 4.035\text{m}$ for Blue)
    - **Ally 1**: Left flank start position ($X = 2.00\text{m}, Y = 5.80\text{m}$ for Blue, facing $0^\circ$)
    - **Ally 2**: Right flank start position ($X = 2.00\text{m}, Y = 2.25\text{m}$ for Blue, facing $0^\circ$)
    *(Coordinates automatically mirror to $X = 14.541\text{m}$ (`FIELD_LENGTH − 2.00`), facing $180^\circ$ when on Red Alliance).*

- **Ally Bot Archetypes & Behavior**:
  - Each ally can be assigned an independent behavioral archetype via SmartDashboard:
    - **Ally 1** (`Simulation/Ally1/ArchetypeChooser`): Defaults to `Autonomous Fuel Cycler`. Focuses on collecting midfield fuel and rapid cycling into your Alliance Hub.
    - **Ally 2** (`Simulation/Ally2/ArchetypeChooser`): Defaults to `Adaptive Match Competitor`. Cycles fuel when your Hub is active, and switches to midfield containment or depot defense when the Hub is inactive.
  - **Alliance Awareness**: Unlike opponents, Ally Bots target your alliance's Hub, harvest balls from your alliance depots, never pin the player, and park at your alliance's climbing tower during the endgame.
  - **Multi-Robot Soft Separation**: All 6 robots active on the field (Player, 2 Allies, 3 Opponents) constantly compute mutual bumper distances. When any robot approaches within $1.10\text{m}$, smooth repulsion velocities prevent mechanical jams and scrums.

- **Field2d & Telemetry Representation**:
  - *Field2d Objects*: `AllyBot1`, `AllyTarget1`, `AllyBot2`, `AllyTarget2` displayed in real-time in Elastic Dashboard and AdvantageScope.
  - *Telemetry Channels*:
    - `AI_Telemetry/Ally1/ActualPose`, `Simulation/Ally1/Fuel`, `Simulation/Ally1/Score`, `Simulation/Ally1/StateDetail`
    - `AI_Telemetry/Ally2/ActualPose`, `Simulation/Ally2/Fuel`, `Simulation/Ally2/Score`, `Simulation/Ally2/StateDetail`
    - `Simulation/TotalAllyScore`, `Simulation/TotalAllyFuel`, `Simulation/AllyActiveCount`

---

### 8. Unified 2026 Match Scoring & Scoreboard System (`MatchScoreTracker`)

The simulation runs an automated, authoritative FRC match scoring engine via [`MatchScoreTracker`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/src/main/java/frc/robot/Sim/MatchScoreTracker.java), providing live scoreboards and Ranking Point calculations for both alliances:

- **Scoring Rules**:
  - **Fuel Ball in Active Hub**: $1\text{ point}$ per ball scored.
  - **Wasted Shots**: Balls launched into an inactive Hub during opposing shifts score $0\text{ points}$ and are logged as wasted fuel.
  - **Endgame Tower Climb**: $10\text{ points}$ per robot positioned within $1.20\text{m}$ of the alliance climbing pole during the final 20 seconds of the match ($t \le 20.0\text{s}$).
  - **Fouls** (`RefereeSim` + `MatchScoreTracker`, awarded to the opponent alliance total): MINOR $5\text{ pts}$ / MAJOR $15\text{ pts}$ — AUTO centerline contact (MAJOR), G407 alliance-zone shooting (MAJOR, checked on every player/bot shot), G418 pinning (MINOR at 3 s, MAJOR per extra uncorrected 3 s), G420 tower protection in the last 30 s (MAJOR). Published under `Scoreboard/Referee/*` and the scoreboard tab.

- **FRC Ranking Points (RP)**:
  - **Match Outcome**: $2\text{ RP}$ for a win, $1\text{ RP}$ for a tie.
  - **Energized RP (Fuel)**: $+1\text{ RP}$ awarded to any alliance scoring $\ge 40$ active fuel balls.
  - **Supercharged RP (Climb)**: $+1\text{ RP}$ awarded to any alliance with $\ge 2$ robots successfully climbed.

- **Full Alliance Score Attribution**:
  - Fuel scored and tower climbs achieved by **Ally 1** and **Ally 2** automatically credit your alliance's score and RP totals!
  - Real-time scoring streams published to Elastic Dashboard and AdvantageKit:
    - Main Scoreboard: `Scoreboard/Match/RedScore`, `Scoreboard/Match/BlueScore`, `Scoreboard/Match/LeadMargin`, `Scoreboard/Match/Leader`
    - Player Team Summary: `Scoreboard/Player/ShotsAttempted`, `Scoreboard/Player/ShotsScored`, `Scoreboard/Player/AccuracyPercent`, `Scoreboard/Player/Climbed`
    - Ally Breakdown: `Scoreboard/Allies/Ally1_FuelScored`, `Scoreboard/Allies/Ally2_FuelScored`, `Scoreboard/Allies/TotalFuelScored`, `Scoreboard/Allies/Ally1_Climbed`
    - Opponent Breakdown: `Scoreboard/Opponents/Bot0_FuelScored`, `Scoreboard/Opponents/Bot1_FuelScored`, `Scoreboard/Opponents/Bot2_FuelScored`

---

### 9. AI Coach & Practice Proving Ground
Switch to the **`AI Coach & Practice`** tab in Elastic Dashboard for focused driver training:

- **Practice Drill Modes** (Select via `Practice Drill Mode` chooser):
  - **`Free Play Match`**: Standard match simulation against an autonomous cycling opponent (`AIRobotSim` at 75% speed).
  - **`Rapid Cycling Sprint`**: Disables opponent defense and enables automatic ball respawns for solo time-trial throughput drills.
  - **`Trench Defense & Pirouette Drill`**: Spawns an 80% speed sparring partner patrolling the trenches to practice automated Smart Tunnel diversions, bumper pirouettes, and legal pinning evasion (warn 1.8 s, max 2.4 s per `ContactWatchdog`; G418 fouls at 3 s per `RefereeSim`).
  - **`Anti-Defense SOTF Drill`**: Spawns an 85% speed lead-pursuit interceptor to practice shoot-on-the-fly (SOTF) accuracy while under heavy pursuit.
- **1-Click Proving Ground Reset**:
  - Click **`Reset Practice Arena`** (`Coaching/ResetPractice`).
  - Instantly resets session metrics, teleports the robot back to the alliance starting line, respawns all 54 Fuel balls across the arena, and configures the sparring AI for the selected drill.
- **Dynamic Driver Grade**:
  - Real-time rating from **`A+`** to **`D`** based on cycle speed, shooting accuracy, and Hub active timing discipline.
- **TypeSafe Jev AI Coach Terminal & Markdown Reports**:
  - Launch the terminal coaching tool in another PowerShell window while running simulation:
    ```powershell
    # Live ANSI Telemetry & Tactical Directive HUD
    python tools/coaching/jev_coach.py --live

    # Generate Post-Match Debrief Report (Saved to reports/)
    python tools/coaching/jev_coach.py --report
    ```
  - If a `TYPESAFE_API_KEY` or `OPENROUTER_API_KEY` environment variable is defined, the tool queries the TypeSafe Jev API (`https://api.typesafe.ai/v1/systemone`, `jev_coach.py:174`) for automated System One AI tactical critiques.

---

## 🛠️ Troubleshooting & FAQs

### Q: `bind() to port 1181 failed: Only one usage of each socket address is normally permitted`
- **Cause**: WPILib CameraServer attempts to bind to default RTSP/HTTP ports that may already be in use by another local process. In a headless match the binder is `PhotonCameraSim`'s constructor (via `Sim/VisionSim`), which calls `CameraServer.putVideo` twice and so takes 1181 and 1182. `CameraServer.kBasePort` is a `public static final int` with no system property, so it cannot be offset.
- **Solution**: Harmless to a score. cscore logs the failure and continues, and the `photonvision` NT table that `VisionIOSim` reads is written regardless, so AprilTag results are unaffected. A single sim instance can ignore it. **In a parallel `tools/score/sweep.ps1` run it is expected** - the rig counts these and reports them, but does not fail the sweep. See `ARCHITECTURE.md` for the full port-isolation contract.
- **Not the same as the NT port problem.** A bind failure on **1735/5810** (NT3/NT4) is a different class: it means a worker could have entered ntcore client mode and attached to a sibling worker. The rig eliminates that (`Robot.robotInit` stops the NT server and drops `NT4Publisher` when headless) and fails the sweep if the marker line is missing or a client connects.

### Q: The robot does not respond to controller inputs
- Check the **`Joysticks`** panel in the WPILib SimGUI. Ensure your controller is placed in **`Joystick 0`**.
- Verify that Driver Station is set to **`Teleoperated`** and **`Enabled`**.

### Q: Build failure: `Unsupported class file major version` or Java errors
- Ensure you are running Gradle with the WPILib 2026 JDK:
  ```powershell
  $env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
  $env:Path = "$($env:JAVA_HOME)\bin;$($env:Path)"
  .\gradlew simulateJava
  ```

### Q: How do I reset the match or respawn balls?
- On the **Simulation & Match Info** tab of the Elastic Dashboard, click the **Respawn Balls** or **Reset Sim** buttons.
- Alternatively, disable and re-enable the robot in SimGUI.

---

## 📚 Related Documentation
- 📖 [System Architecture Specification](ARCHITECTURE.md): Deep-dive into subsystem layers and IO abstraction.
- 🎮 [Operator's Guide](OPERATORS_GUIDE.md): Complete driver and operator control mappings.
- 🧪 [Testing & Diagnostics Guide](src/main/java/frc/robot/Test/README.md): Pre-flight routines and SysId characterization.

