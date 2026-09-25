import json
import os
from collections import defaultdict

raw_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json"
type_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\type_catalog.json"
summary_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\per_file_summary.json"
out_handoff = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\handoff.md"

with open(raw_path, 'r', encoding='utf-8') as f:
    raw_files = json.load(f)

with open(type_path, 'r', encoding='utf-8') as f:
    all_types = json.load(f)

with open(summary_path, 'r', encoding='utf-8') as f:
    file_summaries = json.load(f)

# Group types by package
types_by_pkg = defaultdict(list)
for t in all_types:
    types_by_pkg[t['package']].append(t)

# Group files by package
files_by_pkg = defaultdict(list)
for f in file_summaries:
    files_by_pkg[f['package']].append(f)

md = []

# Title & Metadata
md.append("# Code Quality & Javadoc Documentation Audit Report")
md.append("\n**Author**: Explorer 2 (Code Quality & Javadocs)")
md.append("**Target Codebase**: TitanRoboticsBuildSeason (`src/main/java/frc/robot`)")
md.append("**Working Directory**: `C:\\Users\\jumpi\\Documents\\Github\\2026Dreams\\.agents\\teamwork\\explorer_docs_1`")
md.append("**Date**: 2026-09-25")
md.append("**Parent**: orchestrator_1 (Conversation ID: `8b383374-9a92-411a-ba01-ca6b47135c41`)\n")

# Executive Summary
md.append("## Executive Summary\n")
md.append("A comprehensive, full-coverage static audit was performed across all **94 Java source files** comprising **15 packages** and **159 declared types** (classes, interfaces, enums, records, annotations) in the TitanRoboticsBuildSeason repository.")
md.append("\n### High-Level Audit Findings:")
md.append("1. **Documentation Deficit**:")
md.append("   - **Declared Types**: 80 / 159 have Javadoc (50.3% coverage; **79 types completely lack Javadocs**).")
md.append("   - **Public/Protected Methods**: 201 / 720 have Javadoc (27.9% coverage; **519 methods lack Javadocs**).")
md.append("   - **Documented Engineering Units**: Only 85 / 720 methods (11.8%) and 71 / 457 constants (15.5%) have explicit physical units (e.g., meters, radians, seconds, volts, amperes, RPM) documented in signatures, names, or Javadocs.")
md.append("   - **Constants (`static final`)**: Only 8 / 457 constants (1.8%) have formal Javadoc docstrings.")
md.append("2. **Architectural & WPILib Convention Violations**:")
md.append("   - **Custom Multithreaded Actions Framework vs WPILib CommandScheduler**: `AutoMissionExecutor` spawns autonomous routines on an unmanaged background Java thread (`new Thread(...)`), and `MissionBase` runs a polling loop using `Thread.sleep()`. Meanwhile, `CommandScheduler.getInstance().run()` is omitted from `robotPeriodic()` and `teleopPeriodic()`. This creates concurrency hazards and bypasses standard WPILib requirements and safety.")
md.append("   - **Console Output Loop Overrun Risks**: **106 occurrences** of unbuffered `System.out.println`, `System.err.println`, and `printf` during active robot execution, risking RoboRIO loop overruns (>20ms).")
md.append("   - **Encapsulation & Mutable State Leaks**: **167 public mutable fields**, including subsystem state variables (`Shooter.targetRpmLeft`, `AutoMissionChooser.delay`, `Teleop.joystickEnabled`) and exposed internal collections (`SubsystemManager.getSubsystems()`).")
md.append("   - **REVLib Hardware Flash Parameter Burning**: `NeoSparkMaxMotor.java` dynamically executes `configure(..., PersistMode.kPersistParameters)` inside runtime setters (`setInverted`, `setBrakeMode`), wearing out SparkMax flash memory.")
md.append("   - **Dead & Deprecated Code**: Legacy classes such as `Test/SysID.java` and boilerplate missions (`ExampleMission.java`) exist without active usage.\n")

# SECTION 1: OBSERVATION
md.append("## 1. Observation\n")
md.append("This section contains direct, verbatim evidence gathered from the filesystem, tool executions, and line-level code inspections.\n")

# 1.1 Complete Type Catalog
md.append("### 1.1 Complete Catalog of Classes, Interfaces, Enums, Records, and Annotations\n")
md.append("Every declared type in `src/main/java/frc/robot` was cataloged (159 total types):\n")
md.append("| Package | File | Line | Kind | Modifiers | Name | Has Javadoc? | Summary / First Line |")
md.append("|---|---|---|---|---|---|---|---|")

for t in all_types:
    summary = t['javadoc_summary'].replace('|', '\\|')[:60]
    if len(t['javadoc_summary']) > 60:
        summary += "..."
    jd_str = "✅ Yes" if t['has_javadoc'] else "❌ Missing"
    md.append(f"| `{t['package']}` | `{t['file']}` | {t['line']} | `{t['kind']}` | `{t['modifiers']}` | `{t['name']}` | {jd_str} | {summary} |")

# 1.2 Quantitative Audit Summary Tables
md.append("\n### 1.2 Quantitative Javadoc & Engineering Units Coverage by Package\n")
md.append("| Package | Files | Types Total | Types w/ JD | Methods (Pub/Prot) | Methods w/ JD | Methods w/ Units | Constants Total | Consts w/ JD | Consts w/ Units | Smells |")
md.append("|---|---|---|---|---|---|---|---|---|---|---|")

def pct(a, b):
    return f"{a/b*100:.0f}%" if b > 0 else "N/A"

for pkg, flist in sorted(files_by_pkg.items()):
    pkg_name = pkg if pkg else "(default)"
    files_cnt = len(flist)
    t_tot = sum(f['types_total'] for f in flist)
    t_jd = sum(f['types_with_jd'] for f in flist)
    m_tot = sum(f['methods_total'] for f in flist)
    m_jd = sum(f['methods_with_jd'] for f in flist)
    m_u = sum(f['methods_total'] - f['methods_missing_units_count'] for f in flist)
    c_tot = sum(f['constants_total'] for f in flist)
    c_jd = sum(f['constants_with_jd'] for f in flist)
    c_u = sum(f['constants_total'] - f['constants_missing_units_count'] for f in flist)
    sm_tot = sum(f['smells_count'] for f in flist)
    md.append(f"| `{pkg_name}` | {files_cnt} | {t_tot} | {t_jd} ({pct(t_jd, t_tot)}) | {m_tot} | {m_jd} ({pct(m_jd, m_tot)}) | {m_u} ({pct(m_u, m_tot)}) | {c_tot} | {c_jd} ({pct(c_jd, c_tot)}) | {c_u} ({pct(c_u, c_tot)}) | {sm_tot} |")

# Totals row
tot_files = len(file_summaries)
tot_t = len(all_types)
tot_t_jd = sum(1 for t in all_types if t['has_javadoc'])
tot_m = sum(f['methods_total'] for f in file_summaries)
tot_m_jd = sum(f['methods_with_jd'] for f in file_summaries)
tot_m_u = sum(f['methods_total'] - f['methods_missing_units_count'] for f in file_summaries)
tot_c = sum(f['constants_total'] for f in file_summaries)
tot_c_jd = sum(f['constants_with_jd'] for f in file_summaries)
tot_c_u = sum(f['constants_total'] - f['constants_missing_units_count'] for f in file_summaries)
tot_sm = sum(f['smells_count'] for f in file_summaries)
md.append(f"| **TOTAL** | **{tot_files}** | **{tot_t}** | **{tot_t_jd} ({tot_t_jd/tot_t*100:.1f}%)** | **{tot_m}** | **{tot_m_jd} ({tot_m_jd/tot_m*100:.1f}%)** | **{tot_m_u} ({tot_m_u/tot_m*100:.1f}%)** | **{tot_c}** | **{tot_c_jd} ({tot_c_jd/tot_c*100:.1f}%)** | **{tot_c_u} ({tot_c_u/tot_c*100:.1f}%)** | **{tot_sm}** |")

# 1.3 Detailed Smells & Anti-Pattern Observations
md.append("\n### 1.3 Detailed Code Quality Smells, Anti-Patterns & Convention Violations\n")

md.append("#### Issue 1: Custom Actions Framework & Asynchronous Threading Race Conditions")
md.append("- **Files & Lines**:")
md.append("  - `src/main/java/frc/robot/Auto/AutoMissionExecutor.java:16-28`")
md.append("  - `src/main/java/frc/robot/Auto/Missions/MissionBase.java:74-103`")
md.append("  - `src/main/java/frc/robot/Interfaces/Actions.java:8-34`")
md.append("  - `src/main/java/frc/robot/Robot.java:172-181, 270-272`")
md.append("- **Verbatim Code Evidence**:")
md.append("```java\n// AutoMissionExecutor.java:16-28\nmThread = new Thread(new Runnable() {\n    @Override\n    public void run() {\n        if (mAutoMission != null) {\n            try {\n                mAutoMission.run();\n            } catch (Exception e) {\n                edu.wpi.first.wpilibj.DriverStation.reportError(\"AUTO MISSION CRASHED: \" + e.getMessage(), e.getStackTrace());\n            }\n        }\n    }\n});\n```")
md.append("```java\n// MissionBase.java:90-99\nwhile (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {\n    action.update();\n    try {\n        Thread.sleep(waitTime);\n    } catch (InterruptedException e) {\n        e.printStackTrace();\n    }\n}\n```")
md.append("```java\n// Robot.java:172-181\n@Override\npublic void robotPeriodic() {\n    SubsystemManager.updateSubsystems();\n    SubsystemManager.logSubsystems();\n    AlertManager.update();\n    if (testMode != null && (DriverStation.isTest() || testMode.isEnabled())) {\n      testMode.update();\n    }\n    // NOTE: CommandScheduler.getInstance().run() IS MISSING!\n}\n```")
md.append("- **Analysis**: WPILib Command-based architecture is fundamentally single-threaded to prevent data races and CAN bus synchronization issues. In this codebase, autonomous missions run inside a detached raw Java thread sleeping in 20ms slices (`Thread.sleep(waitTime)`). Actions call subsystem methods directly from this worker thread while `SubsystemManager.updateSubsystems()` runs concurrently on the main robot thread. Additionally, `CommandScheduler.getInstance().run()` is only invoked in `testPeriodic()`, preventing any standard WPILib Command/Trigger bindings from functioning during autonomous or teleop.")

md.append("\n#### Issue 2: Console I/O Overrun Hazard (106 Occurrences)")
md.append("- **Files & Lines**: Found across 24 files, most notably:")
md.append("  - `Test/DriveCharacterization.java:144, 148, 152, 156, 163, 174, 179, 187, 192, 205, 210, 218, 223, 241, 245` (15 prints during real-time driver button handling)")
md.append("  - `Sim/GameSim.java:152, 212, 234, 265, 293, 381, 406, 451...` (11 stderr/stdout prints)")
md.append("  - `Sim/AIRobotSim.java:176, 300, 313, 995, 1098` (7 prints)")
md.append("  - `Robot.java:112, 122, 124, 207` (4 prints)")
md.append("  - `Devices/NeoSparkMaxMotor.java:39` (1 print)")
md.append("  - `Subsystems/Vision.java:340, 347, 354` (3 prints)")
md.append("- **Analysis**: Unbuffered console I/O on the RoboRIO over NetworkTables causes serial bus stalls, GC spikes, and frame jitter exceeding the 20ms boundary. In match environments, these calls must be replaced by AdvantageKit `Logger.recordOutput`, `DataLogManager.log`, or `DriverStation.reportWarning` / `DriverStation.reportError`.")

md.append("\n#### Issue 3: Public Mutable Fields (Encapsulation Violations)")
md.append("- **Files & Lines**:")
md.append("  - `Subsystems/Shooter.java:76-77, 86-88`:")
md.append("    ```java\n    public double targetRpmLeft = 0;\n    public double targetRpmRight = 0;\n    public double normalDistanceToHub = 0;\n    public double leftShooterVoltageCalc = 0;\n    public double rightShooterVoltageCalc = 0;\n    ```")
md.append("  - `Auto/AutoMissionChooser.java:28`: `public static double delay;`")
md.append("  - `Teleop.java:47`: `public static boolean joystickEnabled = false;`")
md.append("  - `Auto/DynamicRouter.java:380-382`: `GridNode` public mutable `gCost`, `hCost`, `parent`")
md.append("- **Analysis**: Subsystem internal states are exposed as public mutable primitives. Audit confirmed `Shooter.targetRpmLeft` is only modified internally, meaning its `public` visibility is an encapsulation anti-pattern.")

md.append("\n#### Issue 4: REVLib 2025/2026 SparkMax Flash Wear Anti-Pattern")
md.append("- **File & Lines**: `src/main/java/frc/robot/Devices/NeoSparkMaxMotor.java:85-100`")
md.append("- **Verbatim Code Evidence**:")
md.append("```java\npublic void setInverted(boolean inverted) {\n    this.isInverted = inverted;\n    SparkMaxConfig config = new SparkMaxConfig();\n    config.inverted(inverted);\n    if (m_motor != null) {\n        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);\n    }\n}\n\npublic void setBrakeMode(boolean brake) {\n    SparkMaxConfig config = new SparkMaxConfig();\n    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);\n    if (m_motor != null) {\n        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);\n    }\n}\n```")
md.append("- **Analysis**: `PersistMode.kPersistParameters` burns configuration into the SparkMax EEPROM/flash memory. Flash memory has a limited lifetime write cycle (~10,000 to 100,000 writes). Dynamically calling `configure` with `kPersistParameters` during runtime (e.g., toggling brake mode when disabling) degrades hardware memory and blocks CAN bus communications. Dynamic runtime updates must use `PersistMode.kNoPersistParameters`.")

md.append("\n#### Issue 5: SubsystemManager Mutable Exposure & Thread Safety Contradiction")
md.append("- **File & Lines**: `src/main/java/frc/robot/Subsystems/SubsystemManager.java:52-62`")
md.append("- **Verbatim Code Evidence**:")
md.append("```java\n/**\n * Returns the list of registered subsystems. This list is unmodifiable, as the purpose is to...\n */\npublic static List<Subsystem> getSubsystems() {\n    return subsystems;\n}\n```")
md.append("- **Analysis**: The Javadoc explicitly promises an unmodifiable list, but the implementation returns the raw, mutable `ArrayList<Subsystem> subsystems`. Any external caller can clear, reorder, or corrupt registered subsystems. It must return `Collections.unmodifiableList(subsystems)`.")

md.append("\n#### Issue 6: Dead, Deprecated, and Duplicate Code")
md.append("- **Files & Lines**:")
md.append("  - `Test/SysID.java:1-50`: Completely unreferenced legacy SysID class. All system identification is already handled by `SysIdManager.java`.")
md.append("  - `Devices/Controller.java:58-87`: Overloads `getDebouncedButton(int button)` and `getDebouncedButton(Button button)` duplicate 15 lines of identical logic instead of `getDebouncedButton(button.value)`.")
md.append("  - `Devices/NeoSparkMaxMotor.java:67-76`: `getVelocity()` is an exact duplicate of `getSpeed()`.")
md.append("  - `Auto/Missions/ExampleMission.java:1-25`: Unused placeholder mission class.")

md.append("\n#### Issue 7: In-Loop Heap Allocations & Autoboxing")
md.append("- **Files & Lines**:")
md.append("  - `Devices/Controller.java:25, 58-87`: `HashMap<Integer, Boolean> debounceButtons` creates autoboxed `Integer` and `Boolean` heap objects inside 20ms teleop cycles.")
md.append("  - `Subsystems/SwerveBase.java:988-1002`: `getDriveMotorVoltages()` and `getDriveMotorPositions()` instantiate `new ArrayList<>()` every call.")

md.append("\n#### Issue 8: Contradictory Constants & Undocumented Magic Numbers")
md.append("- **Files & Lines**:")
md.append("  - `Data/Constants.java:55`: `public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32 lbs * kg per pound`. 148 - 20.3 = 127.7 lbs, not 32 lbs! The comment is wildly contradictory.")
md.append("  - `Teleop.java:340-341, 446-447`: Hardcoded slow mode scale factors `0.35` (translation) and `0.50` (rotation) are duplicated as raw magic numbers instead of residing in `Constants.OperatorConstants`.")

md.append("\n#### Issue 9: Exception Swallowing via `printStackTrace()`")
md.append("- **Files & Lines**:")
md.append("  - `Auto/Missions/MissionBase.java:83, 97`: InterruptedException swallowed with `e.printStackTrace()` inside thread sleep loops.")
md.append("  - `Sim/AIRobotSim.java:301, 314`: Opponent/Ally spawn failures printed to stderr.")
md.append("  - `Sim/GameSim.java:213`: `simulationUpdate` exception printed to stderr.")

# 1.4 Detailed Inventory Per File
md.append("\n### 1.4 Detailed Documentation & Cleanup Inventory Per File (All 94 Files)\n")
md.append("Below is the complete, file-by-file audit of documentation status, missing Javadocs, missing engineering units, and cleanup recommendations for all 94 files in `src/main/java/frc/robot`:\n")

for i, f in enumerate(file_summaries, 1):
    relpath = f['file']
    pkg = f['package']
    types_tot = f['types_total']
    types_jd = f['types_with_jd']
    missing_types = f['types_missing_jd']
    m_tot = f['methods_total']
    m_jd = f['methods_with_jd']
    m_missing_cnt = f['methods_missing_jd_count']
    m_missing_names = f['methods_missing_jd_names']
    c_tot = f['constants_total']
    c_jd = f['constants_with_jd']
    c_missing_cnt = f['constants_missing_jd_count']
    c_missing_u = f['constants_missing_units_count']
    c_missing_u_names = f['constants_missing_units_names']
    smells = f['smells']

    md.append(f"#### {i}. `{relpath}`")
    md.append(f"- **Package**: `{pkg}`")
    md.append(f"- **Types Declared ({types_tot})**: {types_jd}/{types_tot} have Javadoc" + (f" (Missing Javadoc: `{', '.join(missing_types)}`)" if missing_types else ""))
    md.append(f"- **Public/Protected Methods ({m_tot})**: {m_jd}/{m_tot} have Javadoc ({m_missing_cnt} missing)")
    if m_missing_names:
        sample_m = ', '.join([f"`{n}()`" for n in m_missing_names[:8]])
        if len(m_missing_names) > 8:
            sample_m += f", ... (+{len(m_missing_names)-8} more)"
        md.append(f"  - *Missing Method Javadoc*: {sample_m}")
    md.append(f"- **Constants ({c_tot})**: {c_jd}/{c_tot} have Javadoc, {c_missing_u}/{c_tot} lack explicit units")
    if c_missing_u_names:
        sample_c = ', '.join([f"`{n}`" for n in c_missing_u_names[:6]])
        if len(c_missing_u_names) > 6:
            sample_c += f", ... (+{len(c_missing_u_names)-6} more)"
        md.append(f"  - *Constants Lacking Units*: {sample_c}")
    if smells:
        md.append(f"- **Smells/Issues ({len(smells)})**:")
        for s in smells[:5]:
            md.append(f"  - Line {s['line']}: `[{s['type']}]` {s['message'][:90]}")
        if len(smells) > 5:
            md.append(f"  - ... (+{len(smells)-5} more smells)")
    else:
        md.append("- **Smells/Issues**: None detected.")

    # Action recommendations
    actions = []
    if missing_types:
        actions.append(f"Add class-level Javadoc to {', '.join(missing_types)}")
    if m_missing_cnt > 0:
        actions.append(f"Add descriptive Javadocs with `@param` and `@return` to {m_missing_cnt} public/protected methods")
    if c_missing_cnt > 0:
        actions.append(f"Add formal Javadoc comments to {c_missing_cnt} constants")
    if c_missing_u > 0:
        actions.append(f"Document physical engineering units on {c_missing_u} constants or add unit suffixes")
    for s in smells:
        if s['type'] == 'CONSOLE_IO':
            actions.append("Replace console prints with AdvantageKit Logger or DriverStation telemetry")
            break
    for s in smells:
        if s['type'] == 'PUBLIC_MUTABLE_FIELD':
            actions.append("Encapsulate public mutable fields with private access and getters/setters")
            break
    for s in smells:
        if s['type'] == 'THREAD_SLEEP':
            actions.append("Eliminate Thread.sleep and migrate to non-blocking WPILib command loops")
            break
    if not actions:
        actions.append("Verify unit documentation and formatting compliance")
    md.append(f"- **Required Cleanup & Documentation Action**: {'; '.join(actions)}.\n")

# SECTION 2: LOGIC CHAIN
md.append("## 2. Logic Chain\n")
md.append("1. **Observation**: `AutoMissionExecutor.java:16` spawns autonomous routines inside a detached raw Java thread (`mThread = new Thread(...)`), and `MissionBase.java:90-99` runs a polling `while` loop calling `Thread.sleep(waitTime)` to tick actions.")
md.append("   → **Inference**: Autonomous actions execute asynchronously from the RoboRIO's main periodic loop.")
md.append("   → **Inference**: Actions call subsystem hardware methods (e.g. `swerveBase.drive(...)`, `intake.runRollers(...)`) from the background thread while `Robot.robotPeriodic()` executes `SubsystemManager.updateSubsystems()` on the main thread.")
md.append("   → **Conclusion**: This is a direct multithreading race condition without locks or synchronization. In addition, `CommandScheduler.getInstance().run()` is completely missing from `robotPeriodic()`, breaking standard WPILib Command-based paradigms.")

md.append("\n2. **Observation**: 106 occurrences of `System.out.println`, `System.err.println`, and `printf` were cataloged across 24 files, including 15 inside `Test/DriveCharacterization.java` and 11 inside `Sim/GameSim.java`.")
md.append("   → **Inference**: Standard console output on the RoboRIO is synchronous and routed across NetConsole and NetworkTables.")
md.append("   → **Conclusion**: Frequent or unbuffered console prints during robot operations cause loop time overruns (>20ms), degraded odometry integration, and intermittent communication stalls.")

md.append("\n3. **Observation**: 167 public mutable fields were cataloged across the codebase, including `Shooter.targetRpmLeft`, `AutoMissionChooser.delay`, and `Teleop.joystickEnabled`.")
md.append("   → **Inference**: External classes can mutate critical subsystem state without validation, side-effect triggers, or synchronization.")
md.append("   → **Conclusion**: Violates core object-oriented encapsulation and introduces hidden state dependencies across disparate files.")

md.append("\n4. **Observation**: In `NeoSparkMaxMotor.java:85-100`, runtime setter methods `setInverted(boolean)` and `setBrakeMode(boolean)` call `m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters)`.")
md.append("   → **Inference**: `PersistMode.kPersistParameters` writes configuration parameters directly to SparkMax non-volatile flash EEPROM.")
md.append("   → **Conclusion**: Repeated invocations of these methods during a match or teleop toggle will exhaust SparkMax hardware write cycles and introduce CAN bus frame delays.")

md.append("\n5. **Observation**: Quantitative Javadoc inspection showed only 50.3% of types, 27.9% of public/protected methods, and 1.8% of constants have formal Javadocs. Only 11.8% of methods and 15.5% of constants document physical engineering units.")
md.append("   → **Inference**: Developers and autonomous tuning algorithms have no contractual guarantee whether angles are in radians or degrees, distances in meters or inches, speeds in RPM or duty cycle, or times in seconds or milliseconds.")
md.append("   → **Conclusion**: A systematic documentation overhaul documenting explicit units on every method and constant is mandatory before competition deployment.")

# SECTION 3: CAVEATS
md.append("\n## 3. Caveats\n")
md.append("1. **Third-Party Vendor Code Isolation**: `src/main/java/frc/robot/ThirdParty/LimelightHelpers.java` is an official, upstream vendor-supplied single-file utility from Limelight. While it contains 132 code smells (public fields, console prints), altering its core structure could complicate future upstream vendor drops. Refactoring should focus primarily on team-owned code in `frc.robot.*` while wrapping or safely consuming `LimelightHelpers`.")
md.append("2. **AdvantageKit `@AutoLog` Struct Convention**: In files like `DriveIO.java`, `IntakeIO.java`, `ShooterIO.java`, and `VisionIO.java`, inner classes annotated with `@AutoLog` (`DriveIOInputs`, etc.) use public fields by design according to the AdvantageKit code generation pattern. These fields should NOT be encapsulated into private fields with getters/setters, but MUST have explicit engineering units documented via Javadoc docstrings and variable name suffixes.")
md.append("3. **Simulation Code Scope**: The 14 files in `frc.robot.Sim.*` (`GameSim`, `AIRobotSim`, `JevDecisionEngine`, etc.) are desktop simulation models designed to run on development workstations, not on the RoboRIO during FRC matches. While code quality and Javadoc improvements are necessary, performance constraints (such as heap allocations) are less critical in simulation than on the physical RoboRIO embedded processor.")
md.append("4. **ChoreoLib Integration**: Autonomous trajectories are defined via Choreo `.traj` files in `src/main/deploy/choreo`. Refactoring the autonomous action execution architecture must preserve compatibility with Choreo's trajectory sample format (`SwerveSample`) and timestamp tracking.")

# SECTION 4: CONCLUSION
md.append("\n## 4. Conclusion\n")
md.append("The TitanRoboticsBuildSeason codebase possesses a rich feature set (holonomic pathfinding, AdvantageKit replay logging, physics simulation, multi-agent AI sparring), but suffers from significant documentation gaps and critical architectural anti-patterns that jeopardize competition reliability.")
md.append("\n### Actionable Overhaul Priorities for Implementers:")
md.append("1. **Priority 1: Concurrency & Autonomous Unification**")
md.append("   - Deprecate raw thread spawning in `AutoMissionExecutor` and blocking `Thread.sleep` loops in `MissionBase`.")
md.append("   - Ensure `CommandScheduler.getInstance().run()` is called in `Robot.robotPeriodic()` so that WPILib commands, triggers, and scheduled actions run predictably and safely on the main thread.")
md.append("2. **Priority 2: SparkMax Flash Memory Protection**")
md.append("   - In `NeoSparkMaxMotor.java`, replace `PersistMode.kPersistParameters` with `PersistMode.kNoPersistParameters` in runtime methods (`setInverted`, `setBrakeMode`). Ensure parameter persistence occurs ONLY once during robot initialization.")
md.append("3. **Priority 3: Console I/O Elimination & Telemetry Hygiene**")
md.append("   - Replace all 106 occurrences of `System.out.println` and `System.err.println` with AdvantageKit `Logger.recordOutput`, `DataLogManager.log`, or `DriverStation.reportError`.")
md.append("4. **Priority 4: Encapsulation & Mutability Protection**")
md.append("   - Convert public mutable fields in `Shooter.java` (`targetRpmLeft`, `targetRpmRight`, etc.) and `AutoMissionChooser.java` (`delay`) to private fields with type-safe accessors.")
md.append("   - Return `Collections.unmodifiableList(subsystems)` in `SubsystemManager.getSubsystems()`.")
md.append("5. **Priority 5: Comprehensive Javadoc & Engineering Units Overhaul**")
md.append("   - Add complete class-level Javadocs to the 79 missing types.")
md.append("   - Add Javadoc docstrings with explicit `@param`, `@return`, and engineering units (meters, radians, degrees, seconds, volts, amperes, RPM) to all 519 missing methods and 449 missing constants.")
md.append("   - Resolve contradictory comments (e.g. `Constants.ROBOT_MASS`).")
md.append("6. **Priority 6: Dead Code Elimination**")
md.append("   - Remove obsolete `Test/SysID.java` and redundant methods (`NeoSparkMaxMotor.getVelocity()`).")

# SECTION 5: VERIFICATION METHOD
md.append("\n## 5. Verification Method\n")
md.append("Any implementer executing the refactoring and documentation tasks can independently verify code integrity, build stability, and documentation coverage using the following commands and checks:\n")
md.append("### 1. Build Compilation Verification")
md.append("The project must compile cleanly without errors using the official WPILib 2026 JDK:\n")
md.append("```powershell\n$env:JAVA_HOME=\"C:\\Users\\Public\\wpilib\\2026\\jdk\"\n.\\gradlew build `\"-Dorg.gradle.java.home=C:\\Users\\Public\\wpilib\\2026\\jdk`\"\n```")
md.append("**Expected Outcome**: Exit code 0, BUILD SUCCESSFUL, with zero compile errors or broken symbol references.\n")

md.append("### 2. Unit Test Suite Verification")
md.append("Execute all existing unit tests in `src/test/java`:\n")
md.append("```powershell\n$env:JAVA_HOME=\"C:\\Users\\Public\\wpilib\\2026\\jdk\"\n.\\gradlew test `\"-Dorg.gradle.java.home=C:\\Users\\Public\\wpilib\\2026\\jdk`\"\n```")
md.append("**Expected Outcome**: Exit code 0, all 18 test suites pass without regressions.\n")

md.append("### 3. Automated Documentation & Code Quality Audit Verification")
md.append("Re-run the audit scripts created during this investigation to ensure coverage metrics increase to 100% and smells drop to 0:")
md.append("```powershell\npython \"C:\\Users\\jumpi\\Documents\\Github\\2026Dreams\\.agents\\teamwork\\explorer_docs_1\\audit_script.py\"\npython \"C:\\Users\\jumpi\\Documents\\Github\\2026Dreams\\.agents\\teamwork\\explorer_docs_1\\summarize_audit.py\"\n```")
md.append("**Target Metrics for Completion**:")
md.append("- Types with Javadoc: 100% (159/159)")
md.append("- Public/Protected Methods with Javadoc: 100% (720/720)")
md.append("- Public/Protected Methods with Units: 100% of numeric methods")
md.append("- Constants with Javadoc & Units: 100% (457/457)")
md.append("- Console I/O Smells: 0 in robot code (`frc.robot.*`)")
md.append("- Thread.sleep Occurrences: 0 in robot code (`frc.robot.*`)")
md.append("- Public Mutable Fields: 0 (outside of AdvantageKit `@AutoLog` data classes)\n")

content = "\n".join(md)
with open(out_handoff, 'w', encoding='utf-8') as f:
    f.write(content)

print(f"Handoff report successfully written to {out_handoff} ({len(content)} bytes, {len(md)} lines).")
