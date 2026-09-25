# Baseline Build & Dependencies Investigation Report (Explorer 3)

**Target Codebase**: `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason`  
**Execution Context**: WPILib 2026 Toolchain (`C:\Users\Public\wpilib\2026\jdk`)  
**Report File**: `C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_build_1\handoff.md`  

---

## 1. Observation

### 1.1 Baseline Build Execution & Environment

#### Command 1: Direct Execution
```powershell
.\gradlew build -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"
```
- **Exit Code**: `1`
- **Verbatim Output**:
```
ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH. 

Please set the JAVA_HOME variable in your environment to match the 
location of your Java installation. 
```
- **File / Code Reference**: `gradlew.bat` lines 41–54:
```bat
@rem Find java.exe
if defined JAVA_HOME goto findJavaFromJavaHome

set JAVA_EXE=java.exe
%JAVA_EXE% -version >NUL 2>&1
if %ERRORLEVEL% equ 0 goto execute

echo. 1>&2
echo ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH. 1>&2
echo. 1>&2
echo Please set the JAVA_HOME variable in your environment to match the 1>&2
echo location of your Java installation. 1>&2

goto fail
```
- **Observation Detail**: The standard Gradle wrapper batch script checks for an existing `JAVA_HOME` or a system `java.exe` in `PATH` before invoking Gradle. Passing `-Dorg.gradle.java.home=...` is a Gradle argument and is never reached if `JAVA_HOME` is unset in the launching shell. Furthermore, `.cursorrules` (lines 5–8) explicitly notes:
```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'; .\gradlew build
```

#### Command 2: Execution with Pre-set `JAVA_HOME`
```powershell
cmd.exe /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& gradlew.bat build -Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```
- **Exit Code**: `1` (First run failed on Windows test-results daemon lock, resolved by `gradlew.bat --stop`; second run executed the full test suite).
- **Compilation Status**:
  - `Task :generateBuildConstants`: EXECUTED
  - `Task :compileJava`: SUCCESS (compiled with toolchain `C:\Users\Public\wpilib\2026\jdk`, OpenJDK 17.0.16 Temurin)
  - `Task :classes`: SUCCESS
  - `Task :jar`: SUCCESS
  - `Task :assemble`: SUCCESS
  - `Task :compileTestJava`: SUCCESS
  - `Task :testClasses`: SUCCESS
  - `Task :test`: FAILED
- **Verbatim Failure Output**:
```
144 tests completed, 1 failed

FAILURE: Build failed with an exception.

* What went wrong:
Execution failed for task ':test'.
> There were failing tests. See the report at: file:///C:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/build/reports/tests/test/index.html
```
- **Test Results XML / HTML**:
  - Total Tests: 144
  - Passed: 143
  - Failed: 1 (`frc.robot.Auto.DriverAssistTest.testSmartAssistStandoffHoldPositionWithoutRestartStutter`)
  - File: `build/test-results/test/TEST-frc.robot.Auto.DriverAssistTest.xml`
  - Verbatim Assertion Error:
```
org.opentest4j.AssertionFailedError: With fuel held and active hub, Smart Assist must enter CYCLE_SCORE_HUB ==> expected: <CYCLE_SCORE_HUB> but was: <VACUUM_MIDFIELD>
	at app//org.junit.jupiter.api.AssertionFailureBuilder.build(AssertionFailureBuilder.java:151)
	at app//org.junit.jupiter.api.AssertionFailureBuilder.buildAndThrow(AssertionFailureBuilder.java:132)
	at app//org.junit.jupiter.api.AssertEquals.failNotEqual(AssertEquals.java:197)
	at app//org.junit.jupiter.api.AssertEquals.assertEquals(AssertEquals.java:182)
	at app//org.junit.jupiter.api.Assertions.assertEquals(Assertions.java:1156)
	at app//frc.robot.Auto.DriverAssistTest.testSmartAssistStandoffHoldPositionWithoutRestartStutter(DriverAssistTest.java:290)
```
- **Root Cause Code Reference**: `AutonomousTeleopAgent.java` lines 81–89:
```java
        // 1. Ingest Ground Truth Snapshot
        int heldCount;
        if (intake.getMapleIntakeSim() != null) {
            heldCount = intake.getMapleIntakeSim().getGamePiecesAmount();
        } else {
            if (!intake.hasFuel()) {
                estimatedHeldBalls = 0;
            }
            heldCount = intake.hasFuel() ? Math.max(1, estimatedHeldBalls) : estimatedHeldBalls;
        }
```
In `DriverAssistTest.java` lines 283–288:
```java
        // Give robot 1 ball so it wants to score at active hub
        agent.incrementBallCount();
        agent.startSmartAssist();
        agent.updateSmartAssist(0.0, 0.0, 0.0);

        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, agent.getActiveObjective(),
                "With fuel held and active hub, Smart Assist must enter CYCLE_SCORE_HUB");
```
When `agent.startSmartAssist()` calls `updateSmartAssist(0.0, 0.0, 0.0)`, `intake.getMapleIntakeSim()` is null in headless testing, and `intake.hasFuel()` is `false`. Because `!intake.hasFuel()` is true, line 86 wipes `estimatedHeldBalls = 0`. Consequently `heldCount = 0`, causing `JevDecisionEngine` to evaluate `StrategicObjective.VACUUM_MIDFIELD` instead of `StrategicObjective.CYCLE_SCORE_HUB`.

---

### 1.2 Documentation Generation Status (`gradlew javadoc`)

```powershell
cmd.exe /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& gradlew.bat javadoc -Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```
- **Exit Code**: `1`
- **Output Summary**: `31 errors, 100 warnings`
- **Key Verbatim Errors**:
  - `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\src\main\java\frc\robot\Utils\AllianceFlipUtil.java:113: error: malformed HTML` (`X <= 4.597m`)
  - `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\src\main\java\frc\robot\ThirdParty\LimelightHelpers.java:1534: error: unknown tag: throttle` (`<throttle>`)
  - `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\src\main\java\frc\robot\Subsystems\MatchCoach.java:313: error: bad HTML entity` (`arena & practice`)
  - `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\src\main\java\frc\robot\Sim\MatchScoreTracker.java:244: error: malformed HTML` (`t <= 20.0s`)
  - 100 warnings of `warning: no comment` on public/protected methods and constants across `Sim/AIRobotSim.java`, `Interfaces/Subsystem.java`, and `Utils/Alert.java`.

---

### 1.3 Dependencies & Vendor Library Inspection

| Vendordep File | Name | Version | frcYear | Actual Usage in Codebase |
|---|---|---|---|---|
| `AdvantageKit.json` | AdvantageKit | 26.0.2 | 2026 | Active. Core logging (`LoggedRobot`, `Logger`, `@AutoLog`, NT4Publisher, WPILOGWriter). Offline annotation processor jar present in `lib/akit-autolog-26.0.2.jar`. |
| `ChoreoLib2026.json` | ChoreoLib | 2026.0.3 | 2026 | Active. `FollowChoreoPath.java`, autonomous missions (`AdvancedChoreoMission`, `DynamicChoreoMission`), `AutoMissionChooser`. |
| `Phoenix6-replay-26.1.0.json` | CTRE-Phoenix (v6) Replay | 26.1.0 | 2026 | Active via YAGSL. CANcoder swerve absolute steering encoders (`frontleft.json`, `frontright.json`, etc.). |
| `Phoenix5-replay-5.36.0.json` | CTRE-Phoenix (v5) | 5.36.0 | 2026 | Legacy / Transitive. Mandated by `yagsl-2026.1.14.json` `requires` section. No direct imports in `src/`. |
| `REVLib.json` | REVLib | 2026.0.5 | 2026 | Active. Modern SparkMax API (`SparkMax`, `SparkMaxConfig`, `configure(config, ResetMode, PersistMode)`). Used in `NeoSparkMaxMotor`, `ShooterIOSparkMax`, `Intake`. |
| `Studica.json` | Studica | 2026.0.0 | 2026 | Active via YAGSL. NavX MXP IMU (`swervedrive.json`). |
| `ReduxLib-2026.1.2.json` | ReduxLib | 2026.1.2 | 2026 | Transitive. Mandated by `yagsl-2026.1.14.json` `requires` section. No direct imports in `src/`. |
| `ThriftyLib-2026.json` | ThriftyLib | 2026.1.2 | 2026 | Transitive. Mandated by `yagsl-2026.1.14.json` `requires` section. No direct imports in `src/`. |
| `WPILibNewCommands.json` | WPILib-New-Commands | 1.0.0 | 2026 | Active. Command-based framework. |
| `photonlib.json` | photonlib | v2026.3.4 | 2026 | Active. PhotonVision coprocessor vision targeting (`VisionIOPhotonVision.java`). |
| `yagsl-2026.1.14.json` | YAGSL | 2026.1.14 | 2026 | Active. Swerve drive subsystem (`SwerveBase.java`), IronMaple simulation physics (`swervelib.simulation.ironmaple`). |
| `yams.json` | Yet Another Mechanism System | 2026.9.16 | 2026 | **UNUSED**. Not imported anywhere in `src/`, not referenced in `deploy/`, has empty `"requires": []`. Dead dependency. |

---

### 1.4 Gradle & Build Script Configuration Observations

1. **Incremental Build Cache Invalidation**:
   `build.gradle` lines 121–176 defines:
   ```groovy
   task generateBuildConstants {
       doLast { ... }
   }
   compileJava.dependsOn generateBuildConstants
   ```
   Gradle `--info` logs confirm:
   - `Task :generateBuildConstants is not up-to-date because: Task has not declared any outputs despite executing actions.`
   - `Task :compileJava is not up-to-date because: Input property 'stableSources' file .../BuildConstants.java has changed.`
   Because `BUILD_DATE = new Date()` is written every execution and no task outputs are declared, Gradle rebuilds all Java classes on every build, defeating incremental compilation.

2. **Jar Packaging**:
   `build.gradle` line 102 specifies `duplicatesStrategy = DuplicatesStrategy.INCLUDE`. This can bundle duplicate class definitions into the deploy jar without warning. Best practice is `DuplicatesStrategy.EXCLUDE`.

3. **JDK Toolchain Alignment**:
   `build.gradle` specifies:
   ```groovy
   java {
       sourceCompatibility = JavaVersion.VERSION_17
       targetCompatibility = JavaVersion.VERSION_17
   }
   ```
   WPILib 2026 JDK at `C:\Users\Public\wpilib\2026\jdk` is OpenJDK 17.0.16 Temurin, perfectly aligned with Java 17.

---

## 2. Logic Chain

1. **From Observation 1.1**: Direct invocation of `.\gradlew build -Dorg.gradle.java.home=...` failed before Gradle started because Windows batch scripts do not parse JVM system properties passed on the command line. `gradlew.bat` checks for `%JAVA_HOME%` or `java.exe` in `PATH`.
2. **Inference**: In developer environments where WPILib VS Code is not used (e.g. standalone terminals, scripts, CI agents), builds fail immediately unless `JAVA_HOME` is exported or `gradlew.bat` includes an automatic fallback check for `C:\Users\Public\wpilib\2026\jdk`.
3. **From Observation 1.1**: With `JAVA_HOME` set, `compileJava` and all artifact assembly tasks pass with exit code 0. No missing symbols or compilation errors exist in main source.
4. **From Observation 1.1**: Test task executed 144 tests. 143 passed, 1 failed (`testSmartAssistStandoffHoldPositionWithoutRestartStutter`).
5. **From Observation 1.1 & Code Analysis**: `AutonomousTeleopAgent.java` lines 85–87 resets `estimatedHeldBalls = 0` whenever `!intake.hasFuel()`. In unit tests without simulated physical game pieces triggering the intake beam break, any programmatic ball increment is erased on the first cycle of `updateSmartAssist`. As a result, the strategic decision engine chooses `VACUUM_MIDFIELD` instead of `CYCLE_SCORE_HUB`.
6. **From Observation 1.2**: `gradlew javadoc` fails with 31 HTML syntax errors in existing comments and 100 missing-doc warnings, directly establishing the exact scope of work for Requirement R2.
7. **From Observation 1.3**: All active vendor dependencies (`AdvantageKit`, `ChoreoLib`, `REVLib`, `Phoenix6`, `Studica`, `PhotonLib`, `YAGSL`) are on current WPILib 2026 versions. `REVLib` is using the modern `SparkMax` and `SparkMaxConfig` 2026 API. `yams.json` is completely unused and can be pruned. `Phoenix5-replay` is required solely as a transitive descriptor for YAGSL.

---

## 3. Caveats

1. **Hardware In-The-Loop (HITL)**: This investigation was conducted in simulated / offline desktop mode using the WPILib 2026 Desktop toolchain and JNI libraries (`windowsx86-64`). Physical RoboRIO deploy (`deployfrcJavaroborio`) and real CAN bus timings were not tested on physical hardware.
2. **Transitive Vendordeps**: `Phoenix5-replay`, `ReduxLib`, and `ThriftyLib` cannot be removed from `vendordeps/` because `yagsl-2026.1.14.json` enforces their presence through its `"requires"` schema, even though the robot does not directly import them.
3. **No Code Modified**: In strict adherence to the read-only explorer constraint, no files in `TitanRoboticsBuildSeason/` were edited during this investigation.

---

## 4. Conclusion

1. **Baseline Compilation Status**:
   - **Main Java compilation (`compileJava`)**: **PASSED (100% clean)**.
   - **Jar assembly (`jar`, `assemble`)**: **PASSED**.
   - **Test compilation (`compileTestJava`)**: **PASSED**.
   - **Unit Tests (`test`)**: **143 PASSED, 1 FAILED (99.3% pass rate)**.
   - **Javadoc generation (`javadoc`)**: **FAILED (31 HTML entity errors, 100 missing comment warnings)**.
2. **Primary Issues Identified**:
   - `gradlew.bat` lacks fallback auto-detection of `C:\Users\Public\wpilib\2026\jdk`.
   - `AutonomousTeleopAgent.java` line 86 contains a logic bug wiping `estimatedHeldBalls` when `!intake.hasFuel()`, causing the single unit test failure.
   - `generateBuildConstants` lacks Gradle input/output annotations and regenerates files with timestamps on every build, preventing incremental compile caching.
   - Malformed HTML entities (`<`, `&`) in Javadocs break `gradlew javadoc`.
   - `yams.json` is dead dependency clutter.

---

## 5. Verification Method & Actionable Recommendations

### 5.1 Verification Commands
To independently verify these findings, run the following commands from `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason`:

1. **Baseline Build & Full Test Suite**:
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"; .\gradlew build -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"
```
*Expected Result*: `compileJava` passes; `test` completes with 144 tests, 1 failure (`testSmartAssistStandoffHoldPositionWithoutRestartStutter`), exit code 1.

2. **Isolated Failing Test**:
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"; .\gradlew test --tests frc.robot.Auto.DriverAssistTest.testSmartAssistStandoffHoldPositionWithoutRestartStutter -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"
```

3. **Javadoc Verification**:
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"; .\gradlew javadoc -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"
```
*Expected Result*: Fails with 31 malformed HTML errors and 100 missing comment warnings.

---

### 5.2 Proposed Implementation Modernizations

#### Proposal 1: Fix `gradlew.bat` WPILib JDK Auto-Detection
Add at line 41 of `gradlew.bat`:
```bat
@rem Auto-detect WPILib 2026 JDK if JAVA_HOME is not set
if not defined JAVA_HOME (
    if exist "C:\Users\Public\wpilib\2026\jdk\bin\java.exe" (
        set "JAVA_HOME=C:\Users\Public\wpilib\2026\jdk"
    )
)
```

#### Proposal 2: Fix `AutonomousTeleopAgent.java` Ball Count Reset Bug
Target: `src/main/java/frc/robot/Auto/AutonomousTeleopAgent.java` lines 85–88:
```java
// BEFORE:
if (!intake.hasFuel()) {
    estimatedHeldBalls = 0;
}
heldCount = intake.hasFuel() ? Math.max(1, estimatedHeldBalls) : estimatedHeldBalls;

// AFTER:
// Only trust beam break when it explicitly detects fuel; do not wipe manual/test estimate when beam break is clear
if (intake.hasFuel()) {
    estimatedHeldBalls = Math.max(1, estimatedHeldBalls);
}
heldCount = estimatedHeldBalls;
```

#### Proposal 3: Optimize `generateBuildConstants` for Incremental Compilation
Target: `build.gradle` lines 121–175:
Declare outputs and only overwrite files if content changes:
```groovy
task generateBuildConstants {
    def buildConstantsFile = file("src/main/java/frc/robot/BuildConstants.java")
    def deployJson = file("src/main/deploy/git_info.json")
    outputs.file buildConstantsFile
    outputs.file deployJson

    doLast {
        ...
        if (!buildConstantsFile.exists() || buildConstantsFile.text != newContent) {
            buildConstantsFile.text = newContent
        }
    }
}
```

#### Proposal 4: Clean Dead Vendordep
Delete `vendordeps/yams.json` as it is unused by any subsystem, command, or vendordep requirement.

#### Proposal 5: Escape HTML Entities in Javadocs
Replace unescaped `<` with `&lt;` or `{@code ...}` and `&` with `&amp;` in:
- `src/main/java/frc/robot/Utils/AllianceFlipUtil.java:113`
- `src/main/java/frc/robot/ThirdParty/LimelightHelpers.java:1534`
- `src/main/java/frc/robot/Subsystems/MatchCoach.java:313`
- `src/main/java/frc/robot/Sim/MatchScoreTracker.java:244`
