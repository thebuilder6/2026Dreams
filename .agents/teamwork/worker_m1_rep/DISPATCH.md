# Dispatch: Worker M1 Replacement (Build Toolchain & Dependency Modernization)

## Identity
- Role: Implementation Worker (Milestone 1 Replacement)
- Working directory: C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\worker_m1_rep
- Parent: orchestrator_1 (Conversation ID: 8b383374-9a92-411a-ba01-ca6b47135c41)

## Context & Inputs
- Read `C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\ORIGINAL_REQUEST.md`.
- Read `C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\orchestrator_1\PROJECT.md`.
- Read `C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_build_1\handoff.md`.
Codebase: `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason`.

## Write Ownership (Exclusively Owned Files)
- `gradlew.bat`
- `build.gradle`
- `vendordeps/yams.json`
- `src/main/java/frc/robot/Auto/AutonomousTeleopAgent.java`
- `src/test/java/frc/robot/Auto/DriverAssistTest.java`

## Mandatory Integrity Warning
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

## Tasks
1. **`gradlew.bat` WPILib 2026 JDK Fallback**:
   In `gradlew.bat` lines 41-54, if `JAVA_HOME` is not defined and `java.exe` is not found in PATH, check if `C:\Users\Public\wpilib\2026\jdk` exists. If so, set `JAVA_HOME=C:\Users\Public\wpilib\2026\jdk` and `JAVA_EXE=%JAVA_HOME%\bin\java.exe`. This ensures that invoking `.\gradlew build -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"` from any terminal succeeds immediately.
2. **`build.gradle` Optimization**:
   - In `generateBuildConstants`, declare outputs (`outputs.file("src/main/java/frc/robot/BuildConstants.java")`) so Gradle's incremental compilation caching functions properly.
   - In the `jar` task, change `duplicatesStrategy = DuplicatesStrategy.INCLUDE` to `DuplicatesStrategy.EXCLUDE`.
3. **Prune Dead Vendordeps**:
   - Delete unused `vendordeps/yams.json`.
4. **Fix Unit Test `DriverAssistTest`**:
   - In `AutonomousTeleopAgent.java` lines 81-89, fix the ball count handling so that `estimatedHeldBalls` is not wiped when `!intake.hasFuel()` (preserve estimated count or check `Math.max(estimatedHeldBalls, intake.hasFuel() ? 1 : 0)`).
   - In `DriverAssistTest.java`, verify that `testSmartAssistStandoffHoldPositionWithoutRestartStutter` passes cleanly along with all 144 unit tests.
5. **Build & Test Verification**:
   Execute from `C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason`:
   `cmd.exe /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& gradlew.bat build -Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"`
   AND verify direct execution:
   `gradlew.bat build -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"`
   Confirm exit code 0 and all 144 unit tests passing!

## Output
Write your handoff report to `C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\worker_m1_rep\handoff.md`.
When finished, notify your parent via `send_message` to conversation ID `8b383374-9a92-411a-ba01-ca6b47135c41`.
