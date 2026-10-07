# Sprint 0 Tickets — Scaffold (goal: empty project compiles)

Demo: `.\gradlew compileJava --offline` green on template + swerve JSON present. Total: 5 pts.

---

## S0-1 Env + repo setup (1 pt) — no dependencies

**Description:** Prepare the machine and repo so every later ticket runs the same JDK/Gradle. Nothing compiles yet.

**Tasks:**
1. Install WPILib 2026 (VS Code extension + toolchain). Confirm these exist:
   - `C:\Users\Public\wpilib\2026\jdk` (JDK used for ALL builds)
   - `C:\Users\Public\wpilib\2026\maven` (local maven — what makes `--offline` work)
2. In every PowerShell shell, export first:
   ```powershell
   $env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
   $env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
   ```
3. Create project dir (e.g. `MinimalRobot/`). Write `.wpilib/wpilib_preferences.json`:
   ```json
   { "enableCppIntellisense": false, "currentLanguage": "java", "projectYear": "2026", "teamNumber": 8334 }
   ```

**Acceptance:**
- [ ] `java -version` reports the WPILib JDK (17.x from the path above).
- [ ] Prefs file present with team 8334 / java / 2026.

**Verify:** `java -version` + eyeball prefs file.

---

## S0-2 Gradle files (2 pts) — depends: S0-1

**Description:** Minimal GradleRIO build that compiles the WPILib template offline.

**Tasks:**
1. `settings.gradle`: plugin resolution order — `mavenLocal()`, `gradlePluginPortal()`, then `maven { url = <PUBLIC>/wpilib/2026/maven }` (Windows: read `PUBLIC` env, default `C:\Users\Public`). Resolve `PUBLIC`→`wpilib\2026` in code so mac/Linux teammates (`~/wpilib/2026`) also work.
2. `build.gradle`:
   - `plugins { id "java"; id "edu.wpi.first.GradleRIO" version "2026.2.1" }`
   - `java { sourceCompatibility = JavaVersion.VERSION_17; targetCompatibility = JavaVersion.VERSION_17 }`
   - `ROBOT_MAIN_CLASS = "frc.robot.Main"`; standard `deploy { targets { roborio { team = 8334; artifacts { frcJava(...); frcStaticFileDeploy(files = fileTree('src/main/deploy')) } } } }`
   - Deps: `implementation wpi.java.deps.wpilib()`, `implementation wpi.java.vendor.java()`, `testImplementation junit-jupiter:5.10.1` + `testRuntimeOnly junit-platform-launcher`; `test { useJUnitPlatform() }`.
   - AdvantageKit annotation processor (generates `*InputsAutoLogged` — Sprint 5 AK-1 has the snippet; add it now so IO work never blocks).
3. Generate the template robot (WPILib extension: "Create a new robot project") so `src/main/java/frc/robot/{Main,Robot}.java` exist as placeholders.

**Acceptance:**
- [ ] `.\gradlew compileJava --offline` green on the untouched template.
- [ ] Never invoked system `gradle`; never omitted `--offline`.

**Verify:** `.\gradlew compileJava --offline`

---

## S0-3 Vendordeps + swerve JSON (2 pts) — depends: S0-2

**Description:** Vendor libraries + YAGSL chassis config in place. No robot code yet.

**Tasks:**
1. Via WPILib extension ("Manage Vendor Libraries → Install"), add: **YAGSL `2026.1.14`**, **REVLib `2026.0.5`**, **AdvantageKit 26.x**, **WPILibNewCommands**. Do NOT add Choreo/Photon/Limelight/CTRE.
   - Trap: YAGSL's JSON `requires` REVLib + Phoenix + Redux + Thrifty JSONs to be present. Keep those JSON files in `vendordeps/` even though no code calls them.
2. Place all 8 files under `src/main/deploy/swerve/`: `swervedrive.json` (`imu.type navx`, `invertedIMU false`, 4 module names), `controllerproperties.json`, `modules/{physicalproperties,pidfproperties,frontleft,frontright,backleft,backright}.json`.
3. Fill module CAN IDs/encoder offsets/gear ratios via the YAGSL configurator against YOUR chassis. Never hand-guess offsets — a wrong offset drives sideways.

**Acceptance:**
- [ ] `vendordeps/` contains the 4 needed JSONs (+ required stubs); versions match above.
- [ ] `deploy/swerve/swervedrive.json` lists exactly the 4 module files present on disk.
- [ ] `.\gradlew compileJava --offline` still green.

**Verify:** `.\gradlew compileJava --offline` + `dir src\main\deploy\swerve\modules`
