# 2026/2027 Robot Software Architecture Review

## 1. Overview
This report reviews the current implementation of Team 8334's 2026/2027 robot platform against the documented architecture defined in `ARCHITECTURE.md`. The overall system correctly follows the specified layered architecture:
- **Decision & Tactical Strategy Layer:** Jev AI engine, Choreo Trajectory tracking, Auto Mission chooser.
- **Subsystem Logic Layer:** SwerveBase, Shooter, Intake, Vision, Diagnostics.
- **Hardware IO Abstraction:** AdvantageKit pattern used consistently across subsystems.
- **Telemetry & Visualization:** Elastic Dashboard, AdvantageScope, and `.wpilog` replay integrations.

## 2. Component Analysis & Compliance

### A. Decision & Autonomous Layer (Jev AI)
- **Status:** **Compliant with Minor Considerations**
- **Implementation:** `JevDecisionEngine.java`, `StrategicObjective.java`, `WorldState.java`, `MatchKnowledge.java`, `AIActionIntent.java`.
- **Observations:**
  - The System 1 (Tactical Reflex) and System 2 (Executive Strategy) split is well implemented. The engine statelessness contract is maintained by passing state variables (`ObjectiveCommitment`, `FuelTargetMemory`) from the agent to the engine.
  - The `MatchKnowledge` split (`ClairvoyantKnowledge` vs `ObservedKnowledge`) matches the documentation. The engine defaults to `ObservedKnowledge.selfOnly()` when null or in legacy overloads.
  - `AIActionIntent` effectively couples the strategy layer to the subsystem layer.
  - *Note:* `JevDecisionEngine` contains a hardcoded fallback to `StrategicObjective.RUSH_CLIMB` in the `evaluateLocalUtilityMatrix` fallback path. However, `RUSH_CLIMB` has a protective `if (archetype != Archetype.CO_PILOT)` block to prevent sim bots from climbing, which aligns with the safety requirement in `KNOWN_ISSUES.md`.

### B. Navigation & Pathfinding Layer
- **Status:** **Compliant**
- **Implementation:** `StaticPathfinder.java`, `DynamicRouter.java`, `ContactWatchdog.java`, `TargetProgressWatchdog.java`, `TrajectoryController.java`, `FieldMap.java`.
- **Observations:**
  - **Obstacle Avoidance:** `StaticPathfinder` correctly uses topological roadmaps and String-Pulling smoothing. `DynamicRouter` employs Artificial Potential Fields (APF) with the exact bounded derivative fixes (e.g., `REPULSION_MIN_DISTANCE_M = 0.35`) and the smooth `MAX_BACKPRESSURE_FRACTION` constraint described in `ARCHITECTURE.md`.
  - **Stuck Recovery:**
    - `ContactWatchdog` correctly aggregates stall, jerk, deadlock, and pinning detections.
    - `TargetProgressWatchdog` successfully tracks progress independently of peers, escalating to blacklisting targets when stuck.
  - **Geometry:** `FieldMap` centralizes coordinates (Blue-origin) with the single documented exception for `Depots`. The red alliance geometries are mirrored as expected.

### C. Subsystem Logic & IO Abstraction (AdvantageKit)
- **Status:** **Compliant**
- **Implementation:** `SwerveBase.java`, `Shooter.java`, `Intake.java`, `Vision.java`, `SubsystemManager.java`.
- **Observations:**
  - The codebase stringently adheres to the AdvantageKit IO abstraction pattern. Every major subsystem defines an IO interface (e.g., `DriveIO`, `ShooterIO`) and separates hardware (`*SparkMax`) from simulation (`*Sim`) code.
  - **SwerveBase:** Integrates YAGSL successfully. Applies correct brownout protection mapping simulated voltage/current to speed scales.
  - **Shooter:** Uses interpolated distance-to-RPM tables and incorporates Shooting-On-The-Fly (SOTF) kinematics accurately.
  - **Intake:** Follows state-machine design for articulated pivot, using `ProfiledPIDController` and gravity feedforward.
  - **Vision:** Multi-camera fusion (Limelight MegaTag2 + PhotonVision) implemented with `MedianFilter` gating and dynamic std-dev weighting.

### D. Simulation & Telemetry Layer
- **Status:** **Compliant**
- **Implementation:** `AIRobotSim`, `AIRobotInstance`, `GameSim`, `HubSchedule`, `MatchScoreTracker`, `Dashboard`.
- **Observations:**
  - The multi-robot sparring simulation correctly uses `AIRobotInstance` to spin up parallel sim bots.
  - The scoring attribution invariant (`player + bot_slots + unattributed = total_score`) in `MatchScoreTracker` is fully implemented and reconcilable.
  - The FRC 2026 Hub Schedule logic accurately accounts for the alternating shifts, transitional states, and 3-second grace periods as defined in `HubSchedule.java`.
  - Network port contention under headless matches is handled correctly in `Robot.java` by stopping `NetworkTableInstance` and suppressing desktop web servers.

## 3. Discrepancies & Recommendations
1. **Empty Knowledge Fallback Issue:** The `KNOWN_ISSUES.md` highlights that an empty `MatchKnowledge` object empties the utility matrix and defaults to `RUSH_CLIMB` in `evaluateLocalUtilityMatrix`. While the `RUSH_CLIMB` action is internally blocked for non-`CO_PILOT` archetypes, the fallback mechanic inside `evaluateLocalUtilityMatrix` simply falls back to the last evaluated key or default initialized key in the hash map. A dedicated `IDLE` or `VACUUM_MIDFIELD` default initialization inside the utility evaluator might provide safer behavior than implicitly defaulting to `RUSH_CLIMB`.
2. **Architecture Documentation vs Code Alignment:** Overall, the code and the `ARCHITECTURE.md` are in excellent alignment, specifically showing that recent fixes from late September (e.g. `TargetProgressWatchdog` loops, `DynamicRouter` APF derivative limits, `MatchScoreTracker` attributions) have been properly committed to the codebase.

## 4. Conclusion
The implementation tightly respects the defined architecture contracts. The separation of concerns between AI macro-strategy, navigational pathing, and subsystem execution is clean, performant, and highly testable.
