# AdvantageScope / Elastic topic reference (mirrors SIMULATION_GUIDE.md, do not fork it).

## AdvantageScope 3D Field (2026 Rebuilt)

| Add             | Topic                                | Source                  |
|-----------------|--------------------------------------|-------------------------|
| Robot pose      | `/SmartDashboard/Field/Robot`        | Field2d main robot pose, set every update (`SwerveBase.java:777`) |
| Game pieces     | `Simulation/GamePieces`              | `GameSim.java:129` (`FieldSimulation/Fuel`), 54 standard |
| Intake arm      | `/Subsystems/Intake/ArmPose3d`       | standby 347 deg / ground 250 deg |
| Shooter         | `/Subsystems/Shooter/ShooterPose3d`  | flywheel pose           |

Pre-wired layout: `TitanRoboticsBuildSeason/advantagescope-layout.json`.

## Multi-bot (AdvantageScope + Elastic)

- Bot poses: `/RealOutputs/AI_Telemetry/Bot0..2/ActualPose`, `/RealOutputs/AI_Telemetry/Ally1..2/ActualPose` (`recordOutput` → `RealOutputs` table)
- Bot state: `/Simulation/BotN/StateDetail`, `/Score`, `/Fuel`, `/Archetype` (`SmartDashboard.put*` → `/SmartDashboard/Simulation/...`)
- Shot arcs: `/RealOutputs/FieldSimulation/SuccessfulShotsTrajectory`, `/RealOutputs/FieldSimulation/MissedShotsTrajectory`
- Bot state: `/Simulation/BotN/StateDetail`, `/Score`, `/Fuel`, `/Archetype`
- Aggregates: `/Simulation/TotalOpponentScore`, `/Simulation/TotalOpponentFuel`
- Field2d objects: `/SmartDashboard/Field/OpponentBot0..2`, `AllyBot1..2`

## Connections

- Elastic NT: `127.0.0.1:5810`; layout fetch: robot `WebServer` port `5800`, `Ctrl+D`.
- AdvantageScope NT: `127.0.0.1`.
- Sim reset: Elastic `Simulation & Match Info` tab only.
