package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Auto.Actions.*;
import frc.robot.Data.FieldMap;
import frc.robot.Data.GlideConstants;
import frc.robot.Interfaces.Actions;
import frc.robot.Sim.AIActionIntent;
import frc.robot.Sim.Archetype;
import frc.robot.Sim.JevDecisionEngine;
import frc.robot.Sim.StrategicObjective;
import frc.robot.Sim.WorldState;
import frc.robot.Sim.WorldStateBuilder;
import frc.robot.Subsystems.*;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * AutonomousTeleopAgent: Real-Robot Co-Pilot & One-Button Smart Assist.
 * 
 * Powered by Jev AI (System 1 Reflex + System 2 Executive Utility).
 * Features:
 * - Continuous alliance-aware evaluation via WorldStateBuilder.
 * - Shared Authority Blending: Feeds driver stick translation/rotation to active trajectory actions.
 * - Haptic telegraphing: Signals breakout and lock-on.
 * - Reactive macro-objective switching: Automatically shifts between Polar Standoff Hub, Staging,
 *   Feeder Restock, and Endgame Alliance Parking.
 */
public class AutonomousTeleopAgent {

    private static AutonomousTeleopAgent instance = null;

    public static AutonomousTeleopAgent getInstance() {
        if (instance == null) {
            instance = new AutonomousTeleopAgent();
        }
        return instance;
    }

    private Actions currentAction = null;
    private StrategicObjective activeObjective = null;
    private AIActionIntent latestIntent = null;
    private boolean assistActive = false;
    private boolean breakoutTriggered = false;

    private final SwerveBase swerve = SwerveBase.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();

    private static final int MAX_FUEL_CAPACITY = frc.robot.Data.Constants.IntakeConstants.MAX_HELD_BALLS; // 30
    private int estimatedHeldBalls = 0;

    /**
     * Activates Smart Assist mode. Evaluates current WorldState and launches the appropriate action.
     */
    public void startSmartAssist() {
        assistActive = true;
        breakoutTriggered = false;
        activeObjective = null;
        updateSmartAssist(0.0, 0.0, 0.0);
    }

    /**
     * Periodic update for Smart Assist while driver holds the assist button.
     *
     * @param driverForward Field-relative driver forward velocity command (m/s)
     * @param driverStrafe Field-relative driver strafe velocity command (m/s)
     * @param driverRotation Driver rotation rate command (rad/s)
     * @return true if assist continues running, false if finished or broken out
     */
    public boolean updateSmartAssist(double driverForward, double driverStrafe, double driverRotation) {
        if (!assistActive) return false;

        // 1. Ingest Ground Truth Snapshot
        int heldCount = estimatedHeldBalls;
        if (intake.getMapleIntakeSim() != null) {
            heldCount = Math.max(heldCount, intake.getMapleIntakeSim().getGamePiecesAmount());
        }
        if (intake.hasFuel()) {
            heldCount = Math.max(1, heldCount);
        }
        WorldState world = WorldStateBuilder.buildForPlayerRobot(heldCount);
        latestIntent = JevDecisionEngine.getInstance().evaluatePolicy(world, Archetype.CO_PILOT);
        StrategicObjective objective = latestIntent.objective();

        Logger.recordOutput("CoPilot/ActiveObjective", objective.name());
        Logger.recordOutput("CoPilot/Confidence", latestIntent.confidence());
        Logger.recordOutput("CoPilot/Rationale", latestIntent.rationale());
        Logger.recordOutput("CoPilot/NavigationTarget", latestIntent.navigationTarget());

        // 2. React to Strategic Objective Shifts
        if (currentAction == null || activeObjective != objective) {
            transitionToObjective(objective, latestIntent.navigationTarget());
        } else if (currentAction instanceof DriveToPoseAction dtp && latestIntent.navigationTarget() != null) {
            dtp.setTargetPose(latestIntent.navigationTarget());
        }

        // Apply Aim Override to Active Action (e.g. SOTF pointing at Hub while moving)
        if (currentAction instanceof DriveToPoseAction dtp) {
            if (latestIntent.aimOverride() != null) {
                final Rotation2d aim = latestIntent.aimOverride();
                dtp.setRotationOverride(() -> aim);
            } else {
                dtp.setRotationOverride(null);
            }
        }

        // 3. Inject Driver Authority into Active Action & Check Breakout
        double drvSpeed = Math.hypot(driverForward, driverStrafe);
        double maxSpeed = Math.max(0.1, frc.robot.Data.Constants.MAX_SPEED);
        double normDriverMag = drvSpeed / maxSpeed;
        double maxRotSpeed = Math.max(0.1, frc.robot.Data.Constants.MAX_ROTATION_SPEED);
        double normRotMag = Math.abs(driverRotation) / maxRotSpeed;

        if (normDriverMag > 0.65 || normRotMag > 0.60) {
            breakoutTriggered = true;
            stopAssist();
            return false;
        }

        if (currentAction instanceof DriveToPoseAction dtp) {
            dtp.setDriverInput(driverForward, driverStrafe, driverRotation);
        } else if (currentAction instanceof BallHuntAction bh) {
            bh.setDriverInput(driverForward, driverStrafe);
        }

        // 4. Update Active Action
        if (currentAction != null) {
            currentAction.update();
            if (currentAction instanceof BallHuntAction bh && bh.checkAndClearBallAcquired()) {
                incrementBallCount();
            }
            if (currentAction instanceof DriveToPoseAction dtp && dtp.isBreakoutRequested()) {
                breakoutTriggered = true;
                stopAssist();
                return false;
            }
            if (currentAction.isFinished()) {
                currentAction.done();
                currentAction = null;
                // If objective was finished (e.g. arrived at parking or finished scoring), evaluate next step
                if (objective == StrategicObjective.RUSH_CLIMB) {
                    stopAssist();
                    return false;
                }
            }
        }

        // 5. Co-Pilot Subsystem Reflex Automation
        manageSubsystems(latestIntent, world);

        return true;
    }

    private void transitionToObjective(StrategicObjective objective, Pose2d targetPose) {
        if (currentAction != null) {
            currentAction.done();
            currentAction = null;
        }

        activeObjective = objective;
        boolean isRed = AllianceFlipUtil.isRedAlliance();

        switch (objective) {
            case VACUUM_MIDFIELD:
                currentAction = new BallHuntAction();
                currentAction.start();
                break;

            case STOCKPILE_DEPOT:
                Pose2d depotPose = targetPose != null ? targetPose :
                        new Pose2d(FieldMap.Depots.getDepotApproach(isRed),
                                Rotation2d.fromDegrees(isRed ? 0.0 : 180.0));
                DriveToPoseAction depotAction = new DriveToPoseAction(depotPose);
                depotAction.setHoldPosition(true);
                currentAction = depotAction;
                currentAction.start();
                break;

            case CYCLE_SCORE_HUB:
            case STAGE_STANDOFF:
                Pose2d standoffPose = targetPose != null ? targetPose :
                        JevDecisionEngine.getInstance().calculatePolarStandoffPose(swerve.getPose(), isRed);
                DriveToPoseAction scoreAction = new DriveToPoseAction(standoffPose);
                scoreAction.setHoldPosition(true);
                currentAction = scoreAction;
                currentAction.start();
                break;

            case RUSH_CLIMB:
                // Target Alliance Parking Pose (no climber present)
                String parkKey = isRed ? "Red Right Side Climb" : "Blue Right Side Climb";
                Pose2d parkPose = GlideConstants.GLIDE_POINTS.containsKey(parkKey) ?
                        GlideConstants.GLIDE_POINTS.get(parkKey).pose() :
                        (targetPose != null ? targetPose : new Pose2d(isRed ? 15.48 : 1.05, 2.88, Rotation2d.fromDegrees(isRed ? 0 : 180)));
                DriveToPoseAction parkAction = new DriveToPoseAction(parkPose);
                parkAction.setHoldPosition(false);
                currentAction = parkAction;
                currentAction.start();
                break;

            default:
                if (targetPose != null) {
                    DriveToPoseAction defaultAction = new DriveToPoseAction(targetPose);
                    defaultAction.setHoldPosition(true);
                    currentAction = defaultAction;
                    currentAction.start();
                }
                break;
        }
    }

    private void manageSubsystems(AIActionIntent intent, WorldState world) {
        // Pre-spool flywheels during transit if approaching hub or shift is close
        if (intent.shooterCommand() == Shooter.ShooterState.PREPARING || 
            (intent.objective() == StrategicObjective.CYCLE_SCORE_HUB && world.isAllianceHubActive())) {
            double rpm = intent.targetFlywheelRPM() > 1000 ? intent.targetFlywheelRPM() : 3200.0;
            shooter.setTargetRPM(rpm, rpm);
            shooter.prepareToShoot();
        }

        if (intent.intakeCommand() == Intake.IntakeState.INTAKING || intent.objective() == StrategicObjective.VACUUM_MIDFIELD) {
            intake.setState(Intake.IntakeState.INTAKING);
        }
    }

    /**
     * Halts Smart Assist and clears any running actions.
     */
    public void stopAssist() {
        assistActive = false;
        activeObjective = null;
        if (currentAction != null) {
            currentAction.done();
            currentAction = null;
        }
    }

    public boolean isAssistActive() {
        return assistActive;
    }

    public boolean checkAndClearBreakout() {
        boolean wasBreakout = breakoutTriggered;
        breakoutTriggered = false;
        return wasBreakout;
    }

    public Actions getCurrentAction() {
        return currentAction;
    }

    public StrategicObjective getActiveObjective() {
        return activeObjective;
    }

    public AIActionIntent getLatestIntent() {
        return latestIntent;
    }

    public void incrementBallCount() {
        estimatedHeldBalls = Math.min(MAX_FUEL_CAPACITY, estimatedHeldBalls + 1);
    }

    public void decrementBallCount() {
        estimatedHeldBalls = Math.max(0, estimatedHeldBalls - 1);
    }

    public void resetBallCount() {
        estimatedHeldBalls = 0;
    }

    public int getEstimatedHeldBalls() {
        return estimatedHeldBalls;
    }
}