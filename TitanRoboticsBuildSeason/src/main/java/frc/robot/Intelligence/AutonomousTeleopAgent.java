package frc.robot.Intelligence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.*;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * AutonomousTeleopAgent: pure intent coordinator for the real-robot Co-Pilot.
 *
 * <p>Phase 4: no action wrappers. Teleop fetches an {@link AIActionIntent} via
 * {@link #getCoPilotIntent(int)} and executes it directly through
 * {@code TrajectoryController} + {@code ContactWatchdog}. This class only
 * evaluates the Jev policy, tracks ball bookkeeping, and runs subsystem reflexes.
 */
public class AutonomousTeleopAgent {

    private static AutonomousTeleopAgent instance = null;

    public static synchronized AutonomousTeleopAgent getInstance() {
        if (instance == null) {
            instance = new AutonomousTeleopAgent();
        }
        return instance;
    }

    private StrategicObjective activeObjective = null;
    private AIActionIntent latestIntent = null;
    private boolean assistActive = false;
    private boolean breakoutTriggered = false;

    private final SwerveBase swerve = SwerveBase.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    private static final int MAX_FUEL_CAPACITY = frc.robot.Data.Constants.IntakeConstants.MAX_HELD_BALLS; // 30
    private int estimatedHeldBalls = 0;

    /**
     * Evaluates the Co-Pilot policy for the player robot and caches the result.
     *
     * @param heldBalls estimated fuel held in hopper
     * @return concrete intent (navigation target, aim override, subsystem commands)
     */
    public AIActionIntent getCoPilotIntent(int heldBalls) {
        WorldState world = WorldStateBuilder.buildForPlayerRobot(heldBalls);
        latestIntent = JevDecisionEngine.getInstance().evaluatePolicy(
                world, MatchKnowledge.unknown(), Archetype.CO_PILOT);
        activeObjective = latestIntent.objective();

        SmartDashboard.putString("CoPilot/CurrentObjective", activeObjective.name());
        SmartDashboard.putString("CoPilot/NextObjective", latestIntent.plan().nextObjective().name());
        SmartDashboard.putNumber("CoPilot/TimeToTransitionSec", latestIntent.plan().timeToTransitionSec());

        Logger.recordOutput("CoPilot/ActiveObjective", activeObjective.name());
        Logger.recordOutput("CoPilot/Confidence", latestIntent.confidence());
        Logger.recordOutput("CoPilot/Rationale", latestIntent.rationale());
        Logger.recordOutput("CoPilot/NavigationTarget", latestIntent.navigationTarget());

        manageSubsystems(latestIntent, world);
        return latestIntent;
    }

    /** Resolves held balls from bookkeeping fused with intake sensing. */
    public int resolveHeldBalls() {
        int heldCount = estimatedHeldBalls;
        if (intake.getMapleIntakeSim() != null) {
            heldCount = Math.max(heldCount, intake.getMapleIntakeSim().getGamePiecesAmount());
        }
        if (intake.hasFuel()) {
            heldCount = Math.max(1, heldCount);
        }
        return heldCount;
    }

    /**
     * Activates Smart Assist mode (intent evaluation lifecycle for Teleop).
     */
    public void startSmartAssist() {
        assistActive = true;
        breakoutTriggered = false;
        activeObjective = null;
        getCoPilotIntent(resolveHeldBalls());
    }

    /**
     * Periodic update for Smart Assist while driver holds the assist button.
     *
     * <p>Evaluates fresh intent, applies breakout detection, and runs subsystem
     * reflexes. Trajectory execution lives in Teleop via TrajectoryController.
     *
     * @param driverForward Field-relative driver forward velocity command (m/s)
     * @param driverStrafe Field-relative driver strafe velocity command (m/s)
     * @param driverRotation Driver rotation rate command (rad/s)
     * @return true if assist continues running, false if broken out
     */
    public boolean updateSmartAssist(double driverForward, double driverStrafe, double driverRotation) {
        if (!assistActive) return false;

        getCoPilotIntent(resolveHeldBalls());

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

        if (latestIntent != null && latestIntent.objective() == StrategicObjective.RUSH_CLIMB
                && latestIntent.navigationTarget() != null
                && swerve.getPose().getTranslation()
                        .getDistance(latestIntent.navigationTarget().getTranslation()) < 0.12) {
            stopAssist();
            return false;
        }

        return true;
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
     * Halts Smart Assist.
     */
    public void stopAssist() {
        assistActive = false;
        activeObjective = null;
    }

    public boolean isAssistActive() {
        return assistActive;
    }

    public boolean checkAndClearBreakout() {
        boolean wasBreakout = breakoutTriggered;
        breakoutTriggered = false;
        return wasBreakout;
    }

    public StrategicObjective getActiveObjective() {
        return activeObjective;
    }

    public AIActionIntent getLatestIntent() {
        return latestIntent;
    }

    /** @deprecated Phase 4 removed action wrappers; Teleop executes intent directly. */
    @Deprecated
    public frc.robot.Interfaces.Actions getCurrentAction() {
        return null;
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

    /** Parking fallback used by Teleop when intent lacks a target (no climber present). */
    public static Pose2d getParkingFallback(boolean isRed, Pose2d targetFallback) {
        String parkKey = isRed ? "Red Right Side Climb" : "Blue Right Side Climb";
        if (frc.robot.Navigation.GlidePoints.GLIDE_POINTS.containsKey(parkKey)) {
            return frc.robot.Navigation.GlidePoints.GLIDE_POINTS.get(parkKey).pose();
        }
        if (targetFallback != null) return targetFallback;
        return new Pose2d(isRed ? 15.48 : 1.05, 2.88, Rotation2d.fromDegrees(isRed ? 0 : 180));
    }
}
