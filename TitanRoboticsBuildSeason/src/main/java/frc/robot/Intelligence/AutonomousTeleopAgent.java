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
    // Co-Pilot's own objective commitment, mirroring the sim bots. Without it
    // the real robot re-decides every cycle and walks in circles between
    // "go score" and "go sweep" (the two sit ~0.02 apart in the utility matrix).
    private final ObjectiveCommitment objectiveCommitment = new ObjectiveCommitment();
    private AIActionIntent latestIntent = null;
    private boolean assistActive = false;
    private boolean breakoutTriggered = false;

    private final SwerveBase swerve = SwerveBase.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    private static final int MAX_FUEL_CAPACITY = frc.robot.Data.Constants.IntakeConstants.MAX_HELD_BALLS; // 30
    private int estimatedHeldBalls = 0;

    // Single owner for shared-authority thresholds (Teleop mirrors these —
    // do not re-hardcode 0.65/0.60/0.10 elsewhere).
    public static final double BREAKOUT_TRANSLATION = 0.65;
    public static final double BREAKOUT_ROTATION = 0.60;
    public static final double BLEND_MIN = 0.10;

    /**
     * Evaluates the Co-Pilot policy for the player robot and caches the result.
     *
     * @param heldBalls estimated fuel held in hopper
     * @return concrete intent (navigation target, aim override, subsystem commands)
     */
    public AIActionIntent getCoPilotIntent(int heldBalls) {
        WorldState world = WorldStateBuilder.buildForPlayerRobot(heldBalls);
        latestIntent = JevDecisionEngine.getInstance().evaluatePolicy(
                world, MatchKnowledge.unknown(), Archetype.CO_PILOT, null, null,
                objectiveCommitment);
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
        objectiveCommitment.reset();
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

        if (normDriverMag > BREAKOUT_TRANSLATION || normRotMag > BREAKOUT_ROTATION) {
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

        // CoPilot fire execution: route the intent's SHOOTING + feed request
        // through the Shooter state machine (which gates the kicker on flywheel
        // RPM error < 150). Execution interlocks mirror the driver path
        // (Teleop.shooterControl): alliance zone, open ceiling, live shooting
        // solution, live heading alignment. Driver authority is the assist hold
        // itself — releasing Right Bumper or a strong stick input breaks out
        // (Teleop resumes shooterControl), and Back/Start E-stops. Operator
        // MANUAL states are never overridden.
        boolean autoFeedActive = false;
        Shooter.ShooterState currentShooterState = shooter.getStateEnum();
        boolean operatorManual = currentShooterState == Shooter.ShooterState.MANUAL_FIRE
                || currentShooterState == Shooter.ShooterState.MANUAL_PREP;
        if (!operatorManual && assistActive
                && intent.shooterCommand() == Shooter.ShooterState.SHOOTING
                && intent.triggerFeedKicker()) {
            Pose2d livePose = swerve.getPose();
            Shooter.ShootingSolution solution = shooter.getLatestShootingSolution();
            boolean inZone = frc.robot.Navigation.FieldMap.AllianceZones.isInAllianceZone(
                    livePose, world.isRedAlliance());
            boolean openCeiling = !frc.robot.Navigation.FieldMap.Trenches.isLowClearance(livePose);
            boolean solutionOk = solution != null && solution.shotPossibility();
            boolean aligned = solutionOk && Math.abs(solution.shootingAngle()
                    .minus(livePose.getRotation()).getDegrees())
                    <= frc.robot.Subsystems.shooter.ShooterConstants.ALIGNMENT_HEADING_TOLERANCE_DEG;
            if (inZone && openCeiling && solutionOk && aligned) {
                shooter.setTargetRPM(solution.flywheelRpmLeft(), solution.flywheelRpmRight());
                shooter.shoot();
                autoFeedActive = true;
            } else {
                // Hold readiness without feeding: kicker only fires in SHOOTING.
                shooter.prepareToShoot();
            }
        }
        SmartDashboard.putBoolean("CoPilot/AutoFeedActive", autoFeedActive);

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
        objectiveCommitment.reset();
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
        // Canonical Blue point is (1.05, 2.80) — Y=2.80 keeps the robot center
        // outside the inflated BLUE_TOWER_POST_SOUTH (see GlidePoints). Derive Red
        // via AllianceFlipUtil so this fallback can never drift from GLIDE_POINTS.
        Pose2d blueCanonical = new Pose2d(1.05, 2.80, Rotation2d.fromDegrees(180));
        return frc.robot.Utils.AllianceFlipUtil.apply(blueCanonical, isRed);
    }
}
