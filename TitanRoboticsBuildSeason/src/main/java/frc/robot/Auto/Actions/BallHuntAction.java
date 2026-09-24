package frc.robot.Auto.Actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Data.Constants;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Subsystems.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Intelligent Semi-Autonomous "Auto Ball Pick Up" Action (Ball Hunt).
 * 
 * Features:
 * 1. True 2D Holonomic Vectoring towards detected Fuel balls.
 * 2. Shared Driver Authority: Driver stick input guides the search area when no ball is visible,
 *    and smoothly blends with vision tracking when locked on.
 * 3. Target Memory Hysteresis: Bridges camera blindspot right as the ball dips under the bumper into the rollers.
 * 4. Haptic Feedback Notification: Signals the driver controller upon successful ball ingestion.
 */
public class BallHuntAction implements Actions {

    private final SwerveBase swerve = SwerveBase.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Vision vision = Vision.getInstance();
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");

    // PID controller for centering camera yaw onto the ball
    private final PIDController turnController = new PIDController(0.08, 0, 0.005);

    // Speed configuration
    private static final double MAX_PURSUIT_SPEED = 2.8; // m/s
    private static final double MIN_INGESTION_SPEED = 1.2; // m/s into bumper

    // Target tracking & blindspot memory
    private double lastSeenTimestamp = -1.0;
    private Translation2d lastKnownDir = new Translation2d(1.0, 0.0);
    private double lastKnownDistance = 0.0;
    private static final double TARGET_MEMORY_WINDOW_SEC = 0.35; // 350ms memory

    // Driver assist blending
    private double driverForward = 0.0;
    private double driverStrafe = 0.0;

    // Ball acquisition state
    private boolean ballAcquiredPulse = false;
    private boolean wasTargetLocked = false;
    private boolean wasHoldingFuel = false;
    private double sweepPhase = 0.0;

    public BallHuntAction() {
        turnController.setSetpoint(0);
        turnController.setTolerance(1.5); // 1.5 degrees
    }

    /**
     * Injects driver translation inputs to allow shared steering / search authority.
     */
    public void setDriverInput(double forward, double strafe) {
        this.driverForward = forward;
        this.driverStrafe = strafe;
    }

    @Override
    public void start() {
        intake.setState(IntakeState.INTAKING);
        lastSeenTimestamp = -1.0;
        ballAcquiredPulse = false;
        wasTargetLocked = false;
        wasHoldingFuel = intake.hasFuel();
    }

    @Override
    public void update() {
        // Guarantee intake rollers and deployed arm position stay continuously active
        intake.setState(IntakeState.INTAKING);

        // Check hardware / intake simulation fuel sensor
        boolean currentlyHoldingFuel = intake.hasFuel();
        if (!wasHoldingFuel && currentlyHoldingFuel) {
            ballAcquiredPulse = true;
        }
        wasHoldingFuel = currentlyHoldingFuel;

        double now = Timer.getFPGATimestamp();
        boolean hasBall = vision.hasGamePiece();
        double driverSpeedCmd = Math.hypot(driverForward, driverStrafe);

        if (hasBall) {
            // =================================================================
            // CASE 1: Active Visual Target Lock
            // =================================================================
            double yaw = vision.getGamePieceYaw();
            double distance = vision.getGamePieceDistanceMeters();
            Translation2d robotRel = vision.getGamePieceRobotRelativeTranslation();

            lastSeenTimestamp = now;
            lastKnownDistance = distance;
            wasTargetLocked = true;

            // Rotation Output: Center ball in intake frame
            double rotationOutput = -turnController.calculate(yaw, 0);

            // Vector Direction: Normalize 2D vector towards ball
            Translation2d normDir = (robotRel.getNorm() > 1e-4)
                    ? robotRel.div(robotRel.getNorm())
                    : new Translation2d(1.0, 0.0);
            lastKnownDir = normDir;

            // Speed Scaling: Smooth deceleration with distance, but maintaining vigorous ingestion velocity
            double pursuitSpeed = Math.min(MAX_PURSUIT_SPEED, Math.max(MIN_INGESTION_SPEED, distance * 1.8));

            // If driver is pressing stick forward in the general direction, boost speed
            if (driverForward > 0.15) {
                pursuitSpeed = Math.min(Constants.MAX_SPEED * 0.85, pursuitSpeed + driverForward * 1.5);
            }

            double forwardSpeed = normDir.getX() * pursuitSpeed;
            double strafeSpeed = normDir.getY() * pursuitSpeed;

            swerve.drive(new Translation2d(forwardSpeed, strafeSpeed), rotationOutput, false);

            Logger.recordOutput("Vision/BallHunt/State", "LOCKED_PURSUIT");
            Logger.recordOutput("Vision/BallHunt/TargetDistance", distance);

        } else if (wasTargetLocked && (now - lastSeenTimestamp < TARGET_MEMORY_WINDOW_SEC)) {
            // =================================================================
            // CASE 2: Blindspot Ingestion (Ball passed under camera into rollers)
            // =================================================================
            // Robot drives forward along the last known direction at ingestion speed
            // to ensure ball completes intake cycle
            swerve.drive(lastKnownDir.times(MIN_INGESTION_SPEED), 0.0, false);

            Logger.recordOutput("Vision/BallHunt/State", "BLINDSPOT_INGESTION");

        } else {
            // Target was either just ingested or no ball was ever in frame
            if (wasTargetLocked && lastKnownDistance < 0.65) {
                // Ball was close and disappeared into bumper -> Confirmed Ingested!
                ballAcquiredPulse = true;
            }
            wasTargetLocked = false;

            // =================================================================
            // CASE 3: Search / Driver Navigation Mode
            // =================================================================
            if (driverSpeedCmd > 0.08) {
                // Shared authority: Driver steers robot to search areas, vision continuously seeks
                swerve.drive(new Translation2d(driverForward, driverStrafe), 0.0, true);
                Logger.recordOutput("Vision/BallHunt/State", "DRIVER_GUIDED_SEARCH");
            } else {
                // Stationary auto-sweep: Gently oscillate heading (+/- 15 deg) to detect nearby fuel
                sweepPhase += 0.02 * 3.0; // 3 rad/s oscillation
                double sweepRot = Math.sin(sweepPhase) * 0.75;
                swerve.drive(new Translation2d(0.0, 0.0), sweepRot, false);
                Logger.recordOutput("Vision/BallHunt/State", "AUTO_SWEEP");
            }
        }

        Logger.recordOutput("Vision/BallHunt/BallAcquiredPulse", ballAcquiredPulse);
    }

    /**
     * Checks if a ball was successfully ingested and resets the pulse flag.
     * Used by Teleop to trigger tactile rumble on the controller.
     */
    public boolean checkAndClearBallAcquired() {
        if (ballAcquiredPulse) {
            ballAcquiredPulse = false;
            return true;
        }
        return false;
    }

    public boolean isTargetLocked() {
        return wasTargetLocked;
    }

    @Override
    public boolean isFinished() {
        // Controlled by driver button hold (Left Bumper)
        return false;
    }

    @Override
    public void done() {
        swerve.stop();
        intake.setState(IntakeState.STANDBY);
        wasTargetLocked = false;
        Logger.recordOutput("Vision/BallHunt/State", "INACTIVE");
    }
}
