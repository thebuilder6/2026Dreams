package frc.robot.HMI.Watchdogs;

import frc.robot.Telemetry.Dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * LegalPinningWatchdog enforces FRC G-rule pinning limits during close-quarters robot engagement:
 * - 1.8s Contact: Triggers high-frequency PIN_WARNING driver haptic pulse.
 * - 2.4s Contact: Forces autonomous disengagement / pin release.
 * - Automatic 3-foot (0.9144m) back-off hold for 3.0s cooldown to guarantee clearing the pin count.
 */
public class LegalPinningWatchdog {

    public static final double PIN_WARNING_THRESHOLD_SEC = 1.80;
    public static final double MAX_PIN_DURATION_SEC = 2.40;
    public static final double BACKOFF_DURATION_SEC = 3.00;
    public static final double BACKOFF_DISTANCE_METERS = 0.9144; // 3 feet

    private static LegalPinningWatchdog instance;

    public static synchronized LegalPinningWatchdog getInstance() {
        if (instance == null) {
            instance = new LegalPinningWatchdog();
        }
        return instance;
    }

    private double pinDuration = 0.0;
    private double backoffTimer = 0.0;
    private boolean forcedBackoffActive = false;
    private Pose2d pinContactPose = new Pose2d();

    public LegalPinningWatchdog() {
        reset();
    }

    /**
     * Updates pinning monitor state. Call once per 20ms robot loop.
     *
     * @param isContacting True if physical contact / drivetrain stall against opponent is detected
     * @param robotPose Current pose of our robot
     * @param opponentPose Current pose of the opponent robot (or contact center)
     * @param dt Time elapsed since last update in seconds (typically 0.02)
     */
    public synchronized void update(boolean isContacting, Pose2d robotPose, Pose2d opponentPose, double dt) {
        if (forcedBackoffActive) {
            backoffTimer += dt;
            double distFromContact = robotPose.getTranslation().getDistance(pinContactPose.getTranslation());

            // Disengage completes when 3.0s cooldown expires AND robot has backed off at least 3 feet
            if (backoffTimer >= BACKOFF_DURATION_SEC && distFromContact >= BACKOFF_DISTANCE_METERS) {
                forcedBackoffActive = false;
                backoffTimer = 0.0;
                pinDuration = 0.0;
            }
        } else {
            if (isContacting) {
                pinDuration += dt;
                if (pinDuration >= MAX_PIN_DURATION_SEC) {
                    forcedBackoffActive = true;
                    backoffTimer = 0.0;
                    pinContactPose = robotPose;
                }
            } else {
                // Decay pin duration when contact breaks
                pinDuration = Math.max(0.0, pinDuration - (dt * 2.0));
            }
        }

        publishTelemetry();
    }

    /**
     * Returns true if contact has exceeded warning threshold (1.8s) but has not yet triggered forced backoff.
     */
    public synchronized boolean isWarningActive() {
        return !forcedBackoffActive && pinDuration >= PIN_WARNING_THRESHOLD_SEC;
    }

    /**
     * Returns true if robot is in forced 3.0s backoff state to clear pin count.
     */
    public synchronized boolean isForcedBackoffActive() {
        return forcedBackoffActive;
    }

    public synchronized double getPinDuration() {
        return pinDuration;
    }

    public synchronized double getBackoffRemainingSec() {
        return forcedBackoffActive ? Math.max(0.0, BACKOFF_DURATION_SEC - backoffTimer) : 0.0;
    }

    /**
     * Calculates required disengagement target pose located at least 3 feet away along the contact normal.
     *
     * @param robotPose Current robot pose
     * @param opponentPose Current opponent pose
     * @return Safe back-off target Pose2d clamped within field borders
     */
    public synchronized Pose2d getBackOffTarget(Pose2d robotPose, Pose2d opponentPose) {
        Translation2d rPos = robotPose.getTranslation();
        Translation2d oppPos = opponentPose != null ? opponentPose.getTranslation() : pinContactPose.getTranslation();

        Translation2d awayVector = rPos.minus(oppPos);
        double dist = awayVector.getNorm();
        Translation2d unitAway;
        if (dist > 1e-4) {
            unitAway = awayVector.div(dist);
        } else {
            // Default backwards along robot heading
            unitAway = new Translation2d(-1.0, 0.0).rotateBy(robotPose.getRotation());
        }

        // Back up 1.05m (ensuring > 0.9144m clearance)
        Translation2d backoffPos = rPos.plus(unitAway.times(1.05));

        // Clamp to field perimeter
        double clampedX = Math.max(0.60, Math.min(15.94, backoffPos.getX()));
        double clampedY = Math.max(0.60, Math.min(7.65, backoffPos.getY()));

        // Keep robot facing opponent while reversing
        Rotation2d faceOpponent = oppPos.minus(new Translation2d(clampedX, clampedY)).getAngle();
        return new Pose2d(clampedX, clampedY, faceOpponent);
    }

    public synchronized void reset() {
        pinDuration = 0.0;
        backoffTimer = 0.0;
        forcedBackoffActive = false;
        pinContactPose = new Pose2d();
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("PinWatchdog/IsWarning", isWarningActive());
        SmartDashboard.putBoolean("PinWatchdog/ForcedBackoff", forcedBackoffActive);
        SmartDashboard.putNumber("PinWatchdog/PinDurationSec", pinDuration);
        SmartDashboard.putNumber("PinWatchdog/BackoffRemainingSec", getBackoffRemainingSec());

        Logger.recordOutput("PinWatchdog/IsWarning", isWarningActive());
        Logger.recordOutput("PinWatchdog/ForcedBackoff", forcedBackoffActive);
        Logger.recordOutput("PinWatchdog/PinDurationSec", pinDuration);
    }
}
