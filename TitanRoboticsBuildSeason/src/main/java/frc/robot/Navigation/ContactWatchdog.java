package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * ContactWatchdog unifies all contact / stall / pin safety response:
 *
 * <ul>
 *   <li>IMU jerk impact detection (from CollisionDetector).</li>
 *   <li>Current stall tracking {@code I > 28 A} (from CollisionDetector).</li>
 *   <li>Rule G418 1.8 s warning &amp; 2.4 s forced 3-foot backoff
 *       (from LegalPinningWatchdog).</li>
 *   <li>Deadlock yield-and-jink (from DeadlockResolver).</li>
 *   <li>Escalating pirouette escape (from TrajectoryController).</li>
 * </ul>
 *
 * <p>One singleton serves the player robot via {@link #getInstance()}; sim bots
 * construct their own instances (optionally with a seeded {@link java.util.Random}
 * for deterministic tests).
 */
public class ContactWatchdog {

    // ---- Collision / jerk (CollisionDetector parity) ----
    private static final double COLLISION_JERK_THRESHOLD = 120.0;
    private static final double COLLISION_DECEL_THRESHOLD = 10.0;
    private static final double COLLISION_DEBOUNCE_SEC = 0.35;

    // ---- Stall (CollisionDetector parity) ----
    private static final double STALL_CMD_SPEED_MIN = 1.00;
    private static final double STALL_ACTUAL_SPEED_MAX = 0.20;
    private static final double STALL_CURRENT_AMPS = 28.0;

    // ---- G418 pin (LegalPinningWatchdog parity) ----
    public static final double PIN_WARNING_THRESHOLD_SEC = 1.80;
    public static final double MAX_PIN_DURATION_SEC = 2.40;
    public static final double BACKOFF_DURATION_SEC = 3.00;
    public static final double BACKOFF_DISTANCE_METERS = 0.9144; // 3 feet

    // ---- Deadlock (DeadlockResolver parity) ----
    public static final double TRIGGER_STALL_SEC = 1.0;
    public static final double RECOVERY_SEC = 0.7;
    public static final double COOLDOWN_SEC = 2.0;
    public static final double PROXIMITY_M = 1.10;
    public static final double FORWARD_SCALE = 0.3;
    public static final double JINK_SPEED = 1.2;

    /** Recovery command for one tick (DeadlockResolver parity). */
    public record Resolution(boolean recovering, double forwardScale, double lateralJink) {}

    private static ContactWatchdog instance;

    public static synchronized ContactWatchdog getInstance() {
        if (instance == null) {
            instance = new ContactWatchdog();
        }
        return instance;
    }

    // Jerk filter state
    private double filteredAccelX = 0.0;
    private double filteredAccelY = 0.0;
    private double prevFilteredAccelX = 0.0;
    private double prevFilteredAccelY = 0.0;
    private double lastTimestamp = -1.0;
    private double lastCollisionTimestamp = -1.0;
    private double lastJerkMagnitude = 0.0;
    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();

    // Stall state
    private double stallStartTime = -1.0;
    private double stallDuration = 0.0;

    // Pin state
    private double pinDuration = 0.0;
    private double backoffTimer = 0.0;
    private boolean forcedBackoffActive = false;
    private Pose2d pinContactPose = new Pose2d();

    // Deadlock state
    private final java.util.Random random;
    private double deadlockStallTimeSec = 0.0;
    private double recoveryTimeSec = 0.0;
    private double cooldownTimeSec = 0.0;
    private double jinkSign = 1.0;
    private int recoveryCount = 0;

    // Pirouette escape state (TrajectoryController parity)
    private double unstickEndTime = -1.0;
    private Translation2d unstickVector = new Translation2d();
    private int unstickAttempts = 0;
    private double lastUnstickStartTime = -1.0;

    public ContactWatchdog() {
        this(new java.util.Random());
    }

    /** Injectable RNG for deterministic tests. */
    public ContactWatchdog(java.util.Random random) {
        this.random = (random != null) ? random : new java.util.Random();
        reset();
    }

    /**
     * Advances all contact monitors one tick. Call once per robot loop.
     *
     * @param currentPose current robot pose
     * @param actualVel measured field-relative chassis velocity
     * @param commandedVel requested field-relative chassis velocity
     * @param accelXG robot-forward IMU acceleration in G
     * @param accelYG robot-left IMU acceleration in G
     * @param driveCurrentAmps average drive current in amps
     * @param nearestPeerDist distance to closest peer robot (m, {@code Double.MAX_VALUE} if none)
     * @param opponentPose opponent pose for pin reference (may be null)
     * @param dt tick duration in seconds
     */
    public synchronized void update(
            Pose2d currentPose,
            ChassisSpeeds actualVel,
            ChassisSpeeds commandedVel,
            double accelXG,
            double accelYG,
            double driveCurrentAmps,
            double nearestPeerDist,
            Pose2d opponentPose,
            double dt) {
        if (currentPose == null) currentPose = new Pose2d();
        if (actualVel == null) actualVel = new ChassisSpeeds();
        if (commandedVel == null) commandedVel = new ChassisSpeeds();
        if (dt <= 1e-6) dt = 0.02;

        double now = Timer.getFPGATimestamp();
        double filterDt = lastTimestamp > 0.0 ? (now - lastTimestamp) : dt;
        if (filterDt < 1e-4) filterDt = dt;

        // ---- 1. Jerk / impact (CollisionDetector) ----
        Translation2d fieldAcceleration = new Translation2d(accelXG, accelYG)
                .rotateBy(currentPose.getRotation())
                .times(9.80665);
        double rawAx = fieldAcceleration.getX();
        double rawAy = fieldAcceleration.getY();
        if (Math.hypot(accelXG, accelYG) < 1e-3) {
            rawAx = (actualVel.vxMetersPerSecond - prevSpeeds.vxMetersPerSecond) / filterDt;
            rawAy = (actualVel.vyMetersPerSecond - prevSpeeds.vyMetersPerSecond) / filterDt;
        }
        double alpha = 0.35;
        filteredAccelX = alpha * rawAx + (1.0 - alpha) * filteredAccelX;
        filteredAccelY = alpha * rawAy + (1.0 - alpha) * filteredAccelY;

        double jerkX = (filteredAccelX - prevFilteredAccelX) / filterDt;
        double jerkY = (filteredAccelY - prevFilteredAccelY) / filterDt;
        lastJerkMagnitude = Math.hypot(jerkX, jerkY);

        double prevSpeed = Math.hypot(prevSpeeds.vxMetersPerSecond, prevSpeeds.vyMetersPerSecond);
        boolean isImpact = false;
        if (prevSpeed > 0.40) {
            double uVx = prevSpeeds.vxMetersPerSecond / prevSpeed;
            double uVy = prevSpeeds.vyMetersPerSecond / prevSpeed;
            double decelOpposing = -(filteredAccelX * uVx + filteredAccelY * uVy);
            double jerkOpposing = -(jerkX * uVx + jerkY * uVy);
            if (decelOpposing > COLLISION_DECEL_THRESHOLD && jerkOpposing > COLLISION_JERK_THRESHOLD) {
                isImpact = true;
            }
        } else if (Math.hypot(filteredAccelX, filteredAccelY) > 15.0
                && lastJerkMagnitude > (COLLISION_JERK_THRESHOLD * 1.5)) {
            isImpact = true;
        }

        if (isImpact && (now - lastCollisionTimestamp > COLLISION_DEBOUNCE_SEC)) {
            lastCollisionTimestamp = now;
            Translation2d dir = prevSpeed > 0.40
                    ? new Translation2d(prevSpeeds.vxMetersPerSecond, prevSpeeds.vyMetersPerSecond).div(prevSpeed)
                    : new Translation2d(-filteredAccelX, -filteredAccelY);
            if (dir.getNorm() > 1e-3) dir = dir.div(dir.getNorm());
            Translation2d obsPos = currentPose.getTranslation().plus(dir.times(0.65));
            DynamicRouter.registerObstacle(obsPos, new Translation2d(), 0.55, 0.65, true);
        }

        // ---- 2. Stall (CollisionDetector) ----
        double cmdSpeed = Math.hypot(commandedVel.vxMetersPerSecond, commandedVel.vyMetersPerSecond);
        double actSpeed = Math.hypot(actualVel.vxMetersPerSecond, actualVel.vyMetersPerSecond);
        boolean stallCondition = (cmdSpeed > STALL_CMD_SPEED_MIN)
                && (actSpeed < STALL_ACTUAL_SPEED_MAX)
                && (driveCurrentAmps > STALL_CURRENT_AMPS);
        if (stallCondition) {
            if (stallStartTime < 0) stallStartTime = now;
            stallDuration = now - stallStartTime;
            // Fallback when Timer is frozen (unit tests): integrate by dt.
            if (stallDuration < 1e-9) stallDuration += dt;
        } else {
            stallStartTime = -1.0;
            stallDuration = Math.max(0.0, stallDuration - dt * 2.0);
        }
        boolean stalled = stallDuration > 0.25;

        // ---- 3. Pin (LegalPinningWatchdog) ----
        boolean isContacting = false;
        if (opponentPose != null) {
            double distToOpp = currentPose.getTranslation().getDistance(opponentPose.getTranslation());
            if (distToOpp < 1.05 && stalled) isContacting = true;
            // Close shove without full stall still counts toward pin (matches AIRobotInstance heuristic).
            if (distToOpp < 0.95 && cmdSpeed > 0.5) isContacting = true;
        }
        if (!isContacting && nearestPeerDist < 1.05 && stalled) isContacting = true;

        if (forcedBackoffActive) {
            backoffTimer += dt;
            double distFromContact = currentPose.getTranslation().getDistance(pinContactPose.getTranslation());
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
                    pinContactPose = currentPose;
                }
            } else {
                pinDuration = Math.max(0.0, pinDuration - (dt * 2.0));
            }
        }

        // ---- 4. Deadlock (DeadlockResolver) ----
        Resolution deadlock = updateDeadlock(stalled, nearestPeerDist, dt);

        // ---- 5. Pirouette arming (TrajectoryController) ----
        if (stalled && now > unstickEndTime) {
            if (now - lastUnstickStartTime < 3.0) {
                unstickAttempts = Math.min(unstickAttempts + 1, 5);
            } else {
                unstickAttempts = 0;
            }
            lastUnstickStartTime = now;
            double unstickDuration = 0.40 + unstickAttempts * 0.20;
            unstickEndTime = now + unstickDuration;

            Translation2d vel = new Translation2d(
                    actualVel.vxMetersPerSecond, actualVel.vyMetersPerSecond);
            if (vel.getNorm() < 0.10) {
                vel = new Translation2d(commandedVel.vxMetersPerSecond, commandedVel.vyMetersPerSecond);
            }
            if (vel.getNorm() > 0.10) {
                Translation2d reverseDir = vel.div(vel.getNorm()).times(-1.0);
                if (unstickAttempts > 0) {
                    double escapeAngleDeg =
                            (unstickAttempts % 2 == 1 ? 1.0 : -1.0) * (60.0 + (unstickAttempts * 15.0));
                    reverseDir = reverseDir.rotateBy(Rotation2d.fromDegrees(escapeAngleDeg));
                }
                double escapeSpeed = 1.8 + unstickAttempts * 0.4;
                unstickVector = reverseDir.times(Math.min(escapeSpeed, 3.5));
            } else {
                double escapeAngle = (unstickAttempts * Math.PI / 3.0);
                unstickVector = new Translation2d(Math.cos(escapeAngle), Math.sin(escapeAngle)).times(2.2);
            }
        }

        prevFilteredAccelX = filteredAccelX;
        prevFilteredAccelY = filteredAccelY;
        prevSpeeds = actualVel;
        lastTimestamp = now;

        publishTelemetry(isImpact, deadlock);
    }

    /**
     * Arbitrates a commanded velocity through the safety stack.
     * Priority: forced G418 backoff &gt; pirouette escape &gt; deadlock
     * yield-and-jink &gt; commanded.
     */
    public synchronized ChassisSpeeds arbitrate(
            ChassisSpeeds commanded, Pose2d currentPose, Pose2d opponentPose) {
        if (commanded == null) commanded = new ChassisSpeeds();
        if (currentPose == null) currentPose = new Pose2d();
        double now = Timer.getFPGATimestamp();

        if (forcedBackoffActive) {
            Pose2d backoff = getBackOffTarget(currentPose, opponentPose);
            Translation2d dir = backoff.getTranslation().minus(currentPose.getTranslation());
            if (dir.getNorm() > 1e-4) dir = dir.div(dir.getNorm());
            return new ChassisSpeeds(dir.getX() * 1.5, dir.getY() * 1.5, 0.0);
        }

        if (now < unstickEndTime) {
            return new ChassisSpeeds(unstickVector.getX(), unstickVector.getY(), 6.0);
        }

        if (recoveryTimeSec > 0.0) {
            Translation2d jinkField =
                    new Translation2d(0, jinkSign * JINK_SPEED).rotateBy(currentPose.getRotation());
            return new ChassisSpeeds(
                    commanded.vxMetersPerSecond * FORWARD_SCALE + jinkField.getX(),
                    commanded.vyMetersPerSecond * FORWARD_SCALE + jinkField.getY(),
                    commanded.omegaRadiansPerSecond);
        }

        return commanded;
    }

    // ---- Deadlock internals (DeadlockResolver parity) ----

    private Resolution updateDeadlock(boolean stalled, double nearestPeerDist, double dt) {
        if (cooldownTimeSec > 0.0) {
            cooldownTimeSec -= dt;
            return idleResolution();
        }
        if (recoveryTimeSec > 0.0) {
            recoveryTimeSec -= dt;
            if (recoveryTimeSec <= 0.0) {
                cooldownTimeSec = COOLDOWN_SEC;
                return idleResolution();
            }
            return new Resolution(true, FORWARD_SCALE, jinkSign * JINK_SPEED);
        }
        if (stalled && nearestPeerDist < PROXIMITY_M) {
            deadlockStallTimeSec += dt;
        } else {
            deadlockStallTimeSec = 0.0;
        }
        if (deadlockStallTimeSec >= TRIGGER_STALL_SEC) {
            deadlockStallTimeSec = 0.0;
            recoveryTimeSec = RECOVERY_SEC;
            jinkSign = random.nextBoolean() ? 1.0 : -1.0;
            recoveryCount++;
            return new Resolution(true, FORWARD_SCALE, jinkSign * JINK_SPEED);
        }
        return idleResolution();
    }

    private static Resolution idleResolution() {
        return new Resolution(false, 1.0, 0.0);
    }

    /**
     * Single-tick deadlock helper for callers that already track stall
     * themselves (sim bots). Mirrors {@code DeadlockResolver.update}.
     */
    public synchronized Resolution updateDeadlockOnly(boolean stalled, double nearestPeerDist, double dt) {
        Resolution r = updateDeadlock(stalled, nearestPeerDist, dt);
        publishTelemetry(false, r);
        return r;
    }

    // ---- Queries (legacy parity) ----

    public synchronized boolean isWarningActive() {
        return !forcedBackoffActive && pinDuration >= PIN_WARNING_THRESHOLD_SEC;
    }

    public synchronized boolean isForcedBackoffActive() {
        return forcedBackoffActive;
    }

    public synchronized boolean isImpactDetected() {
        return (Timer.getFPGATimestamp() - lastCollisionTimestamp) < 0.20;
    }

    public synchronized boolean isStalled() {
        return stallDuration > 0.25;
    }

    public synchronized boolean isPirouetteActive() {
        return Timer.getFPGATimestamp() < unstickEndTime;
    }

    public synchronized boolean isDeadlockRecovering() {
        return recoveryTimeSec > 0.0;
    }

    public synchronized double getStallDuration() {
        return stallDuration;
    }

    public synchronized double getLastJerkMagnitude() {
        return lastJerkMagnitude;
    }

    public synchronized double getPinDuration() {
        return pinDuration;
    }

    public synchronized double getBackoffRemainingSec() {
        return forcedBackoffActive ? Math.max(0.0, BACKOFF_DURATION_SEC - backoffTimer) : 0.0;
    }

    public synchronized int getRecoveryCount() {
        return recoveryCount;
    }

    /**
     * Calculates required disengagement target at least 3 feet away along the
     * contact normal (LegalPinningWatchdog parity).
     */
    public synchronized Pose2d getBackOffTarget(Pose2d robotPose, Pose2d opponentPose) {
        if (robotPose == null) robotPose = new Pose2d();
        Translation2d rPos = robotPose.getTranslation();
        Translation2d oppPos =
                opponentPose != null ? opponentPose.getTranslation() : pinContactPose.getTranslation();

        Translation2d awayVector = rPos.minus(oppPos);
        double dist = awayVector.getNorm();
        Translation2d unitAway;
        if (dist > 1e-4) {
            unitAway = awayVector.div(dist);
        } else {
            unitAway = new Translation2d(-1.0, 0.0).rotateBy(robotPose.getRotation());
        }

        Translation2d backoffPos = rPos.plus(unitAway.times(1.05));

        double clampedX = Math.max(0.60, Math.min(15.94, backoffPos.getX()));
        double clampedY = Math.max(0.60, Math.min(7.65, backoffPos.getY()));

        Rotation2d faceOpponent = oppPos.minus(new Translation2d(clampedX, clampedY)).getAngle();
        return new Pose2d(clampedX, clampedY, faceOpponent);
    }

    public synchronized void reset() {
        filteredAccelX = 0.0;
        filteredAccelY = 0.0;
        prevFilteredAccelX = 0.0;
        prevFilteredAccelY = 0.0;
        stallStartTime = -1.0;
        stallDuration = 0.0;
        lastCollisionTimestamp = -1.0;
        lastTimestamp = -1.0;
        lastJerkMagnitude = 0.0;
        prevSpeeds = new ChassisSpeeds();
        pinDuration = 0.0;
        backoffTimer = 0.0;
        forcedBackoffActive = false;
        pinContactPose = new Pose2d();
        deadlockStallTimeSec = 0.0;
        recoveryTimeSec = 0.0;
        cooldownTimeSec = 0.0;
        unstickEndTime = -1.0;
        unstickVector = new Translation2d();
        unstickAttempts = 0;
        lastUnstickStartTime = -1.0;
    }

    private void publishTelemetry(boolean isImpact, Resolution deadlock) {
        SmartDashboard.putBoolean("PinWatchdog/IsWarning", isWarningActive());
        SmartDashboard.putBoolean("PinWatchdog/ForcedBackoff", forcedBackoffActive);
        SmartDashboard.putNumber("PinWatchdog/PinDurationSec", pinDuration);
        SmartDashboard.putNumber("PinWatchdog/BackoffRemainingSec", getBackoffRemainingSec());

        Logger.recordOutput("ContactWatchdog/JerkMagnitude", lastJerkMagnitude);
        Logger.recordOutput("ContactWatchdog/ImpactDetected", isImpactDetected());
        Logger.recordOutput("ContactWatchdog/StallDuration", stallDuration);
        Logger.recordOutput("ContactWatchdog/PinDurationSec", pinDuration);
        Logger.recordOutput("ContactWatchdog/IsWarning", isWarningActive());
        Logger.recordOutput("ContactWatchdog/ForcedBackoff", forcedBackoffActive);
        Logger.recordOutput("ContactWatchdog/DeadlockRecovering", deadlock.recovering());
        Logger.recordOutput("ContactWatchdog/PirouetteActive", isPirouetteActive());
        // Legacy keys for existing dashboards/tests.
        Logger.recordOutput("CollisionDetector/JerkMagnitude", lastJerkMagnitude);
        Logger.recordOutput("CollisionDetector/ImpactDetected", isImpactDetected());
        Logger.recordOutput("CollisionDetector/StallDuration", stallDuration);
        Logger.recordOutput("PinWatchdog/IsWarning", isWarningActive());
        Logger.recordOutput("PinWatchdog/ForcedBackoff", forcedBackoffActive);
        Logger.recordOutput("PinWatchdog/PinDurationSec", pinDuration);
    }
}
