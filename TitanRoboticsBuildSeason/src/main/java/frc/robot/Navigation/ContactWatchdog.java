package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Telemetry.TelemetryKeys;
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
    // CMD threshold matches AIRobotInstance.isStalled so the bot-local stall flag
    // and the watchdog agree. 0.80 was too high: TrajectoryController commands as
    // little as 0.25 m/s (its carpet-friction breakout floor) and TRENCH_YIELD scales
    // by 0.2x, so a robot wedged while commanding 0.25-0.79 m/s was invisible here.
    // This is a "hold" floor, not a stall threshold: a true hold commands ~0 and still
    // decays, and the measured side is already permissive (STALL_ACTUAL_SPEED_MAX).
    // Same reasoning as TargetProgressWatchdog.COMMAND_MIN_MPS.
    public static final double STALL_CMD_SPEED_MIN = 0.12;
    public static final double STALL_ACTUAL_SPEED_MAX = 0.20;
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
    // Trench single-lane: lateral jink drives into the truss walls, so recovery
    // reverses out instead (negative scale) and triggers faster (0.5 s).
    // Recovery/cooldown are jittered per bot so head-on pairs desynchronize
    // instead of re-entering in lockstep, and consecutive failed attempts
    // escalate (longer reverse) until stall clears.
    public static final double TRENCH_TRIGGER_STALL_SEC = 0.5;
    public static final double TRENCH_FORWARD_SCALE = -0.8;
    public static final double TRENCH_RECOVERY_JITTER_SEC = 0.5;
    public static final double TRENCH_COOLDOWN_JITTER_SEC = 1.0;
    public static final double TRENCH_ESCALATION_STEP_SEC = 0.2;
    public static final int TRENCH_ESCALATION_MAX = 3;

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
    private Pose2d lastPose = new Pose2d();

    // Deadlock state
    private final java.util.Random random;
    private double deadlockStallTimeSec = 0.0;
    private double recoveryTimeSec = 0.0;
    private double cooldownTimeSec = 0.0;
    private double jinkSign = 1.0;
    private double recoveryForwardScale = FORWARD_SCALE;
    private double recoveryLateralJink = 0.0;
    private boolean recoveryTrenchActive = false;
    private int trenchEscalation = 0;
    private int recoveryCount = 0;

    // Pirouette escape state (TrajectoryController parity)
    /**
     * Escape spin rate when a peer is close enough to be the wedge (pin or
     * deadlock context). Preserved: rotating out of bumper contact needs the
     * aggressive rate.
     */
    public static final double UNSTICK_SPIN_OMEGA_RPS = 6.0;
    /**
     * Escape spin cap against pure geometry (no peer within {@link #PROXIMITY_M}).
     * A wall/hub/trench wedge is escaped by translation; the heading only turns
     * to face the escape direction. The old unconditional 6.0 rad/s made every
     * static-wedge escape a ~1 rev/s pirouette (measured as the dominant
     * spinning in seed-2026 headless replays).
     */
    public static final double UNSTICK_FACE_OMEGA_MAX_RPS = 2.5;
    /** Proportional gain steering the heading toward the escape direction. */
    public static final double UNSTICK_FACE_KP = 3.0;

    private double unstickEndTime = -1.0;
    private Translation2d unstickVector = new Translation2d();
    private int unstickAttempts = 0;
    private double lastUnstickStartTime = -1.0;

    /**
     * Creates a watchdog whose jitter draws from the shared scenario RNG.
     *
     * <p>The jink and cooldown jitter are intentional (they desynchronise
     * head-on trench pairs), but an unseeded {@link java.util.Random} made a
     * scenario seed meaningless. The stream name keeps each robot's draws
     * independent, so one bot's recovery cannot shift another's.
     *
     * @param streamName per-robot generator name, e.g. {@code "watchdog:Bot0"}
     */
    public ContactWatchdog(String streamName) {
        this(RobotBase.isSimulation()
                ? frc.robot.Sim.MatchDeterminism.random(streamName)
                : new java.util.Random());
    }

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
        this.lastPose = currentPose;
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
        // Trench-aware: single-lane corridors reverse out instead of jinking.
        boolean inTrenchCorridor =
                frc.robot.Navigation.FieldMap.Trenches.isLowClearance(currentPose.getTranslation());
        Resolution deadlock = updateDeadlock(stalled, nearestPeerDist, dt, inTrenchCorridor);

        // ---- 5. Pirouette arming (TrajectoryController) ----
        updateUnstick(stalled, actualVel, commandedVel, now);

        prevFilteredAccelX = filteredAccelX;
        prevFilteredAccelY = filteredAccelY;
        prevSpeeds = actualVel;
        lastTimestamp = now;

        publishTelemetry(isImpact, deadlock);
    }

    /**
     * Advances stall detection and pirouette arming only, with no peer-contact,
     * pinning, or G418 state involved.
     *
     * <p>This exists because a static-obstacle wedge (hub core, ramp, trench wall,
     * tower post) is peer-independent: {@link #updateDeadlock} only accumulates
     * when a peer is within {@link #PROXIMITY_M}, so a bot wedged alone against
     * geometry never armed an escape from the peer path. Previously the only
     * escape for that case was the peer-independent
     * {@link frc.robot.Navigation.TargetProgressWatchdog}, and the sim bots
     * additionally skipped the full {@link #update} entirely for allies, so the
     * escalating pirouette was never applied to their commands at all.
     *
     * <p>Deliberately excludes pin/backoff: {@code update} arms a forced 3-foot
     * G418 backoff from stall + proximity, and applying that to allies (who are
     * not pinning an opponent) would produce rule violations rather than
     * recoveries. Only the geometry-escape state is shared.
     *
     * @param actualVel    measured field-relative chassis velocity
     * @param commandedVel requested field-relative chassis velocity
     */
    public synchronized void updateUnstickOnly(
            ChassisSpeeds actualVel,
            ChassisSpeeds commandedVel,
            double dt) {
        if (actualVel == null) actualVel = new ChassisSpeeds();
        if (commandedVel == null) commandedVel = new ChassisSpeeds();
        if (dt <= 1e-6) dt = 0.02;

        double now = Timer.getFPGATimestamp();
        double cmdSpeed = Math.hypot(commandedVel.vxMetersPerSecond, commandedVel.vyMetersPerSecond);
        double actSpeed = Math.hypot(actualVel.vxMetersPerSecond, actualVel.vyMetersPerSecond);
        boolean stalled = (cmdSpeed > STALL_CMD_SPEED_MIN)
                && (actSpeed < STALL_ACTUAL_SPEED_MAX);

        updateUnstick(stalled, actualVel, commandedVel, now);
        prevSpeeds = actualVel;
        lastTimestamp = now;
    }

    /**
     * Arms or extends the escalating pirouette when the robot is stalled.
     *
     * <p>Shared by {@link #update} (opponents, which additionally get pin and
     * deadlock state) and {@link #updateUnstickOnly} (allies, geometry only), so
     * both agree on what a stall is and on the escape geometry.
     */
    private void updateUnstick(
            boolean stalled, ChassisSpeeds actualVel, ChassisSpeeds commandedVel, double now) {
        if (!stalled || now <= unstickEndTime) {
            return;
        }
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

        if (lastPose != null && lastPose.getTranslation().getNorm() > 1e-4) {
            unstickVector = ensureWallSafe(lastPose, unstickVector);
        }
    }

    private static final double WALL_MARGIN_M = 0.50;

    /**
     * Deflects unstick vector if it points directly into a field perimeter wall,
     * sliding along the wall toward the center rather than pinning the chassis further.
     */
    static Translation2d ensureWallSafe(Pose2d pose, Translation2d vector) {
        if (pose == null || vector == null || vector.getNorm() < 1e-4) {
            return vector;
        }
        double x = pose.getX();
        double y = pose.getY();
        double vx = vector.getX();
        double vy = vector.getY();
        double speed = vector.getNorm();

        boolean hitWall = false;
        if ((x < WALL_MARGIN_M && vx < 0) || (x > FieldMap.FIELD_LENGTH - WALL_MARGIN_M && vx > 0)) {
            vx = 0.0;
            hitWall = true;
        }
        if ((y < WALL_MARGIN_M && vy < 0) || (y > FieldMap.FIELD_WIDTH - WALL_MARGIN_M && vy > 0)) {
            vy = 0.0;
            hitWall = true;
        }
        if (hitWall) {
            Translation2d toCenter = new Translation2d(
                    FieldMap.FIELD_LENGTH / 2.0 - x,
                    FieldMap.FIELD_WIDTH / 2.0 - y);
            if (Math.abs(vx) < 1e-4 && Math.abs(vy) < 1e-4) {
                Translation2d unitToCenter = toCenter.div(Math.max(1e-4, toCenter.getNorm()));
                return unitToCenter.times(speed);
            }
            Translation2d deflected = new Translation2d(vx, vy);
            return deflected.div(deflected.getNorm()).times(speed);
        }
        return vector;
    }

    /**
     * Applies the pirouette escape if one is latched, otherwise returns the
     * command unchanged.
     *
     * <p>Separate from {@link #arbitrate} so a caller that has already arbitrated
     * peer corrections (deadlock, trench yield, separation) can add the
     * peer-independent geometry escape without also re-deriving pin and deadlock
     * state, which would apply them twice.
     */
    public synchronized ChassisSpeeds applyUnstickOnly(ChassisSpeeds commanded) {
        return applyUnstickOnly(commanded, Double.MAX_VALUE, null);
    }

    /**
     * Peer-independent geometry escape with a peer-gated spin rate.
     *
     * @param commanded        fallback speeds when no escape is latched
     * @param nearestPeerDistM distance to the nearest peer robot, or
     *                         {@link Double#MAX_VALUE} when unknown/alone
     * @param robotHeading     current heading, used to face the escape
     *                         direction; {@code null} holds heading
     */
    public synchronized ChassisSpeeds applyUnstickOnly(
            ChassisSpeeds commanded, double nearestPeerDistM, Rotation2d robotHeading) {
        ChassisSpeeds base = (commanded != null) ? commanded : new ChassisSpeeds();
        if (Timer.getFPGATimestamp() < unstickEndTime) {
            return new ChassisSpeeds(
                    unstickVector.getX(), unstickVector.getY(),
                    unstickEscapeOmega(nearestPeerDistM, robotHeading));
        }
        return base;
    }

    /**
     * Escape spin rate, single-owned by both escape paths ({@link #applyUnstickOnly}
     * and {@link #arbitrate}). A nearby peer means bumper contact, which keeps
     * the aggressive pirouette; pure geometry gets a capped face-the-escape
     * turn instead.
     */
    double unstickEscapeOmega(double nearestPeerDistM, Rotation2d robotHeading) {
        if (nearestPeerDistM < PROXIMITY_M) {
            return UNSTICK_SPIN_OMEGA_RPS;
        }
        if (robotHeading == null || unstickVector.getNorm() < 1e-4) {
            return 0.0;
        }
        double err = unstickVector.getAngle().minus(robotHeading).getRadians();
        return Math.max(-UNSTICK_FACE_OMEGA_MAX_RPS,
                Math.min(UNSTICK_FACE_OMEGA_MAX_RPS, UNSTICK_FACE_KP * err));
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
            double nearestPeerDistM = (opponentPose == null) ? Double.MAX_VALUE
                    : currentPose.getTranslation().getDistance(opponentPose.getTranslation());
            return new ChassisSpeeds(unstickVector.getX(), unstickVector.getY(),
                    unstickEscapeOmega(nearestPeerDistM, currentPose.getRotation()));
        }

        if (recoveryTimeSec > 0.0) {
            if (recoveryTrenchActive) {
                // Trench yield: reverse out along the commanded path, no lateral
                // jink (lateral = into the truss walls).
                return new ChassisSpeeds(
                        commanded.vxMetersPerSecond * recoveryForwardScale,
                        commanded.vyMetersPerSecond * recoveryForwardScale,
                        commanded.omegaRadiansPerSecond);
            }
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
        return updateDeadlock(stalled, nearestPeerDist, dt, false);
    }

    private Resolution updateDeadlock(boolean stalled, double nearestPeerDist, double dt, boolean inTrench) {
        if (cooldownTimeSec > 0.0) {
            cooldownTimeSec -= dt;
            return idleResolution();
        }
        if (recoveryTimeSec > 0.0) {
            recoveryTimeSec -= dt;
            if (recoveryTimeSec <= 0.0) {
                // Jittered trench cooldown desynchronizes head-on pairs so they
                // don't re-enter in lockstep; open-field stays deterministic.
                cooldownTimeSec = recoveryTrenchActive
                        ? COOLDOWN_SEC + random.nextDouble() * TRENCH_COOLDOWN_JITTER_SEC
                        : COOLDOWN_SEC;
                recoveryTrenchActive = false;
                return idleResolution();
            }
            return new Resolution(true, recoveryForwardScale, recoveryLateralJink);
        }
        if (stalled && nearestPeerDist < PROXIMITY_M) {
            deadlockStallTimeSec += dt;
        } else {
            deadlockStallTimeSec = 0.0;
            // Stall cleared (moving or peer gone): reset escalation.
            trenchEscalation = 0;
        }
        double triggerSec = inTrench ? TRENCH_TRIGGER_STALL_SEC : TRIGGER_STALL_SEC;
        if (deadlockStallTimeSec >= triggerSec) {
            deadlockStallTimeSec = 0.0;
            recoveryTrenchActive = inTrench;
            if (inTrench) {
                int esc = Math.min(trenchEscalation, TRENCH_ESCALATION_MAX);
                recoveryTimeSec = RECOVERY_SEC + random.nextDouble() * TRENCH_RECOVERY_JITTER_SEC
                        + esc * TRENCH_ESCALATION_STEP_SEC;
                trenchEscalation = Math.min(trenchEscalation + 1, TRENCH_ESCALATION_MAX + 1);
                recoveryForwardScale = TRENCH_FORWARD_SCALE;
                recoveryLateralJink = 0.0;
            } else {
                recoveryTimeSec = RECOVERY_SEC;
                recoveryForwardScale = FORWARD_SCALE;
                jinkSign = random.nextBoolean() ? 1.0 : -1.0;
                recoveryLateralJink = jinkSign * JINK_SPEED;
            }
            recoveryCount++;
            return new Resolution(true, recoveryForwardScale, recoveryLateralJink);
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
        Resolution r = updateDeadlock(stalled, nearestPeerDist, dt, false);
        publishTelemetry(false, r);
        return r;
    }

    /**
     * Trench-aware variant: single-lane corridors reverse out instead of jinking
     * laterally (lateral = into the truss), and trigger after
     * {@link #TRENCH_TRIGGER_STALL_SEC}.
     */
    public synchronized Resolution updateDeadlockOnly(
            boolean stalled, double nearestPeerDist, double dt, boolean inTrench) {
        Resolution r = updateDeadlock(stalled, nearestPeerDist, dt, inTrench);
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

    /** Returns the active deadlock resolution without re-ticking the state. */
    public synchronized Resolution getDeadlockResolution() {
        if (recoveryTimeSec > 0.0) {
            return new Resolution(true, recoveryForwardScale, recoveryLateralJink);
        }
        return idleResolution();
    }

    /**
     * True when ANY high-priority contact recovery (pin forced backoff,
     * pirouette unstick, or deadlock recovery) is actively commanding the robot.
     */
    public synchronized boolean isAnyContactRecoveryActive() {
        return isForcedBackoffActive() || isPirouetteActive() || isDeadlockRecovering();
    }

    /** True while a deadlock recovery cooldown is suppressing re-trigger. */
    public synchronized boolean isDeadlockCooling() {
        return cooldownTimeSec > 0.0;
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

    /** Consecutive trench recoveries without stall clearing (escalation level). */
    public synchronized int getTrenchEscalation() {
        return trenchEscalation;
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
        recoveryForwardScale = FORWARD_SCALE;
        recoveryLateralJink = 0.0;
        recoveryTrenchActive = false;
        trenchEscalation = 0;
        unstickEndTime = -1.0;
        unstickVector = new Translation2d();
        unstickAttempts = 0;
        lastUnstickStartTime = -1.0;
        lastPose = new Pose2d();
    }

    private void publishTelemetry(boolean isImpact, Resolution deadlock) {
        SmartDashboard.putBoolean(TelemetryKeys.PinWatchdog.IS_WARNING, isWarningActive());
        SmartDashboard.putBoolean(TelemetryKeys.PinWatchdog.FORCED_BACKOFF, forcedBackoffActive);
        SmartDashboard.putNumber(TelemetryKeys.PinWatchdog.PIN_DURATION_SEC, pinDuration);
        SmartDashboard.putNumber(TelemetryKeys.PinWatchdog.BACKOFF_REMAINING_SEC, getBackoffRemainingSec());

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
