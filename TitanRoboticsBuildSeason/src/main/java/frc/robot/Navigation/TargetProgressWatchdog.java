package frc.robot.Navigation;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Unreachable-target watchdog: makes progress failure self-correcting.
 *
 * <p>{@link ContactWatchdog} covers stalls caused by a peer or a trench
 * corridor, and therefore only fires when {@code nearestPeerDist} is small.
 * A bot driving alone into an unreachable navigation target (fuel it cannot
 * reach behind a hard footprint, a standoff pose with no legal approach)
 * makes no progress and has no peer, so nothing fires: the bot holds position
 * for the rest of the match. Headless 3v3 replays showed exactly that - two
 * bots frozen for ~21 s in the far wall band, and a whole alliance starved of
 * teleop cycles.
 *
 * <p>This watchdog is peer-independent. While the bot commands real speed but
 * does not reduce its distance to the commanded navigation target, it
 * accumulates a no-progress window. Once that window expires it
 * <ol>
 *   <li>blacklists the target (so fuel selectors skip that piece) for
 *       {@link #BLACKLIST_TTL_SEC},</li>
 *   <li>commands a lateral escape away from the target for
 *       {@link #ESCAPE_SEC}, and</li>
 *   <li>latches {@link Result#recovering()} so callers publish the state.</li>
 * </ol>
 *
 * <p>Thresholds deliberately mirror the stall thresholds used by
 * {@code AIRobotInstance.isStalled()} (which reads
 * {@link ContactWatchdog#STALL_CMD_SPEED_MIN} and
 * {@link ContactWatchdog#STALL_ACTUAL_SPEED_MAX}) so the two detectors agree on
 * what "trying to move" means. See {@link #COMMAND_MIN_MPS} for why the gate is a
 * hold floor rather than a stall threshold.
 */
public final class TargetProgressWatchdog {
    /**
     * Commanded translation below this counts as "not trying to move".
     *
     * <p>This is deliberately a <i>hold</i> floor, not a stall threshold. An earlier
     * value of 0.80 m/s was wrong: {@code TrajectoryController} commands as little as
     * 0.25 m/s (its "carpet friction breakout floor", and the 0.2x TRENCH_YIELD
     * scale), so a bot wedged against a peer or an obstacle while commanding
     * 0.25-0.79 m/s never accumulated a no-progress window and sat there for the rest
     * of the match. Verified by probe: frozen pose, 5 s at 0.30/0.50/0.79 m/s
     * commanded never gave up; 0.85 m/s did.
     *
     * <p>Legitimate slow approaches are not false-positived because the decision is
     * made on <i>progress</i>, not on speed: a converging bot improves
     * {@link #bestDistanceM} and resets the window every cycle.
     */
    public static final double COMMAND_MIN_MPS = 0.12;

    /** Seconds of commanded-but-no-approach before the target is abandoned. */
    public static final double GIVEUP_SEC = 3.0;

    /** Distance improvement (m) that resets the no-progress window. */
    public static final double PROGRESS_MIN_M = 0.25;

    /** A new objective this far from the previous one restarts progress tracking. */
    public static final double NEW_TARGET_RESET_M = 0.50;

    /** Fuel within this radius of a blacklisted point is skipped by selectors. */
    public static final double BLACKLIST_RADIUS_M = 1.00;

    /** How long a blacklisted fuel point stays skipped. */
    public static final double BLACKLIST_TTL_SEC = 20.0;

    /** Duration of the escape maneuver after a give-up. */
    public static final double ESCAPE_SEC = 1.2;

    /** Escape speed, field-relative. */
    public static final double ESCAPE_SPEED_MPS = 1.4;

    /** Fraction of the escape command spent backing off the abandoned piece. */
    private static final double AWAY_WEIGHT = 0.6;

    /** Fraction spent sliding toward open field. */
    private static final double LATERAL_WEIGHT = 0.4;

    private static final Translation2d FIELD_CENTRE = new Translation2d(
            FieldMap.FIELD_LENGTH / 2.0, FieldMap.FIELD_WIDTH / 2.0);

    /** Keep-out band the escape must not breach. */
    private static final double WALL_MARGIN_M = 0.5;

    /** Never abandon a target that is already reached. */
    public static final double ARRIVED_M = 0.60;

    /** Per-update outcome; {@code escapeVector} is field-relative. */
    public record Result(
            boolean recovering,
            Translation2d escapeVector,
            List<Translation2d> newlyBlacklisted,
            double noProgressSec,
            double escapeRemainingSec) {
        public static final Result IDLE =
                new Result(false, new Translation2d(), List.of(), 0.0, 0.0);
    }

    private final Map<Translation2d, Double> blacklist = new LinkedHashMap<>();
    private Translation2d trackedTarget;
    private double windowStartDistanceM = Double.MAX_VALUE;
    private double noProgressSec = 0.0;
    private double escapeRemainingSec = 0.0;
    private Translation2d escapeFrom = new Translation2d();

    /**
     * Advances the watchdog by one cycle.
     *
     * @param pose measured robot pose
     * @param commanded field-relative commanded chassis speeds
     * @param navTarget commanded navigation target (may be null)
     * @param dt cycle time in seconds
     */
    public synchronized Result update(
            Pose2d pose, ChassisSpeeds commanded, Pose2d navTarget, double dt) {
        if (pose == null || commanded == null || !Double.isFinite(dt) || dt <= 0.0) {
            return Result.IDLE;
        }
        expireBlacklist(Timer.getFPGATimestamp());
        if (escapeRemainingSec > 0.0) {
            escapeRemainingSec = Math.max(0.0, escapeRemainingSec - dt);
            return new Result(true, escapeVector(pose), List.of(), noProgressSec, escapeRemainingSec);
        }
        if (navTarget == null) {
            resetProgress();
            return Result.IDLE;
        }

        Translation2d target = navTarget.getTranslation();
        double distance = pose.getTranslation().getDistance(target);
        if (trackedTarget == null || trackedTarget.getDistance(target) > NEW_TARGET_RESET_M) {
            // Adopt the new objective and start its window on this same cycle,
            // so GIVEUP_SEC means GIVEUP_SEC of no progress after adoption.
            resetProgress();
            trackedTarget = target;
            windowStartDistanceM = distance;
        }
        if (distance < ARRIVED_M) {
            resetProgress();
            return Result.IDLE;
        }

        double commandedSpeed = Math.hypot(
                commanded.vxMetersPerSecond, commanded.vyMetersPerSecond);
        if (commandedSpeed < COMMAND_MIN_MPS) {
            // Not trying to move (staging, planting to shoot, parked): decay so
            // an intentional hold never counts as a stall.
            noProgressSec = Math.max(0.0, noProgressSec - dt * 2.0);
            return Result.IDLE;
        }

        // Progress is measured across the whole window, not per cycle: a robot
        // closing at 1 m/s gains 0.02 m per 20 ms cycle, so a per-cycle test
        // would abandon a correctly-approaching bot. Closing PROGRESS_MIN_M
        // since the window started is the bar.
        if (distance < windowStartDistanceM - PROGRESS_MIN_M) {
            windowStartDistanceM = distance;
            noProgressSec = 0.0;
            return Result.IDLE;
        }
        noProgressSec += dt;

        if (noProgressSec < GIVEUP_SEC) {
            return new Result(false, new Translation2d(), List.of(), noProgressSec, 0.0);
        }

        // Give up: blacklist the abandoned point, then escape away from it.
        escapeFrom = target;
        escapeRemainingSec = ESCAPE_SEC;
        List<Translation2d> blocked = List.of(target);
        blacklist(target, Timer.getFPGATimestamp());
        resetProgress();
        return new Result(true, escapeVector(pose), blocked, GIVEUP_SEC, ESCAPE_SEC);
    }

    /**
     * Escape direction: primarily away from the abandoned piece, with a lateral
     * component whose sign points at open field - unless that would drive the
     * bot into a perimeter wall, in which case it slides along the wall
     * instead. The wall case is the one this watchdog exists for: a bot pinned
     * at y = 7.4 with the unreachable piece at y = 6.4 has the piece between
     * itself and the wall, so a plain back-off re-drives it into the wall it
     * is already against.
     */
    private Translation2d escapeVector(Pose2d pose) {
        Translation2d away = pose.getTranslation().minus(escapeFrom);
        if (away.getNorm() < 1e-6) {
            away = new Translation2d(1.0, 0.0);
        }
        Translation2d unit = away.div(away.getNorm());
        Translation2d lateral = new Translation2d(-unit.getY(), unit.getX());
        Translation2d toCentre = FIELD_CENTRE.minus(pose.getTranslation());
        Translation2d toCentreUnit = toCentre.getNorm() < 1e-6
                ? new Translation2d(1.0, 0.0)
                : toCentre.div(toCentre.getNorm());
        double sign = lateral.dot(toCentreUnit) >= 0.0 ? 1.0 : -1.0;

        Translation2d direction = unit.times(AWAY_WEIGHT).plus(lateral.times(LATERAL_WEIGHT * sign));
        if (headsIntoWall(pose, direction)) {
            direction = lateral.times(sign);
        }
        if (direction.getNorm() < 1e-6) {
            direction = lateral;
        }
        return direction.div(direction.getNorm()).times(ESCAPE_SPEED_MPS);
    }

    /** True when one second of travel along {@code direction} breaches the wall band. */
    private static boolean headsIntoWall(Pose2d pose, Translation2d direction) {
        Translation2d predicted = pose.getTranslation().plus(direction);
        return predicted.getX() < WALL_MARGIN_M
                || predicted.getX() > FieldMap.FIELD_LENGTH - WALL_MARGIN_M
                || predicted.getY() < WALL_MARGIN_M
                || predicted.getY() > FieldMap.FIELD_WIDTH - WALL_MARGIN_M;
    }

    private void resetProgress() {
        trackedTarget = null;
        windowStartDistanceM = Double.MAX_VALUE;
        noProgressSec = 0.0;
    }

    /** True when a fuel piece at this point is inside a blacklisted radius. */
    public synchronized boolean isFuelBlocked(Translation2d point) {
        if (point == null || blacklist.isEmpty()) {
            return false;
        }
        for (Translation2d blocked : blacklist.keySet()) {
            if (blocked.getDistance(point) <= BLACKLIST_RADIUS_M) {
                return true;
            }
        }
        return false;
    }

    /** Snapshot of currently blocked points (blacklist keys). */
    public synchronized Set<Translation2d> blockedPoints() {
        return new LinkedHashSet<>(blacklist.keySet());
    }

    private void blacklist(Translation2d point, double nowSec) {
        pruneNear(point);
        blacklist.put(point, nowSec + BLACKLIST_TTL_SEC);
        Logger.recordOutput("TargetProgress/BlockedPoints", (double) blacklist.size());
    }

    /** Keeps one entry per physical piece instead of stacking near-duplicates. */
    private void pruneNear(Translation2d point) {
        Iterator<Map.Entry<Translation2d, Double>> it = blacklist.entrySet().iterator();
        while (it.hasNext()) {
            if (it.next().getKey().getDistance(point) <= 0.25) {
                it.remove();
            }
        }
    }

    private void expireBlacklist(double nowSec) {
        if (nowSec <= 0.0) {
            return; // sim time not stamped (tests); TTL skipped rather than wiping state
        }
        blacklist.entrySet().removeIf(e -> e.getValue() <= nowSec);
    }

    /** Clears all state (match reset). */
    public synchronized void reset() {
        blacklist.clear();
        resetProgress();
        escapeRemainingSec = 0.0;
        escapeFrom = new Translation2d();
    }

    /** No-progress window currently accumulated, in seconds. */
    public synchronized double getNoProgressSec() {
        return noProgressSec;
    }

    /** True while an escape maneuver is latched. */
    public synchronized boolean isRecovering() {
        return escapeRemainingSec > 0.0;
    }

    /** Escape remaining, in seconds. */
    public synchronized double getEscapeRemainingSec() {
        return escapeRemainingSec;
    }

}
