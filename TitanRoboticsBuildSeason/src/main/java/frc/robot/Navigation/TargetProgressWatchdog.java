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

    /** Never abandon a target that is already reached and held. */
    public static final double ARRIVED_M = 0.60;

    /**
     * Measured speed below this counts as "pinned", i.e. commanding motion while
     * going nowhere.
     *
     * <p>Shares its value with {@code ContactWatchdog.STALL_ACTUAL_SPEED_MAX}
     * deliberately: both answer the same question ("is this robot actually
     * moving?"), and the Sep 26 trap was precisely that the two detectors
     * disagreed about it. If one is retuned the other must be.
     */
    public static final double STALL_ACTUAL_SPEED_MAX =
            frc.robot.Navigation.ContactWatchdog.STALL_ACTUAL_SPEED_MAX;

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
     * Seconds since the currently-tracked target was adopted (dt-accumulated,
     * so it works when the WPILib clock is frozen in tests). Phase 0b stall
     * taxonomy: a bot stalled longer than {@link #GIVEUP_SEC} while tracking a
     * target younger than that window is churning targets, not stuck on one.
     */
    private double trackedSec = 0.0;
    /**
     * Seconds spent commanding motion while measured speed stayed below
     * {@link #STALL_ACTUAL_SPEED_MAX}. Survives target changes, unlike
     * {@link #noProgressSec}.
     *
     * <p>Only accumulated when a caller supplies measured velocity; with the
     * distance-only overload this stays 0 and the original recovery is the only
     * path, so legacy call sites are unchanged.
     */
    private double pinnedSec = 0.0;
    /**
     * Target changes since the current target was adopted. Non-zero means the
     * selector is churning and the distance window cannot be trusted to expire;
     * zero means one stable target, which the distance path owns (and which then
     * blacklists the piece, because a single unreachable piece is a real
     * target-selection failure).
     */
    private int recentTargetChanges = 0;
    /**
     * Consecutive churn-rescue fires with no intervening progress or stable
     * give-up. Escape-without-blacklist re-drives into the same trap on a
     * ~10 s cycle (30-row set `nav-joint30`: max-consecutive 10-14 per 20 s,
     * worst stall 143.8 s), so at {@link #STATIC_ESCALATION_COUNT} the rescue
     * escalates to the stable path and blacklists the point: after that many
     * failed escapes the area is guilty even though no single piece is.
     * Survives target changes on purpose (that is the loop); cleared by real
     * progress, by a stable give-up, or by {@link #reset}.
     */
    private int consecutiveStaticEscapes = 0;

    /**
     * Churn escapes before the rescue escalates to blacklisting. Three fires
     * at ~2 s per escape/re-pin cycle lands escalation ~6-8 s into a loop,
     * an order of magnitude inside the measured 60-140 s catastrophes.
     */
    public static final int STATIC_ESCALATION_COUNT = 3;

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
        // Legacy call sites have no measured velocity. Passing null keeps the
        // original distance-only semantics exactly, so this overload is not a
        // behaviour change for existing callers.
        return update(pose, commanded, null, navTarget, dt);
    }

    /**
     * Advances the watchdog with the robot's <i>measured</i> velocity alongside the
     * commanded one.
     *
     * <p><b>Why the actual velocity is a separate input.</b> Without it,
     * "no progress" is inferred purely from the distance to the commanded target,
     * and a target that changes every few ticks resets that window every time. A
     * bot physically pinned against a hub core, a ramp, or a trench wall while a
     * selector oscillated between two fuel pieces therefore never accumulated
     * {@link #GIVEUP_SEC} and was never rescued -- classified
     * {@code STALLED_CHURN}. With the measured velocity, "commanding motion and
     * going nowhere" is directly observable, independent of what is being aimed
     * at, so the rescue fires on a pinned robot no matter how the target moves.
     *
     * <p>The two recoveries are deliberately different. A stable unreachable
     * target is a real target-selection failure, so the piece is blacklisted and
     * the selector can pick something else. A pinned robot under a churning
     * target has no guilty piece: nothing about any one piece caused the stall,
     * so it is escaped <i>without</i> blacklisting. Excluding a piece the bot never
     * had a fair shot at would only starve it of options.
     *
     * @param actual measured field-relative chassis speeds, or {@code null} to use
     *              distance-to-target progress only
     */
    public synchronized Result update(
            Pose2d pose, ChassisSpeeds commanded, ChassisSpeeds actual,
            Pose2d navTarget, double dt) {
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
        double commandedSpeed = Math.hypot(
                commanded.vxMetersPerSecond, commanded.vyMetersPerSecond);

        // Pinned: commanding motion, going nowhere. Tracked across target changes
        // on purpose -- the whole point is that a churning target must not be able
        // to clear it.
        boolean measured = (actual != null);
        double actualSpeed = measured
                ? Math.hypot(actual.vxMetersPerSecond, actual.vyMetersPerSecond)
                : Double.MAX_VALUE;
        boolean pinned = measured
                && commandedSpeed >= COMMAND_MIN_MPS
                && actualSpeed < STALL_ACTUAL_SPEED_MAX;
        if (pinned) {
            pinnedSec += dt;
        } else {
            pinnedSec = 0.0;
        }

        boolean isNewTarget =
                trackedTarget == null || trackedTarget.getDistance(target) > NEW_TARGET_RESET_M;
        if (isNewTarget) {
            // Adopting a target for the first time is not churn. Only a *change*
            // from an already-tracked target counts, so read the previous value
            // before resetProgress() clears it.
            boolean hadTrackedTarget = (trackedTarget != null);
            resetProgress();
            trackedTarget = target;
            windowStartDistanceM = distance;
            if (hadTrackedTarget) {
                recentTargetChanges++;
            }
        }
        trackedSec += dt;

        // The churn rescue. Only claims the tick when the target is *also* moving,
        // because a churning target is what prevents the distance window from ever
        // expiring. A stable target falls through to the distance path below, which
        // blacklists the piece: that is a genuine target-selection failure and the
        // selector should be allowed to pick something else. Claiming the tick here
        // unconditionally would silently delete the blacklist behaviour for every
        // pinned bot, which is the recovery that actually works.
        if (pinnedSec >= GIVEUP_SEC && recentTargetChanges > 0) {
            escapeFrom = target;
            escapeRemainingSec = ESCAPE_SEC;
            consecutiveStaticEscapes++;
            if (consecutiveStaticEscapes >= STATIC_ESCALATION_COUNT) {
                // Loop-breaker: this many escapes with no progress means the
                // area is guilty even though no single piece is. Take the
                // stable path so the selector starves this point, then start
                // the count over. Newly-blacklisted (not empty) is also how
                // the rig tells an escalated escape from a plain churn escape.
                blacklist(target, Timer.getFPGATimestamp());
                List<Translation2d> blocked = List.of(target);
                resetProgress();
                consecutiveStaticEscapes = 0;
                return new Result(true, escapeVector(pose), blocked, pinnedSec, ESCAPE_SEC);
            }
            resetProgress();
            return new Result(true, escapeVector(pose), List.of(), pinnedSec, ESCAPE_SEC);
        }
        if (distance < ARRIVED_M && commandedSpeed < COMMAND_MIN_MPS) {
            // Reached and holding (staging, planting to shoot, parked): an
            // intentional hold is never a stall. Reached but still commanding
            // motion falls through to normal tracking below — Phase 0b
            // measured bots stalled 16+ s within this radius (seed 7, variant
            // nav-phase0b) while the old unconditional idle let them sit.
            resetProgress();
            consecutiveStaticEscapes = 0;
            return Result.IDLE;
        }

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
            consecutiveStaticEscapes = 0;
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
        consecutiveStaticEscapes = 0;
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
        trackedSec = 0.0;
        recentTargetChanges = 0;
        // pinnedSec is deliberately NOT reset here. It is the one accumulator that
        // has to outlive a target change, which is the entire reason it exists.
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
        consecutiveStaticEscapes = 0;
        pinnedSec = 0.0;
    }

    /** No-progress window currently accumulated, in seconds. */
    public synchronized double getNoProgressSec() {
        return noProgressSec;
    }

    /** Consecutive churn escapes with no intervening progress (loop-breaker input). */
    public synchronized int getConsecutiveStaticEscapes() {
        return consecutiveStaticEscapes;
    }

    /**
     * Seconds since the currently-tracked target was adopted, or -1 when no
     * target is tracked (idle, arrived, or escaping). A stalled bot whose
     * tracked target is younger than {@link #GIVEUP_SEC} while the stall itself
     * is older is churning through targets rather than stuck on one.
     */
    public synchronized double getTrackedTargetAgeSec() {
        return trackedTarget == null ? -1.0 : trackedSec;
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
