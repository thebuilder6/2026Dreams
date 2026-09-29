package frc.robot.Sim;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Navigation.ContactWatchdog;

/**
 * Per-bot match instrumentation for the score measurement rig
 * ({@code tools/score/}). Pure bookkeeping: no behaviour, no control authority,
 * no coupling to any other bot.
 *
 * <p>Three metrics, all read by {@code HeadlessMatchDriver} at the end of a headless
 * match and written to the rig's JSONL:
 *
 * <ul>
 *   <li><b>Longest contiguous stall</b> — the freeze ceiling. A sample counts as
 *       stalled when the bot is <i>commanded</i> to move faster than
 *       {@link ContactWatchdog#STALL_CMD_SPEED_MIN} but is <i>measured</i> below
 *       {@link ContactWatchdog#STALL_ACTUAL_SPEED_MAX}. Those two constants are
 *       reused deliberately rather than re-declared, so this cannot drift from the
 *       thresholds the production watchdogs fire on.
 *   <li><b>Consecutive recovery events</b> — rising edges of the watchdog recovery
 *       flags, with the worst count inside any {@link #RECOVERY_WINDOW_SEC} window.
 *       A per-match total is too blunt: a defender legitimately re-engaging a trench
 *       mark can rack up several across a match without being stuck, while four
 *       inside 20 s is a loop.
 *   <li><b>Integrated path length</b> — total metres travelled. This is the
 *       engagement proxy for defensive archetypes, whose fuel share carries no
 *       signal (a defender is designed not to score, and in practice its observed
 *       median fuel share is <i>higher</i> than the adaptive bot's).
 * </ul>
 *
 * <p>Why not a per-tick displacement threshold for the freeze window: at the 50 Hz
 * sim rate a robot at full sprint moves ~0.06 m per tick, so any threshold loose
 * enough to catch a lockup would also flag ordinary driving. Speed is the quantity
 * that separates the two cases.
 *
 * <p>Why not {@code AIRobotInstance.getStallDuration()}: that is a <i>decaying</i>
 * accumulator (it bleeds off at 2x whenever the bot is not stalled), so it reports
 * the current stall depth rather than the worst one a bot suffered. A bot that
 * stalled for 8 s and then drove away reads 0. This class keeps the maximum.
 */
public final class BotMatchMetrics {

    /**
     * Stall-cause taxonomy for the navigation overhaul (Phase 0).
     *
     * <p>Every stalled sample is attributed to exactly one cause, derived from
     * the recovery flags already computed in {@code AIRobotInstance.update}.
     * Pure bookkeeping: nothing reads these except the rig report and tests.
     */
    public enum StallCause {
        /** Deadlock recovery active inside a trench corridor (reverse-out). */
        DEADLOCK_TRENCH,
        /** Deadlock recovery active in open field (yield-and-jink). */
        DEADLOCK_OPEN,
        /** Post-recovery trench yield hold (0.2x creep while peer is near). */
        TRENCH_YIELD,
        /** Unreachable-target escape (peer-independent, watchdog blacklist). */
        TARGET_UNREACHABLE,
        /**
         * Static-geometry pirouette escape (peer-independent, ContactWatchdog
         * unstick). Added 2026-09-29 with the unstick path itself: that recovery
         * was counted in {@code inRecovery} but had no cause of its own, so every
         * sample it produced fell into {@link #STALLED_OTHER} — making the newest
         * recovery the only one the taxonomy could not distinguish, and therefore
         * invisible in exactly the report used to judge whether it helps.
         */
        STATIC_UNSTICK,
        /**
         * Stalled within {@code TargetProgressWatchdog.ARRIVED_M} of the nav
         * target: the watchdog calls that "reached" and stays idle by design,
         * so a blocked-but-close bot never fires any recovery (arrived hole).
         */
        STALLED_ARRIVED,
        /**
         * Stalled longer than the give-up window while tracking a target
         * younger than it: Jev is retargeting faster than the watchdog window,
         * so no single target ever accumulates 3 s of no-progress.
         */
        STALLED_CHURN,
        /** Stalled with no recovery active — the unattributed bucket. */
        STALLED_OTHER
    }

    /** Worst number of recovery events tolerated inside any rolling window. */
    public static final double RECOVERY_WINDOW_SEC = 20.0;

    /** Simulated period between {@code AIRobotInstance} updates. */
    private static final double SIM_DT_SEC = 0.02;

    private double integratedPathLengthM = 0.0;

    private double currentStallSec = 0.0;
    private double maxStallSec = 0.0;

    private final Deque<Double> recoveryTimestamps = new ArrayDeque<>();
    private int maxConsecutiveRecoveries = 0;
    private int recoveryEventCount = 0;
    private boolean wasRecovering = false;

    private final java.util.EnumMap<StallCause, Double> stallSecByCause =
            new java.util.EnumMap<>(StallCause.class);
    private final java.util.EnumMap<StallCause, Integer> recoveryEventsByCause =
            new java.util.EnumMap<>(StallCause.class);

    private Pose2d lastPose = null;

    /**
     * Records one sample.
     *
     * @param pose        measured field pose this tick
     * @param stalled     true when commanded to move but measured not moving
     * @param inRecovery  true while any watchdog recovery manoeuvre is active
     * @param active      false while the bot is not participating (parked at the
     *                    queuing pose, DS disabled, off field). Gates every metric
     *                    so queuing time is never scored as travel or a freeze.
     * @param nowSec      monotonic seconds
     */
    public void sample(Pose2d pose, boolean stalled, boolean inRecovery, boolean active, double nowSec) {
        sample(pose, stalled, inRecovery, active, nowSec, null);
    }

    /**
     * Records one sample with a stall-cause tag.
     *
     * <p>A null cause with {@code stalled=true} attributes to
     * {@link StallCause#STALLED_OTHER}; a null cause while flowing attributes
     * nothing. The cause only feeds the per-cause counters — the legacy totals
     * above are unchanged.
     *
     * @param cause       which recovery (if any) owns this stalled sample
     */
    public void sample(Pose2d pose, boolean stalled, boolean inRecovery, boolean active,
            double nowSec, StallCause cause) {
        if (pose == null) return;

        if (!active) {
            // Not participating: close any open window and drop the edge state so
            // re-enabling does not look like one long continuous stall.
            currentStallSec = 0.0;
            wasRecovering = false;
            lastPose = pose;
            return;
        }

        if (lastPose != null) {
            integratedPathLengthM += pose.getTranslation().getDistance(lastPose.getTranslation());
        }
        lastPose = pose;

        if (stalled) {
            currentStallSec += SIM_DT_SEC;
            if (currentStallSec > maxStallSec) {
                maxStallSec = currentStallSec;
            }
            StallCause owned = (cause != null) ? cause : StallCause.STALLED_OTHER;
            stallSecByCause.merge(owned, SIM_DT_SEC, Double::sum);
        } else {
            currentStallSec = 0.0;
        }

        if (inRecovery && !wasRecovering) {
            recoveryTimestamps.addLast(nowSec);
            recoveryEventCount++;
            StallCause owned = (cause != null) ? cause : StallCause.STALLED_OTHER;
            recoveryEventsByCause.merge(owned, 1, Integer::sum);
            while (!recoveryTimestamps.isEmpty()
                    && nowSec - recoveryTimestamps.peekFirst() > RECOVERY_WINDOW_SEC) {
                recoveryTimestamps.removeFirst();
            }
            if (recoveryTimestamps.size() > maxConsecutiveRecoveries) {
                maxConsecutiveRecoveries = recoveryTimestamps.size();
            }
        }
        wasRecovering = inRecovery;
    }

    public double getIntegratedPathLengthM() { return integratedPathLengthM; }

    /** Longest single contiguous stalled run, in seconds. */
    public double getMaxContiguousStallSec() { return maxStallSec; }

    /** Current (unfinished) stalled run, in seconds. Read-only taxonomy input. */
    public double getCurrentStallSec() { return currentStallSec; }

    /** Worst count of recovery events inside any {@link #RECOVERY_WINDOW_SEC} window. */
    public int getMaxConsecutiveRecoveries() { return maxConsecutiveRecoveries; }

    /** Total recovery events across the whole match. */
    public int getRecoveryEventCount() { return recoveryEventCount; }

    /** Stalled seconds attributed to one cause (0.0 when never observed). */
    public double getStallSecByCause(StallCause cause) {
        if (cause == null) return 0.0;
        return stallSecByCause.getOrDefault(cause, 0.0);
    }

    /** Recovery rising-edges attributed to one cause (0 when never observed). */
    public int getRecoveryEventsByCause(StallCause cause) {
        if (cause == null) return 0;
        return recoveryEventsByCause.getOrDefault(cause, 0);
    }

    public void reset() {
        integratedPathLengthM = 0.0;
        currentStallSec = 0.0;
        maxStallSec = 0.0;
        recoveryTimestamps.clear();
        maxConsecutiveRecoveries = 0;
        recoveryEventCount = 0;
        wasRecovering = false;
        stallSecByCause.clear();
        recoveryEventsByCause.clear();
        lastPose = null;
    }
}
