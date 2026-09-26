package frc.robot.Sim;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * HubSchedule: official 2026 match hub-status schedule (rules 6.4 / 6.4.1).
 *
 * <ul>
 * <li>AUTO (20 s), TRANSITION SHIFT (2:20-2:10), END GAME (0:30-0:00): both hubs active.</li>
 * <li>SHIFT 1-4 (25 s each): exactly one hub active, alternating. The alliance
 * that scored more fuel in AUTO starts with its hub <i>inactive</i> in SHIFT 1
 * (tie &rarr; random, mirroring the FMS behavior).</li>
 * <li>Fuel assessment continues up to 3 s after a hub deactivates, covering
 * in-flight/processing balls (rule 6.5). New launches into an inactive hub
 * remain illegal (G407) &mdash; the grace only covers balls already scored.</li>
 * </ul>
 *
 * <p>Teleop {@code matchTimeRemaining} follows the countdown convention used
 * everywhere else (2:20 teleop start = 140 s). Deactivation edges are stamped
 * on the FPGA clock so the grace window tracks real seconds.
 */
public class HubSchedule {

    /** Match segments from Table 6-2 (plus terminal DONE). */
    public enum Phase {
        AUTO,
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        DONE
    }

    /** Fuel-processing grace after a hub deactivates (rule 6.5). */
    public static final double GRACE_SEC = 3.0;

    // Shift boundaries (teleop seconds remaining).
    private static final double TRANSITION_END = 130.0;
    private static final double SHIFT1_END = 105.0;
    private static final double SHIFT2_END = 80.0;
    private static final double SHIFT3_END = 55.0;
    private static final double SHIFT4_END = 30.0;

    private static char shiftSeed = 'R'; // alliance inactive FIRST in SHIFT 1
    private static Phase lastPhase = Phase.TRANSITION;
    private static boolean lastRedActive = true;
    private static boolean lastBlueActive = true;
    private static double redDeactivatedAt = Double.NaN;
    private static double blueDeactivatedAt = Double.NaN;
    private static java.util.function.DoubleSupplier clockForTests = null;

    private HubSchedule() {
    }

    private static double now() {
        return clockForTests != null ? clockForTests.getAsDouble() : Timer.getFPGATimestamp();
    }

    /** Injectable clock for deterministic tests (null restores FPGA time). */
    static void setClockForTests(java.util.function.DoubleSupplier clock) {
        clockForTests = clock;
    }

    /** Official segment for the given countdown time and mode. */
    public static Phase phaseFor(double matchTimeRemaining, boolean isAuto) {
        if (isAuto) {
            return Phase.AUTO;
        }
        if (matchTimeRemaining > TRANSITION_END) {
            return Phase.TRANSITION;
        }
        if (matchTimeRemaining > SHIFT1_END) {
            return Phase.SHIFT1;
        }
        if (matchTimeRemaining > SHIFT2_END) {
            return Phase.SHIFT2;
        }
        if (matchTimeRemaining > SHIFT3_END) {
            return Phase.SHIFT3;
        }
        if (matchTimeRemaining > SHIFT4_END) {
            return Phase.SHIFT4;
        }
        // Clock exhausted (or no clock yet in a practice sim): fail open on
        // both hubs, matching the pre-schedule behavior. Post-match tails are
        // bounded by the 3 s grace stamps and match-end robot stops.
        return Phase.ENDGAME;
    }

    /**
     * Strict hub activity from Table 6-3.
     *
     * @param isRed which hub to query
     * @param phase current match segment
     * @param inactiveFirstSeed 'R' if Red starts inactive in SHIFT 1, else 'B'
     */
    public static boolean isHubActive(boolean isRed, Phase phase, char inactiveFirstSeed) {
        boolean redStartsInactive = (inactiveFirstSeed == 'R');
        switch (phase) {
            case AUTO:
            case TRANSITION:
            case ENDGAME:
                return true;
            case DONE:
                return false;
            case SHIFT1:
            case SHIFT3:
                // First order: seeded alliance sits out.
                return isRed ? !redStartsInactive : redStartsInactive;
            case SHIFT2:
            case SHIFT4:
            default:
                // Alternating order: flipped from SHIFT 1.
                return isRed ? redStartsInactive : !redStartsInactive;
        }
    }

    /**
     * Advances schedule state; call every sim tick with the current match clock.
     * Deactivation edges are stamped so {@link #isScoringActive} can honor the
     * 3-second processing grace.
     */
    public static synchronized void update(double matchTimeRemaining, boolean isAuto) {
        Phase phase = phaseFor(matchTimeRemaining, isAuto);
        boolean redActive = isHubActiveStrict(true, phase);
        boolean blueActive = isHubActiveStrict(false, phase);
        double t = now();
        if (lastRedActive && !redActive) {
            redDeactivatedAt = t;
        } else if (!lastRedActive && redActive) {
            redDeactivatedAt = Double.NaN;
        }
        if (lastBlueActive && !blueActive) {
            blueDeactivatedAt = t;
        } else if (!lastBlueActive && blueActive) {
            blueDeactivatedAt = Double.NaN;
        }
        lastRedActive = redActive;
        lastBlueActive = blueActive;
        lastPhase = phase;

        SmartDashboard.putString("Scoreboard/Match/Phase", phase.name());
        SmartDashboard.putString("Scoreboard/Match/ShiftSeed", String.valueOf(shiftSeed));
        SmartDashboard.putBoolean("Scoreboard/Match/RedHubActive", redActive);
        SmartDashboard.putBoolean("Scoreboard/Match/BlueHubActive", blueActive);
    }

    private static boolean isHubActiveStrict(boolean isRed, Phase phase) {
        return isHubActive(isRed, phase, shiftSeed);
    }

    /** Strict activity from the latest {@link #update} (no grace). */
    public static synchronized boolean isHubActiveNow(boolean isRed) {
        return isRed ? lastRedActive : lastBlueActive;
    }

    /**
     * Scoring activity: strict-active, or deactivated within {@link #GRACE_SEC}.
     * New launches into a graced-but-inactive hub are still illegal; this only
     * lets already-scored balls count.
     */
    public static synchronized boolean isScoringActive(boolean isRed) {
        if (isRed ? lastRedActive : lastBlueActive) {
            return true;
        }
        double stamped = isRed ? redDeactivatedAt : blueDeactivatedAt;
        if (Double.isNaN(stamped)) {
            return false;
        }
        return (now() - stamped) <= GRACE_SEC;
    }

    /** Refreshes schedule state from live match state (DS clock preferred). */
    public static void refreshFromMatchState() {
        boolean isAuto = false;
        double remaining = -1.0;
        try {
            isAuto = DriverStation.isAutonomous();
        } catch (Exception ignored) {
        }
        try {
            if (RobotBase.isSimulation()) {
                // In sim, GameSim owns the match clock: it mirrors a running
                // DS clock during real play and holds test-set values
                // otherwise. (A bare DS reports 0.0 with no match running,
                // which must not read as "match over".)
                remaining = GameSim.getInstance().getSimTimeRemainingSec();
            } else {
                remaining = DriverStation.getMatchTime();
            }
        } catch (Exception ignored) {
        }
        if (remaining < 0.0) {
            remaining = 150.0; // unknown clock: fail open on both hubs
        }
        update(remaining, isAuto);
    }

    /** 'R' if Red is inactive first in SHIFT 1, else 'B'. */
    public static synchronized char getShiftSeed() {
        return shiftSeed;
    }

    public static synchronized void setShiftSeed(char seed) {
        shiftSeed = (seed == 'B') ? 'B' : 'R';
    }

    /**
     * Decides the SHIFT 1 order from AUTO fuel totals: most AUTO fuel &rarr;
     * own hub inactive first; tie &rarr; random (FMS behavior).
     */
    public static char decideSeed(int redAutoFuel, int blueAutoFuel) {
        if (redAutoFuel > blueAutoFuel) {
            return 'R';
        }
        if (blueAutoFuel > redAutoFuel) {
            return 'B';
        }
        return Math.random() < 0.5 ? 'R' : 'B';
    }

    /** Seeds the shift order from the finished AUTO period's fuel totals. */
    public static char seedFromAutoResult() {
        MatchScoreTracker tracker = MatchScoreTracker.getInstance();
        char seed = decideSeed(tracker.getRedAutoFuelCount(), tracker.getBlueAutoFuelCount());
        setShiftSeed(seed);
        SmartDashboard.putString("Scoreboard/Match/ShiftSeed",
                seed + " (R" + tracker.getRedAutoFuelCount() + "/B" + tracker.getBlueAutoFuelCount()
                        + " auto)");
        return seed;
    }

    /** Resets to the default pre-match state (both active, seed 'R'). */
    public static synchronized void reset() {
        shiftSeed = 'R';
        lastPhase = Phase.TRANSITION;
        lastRedActive = true;
        lastBlueActive = true;
        redDeactivatedAt = Double.NaN;
        blueDeactivatedAt = Double.NaN;
    }
}
