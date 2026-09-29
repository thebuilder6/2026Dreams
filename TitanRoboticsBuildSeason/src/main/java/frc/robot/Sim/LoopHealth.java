package frc.robot.Sim;

/**
 * Main-loop timing counters for the score rig.
 *
 * <p>Armed only by {@link HeadlessMatchDriver} when a headless match is running, so
 * the real-robot hot path pays a single volatile read and nothing else.
 *
 * <p>Why this exists: a 150 s headless match is a 20 ms loop, so a worker that is
 * CPU-starved by its siblings does not fail — it silently produces a *different*
 * match. The archived 12-wide baseline showed 27-53 WPILib overrun warnings per
 * match including a single {@code robotPeriodic()} epoch that took 0.81 s, against
 * 1-3 in a 2-wide batch. Those matches were recorded as successful because the rig
 * only checked for a FINAL line and a JSONL row. Counting overruns in-process makes
 * the perturbation a recorded property of each match instead of a log nobody reads.
 *
 * <p>Only {@code robotPeriodic} is timed, not the whole iteration: it is the phase
 * this project owns and the one that showed the 0.81 s epoch. LiveWindow and
 * Shuffleboard are disabled in {@code Robot}'s constructor, and {@code simulationPeriodic}
 * runs after this measurement, so the figure is a lower bound on loop cost.
 */
public final class LoopHealth {
    /** WPILib's main loop period. An epoch longer than this is an overrun. */
    public static final double PERIOD_SEC = 0.020;

    /**
     * Epochs to discard before counting starts.
     *
     * <p>The first second of a headless match runs one-time initialisation inside
     * {@code robotPeriodic}: AprilTag field layout load, the MapleSim arena build,
     * AdvantageKit struct generation, JIT. That epoch measured 0.81 s in the
     * archived 12-wide sweep and 0.32-0.42 s in a 6-wide calibration, while every
     * later epoch was an order of magnitude smaller. It is a fixed startup cost,
     * identical whether or not the machine is loaded, so counting it as load
     * would make the gate fire on a perfectly idle single worker. 50 epochs is
     * 1 s at the 20 ms loop, which is past the last one-time initialiser observed.
     */
    public static final int WARMUP_EPOCHS = 50;

    private static volatile boolean s_armed;
    private static int s_seen;
    private static int s_overruns;
    private static double s_maxSec;

    private LoopHealth() {}

    /** Enables counting and clears the counters. Called when a headless match starts. */
    public static synchronized void arm() {
        s_armed = true;
        s_seen = 0;
        s_overruns = 0;
        s_maxSec = 0.0;
    }

    /** Disables counting; {@link #overrunCount()} reports unknown afterwards. */
    public static synchronized void disarm() {
        s_armed = false;
    }

    public static boolean isArmed() {
        return s_armed;
    }

    /** Cheap token to hand back to {@link #end(long)}; 0 when disarmed. */
    public static long begin() {
        return s_armed ? System.nanoTime() : 0L;
    }

    /** Records one epoch. Safe to call with the 0 token {@link #begin()} returned. */
    public static synchronized void end(long startNanos) {
        if (!s_armed || startNanos == 0L) {
            return;
        }
        if (s_seen++ < WARMUP_EPOCHS) {
            return;
        }
        double sec = (System.nanoTime() - startNanos) / 1.0e9;
        if (sec > s_maxSec) {
            s_maxSec = sec;
        }
        if (sec > PERIOD_SEC) {
            s_overruns++;
        }
    }

    /** Post-warm-up epochs recorded, or -1 when counting is not armed. */
    public static synchronized int measuredEpochs() {
        return s_armed ? Math.max(0, s_seen - WARMUP_EPOCHS) : -1;
    }

    /** Overrunning epochs since warm-up, or -1 when counting is not armed. */
    public static synchronized int overrunCount() {
        return s_armed ? s_overruns : -1;
    }

    /** Slowest post-warm-up epoch in seconds, or -1.0 when counting is not armed. */
    public static synchronized double maxEpochSec() {
        return s_armed ? s_maxSec : -1.0;
    }
}
