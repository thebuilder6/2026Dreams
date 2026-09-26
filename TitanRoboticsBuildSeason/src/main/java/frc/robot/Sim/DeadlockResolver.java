package frc.robot.Sim;

/**
 * Detects and breaks multi-robot deadlocks (head-on trench meetings, scrums over
 * the same game piece) that the reactive 1.10 m peer-separation nudge cannot
 * resolve: symmetric pushes cancel out and nobody replans.
 *
 * <p>Detection: commanded motion with no progress ({@code stalled}) while pressed
 * against another robot, sustained for {@link #TRIGGER_STALL_SEC}. Recovery is a
 * short yield-and-jink: forward drive is scaled down so the other side can make
 * progress, plus a randomized lateral jink so two bots in the same deadlock do
 * not mirror each other. One instance per robot; not thread-safe by design (the
 * sim ticks each bot on one thread).
 */
public final class DeadlockResolver {

    /** Sustained stall-pressed time before recovery triggers. */
    public static final double TRIGGER_STALL_SEC = 1.0;
    /** How long each recovery maneuver lasts. */
    public static final double RECOVERY_SEC = 0.7;
    /** Quiet period after a recovery so bots do not flap in and out. */
    public static final double COOLDOWN_SEC = 2.0;
    /** Peer distance that counts as "pressed against". Mirrors the separation radius. */
    public static final double PROXIMITY_M = 1.10;
    /** Forward drive scale during recovery (yield so the other side can pass). */
    public static final double FORWARD_SCALE = 0.3;
    /** Lateral jink speed during recovery (robot-relative +Y/-Y, randomized sign). */
    public static final double JINK_SPEED = 1.2;

    /** Recovery command for one tick. */
    public record Resolution(boolean recovering, double forwardScale, double lateralJink) {}

    private final java.util.Random random;
    private double stallTimeSec = 0.0;
    private double recoveryTimeSec = 0.0;
    private double cooldownTimeSec = 0.0;
    private double jinkSign = 1.0;
    private int recoveryCount = 0;

    public DeadlockResolver() {
        this(new java.util.Random());
    }

    /** Injectable RNG for deterministic tests. */
    public DeadlockResolver(java.util.Random random) {
        this.random = (random != null) ? random : new java.util.Random();
    }

    /**
     * Advances the detector one tick.
     *
     * @param stalled True when motion is commanded but no progress is made
     * @param nearestPeerDist Distance to the closest other robot (m)
     * @param dt Tick duration (s)
     */
    public Resolution update(boolean stalled, double nearestPeerDist, double dt) {
        if (cooldownTimeSec > 0.0) {
            cooldownTimeSec -= dt;
            return idle();
        }
        if (recoveryTimeSec > 0.0) {
            recoveryTimeSec -= dt;
            if (recoveryTimeSec <= 0.0) {
                cooldownTimeSec = COOLDOWN_SEC;
                return idle();
            }
            return new Resolution(true, FORWARD_SCALE, jinkSign * JINK_SPEED);
        }
        if (stalled && nearestPeerDist < PROXIMITY_M) {
            stallTimeSec += dt;
        } else {
            stallTimeSec = 0.0;
        }
        if (stallTimeSec >= TRIGGER_STALL_SEC) {
            stallTimeSec = 0.0;
            recoveryTimeSec = RECOVERY_SEC;
            jinkSign = random.nextBoolean() ? 1.0 : -1.0;
            recoveryCount++;
            return new Resolution(true, FORWARD_SCALE, jinkSign * JINK_SPEED);
        }
        return idle();
    }

    private static Resolution idle() {
        return new Resolution(false, 1.0, 0.0);
    }

    /** Clears all state (mirrors bot reset). */
    public void reset() {
        stallTimeSec = 0.0;
        recoveryTimeSec = 0.0;
        cooldownTimeSec = 0.0;
    }

    /** Number of recoveries triggered (telemetry/tuning). */
    public int getRecoveryCount() {
        return recoveryCount;
    }
}
