package frc.robot.Intelligence;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * One agent's fuel-target stickiness.
 *
 * <p><b>Why this exists.</b> {@link JevDecisionEngine}'s cluster-weighted scent
 * is recomputed every cycle from geometry, distance, and a
 * {@code cos(heading - delta)} alignment bonus. Two pieces with comparable
 * density and distance therefore trade places whenever the robot's heading
 * swings by a few degrees, because the alignment term is a function of the
 * robot's instantaneous pose. The chosen target oscillates every few ticks.
 *
 * <p>That is not cosmetic. {@link frc.robot.Navigation.TargetProgressWatchdog}
 * restarts its no-progress window whenever the commanded target moves more than
 * {@code NEW_TARGET_RESET_M}. An oscillating target resets that window forever, so
 * the give-up timer never expires and the bot never abandons the piece it cannot
 * reach. The result is classified in
 * {@code AIRobotInstance.classifyUnattributedStall} as
 * {@code STALLED_CHURN}: a robot stalled for many seconds, tracking a target that
 * is always seconds old.
 *
 * <p><b>State belongs to the agent, not to the engine.</b> This is the same
 * constraint {@link ObjectiveCommitment} documents, and it was learned the hard
 * way: an earlier latch stored in a {@code static} map on the shared singleton
 * put every 3-argument caller (all unit tests, the Co-Pilot) into one shared
 * box, one bot's decision leaked into another's, and six tests failed. Each
 * {@code AIRobotInstance} and each {@code AutonomousTeleopAgent} owns its own
 * instance. A {@code null} memory selects the legacy winner-take-all behaviour.
 *
 * <p><b>Calibration.</b> {@link #SWITCH_RATIO} is a starting value, not a
 * measured optimum. It is pinned by {@code FuelTargetMemoryTest} so a retune is
 * deliberate; real-world validation is the churn telemetry in
 * {@code STALLED_CHURN} share, which should fall as this latch takes effect.
 */
public final class FuelTargetMemory {

    /**
     * Within this distance of the latched piece, a candidate is treated as the
     * same piece. The fuel selector scores positions rather than identities, so
     * this is how a latch is matched back to this cycle's candidate list.
     *
     * <p>Comfortably larger than the per-cycle jitter between two readings of the
     * same piece, and comfortably smaller than the minimum spacing of
     * independently placed fuel.
     */
    public static final double STICK_RADIUS_M = 0.30;

    /**
     * A challenger must beat the incumbent by this ratio to steal the latch.
     *
     * <p>A ratio rather than an absolute margin because scents are dimensionally
     * inverse-metres and scale with cluster density, so an absolute threshold
     * would mean different things for a dense midfield pile and a lone piece.
     */
    public static final double SWITCH_RATIO = 1.25;

    /** A scored candidate, as the selector produces them. */
    public record ScoredTarget(Translation2d position, double scent) {
    }

    private Translation2d latched;
    private double latchedSinceSeconds;

    private void markLatched(Translation2d piece) {
        latched = piece;
        latchedSinceSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /**
     * Picks the target to pursue, holding the latch unless decisively beaten.
     *
     * <p>Releases the latch when the piece is gone from {@code scored} (collected
     * by this agent, collected by another, or filtered out as blocked/unreachable)
     * and adopts the best remaining candidate, so the latch can never pin a bot
     * to a piece that no longer exists.
     *
     * @param scored this cycle's eligible candidates; {@code null} or empty
     *               clears the latch and returns {@code null}
     * @return the position to pursue, or {@code null} if there is nothing to pursue
     */
    public Translation2d resolve(List<ScoredTarget> scored) {
        if (scored == null || scored.isEmpty()) {
            latched = null;
            return null;
        }

        ScoredTarget incumbent = null;
        ScoredTarget best = null;
        for (ScoredTarget t : scored) {
            if (t == null || t.position() == null) {
                continue;
            }
            if (latched != null
                    && t.position().getDistance(latched) <= STICK_RADIUS_M
                    // Nearest-wins if two candidates fall inside the stick radius,
                    // so the match does not depend on list order.
                    && (incumbent == null
                            || t.position().getDistance(latched)
                                    < incumbent.position().getDistance(latched))) {
                incumbent = t;
            }
            if (best == null || t.scent() > best.scent()) {
                best = t;
            }
        }

        if (best == null) {
            latched = null;
            return null;
        }

        if (incumbent == null) {
            // Nothing held, or the held piece vanished: adopt the best available.
            markLatched(best.position());
            return latched;
        }

        // Strictly greater, so an exactly-equal challenger never steals -- ties
        // must resolve toward stability, not toward list order.
        if (best.scent() > incumbent.scent() * SWITCH_RATIO) {
            markLatched(best.position());
        } else {
            // Re-latch the same piece without restarting its age, so holding a
            // target for a long time does not look like two separate commitments.
            latched = incumbent.position();
        }
        return latched;
    }

    /** Clears the latch (match reset, objective change away from harvesting). */
    public void reset() {
        latched = null;
        latchedSinceSeconds = 0.0;
    }

    /** Currently latched piece, or {@code null}. Diagnostic. */
    public Translation2d latched() {
        return latched;
    }

    /**
     * Seconds this latch has been held, or -1.0 when nothing is held. Mirrors
     * {@code TargetProgressWatchdog.getTrackedTargetAgeSec()} so the two
     * "how long has this agent been on this piece" readings are comparable.
     */
    public double heldSeconds() {
        if (latched == null) {
            return -1.0;
        }
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - latchedSinceSeconds;
    }
}
