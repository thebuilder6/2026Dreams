package frc.robot.Intelligence;

import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * MatchKnowledge: what the decision-maker is allowed to know beyond its own
 * sensors, split into the two tiers of the program's information philosophy.
 *
 * <ul>
 * <li><b>Driver-assist tier</b> ({@link #unknown()}): the real robot. It knows
 * only what it could theoretically perceive itself &mdash; own odometry /
 * vision, driver-station and FMS data, and its own mechanism state. Opponent
 * robots are <i>not</i> known unless a vision pipeline is tracking them, so
 * the policy must degrade gracefully (lane assumed clear for the human to
 * judge, opponent-chasing objectives unavailable).</li>
 * <li><b>Sim-sparring tier</b> (populated per tick): an AI bot knows everything
 * its robot would know, plus everything its human player would know from
 * watching the match &mdash; live score, roughly where allies and opponents
 * are and how fast they move, and roughly how many balls each side holds and
 * has scored. Sim values are exact; a real player's estimate would be rough,
 * which the policy must tolerate.</li>
 * </ul>
 *
 * <p>Immutable snapshot, safe to share across ticks. Lists are unmodifiable
 * and capped (allies &le; 2, opponents &le; 3).
 */
public record MatchKnowledge(
        boolean opponentObserved,
        int scoreDifferential,
        int alliesHeldFuel,
        int opponentsHeldFuel,
        int alliesScoredFuel,
        int opponentsScoredFuel,
        List<Pose2d> allyPoses,
        List<Pose2d> opponentPoses,
        List<ChassisSpeeds> allyVelocities,
        List<ChassisSpeeds> opponentVelocities) {

    /** Driver-assist tier: only self-knowable state, nothing about opponents. */
    public static MatchKnowledge unknown() {
        return new MatchKnowledge(false, 0, 0, 0, 0, 0,
                Collections.emptyList(), Collections.emptyList(),
                Collections.emptyList(), Collections.emptyList());
    }

    /**
     * Legacy compatibility: opponent mark trusted as observed, no match
     * context. Preserves the pre-tier behavior for direct unit tests.
     */
    public static MatchKnowledge legacyObserved() {
        return new MatchKnowledge(true, 0, 0, 0, 0, 0,
                Collections.emptyList(), Collections.emptyList(),
                Collections.emptyList(), Collections.emptyList());
    }
}
