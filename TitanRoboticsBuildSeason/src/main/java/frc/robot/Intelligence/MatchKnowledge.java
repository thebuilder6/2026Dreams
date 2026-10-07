package frc.robot.Intelligence;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * What the decision-maker is allowed to know beyond its own on-board sensing.
 *
 * <p>This is a <b>sealed hierarchy of two honest kinds</b>, not one record with a
 * "did we see them" flag. The distinction is what the robot's sensors can
 * physically produce:
 *
 * <ul>
 *   <li>{@link ClairvoyantKnowledge} &mdash; everything a human watching the match
 *       knows: every robot's pose and velocity, the live score differential, what
 *       each side holds and has scored, and the <b>number of fuel pieces in each of
 *       the three field zones</b> (alliance, midfield, opponent). A sim operator has
 *       all of this, so sparring bots run clairvoyant.</li>
 *   <li>{@link ObservedKnowledge} &mdash; only sensor truth: own odometry, own
 *       hopper, a single detected fuel piece, a detected opponent bumper. There is
 *       <b>no sensor for the global fuel distribution</b> and no tracker for other
 *       robots, so the zone counts are zero and other robots are absent. Zero is
 *       not a degraded guess; it is the true answer for real hardware.</li>
 * </ul>
 *
 * <p>Why this is a type rather than a boolean: the previous single record carried
 * an {@code opponentObserved} flag while the engine read fuel counts straight from
 * {@code SimulatedArena}, bypassing the record entirely. The gate was therefore
 * decorative &mdash; {@code unknown()} claimed ignorance while fuel was read
 * perfectly, and the real robot silently took a {@code return 0} fallback with no
 * way to distinguish "no fuel there" from "I cannot see any". With separate types an
 * {@link ObservedKnowledge} instance <b>cannot</b> report a zone count, so a lying
 * combination stops being expressible.
 *
 * <p>Zone geometry is single-owned by {@code FieldMap.AllianceZones}
 * ({@code isInAllianceZone}, {@code isInMidfield}); both kinds bucket by it.
 *
 * <p>Immutable snapshots, safe to share across ticks. Lists are unmodifiable and
 * capped (allies &le; 2, opponents &le; 3). See
 * {@code docs/KNOWLEDGE_MODEL.md} for the full design and the reasoning.
 *
 * <p>Out of scope: target <i>selection</i> still reads {@code SimulatedArena}
 * directly in the engine, because picking a specific piece needs individual poses
 * rather than counts. That is a separate, larger change.
 */
public sealed interface MatchKnowledge
        permits ClairvoyantKnowledge, ObservedKnowledge {

    /** Live score from this robot's point of view: positive means we are ahead. */
    int scoreDifferential();

    /** What our own side is holding, summed across robots we can perceive. */
    int alliesHeldFuel();

    /** Ally poses we know about. Empty for {@link ObservedKnowledge}. */
    List<Pose2d> allyPoses();

    /** Ally velocities, index-aligned with {@link #allyPoses()}. */
    List<ChassisSpeeds> allyVelocities();

    /**
     * Whether any opponent robot is known at all. The policy gates every
     * opponent read on this, so an {@link ObservedKnowledge} instance with no
     * tracker returns {@code false} and opponent-chasing objectives become
     * unreachable rather than acting on a fabricated pose.
     */
    boolean opponentObserved();

    /** Opponent poses we know about. Empty for {@link ObservedKnowledge}. */
    List<Pose2d> opponentPoses();

    /** Opponent velocities, index-aligned with {@link #opponentPoses()}. */
    List<ChassisSpeeds> opponentVelocities();

    /**
     * Fuel pieces visible in our own alliance zone.
     *
     * <p>Only {@link ClairvoyantKnowledge} can answer this meaningfully. It
     * exists on the supertype because the engine needs it in a hot path; for
     * {@link ObservedKnowledge} it is always {@code 0}, which is the honest
     * value &mdash; a real robot has no way to count field fuel.
     */
    int allianceZoneFuel();

    /** Fuel in midfield. {@code 0} for {@link ObservedKnowledge}, as above. */
    int midfieldFuel();

    /** Fuel in the opponent zone. {@code 0} for {@link ObservedKnowledge}. */
    int opponentZoneFuel();

    /**
     * Fuel piece coordinates known on the field.
     *
     * <p>For {@link ClairvoyantKnowledge}, this carries all active, eligible
     * fuel positions on the field. For {@link ObservedKnowledge}, this is empty
     * (or vision-detected coordinates when available).
     */
    List<edu.wpi.first.math.geometry.Translation2d> fieldFuel();
}
