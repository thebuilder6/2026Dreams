package frc.robot.Intelligence;

import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Only what the robot's own sensors can physically produce: own odometry, own
 * hopper state, a single detected fuel piece, a detected opponent bumper.
 *
 * <p>The defining limits are hardware, not caution. There is <b>no sensor for the
 * global fuel distribution</b> and <b>no tracker for other robots</b> today, so:
 *
 * <ul>
 *   <li>all three zone fuel counts are {@code 0} &mdash; the true answer, not a
 *       degraded guess;</li>
 *   <li>opponent poses are empty and {@link #opponentObserved()} is {@code false},
 *       which makes every opponent-chasing objective unreachable rather than
 *       letting the policy act on a fabricated pose.</li>
 * </ul>
 *
 * <p>Because the zone counts live on {@link MatchKnowledge} and this type always
 * reports zero, the engine no longer needs to read {@code SimulatedArena} to count
 * fuel. That reach-through was the defect: the old {@code unknown()} record claimed
 * ignorance while the engine read perfect sim data behind it, and the real robot
 * always took a silent {@code return 0} fallback with no way to tell "no fuel
 * there" from "I cannot see any".
 *
 * <p>Self state (pose, velocity, held fuel, hub status) lives on {@link WorldState},
 * not here, because it is known in both kinds.
 *
 * @see MatchKnowledge
 * @see ClairvoyantKnowledge
 */
public record ObservedKnowledge(
        int scoreDifferential,
        int alliesHeldFuel,
        List<Pose2d> allyPoses,
        List<ChassisSpeeds> allyVelocities) implements MatchKnowledge {

    /**
     * Defensive copies.
     *
     * <p>{@code List.copyOf} is <b>not</b> sufficient here: if the caller passes an
     * already-immutable list it is returned as-is, so a caller holding a reference
     * to that same list can still not mutate it — but a caller that passed a
     * mutable list and kept a reference to *that* would, in principle, observe
     * aliasing through a list we did not copy. The behaviour is identical either
     * way for immutability of *our* snapshot, so {@code copyOf} is kept for
     * clarity rather than a redundant {@code new ArrayList}.
     */
    public ObservedKnowledge {
        allyPoses = List.copyOf(allyPoses);
        allyVelocities = List.copyOf(allyVelocities);
    }

    /**
     * The real robot's default: self-knowable state only. Replaces the old
     * {@code MatchKnowledge.unknown()}, which was the same record as the
     * sim-sparring tier with a {@code false} flag rather than a distinct type.
     */
    public static ObservedKnowledge selfOnly() {
        return new ObservedKnowledge(0, 0,
                Collections.emptyList(), Collections.emptyList());
    }

    @Override
    public int scoreDifferential() {
        return scoreDifferential;
    }

    @Override
    public int alliesHeldFuel() {
        return alliesHeldFuel;
    }

    @Override
    public List<Pose2d> allyPoses() {
        return allyPoses;
    }

    @Override
    public List<ChassisSpeeds> allyVelocities() {
        return allyVelocities;
    }

    /** No tracker for other robots, so nothing is ever observed. */
    @Override
    public boolean opponentObserved() {
        return false;
    }

    @Override
    public List<Pose2d> opponentPoses() {
        return Collections.emptyList();
    }

    @Override
    public List<ChassisSpeeds> opponentVelocities() {
        return Collections.emptyList();
    }

    /** No sensor for field fuel. */
    @Override
    public int allianceZoneFuel() {
        return 0;
    }

    /** No sensor for field fuel. */
    @Override
    public int midfieldFuel() {
        return 0;
    }

    /** No sensor for field fuel. */
    @Override
    public int opponentZoneFuel() {
        return 0;
    }

    /** No sensor for field fuel. */
    @Override
    public List<Translation2d> fieldFuel() {
        return Collections.emptyList();
    }
}
