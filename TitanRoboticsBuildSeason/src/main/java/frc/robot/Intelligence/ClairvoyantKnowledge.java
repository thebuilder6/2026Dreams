package frc.robot.Intelligence;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Everything a human watching the match would know: all robot poses and
 * velocities, the live score, what each side holds and has scored, and the fuel
 * count in each of the three field zones.
 * we would also see our allies strategy, their current target, their committed
 * actions
 *
 * <p>
 * A sim operator has all of this, so a sparring bot runs clairvoyant. The
 * zone counts are computed once per tick by {@code WorldStateBuilder} using
 * {@code FieldMap.AllianceZones}, so both knowledge kinds bucket fuel by the
 * same
 * geometry and no caller can invent its own zoning.
 *
 * <p>
 * Counts exclude fuel inside hard obstacles and near dynamic obstacles, and
 * fuel the {@code TargetProgressWatchdog} has abandoned, matching what the
 * engine's
 * selectors will actually treat as collectable. Counting abandoned fuel here
 * would
 * reintroduce the live-lock: {@code SWEEP_ALLIANCE_ZONE} stayed viable on
 * pieces
 * the policy was simultaneously forbidden to approach.
 *
 * @see MatchKnowledge
 * @see ObservedKnowledge
 */
public record ClairvoyantKnowledge(
        int scoreDifferential,
        int alliesHeldFuel,
        int opponentsHeldFuel,
        int alliesScoredFuel,
        int opponentsScoredFuel,
        List<Pose2d> allyPoses,
        List<Pose2d> opponentPoses,
        List<ChassisSpeeds> allyVelocities,
        List<ChassisSpeeds> opponentVelocities,
        int allianceZoneFuel,
        int midfieldFuel,
        int opponentZoneFuel,
        List<Translation2d> fieldFuel) implements MatchKnowledge {

    /** Defensive copy so a caller cannot mutate the snapshot after construction. */
    public ClairvoyantKnowledge {
        allyPoses = List.copyOf(allyPoses);
        opponentPoses = List.copyOf(opponentPoses);
        allyVelocities = List.copyOf(allyVelocities);
        opponentVelocities = List.copyOf(opponentVelocities);
        fieldFuel = (fieldFuel == null) ? List.of() : List.copyOf(fieldFuel);
        if (allianceZoneFuel < 0 || midfieldFuel < 0 || opponentZoneFuel < 0) {
            throw new IllegalArgumentException(
                    "zone fuel counts cannot be negative: alliance=" + allianceZoneFuel
                            + " midfield=" + midfieldFuel + " opponent=" + opponentZoneFuel);
        }
    }

    /**
     * Backward-compatible 12-parameter constructor defaulting fieldFuel to an empty list.
     */
    public ClairvoyantKnowledge(
            int scoreDifferential,
            int alliesHeldFuel,
            int opponentsHeldFuel,
            int alliesScoredFuel,
            int opponentsScoredFuel,
            List<Pose2d> allyPoses,
            List<Pose2d> opponentPoses,
            List<ChassisSpeeds> allyVelocities,
            List<ChassisSpeeds> opponentVelocities,
            int allianceZoneFuel,
            int midfieldFuel,
            int opponentZoneFuel) {
        this(scoreDifferential, alliesHeldFuel, opponentsHeldFuel, alliesScoredFuel, opponentsScoredFuel,
                allyPoses, opponentPoses, allyVelocities, opponentVelocities,
                allianceZoneFuel, midfieldFuel, opponentZoneFuel, List.of());
    }

    /**
     * Whether any opponent is known. Derived from the pose list rather than passed
     * in separately, so it cannot contradict the data it guards &mdash; the exact
     * inconsistency the old {@code opponentObserved} flag allowed.
     */
    @Override
    public boolean opponentObserved() {
        return !opponentPoses.isEmpty();
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

    @Override
    public List<Pose2d> opponentPoses() {
        return opponentPoses;
    }

    @Override
    public List<ChassisSpeeds> opponentVelocities() {
        return opponentVelocities;
    }

    @Override
    public int allianceZoneFuel() {
        return allianceZoneFuel;
    }

    @Override
    public int midfieldFuel() {
        return midfieldFuel;
    }

    @Override
    public int opponentZoneFuel() {
        return opponentZoneFuel;
    }
}
