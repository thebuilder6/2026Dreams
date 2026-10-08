package frc.robot.Intelligence.utility.ast;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.WorldState;
import frc.robot.Navigation.FieldMap;

/**
 * Maps the engine's {@link WorldState} + {@link MatchKnowledge} onto an
 * {@link EvalContext}, so an expression genome can be scored from the same state
 * the engine sees.
 *
 * <p>This is the data half of the {@code ObjectiveUtility} seam: the engine side
 * (calling a genome instead of its hand-written formula) is the remaining step.
 * Keeping the mapping here means one place fixes what each terminal means, and
 * {@code CycleScoreHubParityTest} pins it against the live engine.
 *
 * <p>Tier honesty is derived, not asked for: a non-{@link ClairvoyantKnowledge}
 * context is marked {@code observed}, so {@link EvalContext} zeroes the
 * clairvoyant terminals even though the fuel counts it was given are already 0.
 */
public final class WorldStateTerminals {

    /** Match length the {@code REMAINING_TIME} terminal normalizes over, s. */
    public static final double MATCH_LENGTH_SEC = 150.0;
    /** Shift countdown the {@code TIME_UNTIL_SHIFT} terminal normalizes over, s. */
    public static final double SHIFT_NORM_SEC = 20.0;
    /** Fuel count the zone terminals normalize over. */
    public static final double FUEL_NORM = 30.0;
    /** Distance within which an ally counts as "near", m. */
    public static final double ALLY_NEAR_M = 3.0;
    /** Distance the hub-proximity terminals normalize over, m. */
    public static final double DEFAULT_DIST_NORM_M = 8.0;

    private WorldStateTerminals() {}

    /** {@link #toContext(WorldState, MatchKnowledge, double)} with an 8 m distance norm. */
    public static EvalContext toContext(WorldState world, MatchKnowledge knowledge) {
        return toContext(world, knowledge, DEFAULT_DIST_NORM_M);
    }

    public static EvalContext toContext(WorldState world, MatchKnowledge knowledge, double distNormMeters) {
        MatchKnowledge k = knowledge == null ? frc.robot.Intelligence.ObservedKnowledge.selfOnly() : knowledge;
        boolean observed = !(k instanceof ClairvoyantKnowledge);

        Translation2d selfHub = FieldMap.Hubs.getHubLocation2d(world.isRedAlliance());
        double distToHub = world.selfPose().getTranslation().getDistance(selfHub);

        double oppDistNorm = 0.0;
        double oppInTrench = 0.0;
        if (k.opponentObserved() && !k.opponentPoses().isEmpty()) {
            Translation2d oppHub = FieldMap.Hubs.getHubLocation2d(!world.isRedAlliance());
            double nearest = Double.MAX_VALUE;
            for (Pose2d opponent : k.opponentPoses()) {
                nearest = Math.min(nearest, opponent.getTranslation().getDistance(oppHub));
                if (FieldMap.Trenches.isLowClearance(opponent)) {
                    oppInTrench = 1.0;
                }
            }
            oppDistNorm = nearness(nearest, distNormMeters);
        }

        double allyNear = 0.0;
        for (Pose2d ally : k.allyPoses()) {
            if (ally.getTranslation().getDistance(world.selfPose().getTranslation()) <= ALLY_NEAR_M) {
                allyNear = 1.0;
                break;
            }
        }

        return EvalContext.builder()
                .set(Terminal.HELD_RATIO, ratio(world.heldFuelCount(), world.ballCapacity()))
                .set(Terminal.REMAINING_TIME, clamp01(world.matchTimeRemaining() / MATCH_LENGTH_SEC))
                .set(Terminal.TIME_UNTIL_SHIFT, clamp01(world.timeUntilHubShift() / SHIFT_NORM_SEC))
                .setFlag(Terminal.MY_HUB_ACTIVE, world.isAllianceHubActive())
                .setFlag(Terminal.OPP_HUB_ACTIVE, world.isOpponentHubActive())
                .setFlag(Terminal.MY_HUB_NEXT, world.isAllianceHubActiveAfterShift())
                .setFlag(Terminal.OPP_HUB_NEXT, world.isOpponentHubActiveAfterShift())
                .setFlag(Terminal.IN_ALLIANCE_ZONE,
                        FieldMap.AllianceZones.isInAllianceZone(world.selfPose(), world.isRedAlliance()))
                .set(Terminal.DIST_TO_HUB, nearness(distToHub, distNormMeters))
                .set(Terminal.OPP_DIST_TO_HUB, oppDistNorm)
                .set(Terminal.SCORE_DIFF, clamp01(0.5 + k.scoreDifferential() / 2.0))
                .set(Terminal.MIDFIELD_FUEL, clamp01(k.midfieldFuel() / FUEL_NORM))
                .set(Terminal.ALLIANCE_FUEL, clamp01(k.allianceZoneFuel() / FUEL_NORM))
                .set(Terminal.OPPONENT_FUEL, clamp01(k.opponentZoneFuel() / FUEL_NORM))
                .set(Terminal.ALLY_NEAR_ME, allyNear)
                .set(Terminal.OPP_IN_TRENCH, oppInTrench)
                .observed(observed)
                .build();
    }

    private static double nearness(double meters, double norm) {
        return clamp01(1.0 - meters / Math.max(1e-6, norm));
    }

    private static double ratio(int numerator, int denominator) {
        return denominator <= 0 ? 0.0 : clamp01((double) numerator / denominator);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
