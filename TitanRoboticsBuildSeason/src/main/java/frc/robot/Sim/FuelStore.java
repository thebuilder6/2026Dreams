package frc.robot.Sim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;
import frc.robot.Navigation.StaticPathfinder;

/**
 * List-backed field fuel for the standalone (MapleSim-free) match runner.
 *
 * <p>Ownership: the runner's bots read positions through this store and remove
 * pieces through {@link #takeWithin}. It deliberately mirrors the filters
 * {@code WorldStateBuilder} applies to arena fuel (field bounds, hard
 * obstacles), so the Jev selectors see the same eligible set they would in the
 * full sim — without a {@code SimulatedArena} existing at all.
 */
public final class FuelStore {
    private final List<Translation2d> pieces = new ArrayList<>();

    public FuelStore(List<Translation2d> initial) {
        if (initial != null) {
            pieces.addAll(initial);
        }
    }

    /**
     * Uniform scatter with the same eligibility filter the sim path uses.
     *
     * @param seed  scenario seed; one {@link Random} stream, so the layout is a
     *              pure function of seed and count
     * @param count pieces to place (retries capped; returns what was placed)
     */
    public static FuelStore scatterSeeded(long seed, int count) {
        Random rng = new Random(seed);
        List<Translation2d> placed = new ArrayList<>();
        for (int attempts = 0; attempts < count * 20 && placed.size() < count; attempts++) {
            Translation2d at = new Translation2d(
                    0.05 + rng.nextDouble() * (FieldMap.FIELD_LENGTH - 0.10),
                    0.05 + rng.nextDouble() * (FieldMap.FIELD_WIDTH - 0.10));
            if (StaticPathfinder.isPointInHardObstacle(at)) {
                continue;
            }
            placed.add(at);
        }
        return new FuelStore(placed);
    }

    /** Unmodifiable snapshot of remaining pieces. */
    public List<Translation2d> positions() {
        return Collections.unmodifiableList(pieces);
    }

    /** Remaining piece count. */
    public int size() {
        return pieces.size();
    }

    /**
     * Removes every piece within {@code radius} of {@code center}.
     *
     * @return number of pieces taken
     */
    public int takeWithin(Translation2d center, double radius) {
        int taken = 0;
        for (int i = pieces.size() - 1; i >= 0; i--) {
            if (pieces.get(i).getDistance(center) <= radius) {
                pieces.remove(i);
                taken++;
            }
        }
        return taken;
    }

    /**
     * Removes pieces inside the intake roller box ahead of {@code robotPos}.
     *
     * <p>Geometry copies {@code AIRobotInstance.checkProximityPickup}: roller
     * center 0.48 m ahead, 0.72 m wide, 0.38 m deep, at most 10 per tick.
     *
     * @return number of pieces taken
     */
    public int takeInRollerBox(Translation2d robotPos, double headingRad, int maxTake) {
        Translation2d rollerCenter = robotPos.plus(
                new Translation2d(0.48, 0.0).rotateBy(new edu.wpi.first.math.geometry.Rotation2d(headingRad)));
        edu.wpi.first.math.geometry.Rotation2d inv =
                new edu.wpi.first.math.geometry.Rotation2d(headingRad).unaryMinus();
        int taken = 0;
        for (int i = pieces.size() - 1; i >= 0 && taken < Math.min(maxTake, 10); i--) {
            Translation2d rel = pieces.get(i).minus(rollerCenter).rotateBy(inv);
            if (Math.abs(rel.getY()) <= 0.72 / 2.0 && Math.abs(rel.getX()) <= 0.38 / 2.0) {
                pieces.remove(i);
                taken++;
            }
        }
        return taken;
    }
}
