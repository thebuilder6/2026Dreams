package frc.robot.Intelligence.spatial;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;

/**
 * Production {@link PointGenerator} factories for the EQS tactical layer.
 *
 * <p>API-only for now: no agent consumes these yet. Switching a targeting call site
 * over needs a score-rig A/B per the measurement gates — same rule as
 * {@code FuelTourOptimizer} (see ARCHITECTURE.md §3J). Covered by
 * {@code StandoffGeneratorsTest}; the optimal-standoff midpoint/sigma come from
 * {@code PolicyWeights} at the future call site, not from here.
 */
public final class StandoffGenerators {

    private StandoffGenerators() {}

    /**
     * Ring of poses at {@code radiusM} around {@code hub}, kept inside the caller's
     * alliance zone and the field walls, all facing the hub.
     */
    public static PointGenerator standoffArc(
            Translation2d hub, double radiusM, int samples, boolean isRedAlliance) {
        return context -> {
            List<Pose2d> out = new ArrayList<>();
            if (hub == null || radiusM <= 0.0 || samples <= 0) {
                return out;
            }
            for (int i = 0; i < samples; i++) {
                double angle = 2.0 * Math.PI * i / samples;
                Translation2d pos = hub.plus(new Translation2d(
                        radiusM * Math.cos(angle), radiusM * Math.sin(angle)));
                if (!FieldMap.isWithinField(pos, 0.45)
                        || !FieldMap.AllianceZones.isInAllianceZone(pos, isRedAlliance)) {
                    continue;
                }
                out.add(new Pose2d(pos, hub.minus(pos).getAngle()));
            }
            return out;
        };
    }

    /**
     * Samples across the {@code opponent → goal} shooting corridor at segment fractions
     * 0.3–0.7 plus lateral offsets in units of {@code lateralStepM}, all facing the
     * opponent. Corridor-mouth points inside the hub shell are left for the
     * {@link PointTest#hubShellExclusion} veto, not culled here.
     */
    public static PointGenerator defenseBarrier(
            Translation2d opponent, Translation2d goal, int samples, double lateralStepM) {
        return context -> {
            List<Pose2d> out = new ArrayList<>();
            if (opponent == null || goal == null || samples <= 0) {
                return out;
            }
            Translation2d along = goal.minus(opponent);
            double length = along.getNorm();
            if (length < 1e-6) {
                return out;
            }
            Translation2d dir = along.div(length);
            Translation2d normal = new Translation2d(-dir.getY(), dir.getX());
            for (int i = 0; i < samples; i++) {
                double frac = 0.3 + 0.4 * ((samples == 1) ? 0.5 : (double) i / (samples - 1));
                int lateral = (i % 3) - 1;
                Translation2d pos = opponent.plus(dir.times(frac * length))
                        .plus(normal.times(lateral * lateralStepM));
                if (!FieldMap.isWithinField(pos, 0.45)) {
                    continue;
                }
                Rotation2d face = opponent.minus(pos).getAngle();
                out.add(new Pose2d(pos, face));
            }
            return out;
        };
    }

    /**
     * Field-fuel pieces as zero-rotation candidate poses, culled only to the field
     * walls. Obstacle and reachability vetoes belong to the {@link PointTest} pipeline.
     */
    public static PointGenerator fuelCandidates(List<Translation2d> pieces, double marginM) {
        return context -> {
            List<Pose2d> out = new ArrayList<>();
            if (pieces == null) {
                return out;
            }
            for (Translation2d piece : pieces) {
                if (piece == null || !FieldMap.isWithinField(piece, marginM)) {
                    continue;
                }
                out.add(new Pose2d(piece, new Rotation2d()));
            }
            return out;
        };
    }
}
