package frc.robot.Intelligence.spatial;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Intelligence.utility.ResponseCurve;
import frc.robot.Navigation.StaticPathfinder;

/**
 * Functional test scoring a candidate pose in an environmental query.
 * Multiplicative test: returns a score in [0.0, 1.0]. A score of 0.0 acts as a hard veto.
 */
@FunctionalInterface
public interface PointTest {

    /**
     * Scores a candidate pose.
     *
     * @param candidate candidate pose being evaluated
     * @param context   query context
     * @return utility multiplier in [0.0, 1.0]. Returns 0.0 to veto the candidate.
     */
    double score(Pose2d candidate, PointGenerator.QueryContext context);

    /**
     * Standard obstacle clearance test: vetoes (returns 0.0) any candidate inside hard or dynamic obstacles.
     */
    static PointTest obstacleClearance() {
        return (candidate, context) -> {
            if (candidate == null) {
                return 0.0;
            }
            if (StaticPathfinder.isPointInHardObstacle(candidate.getTranslation())
                    || StaticPathfinder.isPointNearDynamicObstacle(candidate.getTranslation())) {
                return 0.0; // Hard veto
            }
            return 1.0;
        };
    }

    /**
     * Distance-to-target test evaluated through a response curve.
     */
    static PointTest distanceToTarget(
            edu.wpi.first.math.geometry.Translation2d target,
            double minExpectedDist,
            double maxExpectedDist,
            ResponseCurve curve) {
        return (candidate, context) -> {
            if (candidate == null || target == null) {
                return 0.0;
            }
            double dist = candidate.getTranslation().getDistance(target);
            double range = maxExpectedDist - minExpectedDist;
            double norm = (range > 1e-6) ? (dist - minExpectedDist) / range : 0.0;
            return curve.calculate(norm);
        };
    }

    /**
     * Heading alignment test: scores how well candidate heading points toward a target.
     */
    static PointTest headingAlignment(edu.wpi.first.math.geometry.Translation2d target) {
        return (candidate, context) -> {
            if (candidate == null || target == null) {
                return 0.0;
            }
            edu.wpi.first.math.geometry.Rotation2d desiredAngle =
                    target.minus(candidate.getTranslation()).getAngle();
            double angleDiffRad = Math.abs(candidate.getRotation().minus(desiredAngle).getRadians());
            return Math.max(0.0, Math.cos(angleDiffRad));
        };
    }
}
