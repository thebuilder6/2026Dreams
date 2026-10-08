package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Set;

/**
 * Default {@link ObstacleModel} implementation for FRC Team 8334.
 * Delegates geometric queries directly to {@link StaticPathfinder}'s tested obstacle models.
 */
public class FieldMapObstacleModel implements ObstacleModel {

    public FieldMapObstacleModel() {
    }

    @Override
    public boolean isLineOfSightClear(Translation2d p1, Translation2d p2) {
        return StaticPathfinder.isLineOfSightClear(p1, p2);
    }

    @Override
    public boolean isPointInStaticObstacle(Translation2d p) {
        return StaticPathfinder.isPointInStaticObstacle(p);
    }

    @Override
    public boolean isPointInObstacle(Translation2d p) {
        return StaticPathfinder.isPointInObstacle(p);
    }

    @Override
    public double segmentMinClearance(Translation2d a, Translation2d b) {
        return StaticPathfinder.segmentMinClearance(a, b);
    }

    @Override
    public Set<Integer> getBlockedCorridorNodes(Translation2d egoPosition) {
        return StaticPathfinder.getBlockedTrenchNodes(egoPosition);
    }

    @Override
    public Pose2d ensurePoseOutsideObstacles(Pose2d target, Translation2d referenceFrom) {
        return StaticPathfinder.ensurePoseOutsideObstacles(target, referenceFrom);
    }
}
