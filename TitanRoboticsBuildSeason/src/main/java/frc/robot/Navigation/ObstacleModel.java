package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Set;

/**
 * Interface abstracting obstacle detection, line-of-sight raycasting, and clearance evaluation.
 * Decouples geometric environment queries from graph search algorithms in {@link VisibilityGraphPlanner}.
 */
public interface ObstacleModel {

    /** Checks whether a straight line between two translations is unobstructed by obstacles. */
    boolean isLineOfSightClear(Translation2d p1, Translation2d p2);

    /** Checks if a point is within any static obstacle or outside the field boundary. */
    boolean isPointInStaticObstacle(Translation2d p);

    /** Checks if a point is within any static or dynamic obstacle. */
    boolean isPointInObstacle(Translation2d p);

    /** Returns the minimum clearance distance from the segment to any obstacle. */
    double segmentMinClearance(Translation2d a, Translation2d b);

    /** Returns set of roadmap node IDs blocked by dynamic obstacles or corridor occupancy. */
    Set<Integer> getBlockedCorridorNodes(Translation2d egoPosition);

    /** Ensures a target pose is outside obstacles, projecting to nearest clear carpet if needed. */
    Pose2d ensurePoseOutsideObstacles(Pose2d target, Translation2d referenceFrom);
}
