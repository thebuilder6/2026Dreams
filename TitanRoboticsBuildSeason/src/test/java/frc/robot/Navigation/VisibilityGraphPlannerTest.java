package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.StaticPathfinder.PathResult;
import frc.robot.Navigation.StaticPathfinder.PathStatus;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests verifying algorithmic correctness, edge clearance cost penalty,
 * and exact parity of {@link VisibilityGraphPlanner} against {@link StaticPathfinder}.
 */
class VisibilityGraphPlannerTest {

    private VisibilityGraphPlanner planner;

    @BeforeEach
    void setUp() {
        planner = new VisibilityGraphPlanner(Rebuilt2026Roadmap.getInstance());
    }

    @Test
    void testDirectLineOfSight() {
        Pose2d start = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));
        Pose2d target = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));

        PathResult result = planner.planPathWithStatus(start, target);
        assertEquals(PathStatus.DIRECT, result.status());
        assertFalse(result.waypoints().isEmpty());
        Pose2d finalPose = result.waypoints().get(result.waypoints().size() - 1);
        assertEquals(target.getX(), finalPose.getX(), 1e-6);
        assertEquals(target.getY(), finalPose.getY(), 1e-6);
        assertEquals(target.getRotation().getDegrees(), finalPose.getRotation().getDegrees(), 1e-6);
    }

    @Test
    void testExactParityWithStaticPathfinderAcrossField() {
        // Test diverse pairs:
        // 1. Blue Alliance to Midfield (around Blue Hub)
        // 2. Midfield to Red Alliance (around Red Hub)
        // 3. Diagonal transit across the field
        // 4. Trench transits
        Pose2d[] starts = {
            new Pose2d(1.80, 6.00, Rotation2d.fromDegrees(0)),  // Blue Alliance Top
            new Pose2d(2.40, 4.035, Rotation2d.fromDegrees(0)), // Blue Alliance Center
            new Pose2d(8.27, 4.00, Rotation2d.fromDegrees(180)),// Midfield Center
            new Pose2d(14.74, 2.00, Rotation2d.fromDegrees(180))// Red Alliance Bottom
        };

        Pose2d[] targets = {
            new Pose2d(8.27, 6.20, Rotation2d.fromDegrees(0)),  // Midfield Top
            new Pose2d(14.14, 4.035, Rotation2d.fromDegrees(0)),// Red Alliance Center
            new Pose2d(1.80, 2.00, Rotation2d.fromDegrees(180)),// Blue Alliance Bottom
            new Pose2d(6.20, 5.75, Rotation2d.fromDegrees(0))   // Blue Hub Midfield Top
        };

        for (int i = 0; i < starts.length; i++) {
            Pose2d start = starts[i];
            Pose2d target = targets[i];

            PathResult plannerResult = planner.planPathWithStatus(start, target);
            PathResult pathfinderResult = StaticPathfinder.findPathWithStatus(start, target);

            assertEquals(pathfinderResult.status(), plannerResult.status(),
                    "Status mismatch between planner and StaticPathfinder for pair " + i);
            assertEquals(pathfinderResult.waypoints().size(), plannerResult.waypoints().size(),
                    "Waypoint count mismatch for pair " + i);

            for (int w = 0; w < plannerResult.waypoints().size(); w++) {
                Pose2d pw = plannerResult.waypoints().get(w);
                Pose2d sw = pathfinderResult.waypoints().get(w);
                assertEquals(sw.getX(), pw.getX(), 1e-5,
                        "Waypoint " + w + " X mismatch for pair " + i);
                assertEquals(sw.getY(), pw.getY(), 1e-5,
                        "Waypoint " + w + " Y mismatch for pair " + i);
                assertEquals(sw.getRotation().getRadians(), pw.getRotation().getRadians(), 1e-5,
                        "Waypoint " + w + " Rotation mismatch for pair " + i);
            }
        }
    }

    @Test
    void testCustomObstacleModelInjection() {
        // Verify mockability: planner can run with a synthetic obstacle model
        ObstacleModel mockObstacles = new ObstacleModel() {
            @Override
            public boolean isLineOfSightClear(Translation2d p1, Translation2d p2) {
                // Mock: block everything directly
                return false;
            }

            @Override
            public boolean isPointInStaticObstacle(Translation2d p) {
                return false;
            }

            @Override
            public boolean isPointInObstacle(Translation2d p) {
                return false;
            }

            @Override
            public double segmentMinClearance(Translation2d a, Translation2d b) {
                return 1.0;
            }

            @Override
            public Set<Integer> getBlockedCorridorNodes(Translation2d egoPosition) {
                return Set.of();
            }

            @Override
            public Pose2d ensurePoseOutsideObstacles(Pose2d target, Translation2d referenceFrom) {
                return target;
            }
        };

        VisibilityGraphPlanner customPlanner = new VisibilityGraphPlanner(
                Rebuilt2026Roadmap.getInstance(), mockObstacles, 0.45);

        Pose2d start = new Pose2d(2.40, 4.035, new Rotation2d());
        Pose2d target = new Pose2d(14.14, 4.035, new Rotation2d());

        // When line of sight is mocked to false everywhere, it cannot connect start/target to roadmap
        // and gracefully drops to local recovery / unreachable
        PathResult result = customPlanner.planPathWithStatus(start, target);
        assertNotNull(result);
        assertEquals(PathStatus.UNREACHABLE, result.status());
    }

    @Test
    void testClearancePenalisedCostScaling() {
        Translation2d p1 = new Translation2d(0, 0);
        Translation2d p2 = new Translation2d(1, 0);

        // When clearance is tight (< CLEARANCE_TIGHT_M = 0.30m), cost multiplier scales up to 1.60x
        ObstacleModel tightObstacles = new ObstacleModel() {
            @Override public boolean isLineOfSightClear(Translation2d a, Translation2d b) { return true; }
            @Override public boolean isPointInStaticObstacle(Translation2d p) { return false; }
            @Override public boolean isPointInObstacle(Translation2d p) { return false; }
            @Override public double segmentMinClearance(Translation2d a, Translation2d b) { return 0.0; } // 0 clearance
            @Override public Set<Integer> getBlockedCorridorNodes(Translation2d ego) { return Set.of(); }
            @Override public Pose2d ensurePoseOutsideObstacles(Pose2d t, Translation2d ref) { return t; }
        };

        VisibilityGraphPlanner tightPlanner = new VisibilityGraphPlanner(
                Rebuilt2026Roadmap.getInstance(), tightObstacles);

        double tightCost = tightPlanner.clearancePenalisedCost(p1, p2, 1.0);
        assertEquals(1.60, tightCost, 1e-6, "Zero clearance must scale cost to maximum 1.60x");

        // When clearance is safe (>= 0.30m), cost must equal nominal length
        ObstacleModel safeObstacles = new ObstacleModel() {
            @Override public boolean isLineOfSightClear(Translation2d a, Translation2d b) { return true; }
            @Override public boolean isPointInStaticObstacle(Translation2d p) { return false; }
            @Override public boolean isPointInObstacle(Translation2d p) { return false; }
            @Override public double segmentMinClearance(Translation2d a, Translation2d b) { return 0.50; }
            @Override public Set<Integer> getBlockedCorridorNodes(Translation2d ego) { return Set.of(); }
            @Override public Pose2d ensurePoseOutsideObstacles(Pose2d t, Translation2d ref) { return t; }
        };

        VisibilityGraphPlanner safePlanner = new VisibilityGraphPlanner(
                Rebuilt2026Roadmap.getInstance(), safeObstacles);

        double safeCost = safePlanner.clearancePenalisedCost(p1, p2, 1.0);
        assertEquals(1.0, safeCost, 1e-6, "Safe clearance must preserve 1.0x nominal length");
    }
}
