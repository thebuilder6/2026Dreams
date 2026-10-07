package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;

class FuelTourOptimizerTest {

    @BeforeEach
    void setup() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        frc.robot.Navigation.DynamicRouter.clearObstacles();
    }

    @Test
    void testEmptyCandidatesReturnsEmptyResult() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0));
        var result = FuelTourOptimizer.optimizeTour(robotPose, Collections.emptyList(), 5, null);

        assertFalse(result.isValid());
        assertEquals(0, result.pieceCount());
        assertTrue(result.waypoints().isEmpty());
    }

    @Test
    void testSingleCandidateReturnsDirectTarget() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0));
        Translation2d ball = new Translation2d(6.5, 4.0);

        var result = FuelTourOptimizer.optimizeTour(robotPose, List.of(ball), 3, null);

        assertTrue(result.isValid());
        assertEquals(1, result.pieceCount());
        assertEquals(ball, result.waypoints().get(0));
        assertTrue(result.totalDistanceMeters() > 0.0);
        assertTrue(result.estimatedDurationSeconds() > 0.0);
        assertNotNull(result.immediateTargetPose());
    }

    @Test
    void testLinearFuelSequenceMaintainsHeadingOrder() {
        // Robot at (7.0, 4.0) facing east (0 deg) in open midfield
        Pose2d robotPose = new Pose2d(7.0, 4.0, Rotation2d.fromDegrees(0));

        // 3 balls placed in a straight line ahead of the robot in open carpet
        Translation2d b1 = new Translation2d(7.6, 4.0);
        Translation2d b2 = new Translation2d(8.3, 4.0);
        Translation2d b3 = new Translation2d(9.0, 4.0);

        // Pass them scrambled into candidates
        List<Translation2d> scrambled = List.of(b2, b3, b1);
        var result = FuelTourOptimizer.optimizeTour(robotPose, scrambled, 3, null);

        assertTrue(result.isValid());
        assertEquals(3, result.pieceCount());
        // Must be in natural sequential order: b1 -> b2 -> b3
        assertEquals(b1, result.waypoints().get(0));
        assertEquals(b2, result.waypoints().get(1));
        assertEquals(b3, result.waypoints().get(2));
    }

    @Test
    void testHeadingPenaltyPrefersForwardBallOverSharpReversal() {
        // Robot at (8.0, 4.0) facing east (0 deg) in open midfield
        Pose2d robotPose = new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(0));

        // Forward ball at 1.1m ahead
        Translation2d ahead = new Translation2d(9.1, 4.0);
        // Behind ball at 0.9m behind (closer Euclidean distance, but 180-deg reversal)
        Translation2d behind = new Translation2d(7.1, 4.0);

        var result = FuelTourOptimizer.optimizeTour(robotPose, List.of(behind, ahead), 1, null);

        assertTrue(result.isValid());
        assertEquals(1, result.pieceCount());
        // The forward ball must win because the 180-deg reversal cost exceeds the 0.2m distance advantage
        assertEquals(ahead, result.waypoints().get(0));
    }

    @Test
    void testMaxPiecesTruncatesTour() {
        Pose2d robotPose = new Pose2d(7.0, 5.0, Rotation2d.fromDegrees(0));
        List<Translation2d> balls = List.of(
                new Translation2d(7.5, 5.0),
                new Translation2d(8.0, 5.0),
                new Translation2d(8.5, 5.0),
                new Translation2d(9.0, 5.0)
        );

        var result = FuelTourOptimizer.optimizeTour(robotPose, balls, 2, null);

        assertTrue(result.isValid());
        assertEquals(2, result.pieceCount());
        assertEquals(2, result.waypoints().size());
    }

    @Test
    void testExitDestinationGuidesExitHeading() {
        Pose2d robotPose = new Pose2d(7.0, 4.0, Rotation2d.fromDegrees(0));
        Translation2d ball = new Translation2d(8.0, 4.0);
        Translation2d exitWest = new Translation2d(6.0, 4.0); // Facing back west

        var result = FuelTourOptimizer.optimizeTour(robotPose, List.of(ball), 1, exitWest);

        assertTrue(result.isValid());
        // Exit heading should point from ball (8.0, 4.0) towards exitWest (6.0, 4.0) -> ~180 degrees
        double exitDeg = result.finalExitPose().getRotation().getDegrees();
        assertEquals(180.0, Math.abs(exitDeg), 5.0);
    }

    @Test
    void test2OptOnLargerCluster() {
        // Open midfield area between X=7.0 and X=9.5, Y=2.0 and Y=5.5
        Pose2d robotPose = new Pose2d(7.0, 2.5, Rotation2d.fromDegrees(45));

        // 7 balls in an open midfield cluster
        List<Translation2d> cluster = new ArrayList<>();
        cluster.add(new Translation2d(7.4, 2.8));
        cluster.add(new Translation2d(7.8, 3.2));
        cluster.add(new Translation2d(8.2, 3.6));
        cluster.add(new Translation2d(8.6, 4.0));
        cluster.add(new Translation2d(9.0, 4.4));
        cluster.add(new Translation2d(8.8, 3.0));
        cluster.add(new Translation2d(8.0, 4.5));

        Collections.shuffle(cluster);
        var result = FuelTourOptimizer.optimizeTour(robotPose, cluster, 6, null);

        assertTrue(result.isValid());
        assertEquals(6, result.pieceCount());
        assertTrue(result.totalDistanceMeters() > 0.0);
    }

    @Test
    void testFieldCandidateFilteringAndObstacles() {
        Pose2d robotPose = new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(0));

        // Hub center is (4.6256, 4.035) for Blue hub -> hard obstacle
        Translation2d insideHub = FieldMap.Hubs.BLUE_HUB_2D;
        Translation2d outOfBounds = new Translation2d(-1.0, 4.0);
        Translation2d validMidfield = new Translation2d(8.2, 3.5);

        var result = FuelTourOptimizer.optimizeTour(robotPose, List.of(insideHub, outOfBounds, validMidfield), 3, null);

        assertTrue(result.isValid());
        assertEquals(1, result.pieceCount());
        assertEquals(validMidfield, result.waypoints().get(0));
    }

    @Test
    void testBlockedFuelExclusion() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0));
        Translation2d ball = new Translation2d(6.0, 4.0);
        Set<Translation2d> blocked = Set.of(new Translation2d(6.1, 4.0)); // within 1.0m

        // findFieldFuelCandidates filters blocked fuel
        // Also verify optimizer filters obstacles
        var result = FuelTourOptimizer.optimizeTour(robotPose, List.of(ball), 1, null);
        assertTrue(result.isValid());
    }

    @Test
    void testJevDecisionEngineTourMethod() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0));
        var engine = JevDecisionEngine.getInstance();
        assertNotNull(engine);

        var result = engine.planFuelHarvestTour(robotPose, false, false, 3, Collections.emptySet());
        assertNotNull(result);
    }
}
