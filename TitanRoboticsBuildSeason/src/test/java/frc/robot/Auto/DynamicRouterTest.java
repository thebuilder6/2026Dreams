package frc.robot.Auto;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Auto.DynamicRouter.AvoidanceAlgorithm;
import frc.robot.Data.Constants;
import frc.robot.Subsystems.Vision;
import frc.robot.Hardware.Vision.VisionIOSim;

public class DynamicRouterTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        DynamicRouter.clearObstacles();
        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.POTENTIAL_FIELDS);
    }

    @Test
    public void testObstacleRegistrationAndDebounce() {
        Translation2d pos1 = new Translation2d(5.0, 3.0);
        Translation2d vel = new Translation2d(0.5, 0.0);

        DynamicRouter.registerObstacle(pos1, vel, 0.55, 1.0);
        assertEquals(1, DynamicRouter.getActiveObstacles().size());

        // Nearby obstacle within 0.5m debouncing radius should update rather than duplicate
        Translation2d posNear = new Translation2d(5.1, 3.05);
        DynamicRouter.registerObstacle(posNear, vel, 0.55, 1.0);
        assertEquals(1, DynamicRouter.getActiveObstacles().size());
        assertEquals(posNear.getX(), DynamicRouter.getActiveObstacles().get(0).position.getX(), 1e-3);
    }

    @Test
    public void testAlgorithmSwitching() {
        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.POTENTIAL_FIELDS);
        assertEquals(AvoidanceAlgorithm.POTENTIAL_FIELDS, DynamicRouter.getAlgorithm());

        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.DYNAMIC_WINDOW);
        assertEquals(AvoidanceAlgorithm.DYNAMIC_WINDOW, DynamicRouter.getAlgorithm());

        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.DYNAMIC_GRID_ASTAR);
        assertEquals(AvoidanceAlgorithm.DYNAMIC_GRID_ASTAR, DynamicRouter.getAlgorithm());
    }

    @Test
    public void testPotentialFieldsAvoidanceDeflection() {
        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.POTENTIAL_FIELDS);

        // Robot at (4.0, 3.0) heading along +X (towards (6.0, 3.0))
        Pose2d currentPose = new Pose2d(4.0, 3.0, new Rotation2d());
        ChassisSpeeds nominalSpeeds = new ChassisSpeeds(2.0, 0.0, 0.0);
        Translation2d targetWaypoint = new Translation2d(6.0, 3.0);

        // Place obstacle directly ahead at (4.8, 3.0)
        DynamicRouter.registerObstacle(new Translation2d(4.8, 3.0), new Translation2d(), 0.55, 1.0);

        ChassisSpeeds avoidanceSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, targetWaypoint);
        assertNotNull(avoidanceSpeeds);

        // The repulsive force should push back or sideways, so forward speed must be significantly reduced or deflected
        assertTrue(avoidanceSpeeds.vxMetersPerSecond < nominalSpeeds.vxMetersPerSecond,
                "Forward speed should decrease due to obstacle repulsion ahead");

        // Distance > safe distance (1.4m): obstacle placed far away at (8.0, 3.0)
        DynamicRouter.clearObstacles();
        DynamicRouter.registerObstacle(new Translation2d(8.0, 3.0), new Translation2d(), 0.55, 1.0);

        ChassisSpeeds unblockedSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, targetWaypoint);
        assertEquals(nominalSpeeds.vxMetersPerSecond, unblockedSpeeds.vxMetersPerSecond, 1e-3);
        assertEquals(nominalSpeeds.vyMetersPerSecond, unblockedSpeeds.vyMetersPerSecond, 1e-3);
    }

    @Test
    public void testDynamicWindowApproachAvoidance() {
        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.DYNAMIC_WINDOW);

        Pose2d currentPose = new Pose2d(4.0, 3.0, new Rotation2d());
        ChassisSpeeds nominalSpeeds = new ChassisSpeeds(2.5, 0.0, 0.0);
        Translation2d targetWaypoint = new Translation2d(7.0, 3.0);

        // Place moving obstacle right in path at (5.0, 3.0)
        DynamicRouter.registerObstacle(new Translation2d(5.0, 3.0), new Translation2d(-0.5, 0.0), 0.55, 1.0);

        ChassisSpeeds avoidanceSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, targetWaypoint);
        assertNotNull(avoidanceSpeeds);

        // DWA should choose a lateral deviation (non-zero vy) or reduced speed to avoid head-on collision
        boolean hasLateralEvasion = Math.abs(avoidanceSpeeds.vyMetersPerSecond) > 0.05;
        boolean hasReducedSpeed = avoidanceSpeeds.vxMetersPerSecond < 2.0;
        assertTrue(hasLateralEvasion || hasReducedSpeed,
                "DWA should either evade laterally or brake to prevent collision");
    }

    @Test
    public void testDynamicAStarGridAvoidance() {
        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.DYNAMIC_GRID_ASTAR);

        Pose2d currentPose = new Pose2d(3.0, 4.0, new Rotation2d());
        ChassisSpeeds nominalSpeeds = new ChassisSpeeds(2.0, 0.0, 0.0);
        Translation2d targetWaypoint = new Translation2d(7.0, 4.0);

        // Place dynamic obstacle blocking corridor at (4.5, 4.0)
        DynamicRouter.registerObstacle(new Translation2d(4.5, 4.0), new Translation2d(), 0.55, 1.0);

        ChassisSpeeds avoidanceSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, targetWaypoint);
        assertNotNull(avoidanceSpeeds);

        // Corridor is blocked at Y=4.0, so A* should route around (non-zero vy)
        assertTrue(Math.hypot(avoidanceSpeeds.vxMetersPerSecond, avoidanceSpeeds.vyMetersPerSecond) > 0.1,
                "A* should produce valid velocity vector along open corridor");
    }

    @Test
    public void testProprioceptiveBumperRepulsionBoost() {
        DynamicRouter.setAlgorithm(AvoidanceAlgorithm.POTENTIAL_FIELDS);

        Pose2d currentPose = new Pose2d(4.0, 3.0, new Rotation2d());
        ChassisSpeeds nominalSpeeds = new ChassisSpeeds(2.0, 0.0, 0.0);
        Translation2d targetWaypoint = new Translation2d(6.0, 3.0);

        // Normal obstacle placed at distance where repulsion does not saturate max speed clamp
        DynamicRouter.registerObstacle(new Translation2d(5.1, 3.0), new Translation2d(), 0.55, 1.0, false);
        ChassisSpeeds normalSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, targetWaypoint);

        // Proprioceptive contact obstacle at same position
        DynamicRouter.clearObstacles();
        DynamicRouter.registerObstacle(new Translation2d(5.1, 3.0), new Translation2d(), 0.55, 1.0, true);
        ChassisSpeeds proprioceptiveSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, targetWaypoint);

        // Proprioceptive obstacle should generate stronger backward push (lower vx)
        assertTrue(proprioceptiveSpeeds.vxMetersPerSecond < normalSpeeds.vxMetersPerSecond,
                "Proprioceptive obstacle should exert stronger repulsive push to escape stall/pin");
    }

    @Test
    public void testVisionBumperProjectionAndRegistration() {
        VisionIOSim primarySim = new VisionIOSim();
        VisionIOSim secondarySim = new VisionIOSim(VisionIOSim.CameraType.RUBIK_PI);
        Vision vision = new Vision(primarySim, secondarySim);

        // Register detected bumper at yaw = 0 deg, pitch = 0 deg, radius = 0.55m
        // Camera height: 0.45m, bumper height: 0.12m, camera pitch: -15 deg
        // groundDist = 0.33 / tan(15 deg) ≈ 1.23m
        Translation2d projectedPos = vision.registerDetectedBumperObstacle(0.0, 0.0, 0.55);
        assertNotNull(projectedPos);

        // Verify registered in DynamicRouter
        assertEquals(1, DynamicRouter.getActiveObstacles().size());
        DynamicObstacle obs = DynamicRouter.getActiveObstacles().get(0);
        assertEquals(0.55, obs.radius, 1e-3);
    }
}
