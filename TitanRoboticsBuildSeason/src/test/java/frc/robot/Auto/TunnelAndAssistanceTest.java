package frc.robot.Auto;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Auto.Actions.DriveToPoseAction;
import frc.robot.Data.GlideConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.ShootingSolution;
import frc.robot.Subsystems.SwerveBase;

public class TunnelAndAssistanceTest {

    @BeforeAll
    public static void setup() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @Test
    public void testTrenchLowClearanceGeofence() {
        // Blue Top Trench (X in [3.20, 6.10], Y >= 6.50)
        assertTrue(Intake.isPoseInTrenchLowClearanceZone(new Pose2d(4.5, 7.0, new Rotation2d())),
                "Pose inside Blue Top Trench should be flagged as low clearance");

        // Blue Bottom Trench (X in [3.20, 6.10], Y <= 1.55)
        assertTrue(Intake.isPoseInTrenchLowClearanceZone(new Pose2d(3.5, 0.8, new Rotation2d())),
                "Pose inside Blue Bottom Trench should be flagged as low clearance");

        // Red Top Trench (X in [10.44, 13.34], Y >= 6.50)
        assertTrue(Intake.isPoseInTrenchLowClearanceZone(new Pose2d(11.5, 7.2, new Rotation2d())),
                "Pose inside Red Top Trench should be flagged as low clearance");

        // Red Bottom Trench (X in [10.44, 13.34], Y <= 1.55)
        assertTrue(Intake.isPoseInTrenchLowClearanceZone(new Pose2d(12.0, 1.0, new Rotation2d())),
                "Pose inside Red Bottom Trench should be flagged as low clearance");

        // Open field locations should NOT be flagged
        assertFalse(Intake.isPoseInTrenchLowClearanceZone(new Pose2d(8.27, 4.035, new Rotation2d())),
                "Field center should not be low clearance");
        assertFalse(Intake.isPoseInTrenchLowClearanceZone(new Pose2d(2.0, 4.0, new Rotation2d())),
                "Alliance zone should not be low clearance");
        assertFalse(Intake.isPoseInTrenchLowClearanceZone(new Pose2d(4.5, 4.0, new Rotation2d())),
                "Hub area should not be low clearance");
        assertFalse(Intake.isPoseInTrenchLowClearanceZone(null),
                "Null pose should safely return false");
    }

    @Test
    public void testGlideConstantsMatchingTunnelEntrance() {
        // Match Blue Top Trench entrance
        Pose2d blueTopEntrancePose = new Pose2d(3.50, GlideConstants.Y_TOP_LANE, Rotation2d.fromDegrees(0));
        GlideConstants.GlidePoint blueTopMatch = GlideConstants.getMatchingTunnelEntrance(blueTopEntrancePose);
        assertNotNull(blueTopMatch, "Should find matching GlidePoint for Blue Top Trench");
        assertTrue(blueTopMatch.isTunnelEntrance, "Matched point should be marked as tunnel entrance");
        assertNotNull(blueTopMatch.tunnelExitPose, "Tunnel exit pose should be defined");
        assertEquals(5.75, blueTopMatch.tunnelExitPose.getX(), 0.05);

        // Match with small positional perturbation (within tolerance)
        Pose2d perturbedPose = new Pose2d(3.60, GlideConstants.Y_TOP_LANE + 0.05, Rotation2d.fromDegrees(0));
        assertNotNull(GlideConstants.getMatchingTunnelEntrance(perturbedPose, 0.40));

        // Poses far from tunnels should return null
        Pose2d hubFront = new Pose2d(5.60, 4.035, Rotation2d.fromDegrees(180));
        assertNull(GlideConstants.getMatchingTunnelEntrance(hubFront));
        assertNull(GlideConstants.getMatchingTunnelEntrance(null));
    }

    @Test
    public void testMidfieldGlidePointsNotCorruptedByRedMirroring() {
        assertTrue(GlideConstants.GLIDE_POINTS.containsKey("Midfield Top"));
        assertTrue(GlideConstants.GLIDE_POINTS.containsKey("Midfield Bottom"));

        Pose2d midTop = GlideConstants.GLIDE_POINTS.get("Midfield Top").pose();
        Pose2d midBot = GlideConstants.GLIDE_POINTS.get("Midfield Bottom").pose();

        // Midfield Top must stay at high Y (6.10m), Midfield Bottom at low Y (2.00m)
        assertEquals(6.10, midTop.getY(), 0.05, "Midfield Top must be at Y=6.10m and not inverted to bottom");
        assertEquals(2.00, midBot.getY(), 0.05, "Midfield Bottom must be at Y=2.00m and not inverted to top");
    }

    @Test
    public void testDriveToPoseActionTunnelSequencing() {
        SwerveBase.getInstance().resetOdometry(new Pose2d(2.0, GlideConstants.Y_BOT_LANE, Rotation2d.fromDegrees(0)));
        Pose2d blueBottomEntrance = new Pose2d(3.50, GlideConstants.Y_BOT_LANE, Rotation2d.fromDegrees(0));
        DriveToPoseAction tunnelAction = new DriveToPoseAction(blueBottomEntrance);

        assertTrue(tunnelAction.isTunnelTransit(), "Targeting a tunnel entrance should trigger tunnel transit mode");
        assertEquals(0.0, tunnelAction.getTunnelHeading().getDegrees(), 1e-4, "Tunnel heading should be locked to 0 deg");

        List<Pose2d> waypoints = tunnelAction.getWaypoints();
        assertFalse(waypoints.isEmpty(), "Waypoints should not be empty");
        // SmartTunnelRouter generates [preEntrance, entrance, exit, postExit]
        assertEquals(4, waypoints.size(), "SmartTunnelRouter should generate 4 corridor waypoints");
        assertEquals(5.75, waypoints.get(2).getX(), 0.10, "Third waypoint must be the tunnel exit point");
        assertEquals(6.35, waypoints.get(3).getX(), 0.10, "Final waypoint must be the post-exit point");

        // Non-tunnel destination should NOT activate tunnel transit
        Pose2d midfieldPoint = new Pose2d(8.27, 6.10, Rotation2d.fromDegrees(-90));
        DriveToPoseAction normalAction = new DriveToPoseAction(midfieldPoint);
        assertFalse(normalAction.isTunnelTransit(), "Normal target should not activate tunnel transit mode");
    }

    @Test
    public void testShooterSOTFVectorBallistics() {
        Shooter shooter = Shooter.getInstance();
        Pose2d shootingPose = new Pose2d(3.0, 2.5, Rotation2d.fromDegrees(0));

        // 1. Stationary solution
        ShootingSolution stationarySol = shooter.calculateShootingSolution(shootingPose, new ChassisSpeeds(0, 0, 0));
        assertNotNull(stationarySol);
        assertTrue(stationarySol.possible());

        // 2. Lateral strafing (+Y direction)
        // Tangential exit velocity will drift ball in +Y, so SOTF must lead virtual goal in -Y
        ChassisSpeeds strafingSpeeds = new ChassisSpeeds(0.0, 3.5, 0.0);
        ShootingSolution movingSol = shooter.calculateShootingSolution(shootingPose, strafingSpeeds);
        assertNotNull(movingSol);
        assertTrue(movingSol.possible());

        // Moving solution angle should be biased towards -Y (clockwise/smaller angle) relative to stationary
        double diffDeg = movingSol.shootingAngle().minus(stationarySol.shootingAngle()).getDegrees();
        assertTrue(diffDeg < 0.0,
                "Strafing in +Y should lead aim angle in -Y direction, actual diff: " + diffDeg);

        // Virtual goal distance should be longer due to vector displacement, demanding higher or compensated RPM
        assertTrue(movingSol.flywheelRpmLeft() > 0, "Compensated RPM should be positive");
    }

    @Test
    public void testSwerveBaseCollisionTracking() {
        SwerveBase swerve = SwerveBase.getInstance();
        assertNotNull(swerve);
        swerve.getCollisionDetector().reset();

        // Initial state
        assertFalse(swerve.isCollisionDetected(), "Collision should not be detected at initialization");
        assertTrue(swerve.getCollisionJerkMagnitude() >= 0.0, "Jerk magnitude should be non-negative");

        // Normal drive update cycles should not trigger collision
        swerve.drive(new Translation2d(1.5, 0.0), 0.0, false);
        swerve.update();
        assertFalse(swerve.isCollisionDetected(), "Normal forward acceleration should not trigger collision impact");
    }

    @Test
    public void testSmartTunnelRouterBiDirectional() {
        // 1. Approaching from Alliance Zone (X = 2.0m) -> West to East
        Pose2d alliancePose = new Pose2d(2.0, GlideConstants.Y_TOP_LANE, Rotation2d.fromDegrees(0));
        SmartTunnelRouter.TunnelRoute westToEast = SmartTunnelRouter.planTunnelRoute(alliancePose, true);

        assertTrue(westToEast.isWestToEast(), "Should be West to East when starting in Alliance Zone");
        assertEquals(SmartTunnelRouter.TrenchCorridor.TOP_TRENCH, westToEast.corridor);
        assertEquals(0.0, westToEast.corridorHeading.getDegrees(), 1e-4);
        assertEquals(3.50, westToEast.entrancePose.getX(), 0.05);
        assertEquals(5.75, westToEast.exitPose.getX(), 0.05);

        // 2. Approaching from Midfield (X = 7.0m) -> East to West
        Pose2d midfieldPose = new Pose2d(7.0, GlideConstants.Y_TOP_LANE, Rotation2d.fromDegrees(180));
        SmartTunnelRouter.TunnelRoute eastToWest = SmartTunnelRouter.planTunnelRoute(midfieldPose, true);

        assertFalse(eastToWest.isWestToEast(), "Should be East to West when starting in Midfield");
        assertEquals(SmartTunnelRouter.TrenchCorridor.TOP_TRENCH, eastToWest.corridor);
        assertEquals(180.0, Math.abs(eastToWest.corridorHeading.getDegrees()), 1e-4);
        assertEquals(5.75, eastToWest.entrancePose.getX(), 0.05);
        assertEquals(3.50, eastToWest.exitPose.getX(), 0.05);
    }

    @Test
    public void testSmartTunnelRouterAutoDiversion() {
        // Place an active opponent obstacle inside the Top Trench corridor
        DynamicRouter.clearObstacles();
        DynamicRouter.registerObstacle(new edu.wpi.first.math.geometry.Translation2d(4.5, 7.2), new edu.wpi.first.math.geometry.Translation2d(), 0.50, 2.0);

        Pose2d robotPose = new Pose2d(2.0, GlideConstants.Y_TOP_LANE, Rotation2d.fromDegrees(0));
        // Prefer top trench, but it's blocked by the opponent!
        SmartTunnelRouter.TunnelRoute route = SmartTunnelRouter.planTunnelRoute(robotPose, true);

        assertTrue(route.isDiverted(), "Route should be automatically diverted when preferred trench is blocked");
        assertEquals(SmartTunnelRouter.TrenchCorridor.BOTTOM_TRENCH, route.corridor,
                "Should divert from Top Trench to Bottom Trench");
        assertEquals(GlideConstants.Y_BOT_LANE, route.entrancePose.getY(), 0.05);

        DynamicRouter.clearObstacles();
    }

    @Test
    public void testBallHuntActionSharedAuthorityAndMemory() {
        frc.robot.Auto.Actions.BallHuntAction hunt = new frc.robot.Auto.Actions.BallHuntAction();
        assertNotNull(hunt);

        hunt.start();
        hunt.setDriverInput(1.5, 0.5);

        // Verify ball acquired flag starts false
        assertFalse(hunt.checkAndClearBallAcquired());

        // Update loop should execute cleanly without throwing
        assertDoesNotThrow(hunt::update);

        hunt.done();
        assertFalse(hunt.isFinished());
    }
}
