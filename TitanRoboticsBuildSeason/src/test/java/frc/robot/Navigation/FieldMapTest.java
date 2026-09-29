package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;

public class FieldMapTest {

    @Test
    public void testFieldDimensionsAndCenterline() {
        assertEquals(16.541, FieldMap.FIELD_LENGTH, 1e-3);
        assertEquals(8.069, FieldMap.FIELD_WIDTH, 1e-3);
        assertEquals(8.2705, FieldMap.CENTERLINE_X, 1e-2);
        assertTrue(FieldMap.CENTERLINE_X < FieldMap.FIELD_LENGTH / 2.0 + 0.1);
    }

    @Test
    public void testHubLocationsAndStandoff() {
        assertEquals(4.6256, FieldMap.Hubs.BLUE_HUB_2D.getX(), 1e-3);
        assertEquals(4.0346, FieldMap.Hubs.BLUE_HUB_2D.getY(), 1e-3);
        assertEquals(16.541 - 4.6256, FieldMap.Hubs.RED_HUB_2D.getX(), 1e-3);
        assertEquals(4.0346, FieldMap.Hubs.RED_HUB_2D.getY(), 1e-3);

        assertEquals(FieldMap.Hubs.BLUE_HUB_2D, FieldMap.Hubs.getHubLocation2d(false));
        assertEquals(FieldMap.Hubs.RED_HUB_2D, FieldMap.Hubs.getHubLocation2d(true));
        assertEquals(FieldMap.Hubs.BLUE_HUB_3D, FieldMap.Hubs.getHubLocation3d(false));
        assertEquals(FieldMap.Hubs.RED_HUB_3D, FieldMap.Hubs.getHubLocation3d(true));

        assertTrue(FieldMap.Hubs.SHOOTING_MIN_DISTANCE < FieldMap.Hubs.OPTIMAL_STANDOFF_DISTANCE);
        assertTrue(FieldMap.Hubs.OPTIMAL_STANDOFF_DISTANCE < FieldMap.Hubs.SHOOTING_MAX_DISTANCE);
    }

    @Test
    public void testTrenchLowClearanceZone() {
        // Blue Top Trench (X in [3.20, 6.10], Y >= 6.50)
        assertTrue(FieldMap.Trenches.isLowClearance(new Pose2d(4.5, 7.0, new Rotation2d())));
        // Blue Bottom Trench (X in [3.20, 6.10], Y <= 1.55)
        assertTrue(FieldMap.Trenches.isLowClearance(new Pose2d(4.5, 1.0, new Rotation2d())));

        // Red Top Trench (X in [10.44, 13.34], Y >= 6.50)
        assertTrue(FieldMap.Trenches.isLowClearance(new Pose2d(11.5, 7.2, new Rotation2d())));
        // Red Bottom Trench (X in [10.44, 13.34], Y <= 1.55)
        assertTrue(FieldMap.Trenches.isLowClearance(new Pose2d(12.0, 0.8, new Rotation2d())));

        // Open ceiling areas (should be false)
        assertFalse(FieldMap.Trenches.isLowClearance(new Pose2d(8.27, 4.035, new Rotation2d())));
        assertFalse(FieldMap.Trenches.isLowClearance(new Pose2d(4.5, 4.035, new Rotation2d())));
        assertFalse(FieldMap.Trenches.isLowClearance(new Pose2d(2.0, 4.0, new Rotation2d())));
        assertFalse(FieldMap.Trenches.isLowClearance(new Pose2d(14.0, 4.0, new Rotation2d())));
        assertFalse(FieldMap.Trenches.isLowClearance((Pose2d) null));
    }

    @Test
    public void testAllianceScoringZones() {
        // Blue Alliance Zone (X <= 4.6256, single owner: FieldMap.AllianceZones)
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(2.0, 4.0), false));
        assertFalse(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(2.0, 4.0), true));

        // Red Alliance Zone (X >= 11.9154, single owner: FieldMap.AllianceZones)
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(13.5, 4.0), true));
        assertFalse(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(13.5, 4.0), false));

        // Midfield (Neutral - neither alliance zone)
        assertFalse(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(8.27, 4.0), false));
        assertFalse(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(8.27, 4.0), true));
        assertTrue(FieldMap.AllianceZones.isInMidfield(new Translation2d(8.27, 4.0)));
    }

    @Test
    public void testDepotsAndClimbingTowers() {
        assertEquals(0.09, FieldMap.Depots.BLUE_DEPOT_LOAD_POINT.getX(), 1e-3);
        assertEquals(16.00, FieldMap.Depots.RED_DEPOT_LOAD_POINT.getX(), 1e-3);

        assertEquals(FieldMap.Depots.BLUE_DEPOT_LOAD_POINT, FieldMap.Depots.getDepotLoadPoint(false));
        assertEquals(FieldMap.Depots.RED_DEPOT_LOAD_POINT, FieldMap.Depots.getDepotLoadPoint(true));

        assertEquals(FieldMap.ClimbingTowers.BLUE_TOWER_POLE, FieldMap.ClimbingTowers.getTowerPole(false));
        assertEquals(FieldMap.ClimbingTowers.RED_TOWER_POLE, FieldMap.ClimbingTowers.getTowerPole(true));
    }

    @Test
    public void testSeparatedPhysicalObstaclesAndTrenchClearance() {
        assertEquals(1.1938, FieldMap.Obstacles.BLUE_HUB_CORE.getWidth(), 1e-3);
        assertEquals(1.1938, FieldMap.Obstacles.BLUE_HUB_CORE.getHeight(), 1e-3);
        assertEquals(0.3048, FieldMap.Obstacles.BLUE_NORTH_TRENCH_WALL.getHeight(), 1e-3);
        assertEquals(1.8542, FieldMap.Obstacles.BLUE_NORTH_RAMP.getHeight(), 1e-3);
        assertTrue(FieldMap.TrenchWalls.BLUE_NORTH_WALL.contains(
                FieldMap.TrenchWalls.BLUE_CENTER_X, FieldMap.TrenchWalls.NORTH_CENTER_Y));
        assertTrue(FieldMap.Ramps.isPoseOnRamp(new Translation2d(4.62, 5.5)));
        // Ramps are a deliberate part of the hard-footprint set (decided 2026-09-26).
        // The accepted consequence is that fuel resting on a ramp slope is not a valid
        // Jev fuel target. See StaticPathfinder.isPointInHardObstacle.
        assertTrue(StaticPathfinder.isPointInHardObstacle(new Translation2d(4.62, 5.5)),
                "Ramp slope is intentionally a hard footprint");

        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.PHYSICS);
        assertTrue(StaticPathfinder.isLineOfSightClear(
                new Translation2d(2.0, 7.42), new Translation2d(7.0, 7.42)));
        assertTrue(StaticPathfinder.isLineOfSightClear(
                new Translation2d(2.0, 0.65), new Translation2d(7.0, 0.65)));
    }

    @Test
    public void testObstacleHandlingModes() {
        try {
            int expectedObstacleCount = FieldMap.Obstacles.ALL_OBSTACLES.size();
            for (FieldMap.ObstacleHandling mode : FieldMap.ObstacleHandling.values()) {
                FieldMap.setObstacleHandling(mode);
                assertEquals(mode, FieldMap.getObstacleHandling());
                assertEquals(expectedObstacleCount, FieldMap.Obstacles.getActiveObstacles().size(),
                        "Every compatibility mode must keep all five physical pieces per hub blocking");
                assertFalse(StaticPathfinder.isLineOfSightClear(
                        new Translation2d(4.0, 5.5), new Translation2d(5.2, 5.5)),
                        "Ramp exclusion must remain active in " + mode);
            }
        } finally {
            FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
        }
    }

    @Test
    public void testHubTunnelCornerAndFarSideRoutesStayConnected() {
        FieldMap.ObstacleHandling previous = FieldMap.getObstacleHandling();
        try {
            FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
            assertTrue(StaticPathfinder.isPointInStaticObstacle(new Translation2d(3.60, 5.75)),
                    "Robot center closer than bumper radius to a ramp must be treated as obstructed");
            Pose2d[] starts = {
                    new Pose2d(3.59, 5.75, new Rotation2d()),
                    new Pose2d(3.59, 6.90, new Rotation2d()),
                    new Pose2d(1.80, 6.00, new Rotation2d())
            };
            Pose2d[] targets = {
                    new Pose2d(6.20, 5.75, new Rotation2d()),
                    new Pose2d(6.20, 5.75, new Rotation2d()),
                    new Pose2d(14.74, 6.00, new Rotation2d())
            };

            for (int route = 0; route < starts.length; route++) {
                Pose2d safeStart = StaticPathfinder.ensurePoseOutsideObstacles(
                        starts[route], targets[route].getTranslation());
                assertFalse(StaticPathfinder.isPointInStaticObstacle(safeStart.getTranslation()),
                        "Route " + route + " start must be projected clear of the ramp footprint");
                java.util.List<Pose2d> path = StaticPathfinder.findPath(starts[route], targets[route]);
                assertFalse(path.isEmpty(), "Route " + route + " should remain connected");

                Translation2d previousPoint = safeStart.getTranslation();
                for (Pose2d waypoint : path) {
                    assertTrue(StaticPathfinder.isLineOfSightClear(previousPoint, waypoint.getTranslation()),
                            "Route " + route + " contains a segment through a blocked corner");
                    previousPoint = waypoint.getTranslation();
                }
            }
        } finally {
            FieldMap.setObstacleHandling(previous);
        }
    }

    @Test
    public void testTrenchTurnLookaheadDoesNotClipWall() {
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
        Pose2d start = new Pose2d(6.20, 5.75, new Rotation2d());
        Pose2d target = new Pose2d(1.80, 6.00, new Rotation2d());

        List<Pose2d> path = StaticPathfinder.findPath(start, target);
        assertFalse(path.isEmpty());

        // Simulate lookahead interpolation (0.5m lookahead radius) across all waypoint
        // transitions
        for (int i = 0; i < path.size() - 1; i++) {
            Translation2d p1 = path.get(i).getTranslation();
            Translation2d p2 = path.get(i + 1).getTranslation();
            for (double t = 0.0; t <= 1.0; t += 0.1) {
                Translation2d sample = p1.interpolate(p2, t);
                assertFalse(StaticPathfinder.isPointInStaticObstacle(sample),
                        "Lookahead path clipped corner at (" + sample.getX() + ", " + sample.getY() + ")");
            }
        }
    }

    @Test
    public void testFieldClamping() {
        // Points outside field
        Translation2d wayOff = new Translation2d(-5.0, 20.0);
        Translation2d clamped = FieldMap.clampToField(wayOff);

        assertEquals(FieldMap.WALL_SAFETY_MARGIN, clamped.getX(), 1e-3);
        assertEquals(FieldMap.FIELD_WIDTH - FieldMap.WALL_SAFETY_MARGIN, clamped.getY(), 1e-3);

        // Point inside field
        Translation2d inside = new Translation2d(5.0, 4.0);
        Translation2d clampedInside = FieldMap.clampToField(inside);
        assertEquals(5.0, clampedInside.getX(), 1e-3);
        assertEquals(4.0, clampedInside.getY(), 1e-3);
    }

    @Test
    public void testGlidePointsClearOfInflatedObstacles() {
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
        for (GlidePoints.GlidePoint gp : GlidePoints.BLUE_GLIDE_POINTS) {
            assertFalse(StaticPathfinder.isPointInStaticObstacle(gp.pose.getTranslation()),
                    "GlidePoint '" + gp.name + "' is inside an inflated obstacle!");
        }
    }

    /**
     * Pins the Blue-origin rule: every symmetric feature is defined once for Blue and
     * mirrored, so a geometry retune can never leave a stale parallel Red literal
     * behind. The {@link FieldMap.Depots} asymmetry is asserted separately below.
     */
    @Test
    public void testRedFeaturesAreDerivedMirrorsOfBlue() {
        double L = FieldMap.FIELD_LENGTH;
        double W = FieldMap.FIELD_WIDTH;

        // Trenches: X span mirrors and reverses order.
        assertEquals(L - FieldMap.Trenches.BLUE_TRENCH_MAX_X, FieldMap.Trenches.RED_TRENCH_MIN_X, 1e-9);
        assertEquals(L - FieldMap.Trenches.BLUE_TRENCH_MIN_X, FieldMap.Trenches.RED_TRENCH_MAX_X, 1e-9);

        // Hub, trench-wall centerline and ramps already derive; assert the invariant.
        assertEquals(L - FieldMap.Hubs.BLUE_HUB_X, FieldMap.Hubs.RED_HUB_X, 1e-9);
        assertEquals(L - FieldMap.TrenchWalls.BLUE_CENTER_X, FieldMap.TrenchWalls.RED_CENTER_X, 1e-9);

        // Climbing tower pole mirrors in both axes.
        assertEquals(L - FieldMap.ClimbingTowers.BLUE_TOWER_POLE.getX(),
                FieldMap.ClimbingTowers.RED_TOWER_POLE.getX(), 1e-9);
        assertEquals(W - FieldMap.ClimbingTowers.BLUE_TOWER_POLE.getY(),
                FieldMap.ClimbingTowers.RED_TOWER_POLE.getY(), 1e-9);

        // Tower posts mirror AND swap the north/south labels: the Red SOUTH post is the
        // mirror of the Blue NORTH post. Asserting the pairing explicitly is the whole
        // point -- a naive same-label mirror would silently invert the tower geometry.
        assertObstacleMirrors(FieldMap.Obstacles.RED_TOWER_POST_SOUTH,
                FieldMap.Obstacles.BLUE_TOWER_POST_NORTH);
        assertObstacleMirrors(FieldMap.Obstacles.RED_TOWER_POST_NORTH,
                FieldMap.Obstacles.BLUE_TOWER_POST_SOUTH);
    }

    /**
     * The two depots are the one documented exception to the mirror rule: the real field
     * places them asymmetrically. These assertions exist so nobody "simplifies" them into
     * a Blue + mirror pair without re-measuring. If they ever legitimately change, update
     * the {@link FieldMap.Depots} comment at the same time.
     */
    @Test
    public void testDepotsAreIntentionallyNotMirrored() {
        Translation2d blue = FieldMap.Depots.BLUE_DEPOT_LOAD_POINT;
        Translation2d red = FieldMap.Depots.RED_DEPOT_LOAD_POINT;

        double mirroredX = FieldMap.FIELD_LENGTH - blue.getX();
        double mirroredY = FieldMap.FIELD_WIDTH - blue.getY();

        // Sanity-check the documented asymmetry is still present and still significant.
        assertTrue(Math.hypot(mirroredX - red.getX(), mirroredY - red.getY()) > 0.5,
                "Red depot load point drifted toward a pure mirror of Blue; if the field "
                        + "geometry was re-measured, update the FieldMap.Depots comment too");
    }

    private static void assertObstacleMirrors(FieldMap.AABB red, FieldMap.AABB blue) {
        assertEquals(FieldMap.FIELD_LENGTH - blue.maxX, red.minX, 1e-9, red.name + " minX mirror");
        assertEquals(FieldMap.FIELD_LENGTH - blue.minX, red.maxX, 1e-9, red.name + " maxX mirror");
        assertEquals(FieldMap.FIELD_WIDTH - blue.maxY, red.minY, 1e-9, red.name + " minY mirror");
        assertEquals(FieldMap.FIELD_WIDTH - blue.minY, red.maxY, 1e-9, red.name + " maxY mirror");
    }
}
