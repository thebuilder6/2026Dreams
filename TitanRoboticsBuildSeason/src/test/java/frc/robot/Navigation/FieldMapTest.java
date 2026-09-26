package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;

public class FieldMapTest {

    @Test
    public void testFieldDimensionsAndCenterline() {
        assertEquals(16.535, FieldMap.FIELD_LENGTH, 1e-3);
        assertEquals(8.052, FieldMap.FIELD_WIDTH, 1e-3);
        assertEquals(8.27, FieldMap.CENTERLINE_X, 1e-2);
        assertTrue(FieldMap.CENTERLINE_X < FieldMap.FIELD_LENGTH / 2.0 + 0.1);
    }

    @Test
    public void testHubLocationsAndStandoff() {
        assertEquals(4.5974, FieldMap.Hubs.BLUE_HUB_2D.getX(), 1e-3);
        assertEquals(4.0345, FieldMap.Hubs.BLUE_HUB_2D.getY(), 1e-3);
        assertEquals(11.9380, FieldMap.Hubs.RED_HUB_2D.getX(), 1e-3);
        assertEquals(4.0345, FieldMap.Hubs.RED_HUB_2D.getY(), 1e-3);

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
        // Blue Alliance Zone (X <= 4.597)
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(2.0, 4.0), false));
        assertFalse(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(2.0, 4.0), true));

        // Red Alliance Zone (X >= 11.938)
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
    public void testAABBIntersectionAndContainment() {
        FieldMap.AABB blueObstacle = FieldMap.Obstacles.BLUE_HUB_AND_RAMPS;
        assertTrue(blueObstacle.contains(4.60, 4.035));
        assertFalse(blueObstacle.contains(2.0, 4.035));

        // Segment crossing the obstacle
        Translation2d p1 = new Translation2d(2.0, 4.035);
        Translation2d p2 = new Translation2d(7.0, 4.035);
        assertTrue(blueObstacle.intersectsSegment(p1, p2));

        // Segment through the Top Trench corridor (Y = 7.42)
        Translation2d trench1 = new Translation2d(2.0, 7.42);
        Translation2d trench2 = new Translation2d(7.0, 7.42);
        assertFalse(blueObstacle.intersectsSegment(trench1, trench2));
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
}
