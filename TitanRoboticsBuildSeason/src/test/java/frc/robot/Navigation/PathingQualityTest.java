package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Cover for the 2026-09-28 pathing-quality changes:
 *
 * <ol>
 *   <li><b>Trench-conditional cross-track gate.</b> The open-field 0.45 m gate was
 *       wider than the 0.3787 m trench drivable band, so a 0.18-0.45 m lateral shove
 *       advanced a waypoint while the robot's centre was already inside the inflated
 *       footprint. The gate now tightens to
 *       {@link TrajectoryController#TRENCH_CROSS_TRACK_GATE_M} in low-clearance zones.</li>
 *   <li><b>No-route retry.</b> An empty plan used to stamp {@code lastPlanTimestamp},
 *       so the next replan waited for the target to move 0.85 m -- an indefinite hold
 *       for a static target.</li>
 *   <li><b>Clearance-weighted A*.</b> Edges are costed by length scaled up as their
 *       closest approach to an inflated obstacle shrinks, so comparable routes prefer
 *       the one the tracker can hold.</li>
 * </ol>
 */
class PathingQualityTest {

    private static final double DT = 0.02;
    private static final double TOP_LANE = FieldMap.Trenches.TOP_CORRIDOR_Y;
    private static final double BOT_LANE = FieldMap.Trenches.BOT_CORRIDOR_Y;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
        DynamicRouter.clearObstacles();
    }

    @AfterEach
    void tearDown() {
        DynamicRouter.clearObstacles();
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
    }

    // =====================================================================
    // 1. Trench-conditional cross-track gate
    // =====================================================================

    /**
     * The gate must be strictly narrower than the drivable trench band, otherwise the
     * original bug returns. This is the invariant, not the number.
     */
    @Test
    void trenchGateIsNarrowerThanTheDrivableBand() {
        FieldMap.AABB wall = FieldMap.TrenchWalls.BLUE_NORTH_WALL;
        double band = (FieldMap.FIELD_WIDTH - FieldMap.ROBOT_RADIUS)
                - (wall.maxY + FieldMap.ROBOT_RADIUS);

        assertTrue(TrajectoryController.TRENCH_CROSS_TRACK_GATE_M < band,
                "The trench gate (" + TrajectoryController.TRENCH_CROSS_TRACK_GATE_M
                        + " m) must be narrower than the drivable band (" + band
                        + " m), or a shoved robot advances the path while illegal");
        assertTrue(TrajectoryController.CROSS_TRACK_GATE_M
                        > TrajectoryController.TRENCH_CROSS_TRACK_GATE_M,
                "Open field must keep the looser gate");
    }

    /**
     * A robot shoved off the trench lane centreline must NOT advance the waypoint
     * index. Before the fix it did, at exactly the offsets that made the bug.
     */
    @Test
    void shovedRobotInTrenchDoesNotAdvanceTheWaypoint() {
        double laneClearance = TOP_LANE - (FieldMap.TrenchWalls.BLUE_NORTH_WALL.maxY
                + FieldMap.ROBOT_RADIUS);

        // A shove that is legal-ish under the old 0.45 gate but illegal under the new
        // one: past the lane clearance, still inside the old gate.
        double shovedY = TOP_LANE - (laneClearance + 0.05);
        assertTrue(shovedY < TOP_LANE, "probe pose must be off the centreline");
        assertTrue(TOP_LANE - shovedY < TrajectoryController.CROSS_TRACK_GATE_M,
                "probe offset must have passed the old open-field gate");

        TrajectoryController tc = new TrajectoryController(new PIDController(1.0, 0.0, 0.0));
        // Multi-waypoint path along the lane so there is a plane to cross.
        tc.setExplicitWaypoints(List.of(
                new Pose2d(6.20, TOP_LANE, new Rotation2d()),
                new Pose2d(2.60, TOP_LANE, new Rotation2d()),
                new Pose2d(1.80, 6.00, new Rotation2d())));

        // Just past the first waypoint plane, but shoved sideways off the lane.
        Pose2d pose = new Pose2d(6.05, shovedY, new Rotation2d());
        tc.calculate(pose, new ChassisSpeeds(), new Pose2d(1.80, 6.00, new Rotation2d()),
                3.0, false, false);

        assertEquals(0, tc.getCurrentWaypointIndex(),
                "A robot shoved off the lane centreline must not count the waypoint plane "
                        + "as crossed, or it follows a path that clips the truss");
    }

    /** The guard must not break normal trench transit: driving the lane works. */
    @Test
    void centredTrenchTransitStillAdvances() {
        TrajectoryController tc = new TrajectoryController(new PIDController(1.0, 0.0, 0.0));
        tc.setExplicitWaypoints(List.of(
                new Pose2d(6.20, TOP_LANE, new Rotation2d()),
                new Pose2d(2.60, TOP_LANE, new Rotation2d()),
                new Pose2d(1.80, 6.00, new Rotation2d())));

        // Dead on the lane and just past the first waypoint.
        Pose2d pose = new Pose2d(6.05, TOP_LANE, new Rotation2d());
        tc.calculate(pose, new ChassisSpeeds(), new Pose2d(1.80, 6.00, new Rotation2d()),
                3.0, false, false);

        assertTrue(tc.getCurrentWaypointIndex() >= 1,
                "A centred robot must still advance past a crossed waypoint plane");
    }

    /** Open field must keep the looser 0.45 m gate, so nothing regressed there. */
    @Test
    void openFieldKeepsTheLooseGate() {
        double centreY = 4.035; // midfield, no trench, no obstacle
        assertFalse(FieldMap.Trenches.isLowClearance(new Translation2d(8.27, centreY)));

        TrajectoryController tc = new TrajectoryController(new PIDController(1.0, 0.0, 0.0));
        tc.setExplicitWaypoints(List.of(
                new Pose2d(9.00, centreY, new Rotation2d()),
                new Pose2d(7.00, centreY, new Rotation2d()),
                new Pose2d(6.00, centreY, new Rotation2d())));

        // 0.30 m off the line: legal under the open-field gate, would be rejected in a
        // trench. This must still count as progress.
        Pose2d pose = new Pose2d(8.90, centreY + 0.30, new Rotation2d());
        tc.calculate(pose, new ChassisSpeeds(), new Pose2d(6.00, centreY, new Rotation2d()),
                3.0, false, false);

        assertTrue(tc.getCurrentWaypointIndex() >= 1,
                "Open field must keep advancing at 0.30 m cross-track");
    }

    // =====================================================================
    // 2. No-route retry
    // =====================================================================

    @Test
    void replanRetryIsShorterThanTheTargetMovementThreshold() {
        assertTrue(TrajectoryController.REPLAN_RETRY_SEC < 0.30,
                "The retry cadence must be well under the 0.30 m streaming-replan "
                        + "threshold, otherwise it never actually retries sooner");
    }

    /**
     * A plan that comes back empty must still command zero speed, never a straight
     * line through an obstacle. The 444,550-case probe found no reachable-but-failing
     * routes in practice, so this drives the empty-waypoint branch directly instead of
     * hunting for a field position that triggers it.
     */
    @Test
    void emptyPlanCommandsZeroSpeedRatherThanDrivingBlind() {
        TrajectoryController tc = new TrajectoryController(new PIDController(1.0, 0.0, 0.0));
        tc.setExplicitWaypoints(List.of()); // no route available

        ChassisSpeeds out = tc.calculate(
                new Pose2d(8.27, 4.035, new Rotation2d()), new ChassisSpeeds(),
                new Pose2d(2.40, 4.035, new Rotation2d()), 3.0, false, false);

        assertEquals(0.0, Math.hypot(out.vxMetersPerSecond, out.vyMetersPerSecond), 1e-6,
                "An empty plan must command zero speed rather than drive through an obstacle");
    }

    // =====================================================================
    // 3. Clearance-weighted A*
    // =====================================================================

    @Test
    void openFieldEdgesKeepTheirExactEuclideanCost() {
        double clear = StaticPathfinder.segmentMinClearance(
                new Translation2d(8.27, 4.90), new Translation2d(8.27, 3.17));
        assertTrue(clear > StaticPathfinder.CLEARANCE_TIGHT_M,
                "Midfield-to-midfield should be far from any obstacle so the edge keeps "
                        + "its plain Euclidean weight, got " + clear);
    }

    /** A segment that grazes a ramp must report a tight clearance. */
    @Test
    void huggingAnObstacleReportsTightClearance() {
        // Blue north ramp occupies X 4.0287..5.2225, Y 4.6315..6.4857 before inflation.
        // Its inflated top face is at 6.9357.
        double alongRamp = StaticPathfinder.segmentMinClearance(
                new Translation2d(4.63, 7.00), new Translation2d(4.63, 7.30));
        assertTrue(alongRamp < StaticPathfinder.CLEARANCE_TIGHT_M,
                "A segment running just past the inflated ramp face should read as tight, got "
                        + alongRamp);
    }

    /**
     * Sampled across the field so it covers the ramp corners, hub approaches, and
     * both trench lanes. The bound is deliberately the trench lane clearance rather
     * than an open-field figure: a trench route runs down a corridor whose drivable
     * centre band is only ~0.3787 m wide, so ~0.18 m is the tightest a <i>correct</i>
     * trench route can be. The property worth protecting is that no route is tighter
     * than the geometry allows, i.e. nothing has been squeezed below legal.
     */
    @Test
    void plannerRoutesKeepHealthyClearance() {
        double worst = Double.MAX_VALUE;
        String worstWhere = "none";
        int checked = 0;

        for (double x = 1.0; x <= 15.5; x += 0.6) {
            for (double y = 0.7; y <= 7.35; y += 0.6) {
                Translation2d start = new Translation2d(x, y);
                if (StaticPathfinder.isPointInStaticObstacle(start)) continue;
                for (double gx : new double[] {2.40, 8.27, 14.14}) {
                    for (double gy : new double[] {0.65, 4.035, 7.42}) {
                        List<Pose2d> path = StaticPathfinder.findPath(
                                new Pose2d(x, y, new Rotation2d()),
                                new Pose2d(gx, gy, new Rotation2d()));
                        assertFalse(path.isEmpty(),
                                "findPath must stay connected at (" + x + ", " + y + ")");
                        checked++;
                        for (int i = 0; i < path.size() - 1; i++) {
                            double c = StaticPathfinder.segmentMinClearance(
                                    path.get(i).getTranslation(),
                                    path.get(i + 1).getTranslation());
                            if (c < worst) {
                                worst = c;
                                worstWhere = String.format("(%.2f,%.2f)->(%.2f,%.2f)",
                                        path.get(i).getX(), path.get(i).getY(),
                                        path.get(i + 1).getX(), path.get(i + 1).getY());
                            }
                        }
                    }
                }
            }
        }
        assertTrue(checked > 400, "probe should cover a meaningful sample, got " + checked);

        // The tightest lane clearance is the floor for any legal route. Top is
        // 0.1797 m and bottom 0.1787 m, so take the smaller of the two.
        double topFloor = FieldMap.Trenches.TOP_CORRIDOR_Y
                - (FieldMap.TrenchWalls.BLUE_NORTH_WALL.maxY + FieldMap.ROBOT_RADIUS);
        double botFloor = (FieldMap.TrenchWalls.BLUE_SOUTH_WALL.minY - FieldMap.ROBOT_RADIUS)
                - FieldMap.Trenches.BOT_CORRIDOR_Y;
        double laneFloor = Math.min(topFloor, botFloor);
        assertTrue(worst > laneFloor - 1e-6,
                "No planned route may be tighter than the tightest trench lane ("
                        + String.format("%.4f", laneFloor) + " m). Worst was "
                        + String.format("%.4f", worst) + " m at " + worstWhere
                        + " over " + checked + " routes");
    }

    /**
     * The weighting must not break the trench lanes: those are narrow by design, so
     * every lane must remain traversable end to end and every published node clear.
     */
    @Test
    void trenchLanesSurviveClearanceWeighting() {
        for (double lane : new double[] {FieldMap.Trenches.TOP_CORRIDOR_Y,
                FieldMap.Trenches.BOT_CORRIDOR_Y}) {
            assertTrue(StaticPathfinder.isLineOfSightClear(
                            new Translation2d(2.90, lane), new Translation2d(6.35, lane)),
                    String.format("Lane Y=%.2f must stay traversable", lane));
        }
        for (int id = 5; id <= 12; id++) {
            assertFalse(StaticPathfinder.isPointInStaticObstacle(
                            StaticPathfinder.getNodePosition(id)),
                    "Trench node #" + id + " must stay clear under weighted planning");
        }
    }
}
