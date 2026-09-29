package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Pins the trench corridor geometry so the saved roadmap
 * (<code>docs/nav/roadmap.html</code>) cannot drift from the code.
 *
 * <p><b>Known and accepted:</b> the low-clearance band is anchored to the
 * <i>raw</i> wall face ({@code TOP_TRENCH_MIN_Y = NORTH_CENTER_Y + WALL_Y_LEN/2 =
 * 6.7903}), while the planning footprint inflates the wall by
 * {@code ROBOT_RADIUS} on every face. The inflation therefore intrudes 0.45 m into
 * the band. That is structural, not a bug: it shrinks the drivable band rather than
 * making anything unreachable. Every published lane point, roadmap node, and
 * {@code planTunnelRoute} waypoint stays clear.
 *
 * <p><b>Worth watching:</b> the resulting drivable centre band is 0.3787 m wide,
 * which is <i>narrower</i> than the 0.45 m cross-track gate
 * {@code TrajectoryController} uses to advance waypoints. A robot shoved 0.18-0.45 m
 * off the lane centreline passes that gate while its centre is already inside the
 * inflated footprint. The planner self-corrects with an escape waypoint, so the cost
 * is detour and possible lane re-entry churn, not a stall. If trench replays ever show
 * that churn, narrowing the cross-track gate in trenches is the lever &mdash; not
 * moving the band.
 *
 * <p>Companion to the 2026-09-28 navigation probe; see
 * <code>tools/nav/verify_roadmap.py</code> for the geometry check that guards the
 * HTML, and <code>StuckRecoveryTest</code> for the APF / stall-gate fixes.
 */
class TrenchCorridorClearanceTest {

    private static final double M = FieldMap.ROBOT_RADIUS;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
    }

    @AfterEach
    void tearDown() {
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
    }

    private static double wallFaceY(boolean north) {
        FieldMap.AABB w = north ? FieldMap.TrenchWalls.BLUE_NORTH_WALL
                : FieldMap.TrenchWalls.BLUE_SOUTH_WALL;
        return north ? w.maxY : w.minY;
    }

    /** Clearance from a point to the nearest inflated obstacle face. */
    private static double clearanceToObstacles(double x, double y) {
        double best = Double.MAX_VALUE;
        for (FieldMap.AABB o : FieldMap.Obstacles.getActiveObstacles()) {
            double dx = Math.max(Math.max(o.minX - M - x, 0.0), x - (o.maxX + M));
            double dy = Math.max(Math.max(o.minY - M - y, 0.0), y - (o.maxY + M));
            best = Math.min(best, Math.hypot(dx, dy));
        }
        return best;
    }

    // ---------------------------------------------------------------------
    // The overlap is real, symmetric, and bounded by the bumper margin
    // ---------------------------------------------------------------------

    @Test
    void lowClearanceBandIsAnchoredToTheRawWallFace() {
        double topMinY = FieldMap.Trenches.TOP_TRENCH_MIN_Y;
        double rawFace = wallFaceY(true);

        assertEquals(rawFace, topMinY, 1e-9,
                "The band must start at the raw wall face, not the inflated one, or the "
                        + "clearance zone would not match the real truss");

        double botMaxY = FieldMap.Trenches.BOT_TRENCH_MAX_Y;
        assertEquals(wallFaceY(false), botMaxY, 1e-9,
                "Bottom band must be anchored to the raw wall face too");
    }

    @Test
    void inflationIntrudesExactlyOneBumperMarginIntoEachBand() {
        double topOverlap = (wallFaceY(true) + M) - FieldMap.Trenches.TOP_TRENCH_MIN_Y;
        double botOverlap = FieldMap.Trenches.BOT_TRENCH_MAX_Y - (wallFaceY(false) - M);

        assertEquals(M, topOverlap, 1e-9,
                "Top band overlap is by construction the bumper margin");
        assertEquals(M, botOverlap, 1e-9,
                "Bottom band overlap must be symmetric, else the Blue/Red mirror rule "
                        + "has drifted");
    }

    /**
     * Both trenches must offer the same drivable width. A retune that adjusted one
     * lane but not its mirror would show up here.
     */
    @Test
    void bothTrenchesOfferTheSameDrivableWidth() {
        double topUsable = (FieldMap.FIELD_WIDTH - M) - (wallFaceY(true) + M);
        double botUsable = (wallFaceY(false) - M) - M;

        assertEquals(topUsable, botUsable, 1e-9,
                "Top and bottom trench drivable widths must match exactly");
        assertTrue(topUsable > 0.0,
                "The trench must remain wide enough for the robot centre");
    }

    // ---------------------------------------------------------------------
    // The overlap must not make anything unreachable
    // ---------------------------------------------------------------------

    @Test
    void everyPublishedTrenchWaypointStaysClearOfTheInflatedFootprint() {
        for (int id = 5; id <= 12; id++) { // 4 Blue + 4 Red trench nodes
            Translation2d p = StaticPathfinder.getNodePosition(id);
            assertFalse(StaticPathfinder.isPointInStaticObstacle(p),
                    "Roadmap node #" + id + " " + StaticPathfinder.getNodeName(id)
                            + " is inside the inflated footprint");
        }
    }

    /**
     * The <code>planTunnelRoute</code> funnel waypoints sit closest to the wall, so
     * they are the tightest published geometry. Guards the margin the funnel depends on.
     */
    @Test
    void tunnelRouteFunnelWaypointsKeepClearance() {
        for (double lane : new double[] {FieldMap.Trenches.TOP_CORRIDOR_Y,
                FieldMap.Trenches.BOT_CORRIDOR_Y}) {
            for (double x : new double[] {2.90, 3.50, 5.75, 6.35}) {
                Translation2d p = new Translation2d(x, lane);
                assertFalse(StaticPathfinder.isPointInStaticObstacle(p),
                        String.format("Tunnel waypoint (%.2f, %.2f) is blocked", x, lane));
            }
        }
    }

    /** The whole lane must be line-of-sight traversable, or A* cannot use it. */
    @Test
    void trenchLaneIsLineOfSightClearEndToEnd() {
        for (double lane : new double[] {FieldMap.Trenches.TOP_CORRIDOR_Y,
                FieldMap.Trenches.BOT_CORRIDOR_Y}) {
            assertTrue(StaticPathfinder.isLineOfSightClear(
                            new Translation2d(2.90, lane), new Translation2d(6.35, lane)),
                    String.format("Lane Y=%.2f must be traversable end to end", lane));
        }
    }

    // ---------------------------------------------------------------------
    // The documented band-vs-gate tension
    // ---------------------------------------------------------------------

    /**
     * Records the measured relationship so a future retune cannot quietly widen the
     * gap. The cross-track gate lives in {@code TrajectoryController} as a literal,
     * so it is restated here with a pointer rather than imported.
     */
    @Test
    void drivableBandIsNarrowerThanTheCrossTrackGate() {
        double band = (FieldMap.FIELD_WIDTH - M) - (wallFaceY(true) + M);
        final double crossTrackGate = 0.45; // TrajectoryController waypoint progression

        assertTrue(band < crossTrackGate,
                "The documented tension assumes band < cross-track gate. Band is now "
                        + String.format("%.4f", band) + " m vs gate " + crossTrackGate
                        + " m. If this assertion fails the band was widened; update "
                        + "docs/nav/roadmap.html and the TrenchCorridorClearanceTest "
                        + "javadoc, which both describe this gap.");
    }

    /**
     * Within the gate but outside the legal band, the planner must still recover: the
     * measured pose is projected clear and an escape waypoint is prepended rather than
     * the robot being told to hold.
     */
    @Test
    void robotShovedIntoTheOverlapIsEvacuatedNotLeftStuck() {
        double lane = FieldMap.Trenches.TOP_CORRIDOR_Y;
        double blockedFace = wallFaceY(true) + M;

        // 0.10 m inside the inflated face: past the legal band, still inside the
        // cross-track gate.
        double y = blockedFace - 0.10;
        assertTrue(StaticPathfinder.isPointInStaticObstacle(new Translation2d(4.62, y)),
                "probe point should be inside the inflated footprint");

        Pose2d start = new Pose2d(4.62, y, new Rotation2d());
        Pose2d goal = new Pose2d(1.80, 6.00, new Rotation2d());
        var path = StaticPathfinder.findPath(start, goal);

        assertFalse(path.isEmpty(), "a shoved robot must still get a route");
        assertTrue(path.get(0).getTranslation().getDistance(start.getTranslation()) > 1e-6,
                "the path must lead with an escape waypoint out of the footprint");
        assertFalse(StaticPathfinder.isPointInStaticObstacle(path.get(0).getTranslation()),
                "the escape waypoint itself must be clear");
    }
}
