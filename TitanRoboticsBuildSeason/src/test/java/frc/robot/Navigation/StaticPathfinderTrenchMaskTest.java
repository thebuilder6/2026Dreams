package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Pins trench masking that is aware of the requesting robot's own registered
 * obstacle.
 *
 * <p><b>The defect.</b> Every sim robot registers its own pose as a dynamic
 * obstacle ({@code AIRobotInstance} step 5, and {@code AIRobotSim} for Bot 0), and
 * {@link DynamicRouter#isZoneBlocked} matches any unexpired obstacle against the
 * corridor bounds. A robot standing <i>inside</i> a trench therefore satisfies
 * the very zone test meant to detect a <i>peer</i> in the corridor, and
 * {@code getBlockedTrenchNodes()} masks all four nodes of the corridor the robot
 * is physically in. The planner then refuses the passage it is sitting in, which
 * is the trench jitter/dance: enter, lose the route, reverse out, re-enter.
 *
 * <p><b>What is pinned here.</b> Ego exclusion is scoped to a radius around the
 * requesting robot's own start pose, so a genuine peer in the same corridor still
 * masks it. A robot must not be able to un-block a corridor by entering it, but a
 * second robot in that corridor must still close it.
 *
 * <p>The exclusion radius is intentionally small. It is a de-bounce against
 * ego-position smear, not a "clear the corridor" radius: a peer 0.5 m away
 * inside the same trench must still mask.
 *
 * <p>Related: {@link TrenchCorridorClearanceTest} pins the lane geometry these
 * masks operate on, and {@code TunnelAndAssistanceTest} pins the public
 * {@code isTrenchBlocked} semantics this class must not change.
 */
class StaticPathfinderTrenchMaskTest {

    private static final double LANE_TOP = FieldMap.Trenches.TOP_CORRIDOR_Y;
    private static final double LANE_BOT = FieldMap.Trenches.BOT_CORRIDOR_Y;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DynamicRouter.clearObstacles();
    }

    @AfterEach
    void tearDown() {
        DynamicRouter.clearObstacles();
    }

    /** Registers a persistent obstacle at a pose, standing in for a robot. */
    private static void robotAt(double x, double y) {
        DynamicRouter.registerObstacle(
                new Translation2d(x, y), new Translation2d(), 0.55, 30.0, false);
    }

    // ---------------------------------------------------------------------
    // Ego exclusion
    // ---------------------------------------------------------------------

    /**
     * The core case: a robot inside the top trench must not report its own
     * corridor as peer-blocked, and the nodes it is standing in must survive
     * masking so the planner can route it out.
     */
    @Test
    void ownPositionDoesNotMaskItsOwnCorridor() {
        Translation2d ego = new Translation2d(4.50, LANE_TOP);
        robotAt(ego.getX(), ego.getY());

        // The raw, robot-agnostic query still reports the corridor occupied --
        // this is a true statement about the arena and is what peer logic and
        // the published API must keep meaning.
        assertTrue(StaticPathfinder.isTrenchBlocked(true, true),
                "an obstacle really is inside the top corridor, so the public "
                        + "robot-agnostic query must report it blocked");

        // The ego-aware query must not.
        assertFalse(StaticPathfinder.isTrenchBlocked(true, true, ego),
                "a robot must not treat its own registered obstacle as a peer "
                        + "blocking the corridor it occupies");

        // And the node mask it drives must keep the corridor's own nodes.
        Set<Integer> masked = StaticPathfinder.getBlockedTrenchNodes(ego);
        for (int id : StaticPathfinder.BLUE_TOP_TRENCH_NODES) {
            assertFalse(masked.contains(id),
                    "node " + id + " ("
                            + StaticPathfinder.getNodeName(id)
                            + ") must not be masked by the occupant itself");
        }
    }

    /**
     * Ego exclusion is a radius, not a blanket amnesty. A peer inside the same
     * corridor but far enough away must still mask it, otherwise two robots in
     * one trench would both think the passage is open.
     */
    @Test
    void peerInsideTheSameCorridorStillMasks() {
        Translation2d ego = new Translation2d(3.60, LANE_TOP);
        robotAt(ego.getX(), ego.getY());
        // A peer 1.0 m away in the same lane, well outside the exclusion radius.
        robotAt(4.60, LANE_TOP);

        assertTrue(StaticPathfinder.isTrenchBlocked(true, true, ego),
                "a peer 1.0 m away in the same corridor must still mask it");

        Set<Integer> masked = StaticPathfinder.getBlockedTrenchNodes(ego);
        for (int id : StaticPathfinder.BLUE_TOP_TRENCH_NODES) {
            assertTrue(masked.contains(id),
                    "node " + id + " must be masked while a peer occupies the corridor");
        }
    }

    /**
     * The ego is in the top trench and a peer is in the bottom one. Excluding the
     * ego must not accidentally clear the corridor the peer is actually in.
     */
    @Test
    void peerInTheOtherCorridorIsUnaffectedByEgoExclusion() {
        Translation2d ego = new Translation2d(4.50, LANE_TOP);
        robotAt(ego.getX(), ego.getY());
        robotAt(4.50, LANE_BOT);

        assertFalse(StaticPathfinder.isTrenchBlocked(true, true, ego),
                "the ego's own corridor must be clear");
        assertTrue(StaticPathfinder.isTrenchBlocked(false, true, ego),
                "the peer's corridor must remain blocked");
    }

    // ---------------------------------------------------------------------
    // Behavioural consequence: routing through the corridor the robot is in
    // ---------------------------------------------------------------------

    /**
     * The user-visible consequence. A robot sitting in the top trench that wants
     * to leave eastward must get a route. Before ego exclusion the corridor it
     * occupies was masked, so the planner had to either route it the long way
     * round or (when both corridors were contested) return nothing at all.
     */
    @Test
    void robotInATrenchCanStillPlanAnExitRoute() {
        Translation2d ego = new Translation2d(4.50, LANE_TOP);
        robotAt(ego.getX(), ego.getY());

        Pose2d start = new Pose2d(ego, Rotation2d.fromDegrees(0));
        Pose2d goal = new Pose2d(8.27, 6.20, new Rotation2d()); // Midfield Top

        List<Pose2d> path = StaticPathfinder.findPath(start, goal);

        assertFalse(path.isEmpty(),
                "a robot occupying a trench must still receive a route out of it");
        assertEquals(goal.getTranslation(), path.get(path.size() - 1).getTranslation(),
                "the route must terminate at the goal");
    }

    /**
     * Characterization for the "both corridors contested" case, which is the one
     * the original assessment claimed severed the field into two components.
     *
     * <p>Two peers (not the ego) occupy both Blue corridors while the ego sits in
     * open field. The centreline/midfield crossing does not run through a trench,
     * so a route is expected to exist. This test records that; it is the
     * evidence a future penalty-based change (A2) would have to preserve.
     */
    @Test
    void twoPeerOccupiedCorridorsDoNotSeverTheField() {
        Translation2d ego = new Translation2d(2.40, 4.035); // Blue Alliance Center
        robotAt(4.50, LANE_TOP);
        robotAt(4.50, LANE_BOT);

        assertTrue(StaticPathfinder.isTrenchBlocked(true, true, ego));
        assertTrue(StaticPathfinder.isTrenchBlocked(false, true, ego));

        List<Pose2d> path = StaticPathfinder.findPath(
                new Pose2d(ego, new Rotation2d()),
                new Pose2d(8.27, 6.20, new Rotation2d()));

        assertFalse(path.isEmpty(),
                "two occupied trenches must not disconnect the roadmap: the "
                        + "midfield crossing bypasses both corridors");
    }

    // ---------------------------------------------------------------------
    // Backward compatibility of the published API
    // ---------------------------------------------------------------------

    /**
     * The two-argument overload is public API used by
     * {@code TunnelAndAssistanceTest} and by {@code planTunnelRoute}. It must
     * keep its robot-agnostic meaning: null exclusion == no exclusion.
     */
    @Test
    void publicOverloadMatchesNullExclusion() {
        Translation2d ego = new Translation2d(4.50, LANE_TOP);
        robotAt(ego.getX(), ego.getY());

        assertEquals(
                StaticPathfinder.isTrenchBlocked(true, true),
                StaticPathfinder.isTrenchBlocked(true, true, null),
                "a null exclusion must behave exactly like the two-argument form");
    }

    /**
     * With nothing registered there is nothing to mask, and the ego-aware query
     * must agree with the plain one.
     */
    @Test
    void clearArenaMasksNothing() {
        Translation2d anywhere = new Translation2d(4.50, LANE_TOP);
        assertFalse(StaticPathfinder.isTrenchBlocked(true, true, anywhere));
        assertFalse(StaticPathfinder.isTrenchBlocked(false, true, anywhere));
        assertTrue(StaticPathfinder.getBlockedTrenchNodes(anywhere).isEmpty(),
                "an empty arena must produce an empty mask");
    }
}
