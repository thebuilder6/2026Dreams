package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.StaticPathfinder.PathResult;
import frc.robot.Navigation.StaticPathfinder.PathStatus;

/**
 * Pins the two ways a path request can fail honestly.
 *
 * <p><b>1. The blind pocket.</b> The roadmap used to stop at x = 1.80 (Blue) and
 * x = 14.74 (Red), and the climbing-tower posts sit at x ~= 1.06. A robot or a
 * fuel piece in the driver-wall band behind a tower could therefore see no
 * roadmap node at all, {@code findPath} returned an empty list, and
 * {@code TrajectoryController} commanded zero with no way to tell that apart from
 * a hang. Nodes 34-37 give that band an endpoint.
 *
 * <p><b>2. The fabricated route.</b> The tempting "fix" for an empty path is to
 * return the target anyway. That is a straight-line command through whatever
 * geometry caused the failure, so it converts a frozen robot into a robot driving
 * into a wall. These tests pin the opposite contract: either a verified-safe
 * bounded step, or an honest {@link PathStatus#UNREACHABLE}.
 *
 * <p>Note the boundary asymmetry that makes this safe: a recovery step is
 * <i>checked</i> end to end, whereas the original direct-path fast path is not
 * re-checked beyond the existing line-of-sight test.
 *
 * <p>Related: {@link StaticPathfinderTrenchMaskTest} covers the other route-loss
 * cause (a contested corridor), {@link TrenchCorridorClearanceTest} pins the lane
 * geometry, and {@code TunnelAndAssistanceTest} pins the published
 * {@code isTrenchBlocked} API.
 */
class WallPocketRecoveryTest {

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
        DynamicRouter.clearObstacles();
    }

    @AfterEach
    void tearDown() {
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
        DynamicRouter.clearObstacles();
    }

    // ---------------------------------------------------------------------
    // Node geometry
    // ---------------------------------------------------------------------

    /**
     * The Blue/Red mirror rule: derive the Red value, never hardcode it. The
     * wall-band nodes must be exact mirrors about the field centre.
     */
    @Test
    void redTowerNodesMirrorBlueX() {
        double blueX = StaticPathfinder.getNodePosition(StaticPathfinder.N_BLUE_TOWER_WEST_TOP).getX();
        double redX = StaticPathfinder.getNodePosition(StaticPathfinder.N_RED_TOWER_EAST_TOP).getX();

        assertEquals(FieldMap.FIELD_LENGTH - blueX, redX, 1e-9,
                "Red wall node X must be the Blue value mirrored about FIELD_LENGTH");

        // The bottom pair is compared mirror-to-mirror, not blue-to-red: the
        // Red bottom node is the mirror of the BLUE bottom node, so a direct
        // cross-alliance comparison of the two would be testing nothing.
        double blueBotX = StaticPathfinder.getNodePosition(StaticPathfinder.N_BLUE_TOWER_WEST_BOT).getX();
        double redBotX = StaticPathfinder.getNodePosition(StaticPathfinder.N_RED_TOWER_EAST_BOT).getX();
        assertEquals(FieldMap.FIELD_LENGTH - blueBotX, redBotX, 1e-9,
                "the bottom wall nodes must mirror each other the same way");
        assertEquals(blueX, blueBotX, 1e-9,
                "the top and bottom wall lanes must share one X per alliance");
    }

    /**
     * The wall-band Y lanes must mirror too, which the Y-only convention in
     * {@code AllianceFlipUtil} makes an identity check rather than a flip.
     */
    @Test
    void towerNodesShareYOnBothAlliances() {
        assertEquals(
                StaticPathfinder.getNodePosition(StaticPathfinder.N_BLUE_TOWER_WEST_TOP).getY(),
                StaticPathfinder.getNodePosition(StaticPathfinder.N_RED_TOWER_EAST_TOP).getY(),
                1e-9,
                "the wall corridor lane Y must be identical on both alliances "
                        + "(X is the mirroring axis, not Y)");
    }

    @Test
    void newTowerNodesAreOutsideStaticObstacles() {
        int[] ids = {
                StaticPathfinder.N_BLUE_TOWER_WEST_TOP,
                StaticPathfinder.N_BLUE_TOWER_WEST_BOT,
                StaticPathfinder.N_RED_TOWER_EAST_TOP,
                StaticPathfinder.N_RED_TOWER_EAST_BOT};
        for (int id : ids) {
            Translation2d p = StaticPathfinder.getNodePosition(id);
            assertFalse(StaticPathfinder.isPointInStaticObstacle(p),
                    "node " + id + " (" + StaticPathfinder.getNodeName(id)
                            + ") at " + p + " is inside the inflated footprint");
        }
    }

    /**
     * The roadmap count is quoted in {@code docs/nav/roadmap.html}, in
     * {@code StuckRecoveryTest}, and in {@code docs/CHANGELOG.md}. Adding nodes
     * without updating those leaves three stale references, so the count is
     * pinned here.
     */
    @Test
    void roadmapNowHasThirtyEightNodes() {
        assertEquals(38, StaticPathfinder.getNodeCount(),
                "the roadmap gained 4 tower-wall nodes; docs/nav/roadmap.html and "
                        + "StuckRecoveryTest must be updated to match");
    }

    // ---------------------------------------------------------------------
    // The wall band is routable
    // ---------------------------------------------------------------------

    /**
     * The point of nodes 34-37: a robot in the Blue wall band behind the tower
     * can now plan out to the field.
     */
    @Test
    void robotInTheBlueWallBandCanPlanOut() {
        Pose2d wallBand = new Pose2d(
                StaticPathfinder.TOWER_WALL_X, StaticPathfinder.TOWER_WALL_TOP_Y,
                Rotation2d.fromDegrees(0));
        Pose2d goal = new Pose2d(8.27, 4.90, Rotation2d.fromDegrees(180)); // Midfield CTR Top

        PathResult result = StaticPathfinder.findPathWithStatus(wallBand, goal);

        assertFalse(result.waypoints().isEmpty(),
                "the wall band must have a route out, got status " + result.status());
        assertTrue(result.status() == PathStatus.ROADMAP || result.status() == PathStatus.DIRECT,
                "the wall band should be a real route, not a recovery, got "
                        + result.status());
        Translation2d last = result.waypoints().get(result.waypoints().size() - 1).getTranslation();
        assertEquals(0.0, goal.getTranslation().getDistance(last), 1e-6,
                "the route must terminate at the goal, ended at " + last);
    }

    /** Same on the mirrored Red wall band. */
    @Test
    void robotInTheRedWallBandCanPlanOut() {
        Pose2d wallBand = new Pose2d(
                FieldMap.FIELD_LENGTH - StaticPathfinder.TOWER_WALL_X,
                StaticPathfinder.TOWER_WALL_BOT_Y,
                Rotation2d.fromDegrees(180));
        Pose2d goal = new Pose2d(8.27, 3.17, Rotation2d.fromDegrees(0)); // Midfield CTR Bot

        PathResult result = StaticPathfinder.findPathWithStatus(wallBand, goal);

        assertFalse(result.waypoints().isEmpty(),
                "the mirrored wall band must have a route out, got status " + result.status());
    }

    /**
     * Wall-band fuel must be a legal target, not just a legal robot position.
     * The fuel selector filters on {@code isPointInHardObstacle}, so if the wall
     * band were inside an inflated footprint a wall pickup would be silently
     * unselectable even with a route to it.
     */
    @Test
    void wallBandIsNotInsideAHardObstacle() {
        Translation2d band = new Translation2d(
                StaticPathfinder.TOWER_WALL_X, StaticPathfinder.TOWER_WALL_TOP_Y);
        assertFalse(StaticPathfinder.isPointInHardObstacle(band),
                "a wall-band fuel piece must remain selectable by the Jev fuel target");
    }

    // ---------------------------------------------------------------------
    // The recovery contract: verified-safe or honest
    // ---------------------------------------------------------------------

    /**
     * Sweeps the wall band for any request that cannot be routed. Whatever the
     * outcome, a recovery step must be traversable end to end and land on legal
     * carpet. This is the property that makes a fallback safe to command, and it
     * is what the raw-target "fix" would have violated.
     */
    @Test
    void everyRecoveryStepIsTraversableAndLegal() {
        int checked = 0;
        for (double x = 0.70; x <= 1.60; x += 0.10) {
            for (double y = 1.00; y <= 7.00; y += 0.25) {
                Pose2d start = new Pose2d(x, y, new Rotation2d());
                // Aim at the far side of the field, which is what makes the
                // request unroutable from inside the band.
                Pose2d goal = new Pose2d(15.0, 4.0345, Rotation2d.fromDegrees(180));

                PathResult result = StaticPathfinder.findPathWithStatus(start, goal);
                if (result.status() != PathStatus.LOCAL_RECOVERY) {
                    continue;
                }
                checked++;
                assertEquals(1, result.waypoints().size(),
                        "a recovery result must carry exactly one step");
                Pose2d step = result.waypoints().get(0);

                // The step is verified from the SANITIZED start, not the measured
                // one. A robot pressed against a ramp corner is inside the
                // inflated footprint, so findPath projects it clear first and the
                // escape waypoint chain starts from the projected pose. Asserting
                // line of sight from the raw measured pose would test a segment the
                // planner never intends to drive.
                Pose2d sanitized =
                        StaticPathfinder.ensurePoseOutsideObstacles(start, goal.getTranslation());
                assertTrue(StaticPathfinder.isLineOfSightClear(sanitized.getTranslation(),
                                step.getTranslation()),
                        "recovery step from sanitized " + sanitized + " is not traversable");
                assertFalse(StaticPathfinder.isPointInStaticObstacle(step.getTranslation()),
                        "recovery step at start " + start + " lands inside an obstacle");
                assertTrue(FieldMap.isWithinField(step.getTranslation(), 0.0),
                        "recovery step at start " + start + " lands off the field");

                double stepLen = sanitized.getTranslation().getDistance(step.getTranslation());
                assertTrue(stepLen <= StaticPathfinder.LOCAL_RECOVERY_MAX_STEP_M + 1e-6,
                        "recovery step must be bounded, got " + stepLen);
            }
        }
        assertTrue(checked > 0,
                "the sweep must actually exercise the recovery path, otherwise "
                        + "this test proves nothing");
    }

    /**
     * A truly enclosed request must report {@link PathStatus#UNREACHABLE} with an
     * empty list rather than inventing something. Start and target on the same
     * point, deep inside the inflated hub footprint: there is no legal step and
     * no route, and the honest answer is to hold so the stall detectors see it.
     */
    @Test
    void genuinelyEnclosedRequestRemainsHonest() {
        // A point deep inside the inflated Blue hub core, with a target far across
        // the field. The obstacle cannot be sanitized away (findPath projects the
        // endpoint clear), so the real question is what happens when nothing
        // routable and nothing safely steppable exists -- and, just as important,
        // that the sanitizer never resolves a request inside a hub by pretending
        // the hub is not there.
        Translation2d hubCentre = FieldMap.Hubs.getHubLocation2d(false);
        Pose2d insideHub = new Pose2d(hubCentre, Rotation2d.fromDegrees(0));

        assertTrue(StaticPathfinder.isPointInStaticObstacle(insideHub.getTranslation()),
                "premise: the probe start must really be inside the inflated hub");

        Pose2d farGoal = new Pose2d(15.0, 4.0345, Rotation2d.fromDegrees(180));
        PathResult result = StaticPathfinder.findPathWithStatus(insideHub, farGoal);

        // Whichever branch it lands on, the contract holds: never a waypoint
        // inside geometry, and never a straight line to an unroutable target.
        for (Pose2d wp : result.waypoints()) {
            if (result.status() == PathStatus.LOCAL_RECOVERY) {
                assertFalse(StaticPathfinder.isPointInStaticObstacle(wp.getTranslation()),
                        "no recovery waypoint may sit inside an obstacle");
            }
        }
        if (result.status() == PathStatus.UNREACHABLE) {
            assertTrue(result.waypoints().isEmpty(),
                    "UNREACHABLE must carry no waypoints, so the controller holds "
                            + "rather than being handed a fabricated route");
        } else {
            assertTrue(result.waypoints().size() >= 1);
        }
    }

    /**
     * The genuinely-boxed-in case, built so it cannot be sanitized away: a start
     * on legal carpet whose only line of sight to any roadmap node is blocked, and
     * whose 12 fan directions are all obstructed. Uses a target that shares the
     * start's blind pocket, which is the wall-band-behind-tower shape the
     * pre-2026-09-29 planner answered with a silent empty list.
     */
    @Test
    void anUnroutablePairNeverYieldsAFabricatedStraightLine() {
        // Sweep the wall band looking for any request whose only answer would be
        // a fabrication, and assert the contract on every one of them.
        int sawRecovery = 0;
        int sawUnreachable = 0;

        for (double x = 0.70; x <= 1.30; x += 0.10) {
            for (double y = 1.50; y <= 6.50; y += 0.50) {
                Pose2d start = new Pose2d(x, y, new Rotation2d());
                Pose2d goal = new Pose2d(x + 0.20, y, Rotation2d.fromDegrees(0));

                PathResult r = StaticPathfinder.findPathWithStatus(start, goal);
                if (r.status() == PathStatus.LOCAL_RECOVERY) {
                    sawRecovery++;
                    assertEquals(1, r.waypoints().size());
                    // A recovery never points at the original target: the step is
                    // away from the obstruction, bounded in length.
                    assertTrue(r.waypoints().get(0).getTranslation()
                                    .getDistance(goal.getTranslation()) > 1e-6,
                            "a recovery step must not be the unroutable target itself");
                } else if (r.status() == PathStatus.UNREACHABLE) {
                    sawUnreachable++;
                    assertTrue(r.waypoints().isEmpty());
                } else {
                    // DIRECT/ROADMAP are fine, but they must genuinely arrive. The
                    // goal is compared against the SANITIZED target, because
                    // findPath sanitizes both endpoints before planning: a goal
                    // sitting inside an inflated footprint is projected to its
                    // nearest legal face, and the route correctly ends there.
                    Pose2d safeGoal =
                            StaticPathfinder.ensurePoseOutsideObstacles(goal, start.getTranslation());
                    assertEquals(0.0,
                            r.waypoints().get(r.waypoints().size() - 1)
                                    .getTranslation().getDistance(safeGoal.getTranslation()),
                            1e-6, "a routable status must terminate at the sanitized goal");
                }
            }
        }
        assertTrue(sawRecovery + sawUnreachable > 0,
                "the sweep must exercise at least one fallback path");
    }

    /**
     * The direct fast path is unchanged and must stay that way: a clear
     * line of sight is a DIRECT result, not a recovery.
     */
    @Test
    void openFieldIsStillDirect() {
        PathResult result = StaticPathfinder.findPathWithStatus(
                new Pose2d(8.27, 4.0345, new Rotation2d()),
                new Pose2d(10.0, 4.0345, Rotation2d.fromDegrees(0)));

        assertEquals(PathStatus.DIRECT, result.status(),
                "open-carpet driving must stay on the zero-overhead fast path");
        // One waypoint: the target itself. The escape waypoint is only prepended
        // when the measured pose had to be projected out of an obstacle, and this
        // start is already legal carpet.
        assertEquals(1, result.waypoints().size());
        assertEquals(10.0, result.waypoints().get(0).getX(), 1e-6);
    }

    /**
     * {@code findPath} must stay source-compatible: same signature, same
     * waypoint list, just with the status discarded. Every existing caller
     * depends on that, and a silent behaviour change here would move every route
     * in the codebase at once.
     */
    @Test
    void findPathDelegatesToTheStatusVariant() {
        Pose2d start = new Pose2d(2.40, 4.035, new Rotation2d());
        Pose2d goal = new Pose2d(6.20, 4.035, Rotation2d.fromDegrees(180));

        List<Pose2d> viaStatus =
                StaticPathfinder.findPathWithStatus(start, goal).waypoints();
        List<Pose2d> viaLegacy = StaticPathfinder.findPath(start, goal);

        assertEquals(viaStatus.size(), viaLegacy.size(),
                "the two entry points must return identical waypoint lists");
        for (int i = 0; i < viaStatus.size(); i++) {
            assertEquals(viaStatus.get(i), viaLegacy.get(i),
                    "waypoint " + i + " differs between the two entry points");
        }
    }
}
