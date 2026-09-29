package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Regression cover for the stall chain found by the 2026-09-28 navigation probe.
 *
 * <p>The probe swept 444,550 (start, peer, goal) combinations through
 * {@link StaticPathfinder#findPath} and found <b>zero</b> route failures, with a
 * minimum segment clearance of 0.476 m across 203,401 paths. The hand-placed
 * 34-node roadmap is therefore not a stuck source, and no grid fallback planner
 * is warranted. The two real defects were elsewhere:
 *
 * <ol>
 *   <li><b>APF drive inversion.</b> With a peer on the path, pure vector summation
 *       let repulsion exceed the nominal command, so the robot was told to drive
 *       <i>away</i> from its target (probe: nominal -1.500 west, output +2.270 east)
 *       or was cancelled to a 0.345 m/s crawl.</li>
 *   <li><b>Slow-creep blind spot.</b> Both stall detectors gated on commanded speed
 *       &gt; 0.80 m/s, but {@code TrajectoryController} commands as little as 0.25 m/s
 *       (carpet-friction breakout floor) and TRENCH_YIELD scales by 0.2x. A robot wedged
 *       while commanding 0.25-0.79 m/s never accumulated a no-progress window and sat
 *       there for the rest of the match. Both now gate on 0.12 m/s, which is a "hold"
 *       floor rather than a stall threshold: a true hold commands ~0 and still decays.</li>
 * </ol>
 *
 * <p>Together these formed a closed trap: APF cancelled the drive, the resulting crawl
 * fell into the sub-0.80 m/s blind band, and no watchdog fired. Either fix alone leaves
 * the trap open, so both are pinned here.
 */
class StuckRecoveryTest {

    private static final double DT = 0.02;
    private static final Translation2d TARGET = new Translation2d(6.0, 4.0);
    /** Robot drives west (-x) toward the target. */
    private static final ChassisSpeeds WEST = new ChassisSpeeds(-1.5, 0.0, 0.0);

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        FieldMap.setObstacleHandling(FieldMap.ObstacleHandling.IMPASSABLE);
        DynamicRouter.clearObstacles();
    }

    @AfterEach
    void tearDown() {
        DynamicRouter.clearObstacles();
    }

    // ---------------------------------------------------------------------
    // 1. APF must never invert or cancel the commanded drive
    // ---------------------------------------------------------------------

    /**
     * The peer sits between the robot and its target, so repulsion genuinely opposes
     * the drive. Before the fix the output flipped sign (drive away from target) or
     * collapsed to a crawl; now the forward component always survives.
     */
    @Test
    void peerOnPathNeverInvertsOrCancelsTheDrive() {
        Pose2d robot = new Pose2d(8.0, 4.0, new Rotation2d());

        for (double gap = 0.6; gap <= 1.8; gap += 0.1) {
            DynamicRouter.clearObstacles();
            DynamicRouter.registerObstacle(
                    new Translation2d(robot.getX() - gap, robot.getY()),
                    new Translation2d(), 0.55, 5.0, false);

            ChassisSpeeds out = DynamicRouter.computeAvoidanceSpeeds(robot, WEST, TARGET);
            String where = String.format("gap=%.2f", gap);

            assertTrue(out.vxMetersPerSecond < 0.0,
                    "Must keep driving west toward the target at " + where
                            + ", got vx=" + out.vxMetersPerSecond);
            // Above the 0.12 m/s hold floor, so the stall detectors can now see it.
            assertTrue(Math.hypot(out.vxMetersPerSecond, out.vyMetersPerSecond) > 0.12,
                    "Must not fall into the sub-0.12 m/s blind band at " + where);
        }
    }

    /**
     * In the override branch -- repulsion strictly greater than the nominal command --
     * forward progress is floored at {@link DynamicRouter#MIN_FORWARD_FRACTION}. Asserted
     * separately because at large gaps repulsion falls below nominal and the router uses
     * the plain sum instead, which legitimately leaves a smaller residual.
     */
    @Test
    void dominantRepulsionRetainsFlooredForwardProgress() {
        Pose2d robot = new Pose2d(8.0, 4.0, new Rotation2d());

        // 0.8 m: repulsion (8.21 m/s) far exceeds the 1.5 m/s nominal command.
        DynamicRouter.registerObstacle(new Translation2d(robot.getX() - 0.8, robot.getY()),
                new Translation2d(), 0.55, 5.0, false);
        ChassisSpeeds out = DynamicRouter.computeAvoidanceSpeeds(robot, WEST, TARGET);

        assertTrue(out.vxMetersPerSecond <= -DynamicRouter.MIN_FORWARD_FRACTION * 1.5,
                "Dominant repulsion must still command the floored forward fraction, got vx="
                        + out.vxMetersPerSecond);
        assertTrue(Math.abs(out.vyMetersPerSecond) > 0.5,
                "The surplus must become a tangential slide rather than vanish, got vy="
                        + out.vyMetersPerSecond);
    }

    /**
     * Two peers straddling the path produce symmetric lateral terms that cancel, which
     * is the local-minimum case a pure-sum APF cannot escape. The deterministic parity
     * tie-break must still yield a real command.
     */
    @Test
    void flankingPeersStillProduceForwardProgress() {
        Pose2d robot = new Pose2d(8.0, 4.0, new Rotation2d());

        for (double dy = 0.3; dy <= 1.2; dy += 0.3) {
            DynamicRouter.clearObstacles();
            DynamicRouter.registerObstacle(new Translation2d(robot.getX() - 0.9, robot.getY() + dy),
                    new Translation2d(), 0.55, 5.0, false);
            DynamicRouter.registerObstacle(new Translation2d(robot.getX() - 0.9, robot.getY() - dy),
                    new Translation2d(), 0.55, 5.0, false);

            ChassisSpeeds out = DynamicRouter.computeAvoidanceSpeeds(robot, WEST, TARGET);
            String where = String.format("dy=+/-%.2f", dy);

            assertTrue(out.vxMetersPerSecond < 0.0,
                    "Symmetric lateral repulsion must not cancel forward progress at " + where
                            + ", got vx=" + out.vxMetersPerSecond);
            assertTrue(Math.hypot(out.vxMetersPerSecond, out.vyMetersPerSecond) > 0.12,
                    "Flanked robot must keep moving at " + where);
        }
    }

    /** Far from any peer, the router must not alter the nominal command at all. */
    @Test
    void clearFieldLeavesTheNominalCommandUntouched() {
        Pose2d robot = new Pose2d(8.0, 4.0, new Rotation2d());
        ChassisSpeeds out = DynamicRouter.computeAvoidanceSpeeds(robot, WEST, TARGET);
        assertTrue(Math.abs(out.vxMetersPerSecond - WEST.vxMetersPerSecond) < 1e-6);
        assertTrue(Math.abs(out.vyMetersPerSecond - WEST.vyMetersPerSecond) < 1e-6);
    }

    // ---------------------------------------------------------------------
    // 2. Slow-creep blind spot
    // ---------------------------------------------------------------------

    /**
     * A robot physically wedged (pose frozen) while commanding a slow speed must
     * eventually be abandoned. Before the fix, 0.30/0.50/0.79 m/s never fired.
     */
    @Test
    void wedgedRobotIsAbandonedAtSlowCommandSpeeds() {
        for (double cmd : new double[] {0.25, 0.30, 0.50, 0.79}) {
            TargetProgressWatchdog watchdog = new TargetProgressWatchdog();
            Pose2d wedged = new Pose2d(8.0, 4.0, new Rotation2d()); // never moves
            boolean fired = false;

            for (int i = 0; i < 400 && !fired; i++) {
                fired = watchdog.update(wedged, new ChassisSpeeds(-cmd, 0, 0),
                        new Pose2d(TARGET.getX(), TARGET.getY(), new Rotation2d()), DT)
                        .recovering();
            }
            assertTrue(fired,
                    "Wedged robot commanding " + cmd + " m/s must be abandoned; the old "
                            + "0.80 m/s gate left it invisible to every watchdog");
        }
    }

    /**
     * The complementary guard: a genuine hold (staging, planting to shoot) still decays
     * instead of accumulating a window, so lowering the floor does not create
     * false-positive escapes.
     */
    @Test
    void intentionalHoldStillDecays() {
        TargetProgressWatchdog watchdog = new TargetProgressWatchdog();
        Pose2d holding = new Pose2d(8.0, 4.0, new Rotation2d());

        for (int i = 0; i < 400; i++) {
            TargetProgressWatchdog.Result r = watchdog.update(holding,
                    new ChassisSpeeds(-0.05, 0, 0), // below the 0.12 floor
                    new Pose2d(TARGET.getX(), TARGET.getY(), new Rotation2d()), DT);
            assertFalse(r.recovering(), "an intentional hold must not trigger recovery");
        }
        assertTrue(watchdog.blockedPoints().isEmpty());
    }

    /**
     * A slow but genuinely converging approach must never be abandoned: the decision
     * is made on progress, not on speed.
     */
    @Test
    void slowConvergingApproachIsNotAbandoned() {
        TargetProgressWatchdog watchdog = new TargetProgressWatchdog();
        double startX = 8.0;
        double stepM = 0.30 * DT; // 0.30 m/s of real closing

        for (int i = 0; i < 200; i++) {
            Pose2d pose = new Pose2d(Math.max(TARGET.getX() + 0.7, startX - i * stepM),
                    4.0, new Rotation2d());
            TargetProgressWatchdog.Result r = watchdog.update(pose,
                    new ChassisSpeeds(-0.30, 0, 0),
                    new Pose2d(TARGET.getX(), TARGET.getY(), new Rotation2d()), DT);
            assertFalse(r.recovering(), "a converging robot must never be abandoned");
        }
    }

    // ---------------------------------------------------------------------
    // 3. The two detectors must agree on their gate
    // ---------------------------------------------------------------------

    /**
     * A mismatch between the watchdog's gate and the bot-local stall flag previously
     * left pirouette/pin blind to stuck robots while deadlock still fired. Pin the
     * shared constant and the ordering that makes it meaningful.
     */
    @Test
    void stallGateIsBelowTheControllersMinimumUsefulCommand() {
        assertTrue(ContactWatchdog.STALL_CMD_SPEED_MIN <= 0.25,
                "The stall gate must sit below TrajectoryController's 0.25 m/s breakout "
                        + "floor or slow-creep stalls are invisible");
        assertTrue(TargetProgressWatchdog.COMMAND_MIN_MPS <= 0.25,
                "The progress gate must sit below TrajectoryController's 0.25 m/s "
                        + "breakout floor for the same reason");
        assertTrue(ContactWatchdog.STALL_CMD_SPEED_MIN < ContactWatchdog.STALL_ACTUAL_SPEED_MAX,
                "A commanded speed must be able to clear the measured-speed ceiling, "
                        + "otherwise no stall is ever detectable");
    }
}
