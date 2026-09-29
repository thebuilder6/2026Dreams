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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Cover for the bounded-derivative repulsion fix (action 2 of the 2026-09-28 plan).
 *
 * <p>The textbook inverse-distance form {@code k * (1/d - 1/d0)} diverges as
 * {@code d -> 0}. Peers used a 0.05 m guard, so the normal term reached
 * {@code 2.5 * (1/0.05 - 1/1.4) ~= 48 m/s}; walls used 0.04 m and reached
 * {@code 2.0 * (1/0.04 - 1/0.65) ~= 47 m/s}. The {@code MAX_SPEED} clamp hid the
 * magnitude but the commanded direction became a near-step function of position at
 * contact, so a robot pressed against a peer saturated and behaved discontinuously.
 * This is the derivative blow-up arXiv:2402.11601 addresses with subharmonic
 * potentials.
 *
 * <p>The fix raises each floor far enough to bound the term, chosen so the entire
 * pre-contact band (d above the floor) is bit-identical to the old curve.
 */
class RepulsionBoundsTest {

    private static final double OLD_PEER_FLOOR = 0.05;
    private static final double OLD_WALL_FLOOR = 0.04;
    private static final double OLD_PEER_GUARD = 0.05;
    private static final double OLD_WALL_GUARD = 0.04;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DynamicRouter.clearObstacles();
    }

    @AfterEach
    void tearDown() {
        DynamicRouter.clearObstacles();
    }

    private static double oldPeerForce(double k, double d) {
        return k * (1.0 / d - 1.0 / DynamicRouter.SAFE_DISTANCE_METERS);
    }

    private static double oldWallForce(double k, double d, double margin) {
        return k * (1.0 / d - 1.0 / margin);
    }

    // ---------------------------------------------------------------------
    // Bounded magnitude
    // ---------------------------------------------------------------------

    @Test
    void peerRepulsionStaysBoundedAtContact() {
        double worst = 0.0;
        for (double d = 0.0; d < DynamicRouter.SAFE_DISTANCE_METERS; d += 0.01) {
            worst = Math.max(worst, DynamicRouter.repulsiveFalloff(d));
        }
        double bounded = 2.5 * worst;
        assertTrue(bounded < 6.0,
                "Peer repulsion must stay bounded near contact, worst was " + bounded
                        + " m/s (was ~48 m/s with the 0.05 m guard)");
    }

    @Test
    void wallRepulsionStaysBoundedAtContact() {
        double margin = DynamicRouter.WALL_SAFETY_MARGIN_METERS;
        double worst = 0.0;
        for (double d = 0.0; d < margin; d += 0.01) {
            worst = Math.max(worst, DynamicRouter.wallFalloff(d, margin));
        }
        double bounded = 2.0 * worst;
        assertTrue(bounded < 8.0,
                "Wall repulsion must stay bounded near contact, worst was " + bounded
                        + " m/s (was ~47 m/s with the 0.04 m guard)");
    }

    /** The old curves, for the record: this is the magnitude that was removed. */
    @Test
    void oldFormDivergedAtTheOldGuards() {
        double oldPeer = oldPeerForce(2.5, OLD_PEER_GUARD);
        double oldWall = oldWallForce(2.0, OLD_WALL_FLOOR,
                DynamicRouter.WALL_SAFETY_MARGIN_METERS);
        assertTrue(oldPeer > 40.0,
                "Sanity: the old peer form really did blow up, got " + oldPeer);
        assertTrue(oldWall > 40.0,
                "Sanity: the old wall form really did blow up, got " + oldWall);
    }

    // ---------------------------------------------------------------------
    // Behaviour preserved where it matters
    // ---------------------------------------------------------------------

    /**
     * The fix must only touch the near-contact tail. Above the new floor the curve
     * is algebraically identical to the old one, so every ordinary approach behaves
     * exactly as before.
     */
    @Test
    void curveIsUnchangedAboveTheNewFloor() {
        for (double d = DynamicRouter.REPULSION_MIN_DISTANCE_M;
                d < DynamicRouter.SAFE_DISTANCE_METERS; d += 0.01) {
            assertEquals(oldPeerForce(1.0, d), DynamicRouter.repulsiveFalloff(d), 1e-12,
                    "Falloff diverged from the original at d=" + d);
        }
        double margin = DynamicRouter.WALL_SAFETY_MARGIN_METERS;
        for (double d = DynamicRouter.WALL_MIN_DISTANCE_M; d < margin; d += 0.01) {
            assertEquals(oldWallForce(1.0, d, margin), DynamicRouter.wallFalloff(d, margin), 1e-12,
                    "Wall falloff diverged from the original at d=" + d);
        }
    }

    @Test
    void falloffIsMonotonicAndZeroAtTheSafeDistance() {
        assertEquals(0.0, DynamicRouter.repulsiveFalloff(
                DynamicRouter.SAFE_DISTANCE_METERS), 1e-12,
                "No force at the edge of the influence radius");
        assertEquals(0.0, DynamicRouter.repulsiveFalloff(99.0), 1e-12,
                "No force beyond the influence radius");

        // Iterating d upward walks away from the obstacle, so the force must fall
        // monotonically. Checked in the direction the loop actually traverses.
        double prev = Double.MAX_VALUE;
        for (double d = 0.0; d < DynamicRouter.SAFE_DISTANCE_METERS; d += 0.02) {
            double f = DynamicRouter.repulsiveFalloff(d);
            assertTrue(f <= prev + 1e-12,
                    "Falloff must decrease monotonically as the gap widens, at d=" + d);
            assertTrue(f > 0.0, "Falloff must be positive inside the influence radius");
            prev = f;
        }
        // ...and the strongest force is at the closest approach.
        assertTrue(DynamicRouter.repulsiveFalloff(0.0)
                        > DynamicRouter.repulsiveFalloff(0.5),
                "Force must be strongest at the closest approach");
    }

    /** The proprioceptive 2.2x boost must still order correctly at every distance. */
    @Test
    void proprioceptiveBoostStillDominatesAtEveryDistance() {
        for (double d = 0.0; d < DynamicRouter.SAFE_DISTANCE_METERS; d += 0.05) {
            double normal = 2.5 * DynamicRouter.repulsiveFalloff(d);
            double proprio = 2.5 * 2.2 * DynamicRouter.repulsiveFalloff(d);
            assertTrue(proprio > normal,
                    "Proprioceptive force must exceed normal force at d=" + d);
        }
    }

    // ---------------------------------------------------------------------
    // End-to-end: the commanded command is no longer a step function
    // ---------------------------------------------------------------------

    /**
     * The observable consequence of the old blow-up: a peer sitting a few centimetres
     * further away produced a wildly different commanded speed. The bound means small
     * position changes near contact produce proportionate changes.
     */
    @Test
    void commandedSpeedIsProportionateNearContact() {
        Pose2d robot = new Pose2d(8.0, 4.0, new Rotation2d());
        ChassisSpeeds nominal = new ChassisSpeeds(-1.5, 0.0, 0.0);
        Translation2d target = new Translation2d(6.0, 4.0);

        // Distance from the robot centre to the peer centre, minus the peer radius,
        // i.e. the gap DynamicRouter actually integrates on.
        double radius = 0.55;
        double atTenCm = 0.10;
        double atElevenCm = 0.11;

        DynamicRouter.clearObstacles();
        DynamicRouter.registerObstacle(
                new Translation2d(robot.getX() - (atTenCm + radius), robot.getY()),
                new Translation2d(), radius, 5.0, false);
        ChassisSpeeds near = DynamicRouter.computeAvoidanceSpeeds(robot, nominal, target);

        DynamicRouter.clearObstacles();
        DynamicRouter.registerObstacle(
                new Translation2d(robot.getX() - (atElevenCm + radius), robot.getY()),
                new Translation2d(), radius, 5.0, false);
        ChassisSpeeds nearer = DynamicRouter.computeAvoidanceSpeeds(robot, nominal, target);

        double jump = Math.abs(near.vxMetersPerSecond - nearer.vxMetersPerSecond);
        assertTrue(jump < 0.5,
                "A 1 cm gap change must not swing the command by " + jump
                        + " m/s; that discontinuity is the bug the bound removes");
    }

    /** Clear-field behaviour must be completely untouched. */
    @Test
    void noObstaclesLeavesTheNominalCommandUnchanged() {
        Pose2d robot = new Pose2d(8.0, 4.0, new Rotation2d());
        ChassisSpeeds nominal = new ChassisSpeeds(-1.5, 0.4, 0.2);
        ChassisSpeeds out = DynamicRouter.computeAvoidanceSpeeds(
                robot, nominal, new Translation2d(6.0, 4.0));
        assertEquals(nominal.vxMetersPerSecond, out.vxMetersPerSecond, 1e-9);
        assertEquals(nominal.vyMetersPerSecond, out.vyMetersPerSecond, 1e-9);
        assertEquals(nominal.omegaRadiansPerSecond, out.omegaRadiansPerSecond, 1e-9);
    }

    /** A far-away peer inside the influence radius still evades. */
    @Test
    void farPeerStillProducesAvoidance() {
        Pose2d robot = new Pose2d(8.0, 4.0, new Rotation2d());
        DynamicRouter.registerObstacle(new Translation2d(8.0, 5.2),
                new Translation2d(), 0.55, 5.0, false);
        ChassisSpeeds out = DynamicRouter.computeAvoidanceSpeeds(
                robot, new ChassisSpeeds(-1.5, 0.0, 0.0), new Translation2d(6.0, 4.0));
        assertTrue(out.vyMetersPerSecond < -0.05,
                "A peer to the north must still push the robot south, got vy="
                        + out.vyMetersPerSecond);
    }
}
