package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.List;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Navigation.ContactWatchdog;
import frc.robot.Navigation.TargetProgressWatchdog;
import frc.robot.Sim.AIRobotInstance.DriveCorrection;
import frc.robot.Sim.AIRobotInstance.ResolvedDrive;
import frc.robot.Sim.BotMatchMetrics;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Phase 1 single-owner arbitration cover: the trajectory speeds get exactly
 * one peer correction per tick — deadlock, yield, or separation — never a sum.
 *
 * <p>The defect this pins: the separation nudge used to be added first and
 * then scaled again by the deadlock recovery, so a trench reverse-out at
 * −0.8x flipped an away-from-peer nudge into a toward-peer push, and an
 * open-field jink double-counted the same peer.
 */
class AIRobotInstanceNavArbitrationTest {

    private static final Pose2d ORIGIN = new Pose2d(0.0, 4.0, new Rotation2d());

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
    }

    @Test
    void trenchReverseSkipsSeparationNudge() {
        // Peer 0.5 m ahead: close enough that the old code added a nudge AND
        // then reversed the sum. The nudge must not appear in the output.
        List<Pose2d> peers = List.of(new Pose2d(0.5, 4.0, new Rotation2d()));
        ContactWatchdog.Resolution deadlock = new ContactWatchdog.Resolution(true, -0.8, 0.0);

        ResolvedDrive out = AIRobotInstance.resolvePreProgressCommand(
                new ChassisSpeeds(1.0, 0.0, 0.0), ORIGIN, peers, deadlock, false);

        assertEquals(DriveCorrection.DEADLOCK, out.correction());
        assertEquals("DEADLOCK_RECOVERY", out.detail());
        assertEquals(-0.8, out.speeds().vxMetersPerSecond, 1e-9);
        assertEquals(0.0, out.speeds().vyMetersPerSecond, 1e-9);
    }

    @Test
    void openFieldJinkDoesNotDoubleCountThePeer() {
        List<Pose2d> peers = List.of(new Pose2d(0.5, 4.0, new Rotation2d()));
        ContactWatchdog.Resolution deadlock = new ContactWatchdog.Resolution(true, 0.3, 1.2);

        ResolvedDrive out = AIRobotInstance.resolvePreProgressCommand(
                new ChassisSpeeds(2.0, 0.0, 0.5), ORIGIN, peers, deadlock, false);

        assertEquals(DriveCorrection.DEADLOCK, out.correction());
        // 2.0 * 0.3 forward, 1.2 lateral jink in the robot frame (facing +x),
        // omega carried through untouched. No radial nudge added.
        assertEquals(0.6, out.speeds().vxMetersPerSecond, 1e-9);
        assertEquals(1.2, out.speeds().vyMetersPerSecond, 1e-9);
        assertEquals(0.5, out.speeds().omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void trenchYieldHoldsWithoutNudging() {
        List<Pose2d> peers = List.of(new Pose2d(0.5, 4.0, new Rotation2d()));
        ContactWatchdog.Resolution idle = new ContactWatchdog.Resolution(false, 1.0, 0.0);

        ResolvedDrive out = AIRobotInstance.resolvePreProgressCommand(
                new ChassisSpeeds(2.0, 1.0, 0.0), ORIGIN, peers, idle, true);

        assertEquals(DriveCorrection.TRENCH_YIELD, out.correction());
        assertEquals("TRENCH_YIELD", out.detail());
        assertEquals(0.4, out.speeds().vxMetersPerSecond, 1e-9);
        assertEquals(0.2, out.speeds().vyMetersPerSecond, 1e-9);
    }

    @Test
    void idleBotStillNudgesAwayFromClosePeers() {
        List<Pose2d> peers = List.of(new Pose2d(0.5, 4.0, new Rotation2d()));
        ContactWatchdog.Resolution idle = new ContactWatchdog.Resolution(false, 1.0, 0.0);

        ResolvedDrive out = AIRobotInstance.resolvePreProgressCommand(
                new ChassisSpeeds(1.0, 0.0, 0.0), ORIGIN, peers, idle, false);

        assertEquals(DriveCorrection.SEPARATION, out.correction());
        assertNull(out.detail());
        // diff = (-0.5, 0), scale = (1.1-0.5)/1.1, nudge = -x * scale * 1.5.
        double expected = 1.0 - ((1.10 - 0.5) / 1.10) * 1.5;
        assertEquals(expected, out.speeds().vxMetersPerSecond, 1e-9);
        assertEquals(0.0, out.speeds().vyMetersPerSecond, 1e-9);
    }

    @Test
    void farPeersLeaveTheCommandAlone() {
        List<Pose2d> peers = List.of(new Pose2d(5.0, 4.0, new Rotation2d()));
        ContactWatchdog.Resolution idle = new ContactWatchdog.Resolution(false, 1.0, 0.0);

        ResolvedDrive out = AIRobotInstance.resolvePreProgressCommand(
                new ChassisSpeeds(1.0, 2.0, 0.3), ORIGIN, peers, idle, false);

        assertEquals(DriveCorrection.NONE, out.correction());
        assertNull(out.detail());
        assertEquals(1.0, out.speeds().vxMetersPerSecond, 1e-9);
        assertEquals(2.0, out.speeds().vyMetersPerSecond, 1e-9);
        assertEquals(0.3, out.speeds().omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void nullInputsResolveToZeroWithoutThrowing() {
        ResolvedDrive out = AIRobotInstance.resolvePreProgressCommand(
                null, null, null, null, false);

        assertEquals(DriveCorrection.NONE, out.correction());
        assertEquals(0.0, out.speeds().vxMetersPerSecond, 1e-9);
        assertEquals(0.0, out.speeds().vyMetersPerSecond, 1e-9);
    }

    @Test
    void jinkRotatesWithRobotHeading() {
        // Facing +y: a (0, 1.2) robot-frame jink lands on field -x.
        Pose2d facingY = new Pose2d(0.0, 4.0, Rotation2d.fromDegrees(90));
        ContactWatchdog.Resolution deadlock = new ContactWatchdog.Resolution(true, 0.3, 1.2);

        ResolvedDrive out = AIRobotInstance.resolvePreProgressCommand(
                new ChassisSpeeds(0.0, 0.0, 0.0), facingY, List.of(), deadlock, false);

        Translation2d jink = new Translation2d(0, 1.2).rotateBy(facingY.getRotation());
        assertEquals(jink.getX(), out.speeds().vxMetersPerSecond, 1e-9);
        assertEquals(jink.getY(), out.speeds().vyMetersPerSecond, 1e-9);
    }

    // ---------------------------------------------------------------------
    // Phase 0b: unattributed-stall classification
    // ---------------------------------------------------------------------

    @Test
    void closeStalledTargetIsTheArrivedHole() {
        // 0.3 m < ARRIVED_M 0.60: the watchdog calls this "reached" and stays
        // idle, so a blocked-but-close bot never fires any recovery.
        assertEquals(BotMatchMetrics.StallCause.STALLED_ARRIVED,
                AIRobotInstance.classifyUnattributedStall(0.3, 5.0, 5.0));
    }

    @Test
    void longStallOnAYoungTargetIsChurn() {
        // Stalled 4 s but tracking a 1 s-old target: Jev retargeted mid-stall,
        // so no single target ever accumulates the 3 s give-up window.
        assertEquals(BotMatchMetrics.StallCause.STALLED_CHURN,
                AIRobotInstance.classifyUnattributedStall(5.0, 1.0, 4.0));
    }

    @Test
    void shortStallOnAYoungTargetIsNotChurnYet() {
        // 1 s into a fresh pursuit: too early to call it churn.
        assertEquals(BotMatchMetrics.StallCause.STALLED_OTHER,
                AIRobotInstance.classifyUnattributedStall(5.0, 1.0, 1.0));
    }

    @Test
    void longStallOnAStableTargetIsGenuinelyUnexplained() {
        assertEquals(BotMatchMetrics.StallCause.STALLED_OTHER,
                AIRobotInstance.classifyUnattributedStall(5.0, 5.0, 5.0));
    }

    @Test
    void missingTargetIsUnexplainedNotChurn() {
        assertEquals(BotMatchMetrics.StallCause.STALLED_OTHER,
                AIRobotInstance.classifyUnattributedStall(
                        Double.MAX_VALUE, -1.0, 5.0));
    }
}
