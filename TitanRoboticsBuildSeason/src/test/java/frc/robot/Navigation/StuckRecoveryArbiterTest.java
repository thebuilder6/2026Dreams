package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Navigation.StuckRecoveryArbiter.RecoveryResult;
import frc.robot.Navigation.StuckRecoveryArbiter.RecoveryTier;

public class StuckRecoveryArbiterTest {

    private ContactWatchdog contactWatchdog;
    private TargetProgressWatchdog targetProgressWatchdog;

    @BeforeEach
    public void setup() {
        contactWatchdog = new ContactWatchdog(new java.util.Random(42));
        contactWatchdog.reset();
        targetProgressWatchdog = new TargetProgressWatchdog();
        targetProgressWatchdog.reset();
    }

    @Test
    public void testPinRuleBackoffTakesHighestPrecedence() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());

        // Accumulate 2.5s of contact to trigger forced backoff (threshold 2.4s)
        for (int i = 0; i < 130; i++) {
            contactWatchdog.update(
                    robotPose,
                    new ChassisSpeeds(),
                    new ChassisSpeeds(1.5, 0.0, 0.0),
                    0.0,
                    0.0,
                    30.0,
                    0.5,
                    oppPose,
                    0.02);
        }
        assertTrue(contactWatchdog.isForcedBackoffActive(), "Forced backoff should be active");

        // Target progress watchdog also claims recovery
        TargetProgressWatchdog.Result progress = targetProgressWatchdog.abandonTarget(
                new Translation2d(6.0, 4.0), robotPose);
        assertTrue(progress.recovering());

        // Arbitrate: Tier 1 (PIN_RULE_BACKOFF) must take precedence
        RecoveryResult res = StuckRecoveryArbiter.arbitrate(
                new ChassisSpeeds(2.0, 0.0, 0.0),
                robotPose,
                contactWatchdog,
                targetProgressWatchdog,
                progress,
                oppPose,
                List.of(oppPose),
                true,
                false);

        assertEquals(RecoveryTier.PIN_RULE_BACKOFF, res.activeTier());
        assertTrue(res.isRecovering());
        assertTrue(res.stateDetail().startsWith("PIN_RULE_BACKOFF"));
        // Forced backoff moves away from opponent (robot at 5.0, opp at 5.5 -> move -X)
        assertTrue(res.speeds().vxMetersPerSecond < -0.5);
    }

    @Test
    public void testStaticUnstickPirouetteWinsOverProgress() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());

        // Trigger unstick (stall without peer)
        for (int i = 0; i < 10; i++) {
            contactWatchdog.updateUnstickOnly(
                    new ChassisSpeeds(0.05, 0.0, 0.0),
                    new ChassisSpeeds(2.0, 0.0, 0.0),
                    0.02);
        }
        assertTrue(contactWatchdog.isPirouetteActive());

        // Target progress also reports recovering
        TargetProgressWatchdog.Result progress = targetProgressWatchdog.abandonTarget(
                new Translation2d(5.5, 4.0), robotPose);
        assertTrue(progress.recovering());

        // Arbitrate: Tier 2 (STATIC_UNSTICK) must win over Tier 4 (TARGET_UNREACHABLE)
        RecoveryResult res = StuckRecoveryArbiter.arbitrate(
                new ChassisSpeeds(1.0, 0.0, 0.0),
                robotPose,
                contactWatchdog,
                targetProgressWatchdog,
                progress,
                null,
                null,
                false,
                false);

        assertEquals(RecoveryTier.STATIC_UNSTICK, res.activeTier());
        assertEquals("STATIC_UNSTICK", res.stateDetail());
        // Pirouette applies rotation rate 6.0 rad/s
        assertEquals(6.0, res.speeds().omegaRadiansPerSecond, 1e-4);
    }

    @Test
    public void testTrenchDeadlockAndCooldownYield() {
        Pose2d robotPose = new Pose2d(5.0, 0.65, new Rotation2d()); // Low clearance trench Y=0.65

        // Trench deadlock triggers faster (0.5s stall)
        for (int i = 0; i < 30; i++) {
            contactWatchdog.updateDeadlockOnly(true, 0.8, 0.02, true);
        }
        assertTrue(contactWatchdog.isDeadlockRecovering());

        RecoveryResult res = StuckRecoveryArbiter.arbitrate(
                new ChassisSpeeds(2.0, 0.0, 0.0),
                robotPose,
                contactWatchdog,
                targetProgressWatchdog,
                TargetProgressWatchdog.Result.IDLE,
                null,
                List.of(new Pose2d(5.8, 0.65, new Rotation2d())),
                false,
                true);

        assertEquals(RecoveryTier.TRENCH_YIELD, res.activeTier());
        assertEquals("TRENCH_YIELD", res.stateDetail());
        // In trench, deadlock reverses command without lateral jink
        assertTrue(res.speeds().vxMetersPerSecond < 0.0);
    }

    @Test
    public void testTargetUnreachableEscapeWinsWhenNoContactRecovery() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        TargetProgressWatchdog.Result progress = targetProgressWatchdog.abandonTarget(
                new Translation2d(5.5, 4.0), robotPose);

        RecoveryResult res = StuckRecoveryArbiter.arbitrate(
                new ChassisSpeeds(2.0, 0.0, 0.0),
                robotPose,
                contactWatchdog,
                targetProgressWatchdog,
                progress,
                null,
                null,
                false,
                false);

        assertEquals(RecoveryTier.TARGET_UNREACHABLE, res.activeTier());
        assertTrue(res.isRecovering());
        assertTrue(res.stateDetail().startsWith("TARGET_UNREACHABLE"));
        assertEquals(progress.escapeVector().getX(), res.speeds().vxMetersPerSecond, 1e-6);
        assertEquals(progress.escapeVector().getY(), res.speeds().vyMetersPerSecond, 1e-6);
    }

    @Test
    public void testContactRecoverySuppressesTargetProgressTimeout() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d target = new Pose2d(6.0, 4.0, new Rotation2d());

        // Arm contact pirouette
        for (int i = 0; i < 10; i++) {
            contactWatchdog.updateUnstickOnly(
                    new ChassisSpeeds(0.05, 0.0, 0.0),
                    new ChassisSpeeds(2.0, 0.0, 0.0),
                    0.02);
        }
        assertTrue(contactWatchdog.isAnyContactRecoveryActive());

        // Run target progress with suppressProgressTimeout = true for 4.0 seconds (past 3.0s GIVEUP_SEC)
        for (int i = 0; i < 200; i++) {
            TargetProgressWatchdog.Result res = targetProgressWatchdog.update(
                    robotPose,
                    new ChassisSpeeds(2.0, 0.0, 0.0),
                    new ChassisSpeeds(0.0, 0.0, 0.0),
                    target,
                    true,
                    0.02);
            assertFalse(res.recovering(), "Should NOT trigger recovery while contact recovery is active");
        }
        assertEquals(0.0, targetProgressWatchdog.getNoProgressSec(), 1e-6);
    }

    @Test
    public void testWallSafeUnstickDeflectsAlongPerimeter() {
        // Robot near Blue wall at X = 0.40m, facing North (Y=4.0m)
        Pose2d nearWallPose = new Pose2d(0.40, 4.0, Rotation2d.fromDegrees(90));
        // Vector pointing negative X (into the wall)
        Translation2d intoWall = new Translation2d(-2.0, 0.5);

        Translation2d safe = ContactWatchdog.ensureWallSafe(nearWallPose, intoWall);
        // VX must be zeroed (not driving into wall)
        assertEquals(0.0, safe.getX(), 1e-6);
        // Vy must retain escape magnitude
        assertTrue(Math.abs(safe.getY()) >= 1.5);
    }

    @Test
    public void testStationaryHarvesterWatchdogAbandonsWhenHeldInsideArrived() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d targetPose = new Pose2d(5.2, 4.0, new Rotation2d()); // Within ARRIVED_M (0.60m)

        // Hold for 1.7s (under 1.8s threshold) while intaking without collecting fuel
        for (int i = 0; i < 85; i++) {
            TargetProgressWatchdog.Result res = targetProgressWatchdog.updateHarvestArrivalWatchdog(
                    robotPose, targetPose, true, 0, 0.02);
            assertFalse(res.recovering());
        }

        // Cross 1.8s threshold
        TargetProgressWatchdog.Result triggered = TargetProgressWatchdog.Result.IDLE;
        for (int i = 0; i < 15; i++) {
            triggered = targetProgressWatchdog.updateHarvestArrivalWatchdog(
                    robotPose, targetPose, true, 0, 0.02);
            if (triggered.recovering()) break;
        }

        assertTrue(triggered.recovering(), "Harvester watchdog must abandon target after 1.8s arrival stall");
        assertTrue(targetProgressWatchdog.isRecovering());
    }
}
