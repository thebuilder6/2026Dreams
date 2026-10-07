package frc.robot.Navigation;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * StuckRecoveryArbiter provides unified single-owner drive arbitration across all
 * safety, collision, deadlock, and target progress recovery systems:
 *
 * <ol>
 *   <li><b>Tier 1: Rule G418 Forced Pin Backoff</b> (highest priority, legal compliance)</li>
 *   <li><b>Tier 2: Static Geometry Unstick / Pirouette</b> (geometry/obstacle wedge escape)</li>
 *   <li><b>Tier 3: Peer Deadlock / Trench Yield</b> (head-on traffic resolution)</li>
 *   <li><b>Tier 4: Target Unreachable Escape</b> (progress failure / abandoned target)</li>
 *   <li><b>Tier 5: Peer Soft Separation</b> (multi-bot collision softening)</li>
 *   <li><b>Tier 6: Nominal Trajectory Speeds</b> (clean trajectory tracking)</li>
 * </ol>
 *
 * <p>Guarantees mutual non-interference: exactly one recovery tier commands the
 * chassis per tick, eliminating conflicting vector overrides and clobbered rotation rates.
 */
public final class StuckRecoveryArbiter {

    public enum RecoveryTier {
        NONE,
        PIN_RULE_BACKOFF,
        STATIC_UNSTICK,
        DEADLOCK_RECOVERY,
        TRENCH_YIELD,
        TARGET_UNREACHABLE,
        PEER_SEPARATION
    }

    public record RecoveryResult(
            ChassisSpeeds speeds,
            RecoveryTier activeTier,
            String stateDetail) {
        public boolean isRecovering() {
            return activeTier != RecoveryTier.NONE && activeTier != RecoveryTier.PEER_SEPARATION;
        }
    }

    private StuckRecoveryArbiter() {}

    /** Soft peer separation radius (m). */
    public static final double SEPARATION_RADIUS_M = 1.30;
    /** Soft peer separation nudge magnitude (m/s). */
    public static final double SEPARATION_NUDGE_MPS = 0.40;
    /** Trench yield forward command scale during cooldown. */
    public static final double TRENCH_YIELD_SCALE = 0.20;

    /**
     * Arbitrates commanded drive velocities across all safety, collision, deadlock,
     * and target progress recovery systems in strict priority order.
     *
     * @param trajectorySpeeds speeds from TrajectoryController
     * @param currentPose measured robot pose
     * @param contactWatchdog robot's ContactWatchdog instance
     * @param targetProgressWatchdog robot's TargetProgressWatchdog instance
     * @param progressResult result from TargetProgressWatchdog.update
     * @param pinReference opponent pose for G418 pin reference (or null)
     * @param peerRobotPoses list of peer robot poses (or null)
     * @param isDefensive whether this robot is currently assigned to a defensive archetype
     * @param inTrench whether robot is inside a low-clearance trench corridor
     * @return unified, single-owner RecoveryResult
     */
    public static RecoveryResult arbitrate(
            ChassisSpeeds trajectorySpeeds,
            Pose2d currentPose,
            ContactWatchdog contactWatchdog,
            TargetProgressWatchdog targetProgressWatchdog,
            TargetProgressWatchdog.Result progressResult,
            Pose2d pinReference,
            List<Pose2d> peerRobotPoses,
            boolean isDefensive,
            boolean inTrench) {
        ChassisSpeeds base = (trajectorySpeeds != null) ? trajectorySpeeds : new ChassisSpeeds();
        Pose2d pose = (currentPose != null) ? currentPose : new Pose2d();

        // ── Tier 1: Rule G418 Forced Pin Backoff ────────────────────────────
        if (contactWatchdog != null && contactWatchdog.isForcedBackoffActive()
                && isDefensive && pinReference != null) {
            Pose2d backoff = contactWatchdog.getBackOffTarget(pose, pinReference);
            Translation2d dir = backoff.getTranslation().minus(pose.getTranslation());
            if (dir.getNorm() > 1e-4) {
                dir = dir.div(dir.getNorm());
            }
            ChassisSpeeds speeds = new ChassisSpeeds(dir.getX() * 1.5, dir.getY() * 1.5, 0.0);
            String detail = String.format("PIN_RULE_BACKOFF (%.1fs)", contactWatchdog.getBackoffRemainingSec());
            return new RecoveryResult(speeds, RecoveryTier.PIN_RULE_BACKOFF, detail);
        }

        // ── Tier 2: Static Geometry Unstick / Pirouette (obstacle wedge) ────
        if (contactWatchdog != null && contactWatchdog.isPirouetteActive()) {
            ChassisSpeeds speeds = contactWatchdog.applyUnstickOnly(base);
            return new RecoveryResult(speeds, RecoveryTier.STATIC_UNSTICK, "STATIC_UNSTICK");
        }

        // ── Tier 3: Peer Deadlock / Trench Yield ────────────────────────────
        if (contactWatchdog != null && contactWatchdog.isDeadlockRecovering()) {
            ContactWatchdog.Resolution deadlock = contactWatchdog.getDeadlockResolution();
            if (deadlock != null && deadlock.recovering()) {
                if (inTrench) {
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            base.vxMetersPerSecond * deadlock.forwardScale(),
                            base.vyMetersPerSecond * deadlock.forwardScale(),
                            base.omegaRadiansPerSecond);
                    return new RecoveryResult(speeds, RecoveryTier.TRENCH_YIELD, "TRENCH_YIELD");
                } else {
                    Translation2d jinkField = new Translation2d(0, deadlock.lateralJink())
                            .rotateBy(pose.getRotation());
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            base.vxMetersPerSecond * deadlock.forwardScale() + jinkField.getX(),
                            base.vyMetersPerSecond * deadlock.forwardScale() + jinkField.getY(),
                            base.omegaRadiansPerSecond);
                    return new RecoveryResult(speeds, RecoveryTier.DEADLOCK_RECOVERY, "DEADLOCK_RECOVERY");
                }
            }
        }

        // ── Tier 3b: Trench Cooldown Yield ──────────────────────────────────
        double nearestPeerDist = Double.MAX_VALUE;
        if (peerRobotPoses != null) {
            for (Pose2d peerPose : peerRobotPoses) {
                if (peerPose == null) continue;
                double d = pose.getTranslation().getDistance(peerPose.getTranslation());
                if (d > 0.05 && d < nearestPeerDist) nearestPeerDist = d;
            }
        }
        boolean trenchCoolingYield = inTrench
                && contactWatchdog != null
                && contactWatchdog.isDeadlockCooling()
                && nearestPeerDist < ContactWatchdog.PROXIMITY_M;
        if (trenchCoolingYield) {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    base.vxMetersPerSecond * TRENCH_YIELD_SCALE,
                    base.vyMetersPerSecond * TRENCH_YIELD_SCALE,
                    base.omegaRadiansPerSecond);
            return new RecoveryResult(speeds, RecoveryTier.TRENCH_YIELD, "TRENCH_YIELD");
        }

        // ── Tier 4: Target Unreachable Escape ───────────────────────────────
        if (progressResult != null && progressResult.recovering()) {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    progressResult.escapeVector().getX(),
                    progressResult.escapeVector().getY(),
                    0.0);
            String detail = String.format("TARGET_UNREACHABLE (%.1fs)", progressResult.escapeRemainingSec());
            return new RecoveryResult(speeds, RecoveryTier.TARGET_UNREACHABLE, detail);
        }

        // ── Tier 5: Soft Peer Separation ────────────────────────────────────
        double nudgeX = 0.0;
        double nudgeY = 0.0;
        boolean nudged = false;
        if (peerRobotPoses != null) {
            for (Pose2d peerPose : peerRobotPoses) {
                if (peerPose == null) continue;
                double dist = pose.getTranslation().getDistance(peerPose.getTranslation());
                if (dist > 0.05 && dist < SEPARATION_RADIUS_M) {
                    Translation2d diff = pose.getTranslation().minus(peerPose.getTranslation());
                    double scale = (SEPARATION_RADIUS_M - dist) / SEPARATION_RADIUS_M;
                    Translation2d nudge = diff.div(dist).times(scale * SEPARATION_NUDGE_MPS);
                    nudgeX += nudge.getX();
                    nudgeY += nudge.getY();
                    nudged = true;
                }
            }
        }
        if (nudged) {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    base.vxMetersPerSecond + nudgeX,
                    base.vyMetersPerSecond + nudgeY,
                    base.omegaRadiansPerSecond);
            return new RecoveryResult(speeds, RecoveryTier.PEER_SEPARATION, null);
        }

        // ── Tier 6: Nominal Trajectory Speeds ───────────────────────────────
        return new RecoveryResult(base, RecoveryTier.NONE, null);
    }
}
