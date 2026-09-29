package frc.robot.Intelligence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Immutable snapshot capturing all ground truth field information required for
 * System 2 (Executive Strategy) and System 1 (Tactical Reflex) policy arbitration.
 */
public record WorldState(
        Pose2d selfPose,
        ChassisSpeeds selfVelocity,
        int heldFuelCount,
        Pose2d opponentPose,
        ChassisSpeeds opponentVelocity,
        double matchTimeRemaining,
        boolean isAllianceHubActive,
        boolean isOpponentHubActive,
        double timeUntilHubShift,
        boolean isRedAlliance,
        boolean isAutonomous,
        boolean isAllianceHubActiveAfterShift,
        boolean isOpponentHubActiveAfterShift
) {
    public static final int DEFAULT_MAX_CAPACITY = 30;
    public static final int CO_PILOT_CAPACITY = 30; // Changed from 8 to 30

    /**
     * Backward-compatible 11-parameter constructor defaulting isAutonomous to
     * false and the after-shift flags to "unchanged".
     */
    public WorldState(
            Pose2d selfPose,
            ChassisSpeeds selfVelocity,
            int heldFuelCount,
            Pose2d opponentPose,
            ChassisSpeeds opponentVelocity,
            double matchTimeRemaining,
            boolean isAllianceHubActive,
            boolean isOpponentHubActive,
            double timeUntilHubShift,
            boolean isRedAlliance,
            boolean isAutonomous
    ) {
        this(selfPose, selfVelocity, heldFuelCount, opponentPose, opponentVelocity,
                matchTimeRemaining, isAllianceHubActive, isOpponentHubActive,
                timeUntilHubShift, isRedAlliance, isAutonomous,
                // Default to "no upcoming change". Mirroring the current flags
                // keeps every pre-existing caller behaviourally identical while
                // still describing an honest state: with nothing pending, the
                // post-shift state equals the present one.
                isAllianceHubActive, isOpponentHubActive);
    }

    /**
     * Backward-compatible 10-parameter constructor defaulting isAutonomous to false.
     */
    public WorldState(
            Pose2d selfPose,
            ChassisSpeeds selfVelocity,
            int heldFuelCount,
            Pose2d opponentPose,
            ChassisSpeeds opponentVelocity,
            double matchTimeRemaining,
            boolean isAllianceHubActive,
            boolean isOpponentHubActive,
            double timeUntilHubShift,
            boolean isRedAlliance
    ) {
        this(selfPose, selfVelocity, heldFuelCount, opponentPose, opponentVelocity,
                matchTimeRemaining, isAllianceHubActive, isOpponentHubActive,
                timeUntilHubShift, isRedAlliance, false);
    }

    // Self hub active alias:
    public boolean isSelfHubActive() { return isAllianceHubActive; }

    /**
     * True when a shift flip is actually pending, i.e. this robot's hub is live
     * now but will be dark during the next shift. This is the "last window"
     * signal: the one that should stop a bot from starting a fresh scoring trip.
     */
    public boolean willOwnHubDeactivate() {
        return isAllianceHubActive && !isAllianceHubActiveAfterShift;
    }

    /**
     * True when the opponent's hub is dark now but will be live during the next
     * shift. This is the signal that makes a shuttle or an opponent-zone poach
     * worthwhile: there is somewhere to put the fuel.
     */
    public boolean willOpponentHubActivate() {
        return !isOpponentHubActive && isOpponentHubActiveAfterShift;
    }

    // Inventory fullness check:
    public boolean isInventoryFull() { return heldFuelCount >= DEFAULT_MAX_CAPACITY; }

    // Game-agnostic convenience accessors:
    public int heldGamePieceCount() { return heldFuelCount; }
    public boolean isAllianceGoalActive() { return isAllianceHubActive; }
    public boolean isOpponentGoalActive() { return isOpponentHubActive; }
    public double timeUntilGoalShift() { return timeUntilHubShift; }

    public double inventoryRatio(int capacity) {
        return Math.min(1.0, (double) heldFuelCount / Math.max(1, capacity));
    }
}
