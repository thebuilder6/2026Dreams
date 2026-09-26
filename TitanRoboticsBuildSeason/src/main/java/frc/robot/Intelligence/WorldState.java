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
        boolean isAutonomous
) {
    public static final int DEFAULT_MAX_CAPACITY = 30;
    public static final int CO_PILOT_CAPACITY = 30; // Changed from 8 to 30

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
