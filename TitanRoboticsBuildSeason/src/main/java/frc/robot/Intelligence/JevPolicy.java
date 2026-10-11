package frc.robot.Intelligence;

import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Outsourcing seam for the Jev decision policy.
 *
 * <p>Production callers ({@code AIRobotInstance}, {@code AutonomousTeleopAgent},
 * {@code MatchCoach}, {@code StandaloneBot}) evaluate through this interface via
 * {@link JevDecisionEngine#getPolicy()} instead of the concrete engine, so a
 * training repo can substitute its own policy (recorded, learned, or remote)
 * without forking robot code. The default policy is the local
 * {@link JevDecisionEngine} singleton — behavior is unchanged until
 * {@link JevDecisionEngine#setPolicy} is called.
 *
 * <p>shortcut: process-wide static holder, not per-bot injection — upgrade when
 * heterogeneous policies per bot are needed.
 */
public interface JevPolicy {
    /**
     * Full evaluation: the single method every implementation must provide.
     * Shorter overloads below delegate here.
     */
    AIActionIntent evaluatePolicy(
            WorldState world,
            MatchKnowledge knowledge,
            Archetype archetype,
            String cloudContext,
            Set<Translation2d> blockedFuel,
            ObjectiveCommitment commitmentIn,
            FuelTargetMemory fuelTargetMemory,
            PolicyWeights weights,
            double nowSeconds);

    /** Full evaluation on the wall clock. */
    default AIActionIntent evaluatePolicy(
            WorldState world,
            MatchKnowledge knowledge,
            Archetype archetype,
            String cloudContext,
            Set<Translation2d> blockedFuel,
            ObjectiveCommitment commitmentIn,
            FuelTargetMemory fuelTargetMemory,
            PolicyWeights weights) {
        return evaluatePolicy(world, knowledge, archetype, cloudContext, blockedFuel,
                commitmentIn, fuelTargetMemory, weights, Timer.getFPGATimestamp());
    }

    /** Full evaluation with active weights on the wall clock (production path). */
    default AIActionIntent evaluatePolicy(
            WorldState world,
            MatchKnowledge knowledge,
            Archetype archetype,
            String cloudContext,
            Set<Translation2d> blockedFuel,
            ObjectiveCommitment commitmentIn,
            FuelTargetMemory fuelTargetMemory) {
        return evaluatePolicy(world, knowledge, archetype, cloudContext, blockedFuel,
                commitmentIn, fuelTargetMemory, PolicyWeights.getActive(),
                Timer.getFPGATimestamp());
    }

    /** Stateless evaluation with no caller latches (coach, unit tests). */
    default AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype) {
        return evaluatePolicy(world, knowledge, archetype, null, null, null, null,
                PolicyWeights.getActive(), Timer.getFPGATimestamp());
    }
}
