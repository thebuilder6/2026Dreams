package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.util.Set;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.ShooterState;

/**
 * Pins the {@link JevPolicy} outsourcing seam: production defaults to the local
 * engine, and a substituted policy is what callers actually evaluate through.
 */
class JevPolicySeamTest {

    /** Canned policy standing in for a training-repo implementation. */
    private static final class StubPolicy implements JevPolicy {
        private final AIActionIntent canned;

        StubPolicy(AIActionIntent canned) {
            this.canned = canned;
        }

        @Override
        public AIActionIntent evaluatePolicy(
                WorldState world,
                MatchKnowledge knowledge,
                Archetype archetype,
                String cloudContext,
                Set<Translation2d> blockedFuel,
                ObjectiveCommitment commitmentIn,
                FuelTargetMemory fuelTargetMemory,
                PolicyWeights weights,
                double nowSeconds) {
            return canned;
        }
    }

    @AfterEach
    void resetPolicy() {
        JevDecisionEngine.resetPolicy();
    }

    @Test
    void defaultPolicyIsLocalEngine() {
        assertSame(JevDecisionEngine.getInstance(), JevDecisionEngine.getPolicy());
    }

    @Test
    void substitutedPolicyIsWhatCallersEvaluate() {
        AIActionIntent canned = new AIActionIntent(
                StrategicObjective.VACUUM_MIDFIELD,
                new Pose2d(),
                new Rotation2d(),
                IntakeState.DISABLED,
                ShooterState.STOPPED,
                0.0,
                false,
                1.0,
                "stub",
                new StrategicPlan(
                        StrategicObjective.VACUUM_MIDFIELD,
                        StrategicObjective.VACUUM_MIDFIELD,
                        0.0));
        JevDecisionEngine.setPolicy(new StubPolicy(canned));

        WorldState world = new WorldState(
                new Pose2d(), new ChassisSpeeds(), 0,
                new Pose2d(), new ChassisSpeeds(),
                135.0, false, false, 0.0, false, false, false, false);
        AIActionIntent got = JevDecisionEngine.getPolicy().evaluatePolicy(
                world, ObservedKnowledge.selfOnly(), Archetype.CO_PILOT);

        assertSame(canned, got);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, got.objective());
    }

    @Test
    void resetRestoresLocalEngine() {
        AIActionIntent canned = new AIActionIntent(
                StrategicObjective.VACUUM_MIDFIELD,
                new Pose2d(),
                new Rotation2d(),
                IntakeState.DISABLED,
                ShooterState.STOPPED,
                0.0,
                false,
                1.0,
                "stub",
                new StrategicPlan(
                        StrategicObjective.VACUUM_MIDFIELD,
                        StrategicObjective.VACUUM_MIDFIELD,
                        0.0));
        JevDecisionEngine.setPolicy(new StubPolicy(canned));
        JevDecisionEngine.resetPolicy();

        assertSame(JevDecisionEngine.getInstance(), JevDecisionEngine.getPolicy());
    }
}
