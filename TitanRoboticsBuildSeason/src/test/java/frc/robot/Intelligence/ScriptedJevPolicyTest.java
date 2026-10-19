package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.ShooterState;

/** Pins ScriptedJevPolicy to the card-verified Python decide() gates. */
public class ScriptedJevPolicyTest {

    private ScriptedJevPolicy policy;

    @BeforeEach
    public void setup() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        policy = new ScriptedJevPolicy();
    }

    @AfterEach
    public void teardown() {
        JevDecisionEngine.resetPolicy();
        PolicyWeights.resetToDefault();
    }

    private static WorldState world(double x, double y, int held, boolean hubLive,
            boolean isAuto) {
        return new WorldState(
                new Pose2d(x, y, new Rotation2d()),
                new ChassisSpeeds(), held,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                140.0, hubLive, true, 0.0,
                false, isAuto, hubLive, true);
    }

    private static MatchKnowledge fuelAt(double x, double y) {
        return new ClairvoyantKnowledge(
                0, 0, 0, 0, 0, List.of(), List.of(), List.of(), List.of(),
                0, 0, 0, List.of(new Translation2d(x, y)));
    }

    private AIActionIntent eval(WorldState w, MatchKnowledge k) {
        return policy.evaluatePolicy(w, k, Archetype.AUTONOMOUS_CYCLER, null, null,
                null, null, PolicyWeights.DEFAULT, 0.0);
    }

    @Test
    public void scoresWhenLoadedAndHubLive() {
        AIActionIntent intent = eval(world(2.0, 4.03, 8, true, false), fuelAt(6.0, 4.0));
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective());
        assertEquals(IntakeState.STANDBY, intent.intakeCommand());
        // Facing the hub dead-on inside the envelope: already firing.
        assertEquals(ShooterState.SHOOTING, intent.shooterCommand());
        assertTrue(intent.triggerFeedKicker());
        assertNotNull(intent.aimOverride());
    }

    @Test
    public void harvestsWhenEmpty() {
        AIActionIntent intent = eval(world(2.0, 4.03, 0, true, false), fuelAt(6.0, 4.0));
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, intent.objective());
        assertEquals(IntakeState.INTAKING, intent.intakeCommand());
        assertEquals(6.0, intent.navigationTarget().getX(), 1e-9);
    }

    @Test
    public void inRangeSingleFuelScores() {
        // Cards B6/O8: inside 4 m a single fuel justifies the trip.
        AIActionIntent intent = eval(world(2.0, 4.03, 1, true, false), fuelAt(6.0, 4.0));
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective());
    }

    @Test
    public void autoBatchGateOverridesRange() {
        // Cards A5/N3 vs A6: autonomous needs the full batch of 8 either way.
        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                eval(world(2.0, 4.03, 7, true, true), fuelAt(6.0, 4.0)).objective());
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB,
                eval(world(2.0, 4.03, 8, true, true), fuelAt(6.0, 4.0)).objective());
    }

    @Test
    public void darkHubHolds() {
        AIActionIntent intent = eval(world(2.0, 4.03, 20, false, false), fuelAt(6.0, 4.0));
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, intent.objective());
        assertEquals(ShooterState.STOPPED, intent.shooterCommand());
    }

    @Test
    public void installsThroughSeam() {
        JevDecisionEngine.setPolicy(policy);
        assertSame(policy, JevDecisionEngine.getPolicy());
        AIActionIntent intent = JevDecisionEngine.getPolicy().evaluatePolicy(
                world(2.0, 4.03, 8, true, false), fuelAt(6.0, 4.0),
                Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective());
        JevDecisionEngine.resetPolicy();
        assertNotSame(policy, JevDecisionEngine.getPolicy());
    }
}
