package frc.robot.Intelligence.utility.ast;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.PolicyWeights;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;

/**
 * Proves that {@link ObjectiveExpressions#rushClimb} reproduces the live
 * engine's {@code RUSH_CLIMB} value across a state grid — the fourteenth
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>Pins both ramp arms and every boundary (0/15/20 s, climber fitted or
 * not, teleop or autonomous). No post-processing touches climb, so nothing is
 * skipped.
 */
public class RushClimbParityTest {

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.rushClimb(weights);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (double timeLeft : new double[] {0.0, 5.0, 14.0, 15.0, 16.0, 19.0, 20.0, 25.0, 100.0}) {
            for (boolean hasClimber : new boolean[] {true, false}) {
                checked += check(engine, expression, weights, timeLeft, hasClimber, false);
            }
        }
        // Autonomous kills the role factor on an otherwise open cell.
        checked += check(engine, expression, weights, 10.0, true, true);
        assertTrue(checked > 15, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            double timeLeft, boolean hasClimber, boolean autonomous) {
        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), 0,
                new Pose2d(), new ChassisSpeeds(), timeLeft,
                false, false, 10.0, false, autonomous)
                .withHardware(true, 30, hasClimber);
        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                0, 0, 0, 0, 0,
                List.of(), List.of(), List.of(), List.of(), 0, 0, 0);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.AUTONOMOUS_CYCLER, weights)
                .get(StrategicObjective.RUSH_CLIMB);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, 8.0));

        assertEquals(engineValue, expressionValue, 1e-9,
                "t=" + timeLeft + " climber=" + hasClimber + " auto=" + autonomous);
        return 1;
    }

    @Test
    void affineArmsRoundTripThroughCodec() {
        assertEquals(Terminal.Tier.OBSERVED, Terminal.HAS_CLIMBER.tier());

        ExpressionNode genome = ObjectiveExpressions.rushClimb(PolicyWeights.DEFAULT);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("affine("), text);
        assertTrue(text.contains("has_climber"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
