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
 * Proves that {@link ObjectiveExpressions#stockpileDepot} reproduces the live
 * engine's {@code STOCKPILE_DEPOT} value across a state grid — the fifth
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>The formula reads no fuel count and no clock, so one hub flag plus a fill
 * sweep covers it; no post-processing touches stockpile on the covered
 * archetype, so nothing is skipped.
 */
public class StockpileDepotParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.stockpileDepot(weights, BALL_CAPACITY);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (int held : new int[] {0, 1, 7, 15, 22, 29, 30}) {
            for (boolean hubActive : new boolean[] {true, false}) {
                WorldState world = new WorldState(
                        new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), held,
                        new Pose2d(), new ChassisSpeeds(), 100.0,
                        hubActive, !hubActive, 10.0, false, false);
                ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                        0, held, 0, held, 0,
                        List.of(), List.of(), List.of(), List.of(), 0, 0, 0);

                double engineValue = engine.evaluateUtilityScores(
                        world, Collections.emptySet(), knowledge,
                        Archetype.AUTONOMOUS_CYCLER, weights)
                        .get(StrategicObjective.STOCKPILE_DEPOT);
                double expressionValue = expression.evaluate(
                        WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

                assertEquals(engineValue, expressionValue, 1e-9,
                        "held=" + held + " hub=" + hubActive);
                checked++;
            }
        }
        assertTrue(checked > 10, "grid should cover many states, was " + checked);
    }

    @Test
    void rampAndGatesRoundTripThroughCodec() {
        ExpressionNode genome = ObjectiveExpressions.stockpileDepot(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("held_ratio"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
