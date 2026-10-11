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
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.ObservedKnowledge;
import frc.robot.Intelligence.PolicyWeights;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;

/**
 * Proves that {@link ObjectiveExpressions#shadowMidline} and
 * {@link ObjectiveExpressions#leadIntercept} reproduce the live engine's
 * {@code SHADOW_MIDLINE} and {@code LEAD_INTERCEPT} values across a state
 * grid — the tenth and eleventh behaviour-preserving AST conversions, without
 * touching the engine.
 *
 * <p>Both formulas are a flat weight gated on opponent tracks, so the grid
 * proves the constant plus the gate plus independence from the rest of the
 * state. The defender role is assumed; the grid runs
 * {@code ADAPTIVE_COMPETITOR} with the clock out of the G420 window.
 */
public class ShadowInterceptParityTest {

    @Test
    void shadowMatchesTheEngineAcrossAGrid() {
        checkAll(ObjectiveExpressions.shadowMidline(PolicyWeights.DEFAULT),
                StrategicObjective.SHADOW_MIDLINE, "shadow");
    }

    @Test
    void interceptMatchesTheEngineAcrossAGrid() {
        checkAll(ObjectiveExpressions.leadIntercept(PolicyWeights.DEFAULT),
                StrategicObjective.LEAD_INTERCEPT, "intercept");
    }

    private static void checkAll(ExpressionNode expression, StrategicObjective objective, String label) {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (int held : new int[] {0, 30}) {
            for (boolean hubActive : new boolean[] {true, false}) {
                // Tracked opponent.
                checked += check(engine, expression, weights, objective,
                        new ClairvoyantKnowledge(0, held, 0, held, 0,
                                List.of(), List.of(new Pose2d(9.0, 4.0, Rotation2d.fromDegrees(0))),
                                List.of(), List.of(), 0, 0, 0),
                        held, hubActive, label + " tracked");
                // No tracks.
                checked += check(engine, expression, weights, objective,
                        ObservedKnowledge.selfOnly(), held, hubActive, label + " untracked");
            }
        }
        assertTrue(checked > 6, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            StrategicObjective objective, MatchKnowledge knowledge, int held, boolean hubActive,
            String label) {
        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), held,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                hubActive, false, 10.0, false, false);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.ADAPTIVE_COMPETITOR, weights).get(objective);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, 8.0));

        assertEquals(engineValue, expressionValue, 1e-9,
                "held=" + held + " hub=" + hubActive + " " + label);
        return 1;
    }

    @Test
    void bothGenomesRoundTripThroughCodec() {
        for (ExpressionNode genome : List.of(
                ObjectiveExpressions.shadowMidline(PolicyWeights.DEFAULT),
                ObjectiveExpressions.leadIntercept(PolicyWeights.DEFAULT))) {
            String text = SExpressions.write(genome);
            assertTrue(text.contains("opponent_observed"), text);
            assertEquals(text, SExpressions.write(SExpressions.read(text)));
        }
    }
}
