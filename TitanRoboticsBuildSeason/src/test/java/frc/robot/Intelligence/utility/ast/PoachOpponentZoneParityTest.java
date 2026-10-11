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
 * Proves that {@link ObjectiveExpressions#poachOpponentZone} reproduces the
 * live engine's {@code POACH_OPPONENT_ZONE} value across a state grid — the
 * seventh behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>The objective is deprecated and never wins the argmax
 * ({@code testPoachUtilityIsCurrentlyUnreachable}), but the formula is live and
 * worth pinning. No post-processing touches poach on the covered archetype, so
 * nothing is skipped; the fuel gate is tier-clean, so the grid covers the
 * observed tier too.
 */
@SuppressWarnings("deprecation") // poach weights are deprecated but the formula is live
public class PoachOpponentZoneParityTest {

    private static final int BALL_CAPACITY = 30;

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.poachOpponentZone(weights, BALL_CAPACITY);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (int held : new int[] {0, 19, 20, 21}) {
            for (double tus : new double[] {0.0, 3.0, 6.0, 10.0}) {
                for (int oppFuel : new int[] {0, 5}) {
                    for (boolean oppNext : new boolean[] {false, true}) {
                        checked += check(engine, expression, weights,
                                new ClairvoyantKnowledge(0, held, 0, held, 0,
                                        List.of(), List.of(), List.of(), List.of(), 0, 0, oppFuel),
                                held, tus, oppNext, false);
                    }
                }
            }
        }
        // Teleop gate: autonomous kills it.
        checked += check(engine, expression, weights,
                new ClairvoyantKnowledge(0, 0, 0, 0, 0,
                        List.of(), List.of(), List.of(), List.of(), 0, 0, 5),
                0, 6.0, true, true);
        // Observed tier: opponent count honestly zero, so poach reads 0.
        for (int held : new int[] {0, 19}) {
            checked += check(engine, expression, weights, ObservedKnowledge.selfOnly(), held, 6.0, true, false);
        }
        assertTrue(checked > 50, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            MatchKnowledge knowledge, int held, double tus, boolean oppNext, boolean autonomous) {
        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), held,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                false, false, tus, false, autonomous,
                false, oppNext);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.AUTONOMOUS_CYCLER, weights)
                .get(StrategicObjective.POACH_OPPONENT_ZONE);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, 8.0));

        String label = "held=" + held + " tus=" + tus + " oppNext=" + oppNext + " auto=" + autonomous;
        assertEquals(engineValue, expressionValue, 1e-9, label);
        return 1;
    }

    @Test
    void windowAndCapGatesRoundTripThroughCodec() {
        ExpressionNode genome = ObjectiveExpressions.poachOpponentZone(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("opponent_fuel"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
