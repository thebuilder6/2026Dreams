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
import frc.robot.Navigation.FieldMap;

/**
 * Proves that {@link ObjectiveExpressions#cycleScoreHub} reproduces the live
 * engine's {@code CYCLE_SCORE_HUB} value across a state grid — the
 * behaviour-preserving half of the AST conversion, without touching the engine.
 *
 * <p>States that would trip the engine's hand-written post-processing (the
 * harvest-deadline force) are skipped; those overrides are deliberately not part
 * of the expression.
 */
public class CycleScoreHubParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;

    private static Pose2d poseAtDistance(double distanceM) {
        return new Pose2d(
                FieldMap.Hubs.getHubLocation2d(false).plus(new edu.wpi.first.math.geometry.Translation2d(distanceM, 0.0)),
                Rotation2d.fromDegrees(180));
    }

    private static EvalContext context(int held, double distM, boolean hubActive, int scoreDiff, double timeUntilShift) {
        return EvalContext.builder()
                .set(Terminal.HELD_RATIO, held / (double) BALL_CAPACITY)
                .set(Terminal.DIST_TO_HUB, 1.0 - distM / DIST_NORM)
                .set(Terminal.TIME_UNTIL_SHIFT, timeUntilShift / 20.0)
                .set(Terminal.SCORE_DIFF, 0.5 + scoreDiff / 2.0)
                .setFlag(Terminal.MY_HUB_ACTIVE, hubActive)
                .build();
    }

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.cycleScoreHub(weights, BALL_CAPACITY, DIST_NORM);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (int held : new int[] {0, 1, 3, 4, 7, 8, 15, 16, 17, 30}) {
            for (double distM : new double[] {1.0, 2.0, 3.5, 5.0, 7.0}) {
                for (boolean hubActive : new boolean[] {true, false}) {
                    for (int diff : new int[] {0, -20, 10}) {
                        for (double timeUntilShift : new double[] {0.0, 3.0, 10.0}) {
                            // Skip the harvest-deadline force: engine sets score to a
                            // flat value when hub live, held >= 8, and no time left.
                            double timeLeftToHarvest = timeUntilShift - distM / 3.2;
                            if (hubActive && held >= 8 && timeLeftToHarvest <= 0.0) {
                                continue;
                            }
                            WorldState world = new WorldState(
                                    poseAtDistance(distM), new ChassisSpeeds(), held,
                                    new Pose2d(), new ChassisSpeeds(), 100.0,
                                    hubActive, !hubActive, timeUntilShift, false, false);
                            ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                                    diff, held, 0, held, 0,
                                    List.of(), List.of(), List.of(), List.of(), 0, 0, 0);

                            double engineValue = engine.evaluateUtilityScores(
                                    world, Collections.emptySet(), knowledge,
                                    Archetype.AUTONOMOUS_CYCLER, weights)
                                    .get(StrategicObjective.CYCLE_SCORE_HUB);
                            double expressionValue = expression.evaluate(
                                    context(held, distM, hubActive, diff, timeUntilShift));

                            String label = "held=" + held + " d=" + distM + " hub=" + hubActive
                                    + " diff=" + diff + " tus=" + timeUntilShift;
                            assertEquals(engineValue, expressionValue, 1e-9, label);
                            checked++;
                        }
                    }
                }
            }
        }
        assertTrue(checked > 200, "grid should cover many states, was " + checked);
    }

    @Test
    void scaleAndInclusiveThresholdRoundTrip() {
        ExpressionNode genome = new ExpressionNode.Scale(
                new ExpressionNode.Threshold(
                        new ExpressionNode.TerminalNode(Terminal.HELD_RATIO), 0.5, 0.0, 1.0, true),
                1.5);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("threshold_ge("), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
        EvalContext atThreshold = EvalContext.builder().set(Terminal.HELD_RATIO, 0.5).build();
        assertEquals(1.0, parsed.evaluate(atThreshold), 1e-12, "inclusive gate passes at equality");
    }
}
