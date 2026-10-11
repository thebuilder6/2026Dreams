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
 * Proves that {@link ObjectiveExpressions#chokeTrench} reproduces the live
 * engine's {@code CHOKE_TRENCH} value across a state grid — the twelfth
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>The defender role is assumed; the grid runs {@code ADAPTIVE_COMPETITOR}
 * with the clock out of the G420 window, both of which leave the choke value
 * untouched. The trench terminal reads the nearest listed opponent while the
 * engine reads the tracked pose — the grid keeps the two equal.
 */
public class ChokeTrenchParityTest {

    private static final double DIST_NORM = 8.0;

    // (4.0, 0.65): trench X band [3.2, 6.1] and south of y 1.279. (9, 4): open.
    private static final Pose2d OPP_TRENCH = new Pose2d(4.0, 0.65, Rotation2d.fromDegrees(0));
    private static final Pose2d OPP_CLEAR = new Pose2d(9.0, 4.0, Rotation2d.fromDegrees(0));

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.chokeTrench(weights);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (Pose2d oppPose : new Pose2d[] {OPP_TRENCH, OPP_CLEAR}) {
            for (boolean hubActive : new boolean[] {true, false}) {
                // Tracked: the listed pose equals the tracked pose.
                checked += check(engine, expression, weights, oppPose, List.of(oppPose),
                        hubActive, false);
                // Untracked: no opponent tracks, so choking reads 0.
                checked += check(engine, expression, weights, oppPose, List.of(),
                        hubActive, false);
            }
        }
        // Teleop gate: autonomous kills it on an otherwise open cell.
        checked += check(engine, expression, weights, OPP_TRENCH, List.of(OPP_TRENCH), false, true);
        assertTrue(checked > 8, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            Pose2d oppPose, List<Pose2d> oppPoses, boolean hubActive, boolean autonomous) {
        MatchKnowledge knowledge = oppPoses.isEmpty() ? ObservedKnowledge.selfOnly()
                : new ClairvoyantKnowledge(0, 0, 0, 0, 0,
                        List.of(), oppPoses, List.of(), List.of(), 0, 0, 0);
        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), 0,
                oppPose, new ChassisSpeeds(), 100.0,
                hubActive, false, 10.0, false, autonomous);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.ADAPTIVE_COMPETITOR, weights)
                .get(StrategicObjective.CHOKE_TRENCH);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

        assertEquals(engineValue, expressionValue, 1e-9,
                "opp=" + oppPose.getTranslation() + " tracked=" + !oppPoses.isEmpty()
                        + " hub=" + hubActive + " auto=" + autonomous);
        return 1;
    }

    @Test
    void trenchGateRoundTripsThroughCodec() {
        ExpressionNode genome = ObjectiveExpressions.chokeTrench(PolicyWeights.DEFAULT);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("opp_in_trench"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
