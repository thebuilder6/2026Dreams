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
 * Proves that {@link ObjectiveExpressions#denyShootingLane} reproduces the live
 * engine's {@code DENY_SHOOTING_LANE} value across a state grid — the ninth
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>The defender-role factor is assumed 1 (archetype stays engine
 * scaffolding), so the grid runs {@code ADAPTIVE_COMPETITOR}, whose
 * post-processing leaves the lane value untouched. The range terminal reads
 * the nearest listed opponent while the engine reads the tracked pose — the
 * grid keeps the two equal, as production does.
 */
public class DenyShootingLaneParityTest {

    private static final double DIST_NORM = 8.0;

    // Distances to the Red hub (11.9154, 4.0346): inside, near the edge, outside.
    private static final Pose2d OPP_CLOSE = new Pose2d(11.0, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d OPP_EDGE = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d OPP_FAR = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.denyShootingLane(weights, DIST_NORM);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (Pose2d oppPose : new Pose2d[] {OPP_CLOSE, OPP_EDGE, OPP_FAR}) {
            for (boolean oppHubActive : new boolean[] {true, false}) {
                // Tracked: the listed pose equals the tracked pose.
                checked += check(engine, expression, weights, oppPose, List.of(oppPose), oppHubActive);
                // Untracked: no opponent tracks, so denial reads 0.
                checked += check(engine, expression, weights, oppPose, List.of(), oppHubActive);
            }
        }
        assertTrue(checked > 10, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            Pose2d oppPose, List<Pose2d> oppPoses, boolean oppHubActive) {
        MatchKnowledge knowledge = oppPoses.isEmpty() ? ObservedKnowledge.selfOnly()
                : new ClairvoyantKnowledge(0, 0, 0, 0, 0,
                        List.of(), oppPoses, List.of(), List.of(), 0, 0, 0);
        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), 0,
                oppPose, new ChassisSpeeds(), 100.0,
                false, oppHubActive, 10.0, false, false);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.ADAPTIVE_COMPETITOR, weights)
                .get(StrategicObjective.DENY_SHOOTING_LANE);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

        assertEquals(engineValue, expressionValue, 1e-9,
                "opp=" + oppPose.getTranslation() + " tracked=" + !oppPoses.isEmpty()
                        + " oppHub=" + oppHubActive);
        return 1;
    }

    @Test
    void observedTerminalTierAndCodecRoundTrip() {
        assertEquals(Terminal.Tier.OBSERVED, Terminal.OPPONENT_OBSERVED.tier());

        ExpressionNode genome = ObjectiveExpressions.denyShootingLane(PolicyWeights.DEFAULT);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("opponent_observed"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
