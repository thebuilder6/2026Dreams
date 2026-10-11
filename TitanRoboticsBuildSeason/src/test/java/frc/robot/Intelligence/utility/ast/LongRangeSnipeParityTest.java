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
import frc.robot.Navigation.FieldMap;

/**
 * Proves that {@link ObjectiveExpressions#longRangeSnipe} reproduces the live
 * engine's {@code LONG_RANGE_SNIPE} value across a state grid — the eighth
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>The close/far branch reads the tracked opponent pose through the new
 * {@code OPP_NEAR_MY_HUB} terminal; the unobserved grid row pins the far
 * fallback even when the tracked pose sits on the hub. No post-processing
 * touches snipe on the covered archetype, so nothing is skipped.
 */
public class LongRangeSnipeParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;

    // Blue hub (4.6256, 4.0346): P1 short of the band, P2 inside it, P3 past it.
    private static final Pose2d POSE_NEAR = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d POSE_BAND = new Pose2d(0.5, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d POSE_FAR = new Pose2d(9.0, 4.0, Rotation2d.fromDegrees(0));

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.longRangeSnipe(weights, BALL_CAPACITY, DIST_NORM);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();
        Pose2d contestPose = new Pose2d(
                FieldMap.Hubs.getHubLocation2d(false).plus(new edu.wpi.first.math.geometry.Translation2d(1.0, 0.0)),
                Rotation2d.fromDegrees(0));
        Pose2d clearPose = new Pose2d(12.0, 4.0, Rotation2d.fromDegrees(0));

        int checked = 0;
        for (Pose2d pose : new Pose2d[] {POSE_NEAR, POSE_BAND, POSE_FAR}) {
            for (int held : new int[] {0, 5, 6, 30}) {
                for (boolean hubActive : new boolean[] {true, false}) {
                    // Contested hub.
                    checked += check(engine, expression, weights, pose, contestPose, List.of(contestPose),
                            held, hubActive, true);
                    // Clear hub.
                    checked += check(engine, expression, weights, pose, clearPose, List.of(clearPose),
                            held, hubActive, true);
                    // Unobserved: far branch even though the tracked pose contests the hub.
                    checked += check(engine, expression, weights, pose, contestPose, List.of(),
                            held, hubActive, false);
                }
            }
        }
        // Shooter gate: no fitted shooter kills it on an otherwise open cell.
        checked += checkShooter(engine, expression, weights, POSE_BAND, contestPose);
        assertTrue(checked > 60, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            Pose2d pose, Pose2d oppPose, List<Pose2d> oppPoses, int held, boolean hubActive,
            boolean expectObserved) {
        MatchKnowledge knowledge = oppPoses.isEmpty() ? ObservedKnowledge.selfOnly()
                : new ClairvoyantKnowledge(0, held, 0, held, 0,
                        List.of(), oppPoses, List.of(), List.of(), 0, 0, 0);
        return checkWith(engine, expression, weights, knowledge, pose, oppPose, held, hubActive, true,
                "opp=" + (oppPoses.isEmpty() ? "unobserved" : oppPose.getTranslation()));
    }

    private static int checkShooter(JevDecisionEngine engine, ExpressionNode expression,
            PolicyWeights weights, Pose2d pose, Pose2d oppPose) {
        MatchKnowledge knowledge = new ClairvoyantKnowledge(0, 6, 0, 6, 0,
                List.of(), List.of(oppPose), List.of(), List.of(), 0, 0, 0);
        return checkWith(engine, expression, weights, knowledge, pose, oppPose, 6, true, false,
                "shooter=false");
    }

    private static int checkWith(JevDecisionEngine engine, ExpressionNode expression,
            PolicyWeights weights, MatchKnowledge knowledge, Pose2d pose, Pose2d oppPose,
            int held, boolean hubActive, boolean hasShooter, String label) {
        WorldState world = new WorldState(
                pose, new ChassisSpeeds(), held,
                oppPose, new ChassisSpeeds(), 100.0,
                hubActive, false, 10.0, false, false)
                .withHardware(hasShooter, BALL_CAPACITY, false);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.AUTONOMOUS_CYCLER, weights)
                .get(StrategicObjective.LONG_RANGE_SNIPE);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

        assertEquals(engineValue, expressionValue, 1e-9,
                "pose=" + pose.getTranslation() + " held=" + held + " hub=" + hubActive + " " + label);
        return 1;
    }

    @Test
    void contestTerminalIsClairvoyantAndRoundTripsThroughCodec() {
        assertEquals(Terminal.Tier.CLAIRVOYANT, Terminal.OPP_NEAR_MY_HUB.tier());

        ExpressionNode genome = ObjectiveExpressions.longRangeSnipe(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("opp_near_my_hub"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
