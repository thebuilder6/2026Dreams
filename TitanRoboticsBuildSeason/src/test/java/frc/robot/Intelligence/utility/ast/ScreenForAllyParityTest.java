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
 * Proves that {@link ObjectiveExpressions#screenForAlly} reproduces the live
 * engine's {@code SCREEN_FOR_ALLY} value across a state grid — the thirteenth
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>Covers the base gates and the 0.99 emergency-escort override, including
 * both ally-fuel thresholds (12/20), the self-hold boundary (8), and the
 * observed tier (no tracks, so screening reads 0). No post-processing touches
 * screen on the covered archetype, so nothing is skipped.
 */
public class ScreenForAllyParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;

    private static final Pose2d ALLY = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d OPP_NEAR = new Pose2d(3.5, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d OPP_FAR = new Pose2d(12.0, 4.0, Rotation2d.fromDegrees(0));

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.screenForAlly(weights, BALL_CAPACITY);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (int alliesHeld : new int[] {0, 11, 12, 19, 20, 25}) {
            for (int held : new int[] {0, 8, 9}) {
                for (boolean hubActive : new boolean[] {true, false}) {
                    // Ally in distress.
                    checked += check(engine, expression, weights, alliesHeld, held, hubActive,
                            List.of(ALLY), List.of(OPP_NEAR), true);
                    // Opponent too far: no distress.
                    checked += check(engine, expression, weights, alliesHeld, held, hubActive,
                            List.of(ALLY), List.of(OPP_FAR), false);
                }
            }
        }
        // Observed tier: no tracks, so screening reads 0 even with fuel aboard.
        for (int alliesHeld : new int[] {0, 25}) {
            WorldState world = new WorldState(
                    new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), 8,
                    new Pose2d(), new ChassisSpeeds(), 100.0,
                    true, false, 10.0, false, false);
            MatchKnowledge knowledge = ObservedKnowledge.selfOnly();
            double engineValue = engine.evaluateUtilityScores(
                    world, Collections.emptySet(), knowledge,
                    Archetype.AUTONOMOUS_CYCLER, weights)
                    .get(StrategicObjective.SCREEN_FOR_ALLY);
            double expressionValue = expression.evaluate(
                    WorldStateTerminals.toContext(world, knowledge, DIST_NORM));
            assertEquals(engineValue, expressionValue, 1e-9, "observed alliesHeld=" + alliesHeld);
            checked++;
        }
        assertTrue(checked > 60, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            int alliesHeld, int held, boolean hubActive, List<Pose2d> allies, List<Pose2d> opponents,
            boolean distress) {
        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), held,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                hubActive, false, 10.0, false, false);
        MatchKnowledge knowledge = new ClairvoyantKnowledge(0, alliesHeld, 0, 0, 0,
                allies, opponents, List.of(), List.of(), 0, 0, 0);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.AUTONOMOUS_CYCLER, weights)
                .get(StrategicObjective.SCREEN_FOR_ALLY);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

        assertEquals(engineValue, expressionValue, 1e-9,
                "allies=" + alliesHeld + " held=" + held + " hub=" + hubActive
                        + " distress=" + distress);
        return 1;
    }

    @Test
    void escortOverrideRoundTripsThroughCodec() {
        ExpressionNode genome = ObjectiveExpressions.screenForAlly(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("ally_in_distress"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
