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
import frc.robot.Intelligence.ObservedKnowledge;
import frc.robot.Intelligence.PolicyWeights;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;
import frc.robot.Navigation.FieldMap;

/**
 * Proves that {@link ObjectiveExpressions#sweepAllianceZone} reproduces the
 * live engine's {@code SWEEP_ALLIANCE_ZONE} value across a state grid — the
 * fourth behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>States that would trip the engine's hand-written post-processing (the
 * harvest-deadline force, which zeroes sweep) are skipped. Unlike vacuum, the
 * fuel gate is tier-clean, so the grid also covers the observed tier.
 */
public class SweepAllianceZoneParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;
    private static final double DIST_M = 1.0;

    private static Pose2d poseAtDistance(double distanceM) {
        return new Pose2d(
                FieldMap.Hubs.getHubLocation2d(false).plus(new edu.wpi.first.math.geometry.Translation2d(distanceM, 0.0)),
                Rotation2d.fromDegrees(180));
    }

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.sweepAllianceZone(weights, BALL_CAPACITY);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (int held : new int[] {0, 1, 15, 29, 30}) {
            for (boolean hubActive : new boolean[] {true, false}) {
                for (double tus : new double[] {0.0, 10.0}) {
                    for (int home : new int[] {0, 5}) {
                        // Skip the harvest-deadline force: engine zeroes sweep when
                        // hub live, held >= 8, and no time left to harvest.
                        double timeLeftToHarvest = tus - DIST_M / 3.2;
                        if (hubActive && held >= 8 && timeLeftToHarvest <= 0.0) {
                            continue;
                        }
                        // Clairvoyant: home count flows through the knowledge.
                        checked += check(engine, expression, weights,
                                new ClairvoyantKnowledge(0, held, 0, held, 0,
                                        List.of(), List.of(), List.of(), List.of(), home, 0, 0),
                                held, hubActive, tus, "clairvoyant home=" + home);
                        // Observed: zone counts are honestly zero, so sweep must be 0.
                        checked += check(engine, expression, weights, ObservedKnowledge.selfOnly(),
                                held, hubActive, tus, "observed");
                    }
                }
            }
        }
        assertTrue(checked > 60, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            frc.robot.Intelligence.MatchKnowledge knowledge,
            int held, boolean hubActive, double tus, String label) {
        WorldState world = new WorldState(
                poseAtDistance(DIST_M), new ChassisSpeeds(), held,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                hubActive, !hubActive, tus, false, false);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.AUTONOMOUS_CYCLER, weights)
                .get(StrategicObjective.SWEEP_ALLIANCE_ZONE);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

        assertEquals(engineValue, expressionValue, 1e-9,
                "held=" + held + " hub=" + hubActive + " tus=" + tus + " " + label);
        return 1;
    }

    @Test
    void activeValueAndGatesRoundTripThroughCodec() {
        ExpressionNode genome = ObjectiveExpressions.sweepAllianceZone(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("alliance_fuel"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));
    }
}
