package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class JevDecisionEngineModeTest {
    private final JevDecisionEngine engine = JevDecisionEngine.getInstance();
    private final TypeSafeJevClient cloudClient = TypeSafeJevClient.getInstance();

    @BeforeEach
    void setup() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        cloudClient.setApiKey("");
        SmartDashboard.putBoolean("Features/Use TypeSafe Jev AI", false);
        SmartDashboard.putString("JevAI/DecisionMode", "AUTO_FALLBACK");
        engine.setDecisionMode(JevDecisionEngine.DecisionMode.AUTO_FALLBACK);
    }

    @AfterEach
    void tearDown() {
        SmartDashboard.putBoolean("Features/Use TypeSafe Jev AI", false);
        cloudClient.setApiKey("");
    }

    @Test
    void localModeIsDeterministicAndDoesNotUseCloud() {
        engine.setDecisionMode(JevDecisionEngine.DecisionMode.LOCAL_HEURISTIC);
        SmartDashboard.putBoolean("Features/Use TypeSafe Jev AI", false);
        cloudClient.setApiKey("test-key-must-not-trigger-network");
        WorldState world = world(false);

        AIActionIntent first = engine.evaluatePolicy(world, MatchKnowledge.unknown(), Archetype.CO_PILOT);
        AIActionIntent second = engine.evaluatePolicy(world, MatchKnowledge.unknown(), Archetype.CO_PILOT);

        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, first.objective());
        assertEquals(first.objective(), second.objective());
        assertFalse(SmartDashboard.getBoolean("JevAI/UsingCloudAI", true));
        assertFalse(cloudClient.isRequestInFlight());
    }

    @Test
    void autoFallbackWithoutKeyUsesLocalHeuristics() {
        engine.setDecisionMode(JevDecisionEngine.DecisionMode.AUTO_FALLBACK);
        SmartDashboard.putBoolean("Features/Use TypeSafe Jev AI", true);

        AIActionIntent intent = engine.evaluatePolicy(world(false), MatchKnowledge.unknown(), Archetype.CO_PILOT);

        assertNotNull(intent);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective());
        assertFalse(SmartDashboard.getBoolean("JevAI/UsingCloudAI", true));
        assertFalse(cloudClient.hasValidKey());
    }

    @Test
    void autonomousCloudDefenseChoiceIsRejected() {
        WorldState autonomous = world(true);
        Map<StrategicObjective, Double> localUtilities = Map.of(
                StrategicObjective.LEAD_INTERCEPT, 1.0,
                StrategicObjective.CYCLE_SCORE_HUB, 0.5);

        assertFalse(JevDecisionEngine.isCloudObjectiveAdmissible(
                StrategicObjective.LEAD_INTERCEPT, localUtilities, autonomous,
                MatchKnowledge.legacyObserved(), StrategicObjective.CYCLE_SCORE_HUB, 0.5));
        assertTrue(JevDecisionEngine.isCloudObjectiveAdmissible(
                StrategicObjective.CYCLE_SCORE_HUB, localUtilities, autonomous,
                MatchKnowledge.legacyObserved(), StrategicObjective.CYCLE_SCORE_HUB, 0.5),
                "Own-half scoring remains an eligible autonomous choice.");
    }

    @Test
    void cloudCannotReplaceHighPriorityLocalObjective() {
        WorldState teleop = world(false);
        Map<StrategicObjective, Double> localUtilities = Map.of(
                StrategicObjective.CYCLE_SCORE_HUB, 0.98,
                StrategicObjective.SWEEP_ALLIANCE_ZONE, 0.96);

        assertFalse(JevDecisionEngine.isCloudObjectiveAdmissible(
                StrategicObjective.SWEEP_ALLIANCE_ZONE, localUtilities, teleop,
                MatchKnowledge.unknown(), StrategicObjective.CYCLE_SCORE_HUB, 0.98));
        assertTrue(JevDecisionEngine.isCloudObjectiveAdmissible(
                StrategicObjective.CYCLE_SCORE_HUB, localUtilities, teleop,
                MatchKnowledge.unknown(), StrategicObjective.CYCLE_SCORE_HUB, 0.98));
    }

    @Test
    void parsesDocumentedTypeSafeChoiceAndNoulAnswers() throws Exception {
        String response = """
                {"model":"jev-latest","answers":{
                  "strategic_objective":{"type":"choice","choice":"CYCLE_SCORE_HUB","confidence":0.82,
                    "probabilities":{"CYCLE_SCORE_HUB":0.82,"VACUUM_MIDFIELD":0.18}},
                  "opponent_threat_high":{"type":"noul","noul":0.30}},
                 "usage":{"input_tokens":20,"output_tokens":5}}
                """;

        TypeSafeJevClient.JevDecision decision = TypeSafeJevClient.parseDecision(response, System.nanoTime());

        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, decision.objective());
        assertEquals(0.82, decision.confidence(), 1e-9);
        assertEquals(0.18, decision.probabilityDistribution().get("VACUUM_MIDFIELD"), 1e-9);
        assertEquals(0.30, decision.opponentThreatProbability(), 1e-9);
    }

    private static WorldState world(boolean autonomous) {
        return new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                8,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                100.0,
                true,
                false,
                20.0,
                false,
                autonomous);
    }
}
