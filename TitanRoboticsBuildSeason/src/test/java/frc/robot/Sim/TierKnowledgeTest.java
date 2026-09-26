package frc.robot.Sim;

import frc.robot.Intelligence.State.WorldState;

import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.Sparring.AIRobotSim;
import frc.robot.Intelligence.Sparring.AIRobotInstance;
import frc.robot.Intelligence.State.MatchScoreTracker;
import frc.robot.Intelligence.State.WorldStateBuilder;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Telemetry.Dashboard;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Subsystems.SwerveBase;

/**
 * Two-tier information philosophy: the driver-assist tier knows only what the
 * robot could perceive itself (opponent unobserved), while the sim-sparring
 * tier additionally knows player-visible match context (score, sides).
 */
public class TierKnowledgeTest {

    private JevDecisionEngine engine;

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        engine = JevDecisionEngine.getInstance();
        MatchScoreTracker.getInstance().reset();
        SwerveBase.getInstance();
    }

    private static WorldState world(Pose2d self, int held, Pose2d opp) {
        return new WorldState(
                self, new ChassisSpeeds(), held,
                opp, new ChassisSpeeds(),
                100.0, true, true, 15.0, false, false);
    }

    private static MatchKnowledge knowledgeWithDiff(int differential) {
        return new MatchKnowledge(true, differential, 0, 0, 0, 0,
                List.of(), List.of(), List.of(), List.of());
    }

    @Test
    public void unobservedCoPilotAssumesClearLaneAndFires() {
        // Opponent parked directly in the shooting lane.
        Pose2d self = new Pose2d(3.0, 4.0, new Rotation2d());
        Pose2d laneBlocker = new Pose2d(3.8, 4.02, new Rotation2d());
        WorldState state = world(self, 5, laneBlocker);

        AIActionIntent intent = engine.evaluatePolicy(state, MatchKnowledge.unknown(), Archetype.CO_PILOT);

        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective());
        assertTrue(intent.triggerFeedKicker(),
                "Driver-assist tier leaves lane judgment to the human; geometry valid => fire");
    }

    @Test
    public void unobservedDefenderFallsBackOffOpponentObjectives() {
        Pose2d self = new Pose2d(8.0, 4.0, new Rotation2d());
        Pose2d oppNearHub = new Pose2d(13.5, 4.0, new Rotation2d());
        WorldState state = world(self, 0, oppNearHub);

        AIActionIntent intent = engine.evaluatePolicy(
                state, MatchKnowledge.unknown(), Archetype.DEFENSE_BULLY);

        assertNotEquals(StrategicObjective.LEAD_INTERCEPT, intent.objective());
        assertNotEquals(StrategicObjective.DENY_SHOOTING_LANE, intent.objective());
        assertNotEquals(StrategicObjective.SHADOW_MIDLINE, intent.objective());
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, intent.objective());
    }

    @Test
    public void scoreDifferentialBiasesChaseWhenBehind() {
        Pose2d self = new Pose2d(10.0, 4.0, new Rotation2d());
        Pose2d opp = new Pose2d(13.0, 4.0, new Rotation2d());
        WorldState state = world(self, 20, opp);

        AIActionIntent behind = engine.evaluatePolicy(state, knowledgeWithDiff(-10),
                Archetype.AUTONOMOUS_CYCLER);
        AIActionIntent ahead = engine.evaluatePolicy(state, knowledgeWithDiff(10),
                Archetype.AUTONOMOUS_CYCLER);
        AIActionIntent level = engine.evaluatePolicy(state, MatchKnowledge.legacyObserved(),
                Archetype.AUTONOMOUS_CYCLER);

        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, behind.objective());
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, ahead.objective());
        assertEquals(0.99, behind.confidence(), 1e-9);
        assertEquals(0.98, ahead.confidence(), 1e-9);
        assertEquals(0.98, level.confidence(), 1e-9);
    }

    @Test
    public void simBotKnowledgePopulatesScoreAndSides() {
        // Fresh arena: only the (blue) player is on the carpet.
        MatchKnowledge empty = WorldStateBuilder.buildMatchKnowledgeForSimBot(false);

        assertTrue(empty.opponentObserved());
        assertEquals(0, empty.scoreDifferential());
        assertEquals(1, empty.allyPoses().size(), "Player skates with the blue bot");
        assertEquals(1, empty.allyVelocities().size());
        assertTrue(empty.opponentPoses().isEmpty());
        assertEquals(GameSim.getInstance().getHeldBalls(), empty.alliesHeldFuel());

        MatchScoreTracker.getInstance().recordFuelScore(false);
        MatchScoreTracker.getInstance().recordFuelScore(false);
        MatchKnowledge ahead = WorldStateBuilder.buildMatchKnowledgeForSimBot(false);
        assertEquals(2, ahead.scoreDifferential(), "Blue bot leads 2-0");

        MatchKnowledge redView = WorldStateBuilder.buildMatchKnowledgeForSimBot(true);
        assertEquals(-2, redView.scoreDifferential(), "Red sees the same lead mirrored");
        assertEquals(1, redView.opponentPoses().size(), "Player is Red's opponent here");
    }
}
