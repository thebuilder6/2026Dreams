package frc.robot.Intelligence;

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
import frc.robot.Sim.GameSim;
import frc.robot.Sim.MatchScoreTracker;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;
import frc.robot.Intelligence.WorldStateBuilder;
import frc.robot.Navigation.FieldMap;

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

    /**
     * Clairvoyant knowledge carrying only a score differential. Used where the
     * test is about the score bias specifically and the field picture is
     * irrelevant; zone fuel is 0 so zone-driven objectives stay out of the way.
     */
    private static MatchKnowledge knowledgeWithDiff(int differential) {
        return new ClairvoyantKnowledge(differential,
                0, 0, 0, 0,
                List.of(), List.of(), List.of(), List.of(),
                0, 0, 0);
    }

    @Test
    public void unobservedCoPilotAssumesClearLaneAndFires() {
        // Opponent parked directly in the shooting lane.
        Pose2d self = new Pose2d(3.0, 4.0, new Rotation2d());
        Pose2d laneBlocker = new Pose2d(3.8, 4.02, new Rotation2d());
        WorldState state = world(self, 5, laneBlocker);

        AIActionIntent intent = engine.evaluatePolicy(state, ObservedKnowledge.selfOnly(), Archetype.CO_PILOT);

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
                state, ObservedKnowledge.selfOnly(), Archetype.DEFENSE_BULLY);

        assertNotEquals(StrategicObjective.LEAD_INTERCEPT, intent.objective());
        assertNotEquals(StrategicObjective.DENY_SHOOTING_LANE, intent.objective());
        assertNotEquals(StrategicObjective.VACUUM_MIDFIELD, intent.objective(),
                "Unobserved defender must never abandon defense to harvest midfield");
        assertEquals(StrategicObjective.SHADOW_MIDLINE, intent.objective(),
                "Unobserved defender must degrade safely to midline zone defense");
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
        AIActionIntent level = engine.evaluatePolicy(state, ObservedKnowledge.selfOnly(),
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

        assertInstanceOf(ClairvoyantKnowledge.class, empty,
                "A sim sparring bot runs with the operator's full field picture");
        assertEquals(0, empty.scoreDifferential());
        assertEquals(1, empty.allyPoses().size(), "Player skates with the blue bot");
        assertEquals(1, empty.allyVelocities().size());
        assertTrue(empty.opponentPoses().isEmpty());
        assertEquals(GameSim.getInstance().getHeldBalls(), empty.alliesHeldFuel());
        // opponentObserved is now derived from the pose list, not passed in, so it
        // cannot contradict the data it guards.
        assertFalse(empty.opponentObserved(),
                "With no opponent robots on the carpet, nothing is observed");

        MatchScoreTracker.getInstance().recordFuelScore(false);
        MatchScoreTracker.getInstance().recordFuelScore(false);
        MatchKnowledge ahead = WorldStateBuilder.buildMatchKnowledgeForSimBot(false);
        assertEquals(2, ahead.scoreDifferential(), "Blue bot leads 2-0");

        MatchKnowledge redView = WorldStateBuilder.buildMatchKnowledgeForSimBot(true);
        assertEquals(-2, redView.scoreDifferential(), "Red sees the same lead mirrored");
        assertEquals(1, redView.opponentPoses().size(), "Player is Red's opponent here");
        assertTrue(redView.opponentObserved(), "A non-empty pose list implies observed");
    }

    /**
     * The two kinds must be genuinely different, not one record with a flag: only
     * the clairvoyant kind can report field fuel, and the observed kind cannot.
     */
    @Test
    public void observedKnowledgeReportsNoFieldFuelAndNoOpponents() {
        MatchKnowledge observed = ObservedKnowledge.selfOnly();

        assertInstanceOf(ObservedKnowledge.class, observed);
        assertEquals(0, observed.allianceZoneFuel());
        assertEquals(0, observed.midfieldFuel());
        assertEquals(0, observed.opponentZoneFuel());
        assertFalse(observed.opponentObserved());
        assertTrue(observed.opponentPoses().isEmpty());
        assertTrue(observed.opponentVelocities().isEmpty());
    }

    @Test
    public void observedKnowledgeIsTheSafeDefaultForTheEngine() {
        // The 2-arg overload used to default to a record claiming
        // opponentObserved = true with empty lists. It must default to the tier
        // that claims the least, so a caller that forgets knowledge degrades to
        // sensor-only instead of acting on nothing.
        Pose2d self = new Pose2d(8.0, 4.0, new Rotation2d());
        Pose2d oppNearHub = new Pose2d(13.5, 4.0, new Rotation2d());
        WorldState state = world(self, 0, oppNearHub);

        AIActionIntent defaulted = engine.evaluatePolicy(state, Archetype.DEFENSE_BULLY);
        AIActionIntent explicit = engine.evaluatePolicy(state,
                ObservedKnowledge.selfOnly(), Archetype.DEFENSE_BULLY);

        assertEquals(explicit.objective(), defaulted.objective());
        assertNotEquals(StrategicObjective.DENY_SHOOTING_LANE, defaulted.objective(),
                "A defender with no opponent tracker must not pick an opponent objective");
    }

    /** Zone fuel is a clairvoyant-only capability, so a negative count is rejected. */
    @Test
    public void clairvoyantKnowledgeRejectsNegativeZoneCounts() {
        assertThrows(IllegalArgumentException.class, () -> new ClairvoyantKnowledge(
                0, 0, 0, 0, 0, List.of(), List.of(), List.of(), List.of(),
                -1, 0, 0));
    }

    /** The pose lists are copied, so a caller cannot mutate a published snapshot. */
    @Test
    public void clairvoyantKnowledgeCopiesItsLists() {
        java.util.List<Pose2d> poses = new java.util.ArrayList<>();
        poses.add(new Pose2d(1.0, 2.0, new Rotation2d()));
        // Field order after the five scalars is allyPoses, opponentPoses,
        // allyVelocities, opponentVelocities.
        ClairvoyantKnowledge k = new ClairvoyantKnowledge(
                0, 0, 0, 0, 0, List.of(), poses, List.of(), List.of(), 0, 0, 0);

        poses.clear();
        assertEquals(1, k.opponentPoses().size(), "Snapshot must not alias the caller's list");
        assertTrue(k.opponentObserved());
    }

    @Test
    public void testRosterUnavailableAlertLatchesOnDegradedKnowledge() {
        WorldStateBuilder.setRosterUnavailableAlertForTesting(false);
        assertFalse(WorldStateBuilder.isRosterUnavailableAlertActive(), "Alert must start inactive");

        WorldStateBuilder.reportDegradedForTesting("testFailure", new RuntimeException("Simulated failure"));
        assertTrue(WorldStateBuilder.isRosterUnavailableAlertActive(), "Alert must latch active on failure");

        // Normal successful build should clear the alert
        MatchKnowledge knowledge = WorldStateBuilder.buildMatchKnowledgeForSimBot(false);
        assertNotNull(knowledge);
        assertFalse(WorldStateBuilder.isRosterUnavailableAlertActive(),
                "A completely successful build with no exceptions must clear the alert");
    }

    @Test
    public void testFuelTargetInZoneRespectsFieldMapAllianceZones() {
        Pose2d midFieldPose = new Pose2d(8.0, 4.0, new Rotation2d());

        // For Blue alliance:
        // Home zone target must be inside Blue Alliance Zone
        Pose2d blueHome = engine.findAllianceZoneFuelTarget(midFieldPose, false);
        assertNotNull(blueHome);
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(blueHome.getTranslation(), false),
                "Blue home fuel target must be inside Blue alliance zone");

        // Opponent zone target must be inside Red Alliance Zone
        Pose2d blueOpp = engine.findOpponentZoneFuelTarget(midFieldPose, false);
        assertNotNull(blueOpp);
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(blueOpp.getTranslation(), true),
                "Blue's opponent fuel target must be inside Red alliance zone");

        // For Red alliance:
        // Home zone target must be inside Red Alliance Zone
        Pose2d redHome = engine.findAllianceZoneFuelTarget(midFieldPose, true);
        assertNotNull(redHome);
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(redHome.getTranslation(), true),
                "Red home fuel target must be inside Red alliance zone");

        // Opponent zone target must be inside Blue Alliance Zone
        Pose2d redOpp = engine.findOpponentZoneFuelTarget(midFieldPose, true);
        assertNotNull(redOpp);
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(redOpp.getTranslation(), false),
                "Red's opponent fuel target must be inside Blue alliance zone");
    }
}
