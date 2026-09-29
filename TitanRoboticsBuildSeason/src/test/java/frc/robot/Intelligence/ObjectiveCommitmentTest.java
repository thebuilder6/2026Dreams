package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.LinkedHashMap;
import java.util.Map;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Objective-commitment (hysteresis) coverage.
 *
 * <p>Several objectives sit within ~0.02 of each other in the utility matrix
 * (SWEEP 0.96 vs CYCLE 0.72-0.98), so an unlatched engine re-routes a loaded
 * bot every 20 ms - the reported "constantly moving before trying to shoot".
 * These tests pin the three rules that stop that without suppressing genuine
 * match-state changes, and that commitment is per-agent rather than global.
 */
class ObjectiveCommitmentTest {

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setMatchTime(-1.0);
    }

    private static Map<StrategicObjective, Double> utilities(
            StrategicObjective a, double av, StrategicObjective b, double bv) {
        Map<StrategicObjective, Double> m = new LinkedHashMap<>();
        m.put(a, av);
        m.put(b, bv);
        return m;
    }

    @Test
    void firstCandidateIsAdoptedImmediately() {
        ObjectiveCommitment c = new ObjectiveCommitment();
        assertNull(c.committed());
        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                c.apply(StrategicObjective.VACUUM_MIDFIELD,
                        utilities(StrategicObjective.VACUUM_MIDFIELD, 0.9,
                                StrategicObjective.CYCLE_SCORE_HUB, 0.9)));
    }

    @Test
    void smallUtilityGainCannotStealCommitment() {
        // The thrash case: VACUUM 0.82 vs CYCLE 0.84, a 0.02 gap that flipped
        // every cycle before the latch existed.
        ObjectiveCommitment c = new ObjectiveCommitment();
        Map<StrategicObjective, Double> u = utilities(
                StrategicObjective.VACUUM_MIDFIELD, 0.82,
                StrategicObjective.CYCLE_SCORE_HUB, 0.84);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                c.apply(StrategicObjective.VACUUM_MIDFIELD, u));
        for (int i = 0; i < 20; i++) {
            assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                    c.apply(StrategicObjective.CYCLE_SCORE_HUB, u),
                    "a 0.02 gain must never steal an objective");
        }
    }

    @Test
    void sameObjectiveKeepsHolding() {
        ObjectiveCommitment c = new ObjectiveCommitment();
        Map<StrategicObjective, Double> u = utilities(
                StrategicObjective.CYCLE_SCORE_HUB, 0.9,
                StrategicObjective.VACUUM_MIDFIELD, 0.99);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, c.apply(StrategicObjective.CYCLE_SCORE_HUB, u));
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, c.apply(StrategicObjective.CYCLE_SCORE_HUB, u));
    }

    @Test
    void collapsedIncumbentIsReleasedImmediately() {
        // The hub deactivating: CYCLE_SCORE_HUB's utility drops to 0, so holding
        // it would strand the bot. This must not wait for the minimum hold.
        ObjectiveCommitment c = new ObjectiveCommitment();
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB,
                c.apply(StrategicObjective.CYCLE_SCORE_HUB,
                        utilities(StrategicObjective.CYCLE_SCORE_HUB, 0.95,
                                StrategicObjective.VACUUM_MIDFIELD, 0.10)));

        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                c.apply(StrategicObjective.VACUUM_MIDFIELD,
                        utilities(StrategicObjective.CYCLE_SCORE_HUB, 0.0,
                                StrategicObjective.VACUUM_MIDFIELD, 0.85)),
                "an inadmissible incumbent must be released at once");
    }

    @Test
    void decisiveGainSwitchesWithoutWaiting() {
        ObjectiveCommitment c = new ObjectiveCommitment();
        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                c.apply(StrategicObjective.VACUUM_MIDFIELD,
                        utilities(StrategicObjective.VACUUM_MIDFIELD, 0.50,
                                StrategicObjective.SWEEP_ALLIANCE_ZONE, 0.96)));
        assertEquals(StrategicObjective.SWEEP_ALLIANCE_ZONE,
                c.apply(StrategicObjective.SWEEP_ALLIANCE_ZONE,
                        utilities(StrategicObjective.VACUUM_MIDFIELD, 0.50,
                                StrategicObjective.SWEEP_ALLIANCE_ZONE, 0.96)),
                "a decisive gain must switch immediately");
    }

    @Test
    void commitmentsAreIndependentPerAgent() {
        // The regression that motivated moving this off the engine: a shared
        // latch let one agent's decision leak into another's.
        ObjectiveCommitment bot0 = new ObjectiveCommitment();
        ObjectiveCommitment bot1 = new ObjectiveCommitment();
        Map<StrategicObjective, Double> u = utilities(
                StrategicObjective.CYCLE_SCORE_HUB, 0.98,
                StrategicObjective.VACUUM_MIDFIELD, 0.10);

        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, bot0.apply(StrategicObjective.CYCLE_SCORE_HUB, u));
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, bot1.apply(StrategicObjective.VACUUM_MIDFIELD, u));
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, bot0.committed());
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, bot1.committed());
    }

    @Test
    void resetClearsCommitment() {
        ObjectiveCommitment c = new ObjectiveCommitment();
        c.apply(StrategicObjective.CYCLE_SCORE_HUB,
                utilities(StrategicObjective.CYCLE_SCORE_HUB, 0.9,
                        StrategicObjective.VACUUM_MIDFIELD, 0.0));
        c.reset();
        assertNull(c.committed());
        assertEquals(0.0, c.heldSeconds());
    }

    @Test
    void engineRemainsStatelessWithoutALatch() {
        // evaluatePolicy with a null commitment must ignore any prior decision:
        // the Match Coach and unit tests rely on this.
        WorldState w = new WorldState(
                new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0)),
                new ChassisSpeeds(), 12,
                new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180)),
                new ChassisSpeeds(), 120.0, true, true, 120.0, false, false);

        AIActionIntent first = JevDecisionEngine.getInstance()
                .evaluatePolicy(w, MatchKnowledge.unknown(), Archetype.AUTONOMOUS_CYCLER);
        for (int i = 0; i < 10; i++) {
            assertEquals(first.objective(),
                    JevDecisionEngine.getInstance().evaluatePolicy(
                            w, MatchKnowledge.unknown(), Archetype.AUTONOMOUS_CYCLER).objective(),
                    "no latch must mean a pure function of the snapshot");
        }
    }

    @Test
    void committedRunIsStableWhereUnlatchedRunMayFlip() {
        // End-to-end: the same snapshot repeated must not thrash when a latch
        // is supplied, and the latched objective must be the raw winner.
        WorldState w = new WorldState(
                new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0)),
                new ChassisSpeeds(), 20,
                new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180)),
                new ChassisSpeeds(), 120.0, false, true, 8.0, false, false);

        AIActionIntent raw = JevDecisionEngine.getInstance()
                .evaluatePolicy(w, MatchKnowledge.unknown(), Archetype.AUTONOMOUS_CYCLER);
        ObjectiveCommitment latch = new ObjectiveCommitment();
        StrategicObjective first = null;
        for (int i = 0; i < 30; i++) {
            StrategicObjective o = JevDecisionEngine.getInstance().evaluatePolicy(
                    w, MatchKnowledge.unknown(), Archetype.AUTONOMOUS_CYCLER,
                    null, null, latch).objective();
            if (first == null) first = o;
            assertEquals(first, o, "latched selection must be stable");
        }
        assertNotEquals(null, first);
        assertEquals(raw.objective(), first,
                "the first latched decision must equal the raw utility winner");
    }

    @Test
    void thresholdsAreOrdered() {
        assertTrue(JevDecisionEngine.COMMITMENT_MARGIN
                        < JevDecisionEngine.COMMITMENT_DECISIVE_MARGIN,
                "normal margin must be tighter than the decisive-switch margin");
        assertTrue(JevDecisionEngine.COMMITMENT_MIN_HOLD_SEC > 0.0);
    }
}
