package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PolicyWeightsTest {

    @BeforeEach
    void setUp() {
        PolicyWeights.resetToDefault();
    }

    @AfterEach
    void tearDown() {
        PolicyWeights.resetToDefault();
    }

    @Test
    void testDefaultWeightsMatchExpectedConstants() {
        PolicyWeights def = PolicyWeights.DEFAULT;
        assertEquals(0.72, def.scoreHubBase(), 1e-9);
        assertEquals(0.26, def.scoreHubScale(), 1e-9);
        assertEquals(0.80, def.stageStandoffBase(), 1e-9);
        assertEquals(0.88, def.vacuumInactiveBase(), 1e-9);
        assertEquals(0.82, def.vacuumActiveBase(), 1e-9);
        assertEquals(0.86, def.stockpileDepotBase(), 1e-9);
        assertEquals(0.96, def.sweepAllianceZoneActive(), 1e-9);
        assertEquals(0.78, def.poachOpponentZoneUtility(), 1e-9);
        assertEquals(0.87, def.shuttlePassUtility(), 1e-9);
        assertEquals(0.94, def.snipeCloseUtility(), 1e-9);
        assertEquals(0.86, def.laneDenialActiveUtility(), 1e-9);
        assertEquals(0.91, def.chokeTrenchUtility(), 1e-9);
        assertEquals(0.89, def.screenForAllyUtility(), 1e-9);
        assertEquals(1.15, def.tacticalDefenderLaneMultiplier(), 1e-9);
        assertEquals(0.99, def.bullyInterceptUtility(), 1e-9);

        // Dynamic Action Inertia
        assertEquals(0.06, def.commitmentMargin(), 1e-9);
        assertEquals(0.20, def.commitmentDecisiveMargin(), 1e-9);
        assertEquals(1.5, def.commitmentMinHoldSec(), 1e-9);
        assertEquals(0.20, def.inertiaInitialBoost(), 1e-9);
        assertEquals(1.0, def.inertiaTimeConstantSec(), 1e-9);
        assertEquals(0.04, def.inertiaResidualMargin(), 1e-9);

        // Spatial Fuel Scent / Clustering
        assertEquals(1.30, def.clusterNeighborhoodRadius(), 1e-9);
        assertEquals(0.50, def.clusterKernelSigma(), 1e-9);
        assertEquals(1.50, def.clusterDensityExponent(), 1e-9);
        assertEquals(0.40, def.clusterDistanceFloor(), 1e-9);
        assertEquals(0.30, def.harvestHeadingAlignScale(), 1e-9);
        assertEquals(0.35, def.harvestReturnVectorBonus(), 1e-9);

        // IAUS Consideration Curve Shapes (plan §4)
        assertEquals(1.0, def.scoreHubPayloadExponent(), 1e-9);
        assertEquals(8.0, def.shiftUrgencySigmoidSteepness(), 1e-9);
        assertEquals(3.5, def.shiftUrgencyMidpointSec(), 1e-9);
        assertEquals(2.8, def.optimalStandoffMidpointM(), 1e-9);
        assertEquals(0.5, def.optimalStandoffSigmaM(), 1e-9);
    }

    @Test
    void testCurveShapeOverrides() {
        PolicyWeights custom = PolicyWeights.fromString(
                "scoreHubPayloadExponent=1.80, shiftUrgencySigmoidSteepness=5.0, shiftUrgencyMidpointSec=4.0,"
                        + " optimalStandoffMidpointM=3.0, optimalStandoffSigmaM=0.6");
        assertEquals(1.80, custom.scoreHubPayloadExponent(), 1e-9);
        assertEquals(5.0, custom.shiftUrgencySigmoidSteepness(), 1e-9);
        assertEquals(4.0, custom.shiftUrgencyMidpointSec(), 1e-9);
        assertEquals(3.0, custom.optimalStandoffMidpointM(), 1e-9);
        assertEquals(0.6, custom.optimalStandoffSigmaM(), 1e-9);
        // Unmodified retain default
        assertEquals(PolicyWeights.DEFAULT.scoreHubScale(), custom.scoreHubScale(), 1e-9);
        assertEquals(PolicyWeights.DEFAULT.stageShiftWindowSec(), custom.stageShiftWindowSec(), 1e-9);
    }

    @Test
    void testLegacy53ArgConstructorDefaultsCurveShapes() {
        PolicyWeights legacy = new PolicyWeights(
                0.99, 0.01, 0.85, 0.13,
                0.72, 0.26, 0.03, 0.99, 20.0, 16, 4, 4.0, 4.5,
                0.80, 0.99, 0.95, 3.5, 18,
                0.88, 0.10, 0.82, 0.14, 18, 6, 0.35,
                0.86, 0.10,
                0.96, 0.90, 0.08,
                0.78, 6.0, 20,
                0.87, 6.0, 16,
                0.94, 0.86, 3.6, 6,
                0.86, 6.5, 0.65, 0.75, 0.91, 0.89,
                1.15, 1.10, 0.99, 0.98, 0.98, 0.95, 0.85);
        assertEquals(1.0, legacy.scoreHubPayloadExponent(), 1e-9);
        assertEquals(8.0, legacy.shiftUrgencySigmoidSteepness(), 1e-9);
        assertEquals(3.5, legacy.shiftUrgencyMidpointSec(), 1e-9);
        assertEquals(2.8, legacy.optimalStandoffMidpointM(), 1e-9);
        assertEquals(0.5, legacy.optimalStandoffSigmaM(), 1e-9);
    }

    @Test
    void testPayloadExponentShapesScoreUtility() {
        JevDecisionEngine engine = JevDecisionEngine.getInstance();
        WorldState world = new WorldState(
                new Pose2d(9.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                18, // heldFuelCount: above the 16 teleop threshold, loadRatio 18/20 = 0.9 out of range
                new Pose2d(10.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                100.0, // matchTimeRemaining
                true, // isAllianceHubActive
                false, // isOpponentHubActive
                20.0, // timeUntilHubShift
                false, // isRedAlliance
                false // isAutonomous
        );
        Map<StrategicObjective, Double> linear = engine.evaluateUtilityScores(
                world, Collections.emptySet(), ObservedKnowledge.selfOnly(),
                Archetype.AUTONOMOUS_CYCLER, PolicyWeights.DEFAULT);
        PolicyWeights convex = PolicyWeights.DEFAULT.toBuilder().scoreHubPayloadExponent(1.8).build();
        Map<StrategicObjective, Double> shaped = engine.evaluateUtilityScores(
                world, Collections.emptySet(), ObservedKnowledge.selfOnly(),
                Archetype.AUTONOMOUS_CYCLER, convex);
        // loadRatio 0.9: linear base 0.72+0.26*0.9=0.954; convex uses pow(0.9,1.8).
        assertEquals(0.954, linear.get(StrategicObjective.CYCLE_SCORE_HUB), 1e-9);
        assertEquals(0.72 + 0.26 * Math.pow(0.9, 1.8), shaped.get(StrategicObjective.CYCLE_SCORE_HUB), 1e-9);
    }

    @Test
    void testShiftUrgencyLogisticMatchesLegacyStepOffWindow() {
        JevDecisionEngine engine = JevDecisionEngine.getInstance();
        WorldState midShift = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                20, new Pose2d(10.0, 4.0, new Rotation2d()), new ChassisSpeeds(),
                100.0, false, true, 15.0, false, false);
        WorldState imminent = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                20, new Pose2d(10.0, 4.0, new Rotation2d()), new ChassisSpeeds(),
                100.0, false, true, 1.0, false, false);
        WorldState noFlip = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                20, new Pose2d(10.0, 4.0, new Rotation2d()), new ChassisSpeeds(),
                10.0, false, true, 0.0, false, false);
        Map<StrategicObjective, Double> midScores = engine.evaluateUtilityScores(
                midShift, Collections.emptySet(), ObservedKnowledge.selfOnly(),
                Archetype.AUTONOMOUS_CYCLER, PolicyWeights.DEFAULT);
        Map<StrategicObjective, Double> imminentScores = engine.evaluateUtilityScores(
                imminent, Collections.emptySet(), ObservedKnowledge.selfOnly(),
                Archetype.AUTONOMOUS_CYCLER, PolicyWeights.DEFAULT);
        Map<StrategicObjective, Double> noFlipScores = engine.evaluateUtilityScores(
                noFlip, Collections.emptySet(), ObservedKnowledge.selfOnly(),
                Archetype.AUTONOMOUS_CYCLER, PolicyWeights.DEFAULT);
        // Legacy step values preserved: base 0.80 off-window, 0.95 imminent, base when no flip is coming.
        assertEquals(0.80, midScores.get(StrategicObjective.STAGE_STANDOFF), 1e-6);
        assertEquals(0.95, imminentScores.get(StrategicObjective.STAGE_STANDOFF), 1e-6);
        assertEquals(0.80, noFlipScores.get(StrategicObjective.STAGE_STANDOFF), 1e-9);
    }

    @Test
    void testActionInertiaAndHarvestScentOverrides() {
        PolicyWeights custom = PolicyWeights.fromString(
                "commitmentMargin=0.08, inertiaTimeConstantSec=1.5, clusterNeighborhoodRadius=1.60, clusterKernelSigma=0.65");
        assertEquals(0.08, custom.commitmentMargin(), 1e-9);
        assertEquals(1.5, custom.inertiaTimeConstantSec(), 1e-9);
        assertEquals(1.60, custom.clusterNeighborhoodRadius(), 1e-9);
        assertEquals(0.65, custom.clusterKernelSigma(), 1e-9);
        // Unmodified retain default
        assertEquals(PolicyWeights.DEFAULT.commitmentDecisiveMargin(), custom.commitmentDecisiveMargin(), 1e-9);
        assertEquals(PolicyWeights.DEFAULT.clusterDensityExponent(), custom.clusterDensityExponent(), 1e-9);
    }

    @Test
    void testFromStringAppliesOverrides() {
        PolicyWeights custom = PolicyWeights.fromString("scoreHubBase=0.79, vacuumActiveBase=0.91; stageStandoffBase=0.85");
        assertEquals(0.79, custom.scoreHubBase(), 1e-9);
        assertEquals(0.91, custom.vacuumActiveBase(), 1e-9);
        assertEquals(0.85, custom.stageStandoffBase(), 1e-9);
        // Unmodified values must retain default
        assertEquals(PolicyWeights.DEFAULT.scoreHubScale(), custom.scoreHubScale(), 1e-9);
        assertEquals(PolicyWeights.DEFAULT.poachOpponentZoneUtility(), custom.poachOpponentZoneUtility(), 1e-9);
    }

    @Test
    void testFromStringUnknownKeyThrowsIllegalArgumentException() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () -> {
            PolicyWeights.fromString("fakeWeightName=1.23");
        });
        assertTrue(ex.getMessage().contains("fakeWeightName"), "Error message should mention the unknown key");
    }

    @Test
    void testFromStringMalformedFormatThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () -> {
            PolicyWeights.fromString("invalid_key_value_string");
        });
    }

    @Test
    void testActiveWeightsSetAndReset() {
        PolicyWeights custom = PolicyWeights.DEFAULT.toBuilder().scoreHubBase(0.95).build();
        PolicyWeights.setActive(custom);
        assertEquals(0.95, PolicyWeights.getActive().scoreHubBase(), 1e-9);

        PolicyWeights.resetToDefault();
        assertEquals(PolicyWeights.DEFAULT.scoreHubBase(), PolicyWeights.getActive().scoreHubBase(), 1e-9);
    }

    @Test
    void testJevDecisionEngineHonorsExplicitWeights() {
        JevDecisionEngine engine = JevDecisionEngine.getInstance();
        // Construct world state where alliance hub is active and held fuel allows scoring
        WorldState world = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                20,    // heldFuelCount
                new Pose2d(10.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                100.0, // matchTimeRemaining
                true,  // isAllianceHubActive
                false, // isOpponentHubActive
                20.0,  // timeUntilHubShift
                false, // isRedAlliance
                false  // isAutonomous
        );

        Map<StrategicObjective, Double> defaultScores = engine.evaluateUtilityScores(
                world, Collections.emptySet(), ObservedKnowledge.selfOnly(), Archetype.AUTONOMOUS_CYCLER, PolicyWeights.DEFAULT);

        // Customize scoreHubBase and scoreHubScale
        PolicyWeights boosted = PolicyWeights.DEFAULT.toBuilder()
                .scoreHubBase(0.85)
                .scoreHubScale(0.10)
                .build();

        Map<StrategicObjective, Double> boostedScores = engine.evaluateUtilityScores(
                world, Collections.emptySet(), ObservedKnowledge.selfOnly(), Archetype.AUTONOMOUS_CYCLER, boosted);

        assertNotEquals(defaultScores.get(StrategicObjective.CYCLE_SCORE_HUB),
                boostedScores.get(StrategicObjective.CYCLE_SCORE_HUB));
        // With loadRatio = 1.0, boosted is 0.85 + 0.10 = 0.95
        assertEquals(0.95, boostedScores.get(StrategicObjective.CYCLE_SCORE_HUB), 1e-9);
    }

    @Test
    void testDefaultOverloadIsBitExactWithExplicitDefault() {
        JevDecisionEngine engine = JevDecisionEngine.getInstance();
        WorldState world = new WorldState(
                new Pose2d(3.5, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                5,     // heldFuelCount
                new Pose2d(9.5, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                80.0,  // matchTimeRemaining
                false, // isAllianceHubActive
                true,  // isOpponentHubActive
                15.0,  // timeUntilHubShift
                false, // isRedAlliance
                false  // isAutonomous
        );

        Map<StrategicObjective, Double> implicitScores = engine.evaluateUtilityScores(
                world, Collections.emptySet(), ObservedKnowledge.selfOnly(), Archetype.ADAPTIVE_COMPETITOR);
        Map<StrategicObjective, Double> explicitScores = engine.evaluateUtilityScores(
                world, Collections.emptySet(), ObservedKnowledge.selfOnly(), Archetype.ADAPTIVE_COMPETITOR, PolicyWeights.DEFAULT);

        assertEquals(implicitScores.size(), explicitScores.size());
        for (StrategicObjective obj : StrategicObjective.values()) {
            assertEquals(implicitScores.get(obj), explicitScores.get(obj),
                    "Objective " + obj + " must produce bit-exact identical utility value");
        }
    }
}
