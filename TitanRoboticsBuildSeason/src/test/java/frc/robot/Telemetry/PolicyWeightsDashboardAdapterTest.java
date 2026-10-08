package frc.robot.Telemetry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Intelligence.PolicyWeights;

class PolicyWeightsDashboardAdapterTest {

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        PolicyWeights.resetToDefault();
    }

    @AfterEach
    void tearDown() {
        PolicyWeights.resetToDefault();
    }

    @Test
    void testAdapterUpdateDoesNotThrowAndPreservesDefaults() {
        assertDoesNotThrow(PolicyWeightsDashboardAdapter::update);
        PolicyWeights active = PolicyWeights.getActive();
        assertNotNull(active);
        assertEquals(0.72, active.scoreHubBase(), 1e-9);
        assertEquals(0.06, active.commitmentMargin(), 1e-9);
        assertEquals(1.30, active.clusterNeighborhoodRadius(), 1e-9);
        assertEquals(1.0, active.scoreHubPayloadExponent(), 1e-9);
        assertEquals(8.0, active.shiftUrgencySigmoidSteepness(), 1e-9);
        assertEquals(3.5, active.shiftUrgencyMidpointSec(), 1e-9);
        assertEquals(2.8, active.optimalStandoffMidpointM(), 1e-9);
        assertEquals(0.5, active.optimalStandoffSigmaM(), 1e-9);
    }

    @Test
    void testPreviouslyDeadTunablesNowApply() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.getTable("TunableNumbers").getEntry("JevAI/SweepZoneActive").setDouble(0.50);
        inst.getTable("TunableNumbers").getEntry("JevAI/VacuumInactiveBase").setDouble(0.51);
        inst.getTable("TunableNumbers").getEntry("JevAI/LaneDenialBase").setDouble(0.52);
        inst.getTable("TunableNumbers").getEntry("JevAI/ShadowMidlineBase").setDouble(0.53);
        inst.getTable("TunableNumbers").getEntry("JevAI/InterceptBase").setDouble(0.54);
        try {
            PolicyWeightsDashboardAdapter.update();
            PolicyWeights active = PolicyWeights.getActive();
            assertEquals(0.50, active.sweepAllianceZoneActive(), 1e-9);
            assertEquals(0.51, active.vacuumInactiveBase(), 1e-9);
            assertEquals(0.52, active.laneDenialActiveUtility(), 1e-9);
            assertEquals(0.53, active.shadowMidlineBaseUtility(), 1e-9);
            assertEquals(0.54, active.interceptBaseUtility(), 1e-9);
        } finally {
            inst.getTable("TunableNumbers").getEntry("JevAI/SweepZoneActive")
                    .setDouble(PolicyWeights.DEFAULT.sweepAllianceZoneActive());
            inst.getTable("TunableNumbers").getEntry("JevAI/VacuumInactiveBase")
                    .setDouble(PolicyWeights.DEFAULT.vacuumInactiveBase());
            inst.getTable("TunableNumbers").getEntry("JevAI/LaneDenialBase")
                    .setDouble(PolicyWeights.DEFAULT.laneDenialActiveUtility());
            inst.getTable("TunableNumbers").getEntry("JevAI/ShadowMidlineBase")
                    .setDouble(PolicyWeights.DEFAULT.shadowMidlineBaseUtility());
            inst.getTable("TunableNumbers").getEntry("JevAI/InterceptBase")
                    .setDouble(PolicyWeights.DEFAULT.interceptBaseUtility());
            PolicyWeights.resetToDefault();
        }
    }
}
