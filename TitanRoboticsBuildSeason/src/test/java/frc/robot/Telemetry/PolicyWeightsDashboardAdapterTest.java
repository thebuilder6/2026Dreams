package frc.robot.Telemetry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
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
    }
}
