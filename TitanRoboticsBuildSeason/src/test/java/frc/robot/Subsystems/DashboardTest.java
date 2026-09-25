package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testDashboardSingletonAndFeatureToggles() {
        Dashboard dashboard = Dashboard.getInstance();
        assertNotNull(dashboard);

        assertTrue(Dashboard.isSnapToTurnEnabled());
        assertTrue(Dashboard.isAutoAimEnabled());
        assertTrue(Dashboard.isFieldOriented());
        assertFalse(Dashboard.isHapticCollisionEnabled(), "Haptic collision should default to disabled in simulation");
    }

    @Test
    public void testDashboardLogExecution() {
        Dashboard dashboard = Dashboard.getInstance();
        assertDoesNotThrow(() -> dashboard.update());
        assertDoesNotThrow(() -> dashboard.log());

        assertTrue(SmartDashboard.containsKey("Driver/Hub Active"));
        assertTrue(SmartDashboard.containsKey("Driver/Shoot Alert"));
        assertTrue(SmartDashboard.containsKey("Driver/Hub Shift Time Remaining"));
    }

    @Test
    public void testMultiBotSimulationControls() {
        Dashboard.setOpponentCount(2);
        assertTrue(Dashboard.getOpponentCount() >= 1 && Dashboard.getOpponentCount() <= 3);
        Dashboard.setOpponentCount(1);
        assertTrue(Dashboard.getOpponentSpeedPercent() >= 20.0 && Dashboard.getOpponentSpeedPercent() <= 100.0);
    }
}
