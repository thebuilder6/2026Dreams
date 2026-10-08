package frc.robot.Telemetry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Telemetry.Dashboard;

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
        Dashboard dashboard = Dashboard.getInstance();
        assertNotNull(dashboard.getOpponentCountChooser());

        Dashboard.setOpponentCount(2);
        assertEquals(2, Dashboard.getOpponentCount());
        Dashboard.setOpponentCount(3);
        assertEquals(3, Dashboard.getOpponentCount());
        Dashboard.setOpponentCount(1);
        assertEquals(1, Dashboard.getOpponentCount());
        assertTrue(Dashboard.getOpponentSpeedPercent() >= 20.0 && Dashboard.getOpponentSpeedPercent() <= 100.0);
    }

    @Test
    public void testAllyBotSimulationControls() {
        Dashboard dashboard = Dashboard.getInstance();
        assertNotNull(dashboard.getAllyCountChooser());

        Dashboard.setAllyCount(1);
        assertEquals(1, Dashboard.getAllyCount());
        assertTrue(Dashboard.isAllyBotsEnabled());

        Dashboard.setAllyCount(2);
        assertEquals(2, Dashboard.getAllyCount());
        assertTrue(Dashboard.isAllyBotsEnabled());

        Dashboard.setAllyCount(0);
        assertEquals(0, Dashboard.getAllyCount());

        // Clamping checks
        Dashboard.setAllyCount(-1);
        assertEquals(0, Dashboard.getAllyCount());
        Dashboard.setAllyCount(5);
        assertEquals(2, Dashboard.getAllyCount());

        // Reset to default 0
        Dashboard.setAllyCount(0);
    }

    @Test
    public void testGameDataRefreshesBetweenMatches() {
        Dashboard dashboard = Dashboard.getInstance();
        try {
            edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
            edu.wpi.first.wpilibj.simulation.DriverStationSim.setGameSpecificMessage("R");
            edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
            dashboard.update();
            dashboard.log();
            assertEquals("R", SmartDashboard.getString("Match/GameData", ""));

            // Match 2: the FMS sends a new shift order. The match-1 value must not
            // stick (the robot program persists across matches).
            edu.wpi.first.wpilibj.simulation.DriverStationSim.setGameSpecificMessage("B");
            edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
            dashboard.update();
            dashboard.log();
            assertEquals("B", SmartDashboard.getString("Match/GameData", ""),
                    "the cached shift order must refresh from the live FMS message");
        } finally {
            edu.wpi.first.wpilibj.simulation.DriverStationSim.resetData();
        }
    }
}
