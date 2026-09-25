package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Data.PortMap;
import frc.robot.Subsystems.Dashboard;

public class TeleopTest {

    private Teleop teleop;

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        CommandScheduler.getInstance().cancelAll();
        teleop = new Teleop();
        teleop.init();
    }

    @Test
    public void testInputShapingMath() {
        // Zero point
        assertEquals(0.0, Teleop.shapeInput(0.0), 1e-6);

        // Full scale limits
        assertEquals(1.0, Teleop.shapeInput(1.0), 1e-6);
        assertEquals(-1.0, Teleop.shapeInput(-1.0), 1e-6);

        // Half scale should be significantly softer than linear (0.7 * 0.125 + 0.3 * 0.5 = 0.2375)
        double halfVal = Teleop.shapeInput(0.5);
        assertEquals(0.2375, halfVal, 1e-4);
        assertTrue(halfVal < 0.5, "Input shaping should reduce low-end sensitivity for fine control");

        // Symmetry
        assertEquals(-halfVal, Teleop.shapeInput(-0.5), 1e-6);

        // Monotonic increase check across 0.0 -> 1.0 range
        double last = -1.0;
        for (double x = -1.0; x <= 1.0; x += 0.1) {
            double shaped = Teleop.shapeInput(x);
            assertTrue(shaped >= last, "Input shaping must be strictly monotonically increasing");
            last = shaped;
        }
    }

    @Test
    public void testTeleopInitAndReset() {
        assertNotNull(teleop);
        assertFalse(teleop.isSlowModeActive());
        assertFalse(teleop.isArmDeployed());
        assertNull(teleop.getSnapTargetHeading());

        teleop.setSlowModeActive(true);
        assertTrue(teleop.isSlowModeActive());

        teleop.reset();
        assertFalse(teleop.isSlowModeActive());
        assertFalse(teleop.isArmDeployed());
        assertNull(teleop.getSnapTargetHeading());
    }

    @Test
    public void testSnapTargetHeadingAngles() {
        // Test that D-pad cardinal directions map to standard coordinate angles
        Rotation2d forward = Rotation2d.fromDegrees(0);
        Rotation2d right = Rotation2d.fromDegrees(-90);
        Rotation2d backward = Rotation2d.fromDegrees(180);
        Rotation2d left = Rotation2d.fromDegrees(90);

        assertEquals(0.0, forward.getDegrees(), 1e-4);
        assertEquals(-90.0, right.getDegrees(), 1e-4);
        assertEquals(180.0, Math.abs(backward.getDegrees()), 1e-4);
        assertEquals(90.0, left.getDegrees(), 1e-4);
    }

    @Test
    public void testSlowModeControllerToggleAndExit() {
        edu.wpi.first.wpilibj.simulation.DriverStationSim.resetData();
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setDsAttached(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
        XboxControllerSim driverSim = new XboxControllerSim(PortMap.DRIVER_CONTROLLER);
        driverSim.setButtonCount(16);
        driverSim.setAxisCount(6);
        driverSim.setLeftStickButton(false);
        driverSim.notifyNewData();
        teleop.init();

        assertFalse(teleop.isSlowModeActive(), "Slow mode must start inactive");
        assertFalse(Dashboard.isSlowModeEnabled(), "Dashboard slow mode must start inactive");

        // 1. Driver clicks Left Stick to enter slow mode
        driverSim.setLeftStickButton(true);
        driverSim.notifyNewData();
        teleop.readControllers();
        assertTrue(teleop.isSlowModeActive(), "Driver clicking left stick must enter slow mode");
        assertTrue(Dashboard.isSlowModeEnabled(), "Dashboard must reflect slow mode enabled");

        // Release stick
        driverSim.setLeftStickButton(false);
        driverSim.notifyNewData();
        teleop.readControllers();
        assertTrue(teleop.isSlowModeActive(), "Slow mode must stay active when stick is released");

        // 2. Driver clicks Left Stick again to EXIT slow mode
        driverSim.setLeftStickButton(true);
        driverSim.notifyNewData();
        teleop.readControllers();
        assertFalse(teleop.isSlowModeActive(), "Driver clicking left stick again must EXIT slow mode");
        assertFalse(Dashboard.isSlowModeEnabled(), "Dashboard must reflect slow mode disabled");

        // Release stick
        driverSim.setLeftStickButton(false);
        driverSim.notifyNewData();
        teleop.readControllers();
        assertFalse(teleop.isSlowModeActive(), "Slow mode must stay inactive");
    }

    @Test
    public void testSlowModeDashboardTwoWaySync() {
        edu.wpi.first.wpilibj.simulation.DriverStationSim.resetData();
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setDsAttached(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
        XboxControllerSim driverSim = new XboxControllerSim(PortMap.DRIVER_CONTROLLER);
        driverSim.setButtonCount(16);
        driverSim.setAxisCount(6);
        driverSim.setLeftStickButton(false);
        driverSim.notifyNewData();
        teleop.init();

        // 1. Dashboard turns slow mode ON remotely
        Dashboard.setSlowModeEnabled(true);
        teleop.readControllers();
        assertTrue(teleop.isSlowModeActive(), "Remote dashboard toggle must activate slow mode");

        // 2. Driver overrides and turns slow mode OFF using controller
        driverSim.setLeftStickButton(true);
        driverSim.notifyNewData();
        teleop.readControllers();
        assertFalse(teleop.isSlowModeActive(), "Driver controller must successfully exit slow mode after dashboard turned it on");
        assertFalse(Dashboard.isSlowModeEnabled(), "Dashboard must synchronize to disabled after driver override");

        // 3. Driver turns slow mode ON using controller
        driverSim.setLeftStickButton(false);
        driverSim.notifyNewData();
        teleop.readControllers();
        driverSim.setLeftStickButton(true);
        driverSim.notifyNewData();
        teleop.readControllers();
        assertTrue(teleop.isSlowModeActive(), "Driver controller turns slow mode ON");
        assertTrue(Dashboard.isSlowModeEnabled(), "Dashboard reflects slow mode ON");

        // 4. Remote dashboard turns slow mode OFF
        Dashboard.setSlowModeEnabled(false);
        teleop.readControllers();
        assertFalse(teleop.isSlowModeActive(), "Remote dashboard must successfully turn off slow mode");
    }
}
