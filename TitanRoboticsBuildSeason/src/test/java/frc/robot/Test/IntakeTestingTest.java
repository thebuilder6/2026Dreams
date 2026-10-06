package frc.robot.Test;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Hardware.Controller;
import frc.robot.Subsystems.Intake;

public class IntakeTestingTest {

    private Intake intake;
    private IntakeTesting intakeTesting;
    private Controller driverController;
    private Controller operatorController;

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();

        intake = Intake.getInstance();
        intake.stop();

        intakeTesting = new IntakeTesting();
        driverController = new Controller(0);
        operatorController = new Controller(1);
    }

    @Test
    public void testIntakeTestingLifecycle() {
        assertNotNull(intakeTesting);
        assertFalse(intakeTesting.isTestRunning());

        // Update with no inputs
        intakeTesting.update(driverController, operatorController);
        assertFalse(intakeTesting.isTestRunning());

        // Cleanup
        intakeTesting.cleanup();
        assertFalse(intakeTesting.isTestRunning());
    }

    @Test
    public void testIntakeArmSetpointsValid() {
        // Confirm Intake constants are consistent
        assertTrue(frc.robot.Data.Constants.INTAKE_UP_POSITION > frc.robot.Data.Constants.INTAKE_DOWN_POSITION,
                "Up position must be greater than down position");
        assertEquals(250.0, frc.robot.Data.Constants.INTAKE_HORIZONTAL_POSITION, 0.01,
                "Horizontal datum position must be 250 degrees");
    }
}
