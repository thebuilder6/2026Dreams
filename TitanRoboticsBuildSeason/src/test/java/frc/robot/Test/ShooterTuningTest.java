package frc.robot.Test;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Hardware.Controller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

public class ShooterTuningTest {

    private Shooter shooter;
    private ShooterTuning shooterTuning;
    private Controller driverController;
    private Controller operatorController;

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();

        SwerveBase.getInstance();
        shooter = Shooter.getInstance();
        shooter.stop();

        shooterTuning = new ShooterTuning();
        driverController = new Controller(0);
        operatorController = new Controller(1);
    }

    @Test
    public void testUpdatePIDGainsAppliesDirectly() {
        shooter.updatePIDGains(0.0015, 0.0001, 0.0002, 0.1, 0.0025, 0.05);

        shooter.setTargetRPM(3000.0);
        shooter.prepareToShoot();
        shooter.update();

        assertTrue(shooter.getLeftShooterVoltageCalc() > 0.0, "Calculated voltage should be positive with new gains");
        shooter.stop();
    }

    @Test
    public void testShooterTuningLifecycle() {
        assertNotNull(shooterTuning);
        assertFalse(shooterTuning.isTestRunning());

        // Update with no inputs
        shooterTuning.update(driverController, operatorController);
        assertEquals("stop", shooter.getShooterState());

        // Cleanup
        shooterTuning.cleanup();
        assertFalse(shooterTuning.isTestRunning());
    }
}
