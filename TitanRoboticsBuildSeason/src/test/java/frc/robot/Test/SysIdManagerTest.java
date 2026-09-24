package frc.robot.Test;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Test.SysIdManager.MechanismType;

public class SysIdManagerTest {

    private SysIdManager sysIdManager;

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        CommandScheduler.getInstance().cancelAll();
        sysIdManager = SysIdManager.getInstance();
    }

    @Test
    public void testMechanismSelection() {
        assertNotNull(sysIdManager);

        for (MechanismType mech : MechanismType.values()) {
            sysIdManager.setActiveMechanism(mech);
            assertEquals(mech, sysIdManager.getActiveMechanism());
            assertNotNull(sysIdManager.getActiveRoutine());
        }
    }

    @Test
    public void testStartAndAbortQuasistatic() {
        sysIdManager.setActiveMechanism(MechanismType.SWERVE_DRIVE_LINEAR);
        sysIdManager.startQuasistatic(Direction.kForward);
        assertTrue(sysIdManager.isRunning());

        sysIdManager.abort();
        assertFalse(sysIdManager.isRunning());
        assertEquals("IDLE", sysIdManager.getRoutineState());
    }

    @Test
    public void testStartAndAbortDynamic() {
        sysIdManager.setActiveMechanism(MechanismType.SHOOTER_FLYWHEELS);
        sysIdManager.startDynamic(Direction.kReverse);
        assertTrue(sysIdManager.isRunning());

        sysIdManager.abort();
        assertFalse(sysIdManager.isRunning());
        assertEquals("IDLE", sysIdManager.getRoutineState());
    }

    @Test
    public void testIntakeArmSafetyLimitClamp() {
        Intake intake = Intake.getInstance();
        intake.setArmVoltage(0.0);

        // Safe angle
        intake.setCharacterizationVoltage(2.0);
        assertEquals(2.0, intake.getArmAppliedVoltage(), 0.01);
        assertEquals(Intake.IntakeState.CHARACTERIZATION, intake.getState());

        intake.stop();
        assertEquals(Intake.IntakeState.DISABLED, intake.getState());
    }

    @Test
    public void testShooterCharacterizationState() {
        Shooter shooter = Shooter.getInstance();
        shooter.setFlywheelCharacterizationVoltage(3.0, 3.0);
        assertEquals("characterization", shooter.getShooterState());

        shooter.stop();
        assertEquals("stop", shooter.getShooterState());
    }

    @Test
    public void testSwerveSysIdMethods() {
        SwerveBase swerve = SwerveBase.getInstance();
        assertDoesNotThrow(() -> swerve.setSysIdDriveVoltage(2.0));
        assertDoesNotThrow(() -> swerve.setSysIdRotationVoltage(2.0));
        assertDoesNotThrow(() -> swerve.setSysIdSteerVoltage(1.0));
        assertFalse(swerve.hasAbsoluteEncoderIssues());
        swerve.stop();
    }
}
