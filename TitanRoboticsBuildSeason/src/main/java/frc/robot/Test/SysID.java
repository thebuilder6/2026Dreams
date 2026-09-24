package frc.robot.Test;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Test.SysIdManager.Mechanism;
import frc.robot.Test.SysIdManager.TestType;

/**
 * Legacy SysID compatibility wrapper.
 * Delegates all routines and execution to the unified {@link SysIdManager}.
 */
public class SysID {

    private final SysIdManager manager;

    public SysID(Shooter shooter, Intake intake, SwerveBase swerve) {
        this.manager = SysIdManager.getInstance();
    }

    public void runTest(XboxController controller) {
        if (controller instanceof Controller c) {
            manager.update(c);
            return;
        }

        // Direct D-pad mechanism switching
        if (controller.getPOV() == 0) {
            manager.setActiveMechanism(Mechanism.SHOOTER_FLYWHEELS);
        } else if (controller.getPOV() == 180) {
            manager.setActiveMechanism(Mechanism.INTAKE_ARM);
        } else if (controller.getPOV() == 270) {
            manager.setActiveMechanism(Mechanism.SWERVE_DRIVE_LINEAR);
        } else if (controller.getPOV() == 90) {
            manager.setActiveMechanism(Mechanism.SWERVE_DRIVE_ANGULAR);
        }

        // Button commands
        if (controller.getAButtonPressed()) {
            manager.startTest(manager.getActiveMechanism(), TestType.QUASISTATIC, Direction.kForward);
        } else if (controller.getBButtonPressed()) {
            manager.cancelTest();
        } else if (controller.getXButtonPressed()) {
            manager.startTest(manager.getActiveMechanism(), TestType.DYNAMIC, Direction.kReverse);
        } else if (controller.getYButtonPressed()) {
            manager.startTest(manager.getActiveMechanism(), TestType.DYNAMIC, Direction.kForward);
        }
    }

    public SysIdManager getManager() {
        return manager;
    }
}
