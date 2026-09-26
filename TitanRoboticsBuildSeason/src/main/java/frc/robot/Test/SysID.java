package frc.robot.Test;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Hardware.Controller;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * Legacy wrapper for SysId routines, now delegating to the unified SysIdManager.
 */
public class SysID {

    private final SysIdManager sysIdManager;

    public SysID(Shooter shooter, Intake intake, SwerveBase swerve) {
        this.sysIdManager = SysIdManager.getInstance();
    }

    public void runTest(XboxController controller) {
        if (controller instanceof Controller) {
            sysIdManager.updateController((Controller) controller);
        } else {
            // Basic hold-to-run fallback
            int pov = controller.getPOV();
            if (pov == 0) {
                sysIdManager.setActiveMechanism(SysIdManager.MechanismType.SHOOTER_FLYWHEELS);
            } else if (pov == 180) {
                sysIdManager.setActiveMechanism(SysIdManager.MechanismType.INTAKE_ARM);
            } else if (pov == 270) {
                sysIdManager.setActiveMechanism(SysIdManager.MechanismType.SWERVE_DRIVE_LINEAR);
            }

            if (controller.getAButton()) {
                if (!sysIdManager.isRunning()) sysIdManager.startQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
            } else if (controller.getXButton()) {
                if (!sysIdManager.isRunning()) sysIdManager.startQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
            } else if (controller.getYButton()) {
                if (!sysIdManager.isRunning()) sysIdManager.startDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
            } else if (controller.getBButton()) {
                sysIdManager.abort();
            } else {
                if (sysIdManager.isRunning()) {
                    sysIdManager.abort();
                }
            }
        }
    }
}
