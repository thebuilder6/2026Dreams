package frc.robot.Auto.Actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.SwerveBase;

/**
 * Prototype action to automatically drive towards and intake balls (Fuel).
 * Uses Limelight 'tx' to steer and drive at a constant speed.
 */
public class BallHuntAction implements Actions {
    private final SwerveBase swerve = SwerveBase.getInstance();
    private final Intake intake = Intake.getInstance();
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // PID constants for steering. tx is in degrees.
    // If tx is 10 degrees, we want to rotate at some speed to center it.
    private final PIDController turnController = new PIDController(0.08, 0, 0.005);
    private final double driveSpeed = 2.0; // Meters per second

    public BallHuntAction() {
        // Continuous input not strictly necessary for tx since it's already relative
        // offset
        // but safe to have if tx was absolute.
        turnController.setSetpoint(0);
        turnController.setTolerance(1.0); // 1 degree tolerance
    }

    @Override
    public void start() {
        intake.setState(IntakeState.INTAKING);
    }

    @Override
    public void update() {
        double tv = limelightTable.getEntry("tv").getDouble(0);

        if (tv > 0) {
            double tx = limelightTable.getEntry("tx").getDouble(0);

            // Calculate rotation speed based on tx error
            // Negative tx means ball is to the left, so we want positive rotation (CCW)
            // However, most systems use tx as positive to the right.
            // If tx > 0, ball is to the right, we want negative rotation (CW).
            double rotationOutput = -turnController.calculate(tx, 0);

            // Drive forward in robot-relative coordinates
            swerve.drive(new Translation2d(driveSpeed, 0), rotationOutput, false);
        } else {
            // If no ball is seen, slow down and stop
            swerve.drive(new Translation2d(0, 0), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        // Returns false so it can be controlled by a button hold (SeriesAction or
        // Teleop toggle)
        return false;
    }

    @Override
    public void done() {
        swerve.stop();
        intake.setState(IntakeState.IDLE);
    }
}
