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
    private final frc.robot.Subsystems.Vision vision = frc.robot.Subsystems.Vision.getInstance();
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");

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
        if (vision.hasGamePiece()) {
            double yaw = vision.getGamePieceYaw();
            double rotationOutput = -turnController.calculate(yaw, 0);

            // True 2D Holonomic Vectoring using Rubik Pi ground projection
            Translation2d robotRel = vision.getGamePieceRobotRelativeTranslation();
            double distance = vision.getGamePieceDistanceMeters();

            double forwardSpeed;
            double strafeSpeed;

            if (distance > 0.05) {
                // Scale speed smoothly: full speed at >1.5m, smoothly tapering down near bumper
                double speedScale = Math.min(driveSpeed, Math.max(1.0, distance * 1.5));
                Translation2d normalizedDir = robotRel.div(robotRel.getNorm());
                forwardSpeed = normalizedDir.getX() * speedScale;
                strafeSpeed = normalizedDir.getY() * speedScale;
            } else {
                // Fallback to forward drive if distance calculation is uncalibrated
                forwardSpeed = driveSpeed;
                strafeSpeed = 0.0;
            }

            swerve.drive(new Translation2d(forwardSpeed, strafeSpeed), rotationOutput, false);
        } else {
            double tv = limelightTable.getEntry("tv").getDouble(0);
            if (tv > 0) {
                double tx = limelightTable.getEntry("tx").getDouble(0);
                double rotationOutput = -turnController.calculate(tx, 0);
                swerve.drive(new Translation2d(driveSpeed, 0), rotationOutput, false);
            } else {
                // If no ball is seen, slow down and stop
                swerve.drive(new Translation2d(0, 0), 0, false);
            }
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
