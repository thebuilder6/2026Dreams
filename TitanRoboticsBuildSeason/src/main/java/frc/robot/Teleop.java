package frc.robot;

import frc.robot.Devices.Controller;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Subsystems.SwerveBase;
import frc.robot.Subsystems.Shooter;
import frc.robot.Data.PortMap;
import frc.robot.Data.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Teleop {

    Controller driverController; // object of Controller
    SwerveBase swerveBase; // object of SwerveBase

    private double controllerLeftX; // variable for the left x joystick axis
    private double controllerLeftY; // variable for the left y joystick axis
    private double controllerRightX; // variable for the right x joystick axis
    private double controllerRightY;
    private boolean controllerAButton;
    private boolean controllerRightBumper; // variable for if the right bumper is pressed
    double rotationX;
    double rotationY;

    public Teleop() {
        driverController = new Controller(PortMap.DRIVER_CONTROLLER); // creates a new controller
        swerveBase = SwerveBase.getInstance(); // gets an instance of SwerveBase
    }

    public void teleopPeriodic() // everything in this method will get executed
    {
        driveBaseControl(); // executes the driveBaseControl method
    }

    public void driveBaseControl() {
        controllerLeftY = driverController.getLeftY(); // sets the variable controllerLeftY to the actual data coming
                                                       // from the controller
        controllerLeftX = driverController.getLeftX(); // sets the variable controllerLeftX to the actual data coming
                                                       // from the controller
        controllerRightX = driverController.getRightX(); // sets the variable controllerRightX to the actual data coming
                                                         // from the controller
        controllerRightY = driverController.getRightY(); // sets the variable controllerRightY to the actual data coming
                                                         // from the controller
        controllerAButton = driverController.getAButton();
        controllerRightBumper = driverController.getRightBumperButton(); // sets the variable controllerRightBumper to
                                                                         // the actual data coming from the controller
        double rightTrigger = driverController.getRightTriggerAxis();

        double forward;
        double strafe; // Rhea this means going side to side
        double rotation = 0;
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        boolean isFieldOrriented = true;

        if (Math.abs(controllerLeftY) >= 0.1) {
            forward = -(controllerLeftY) * Constants.MAX_SPEED;
        } else {
            forward = 0;
        }
        if (Math.abs(controllerLeftX) >= 0.1) {
            strafe = -(controllerLeftX) * Constants.MAX_SPEED;
        } else {
            strafe = 0;
        }
        if (Math.abs(controllerRightX) >= 0.1) {
            rotation = -((Math.abs(controllerRightX)) * (controllerRightX)) * Constants.MAX_ROTATION_SPEED;
        } else {
            rotation = 0;
        }

        boolean isSnapMode = Math.abs(controllerRightX) >= 0.9 || Math.abs(controllerRightY) >= 0.9;
        boolean isManualRotation = !isSnapMode && Math.abs(controllerRightX) >= 0.1;

        if (isSnapMode) {
            // Update target rotation from joystick when in snap mode
            rotationX = controllerRightX;
            rotationY = controllerRightY;
        } else if (!isManualRotation) {
            // Reset rotation target when not actively controlling rotation
            // This prevents stale values from causing unwanted snap behavior
            rotationX = 0;
            rotationY = 0;
        }

        if (controllerAButton) {
            swerveBase.zeroGyroWithAlliance();
            rotationX = 0;
            rotationY = isRed ? 1 : -1; // Face away from the alliance wall
            isSnapMode = true; // Force snap to the new zero
        }

        if (rightTrigger >= 0.5) {
            Shooter shooter = Shooter.getInstance();
            var solution = shooter.calculateShootingSolution(swerveBase.getPose(), swerveBase.getFieldVelocity());

            if (solution.possible()) {
                shooter.setFlywheelVelocity(solution.flywheelRPM());

                Rotation2d targetHeading = solution.turretAngle();
                Rotation2d currentHeading = swerveBase.getHeading();
                edu.wpi.first.math.kinematics.ChassisSpeeds targetSpeeds = swerveBase.getTargetSpeeds(forward, strafe,
                        targetHeading);

                System.out.printf("Auto-Aim - Curr: %.2f deg, Target: %.2f deg, Error: %.2f deg, Corr: %.2f rad/s\n",
                        currentHeading.getDegrees(), targetHeading.getDegrees(),
                        targetHeading.minus(currentHeading).getDegrees(), targetSpeeds.omegaRadiansPerSecond);

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Current Heading Deg",
                        currentHeading.getDegrees());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Target Heading Deg",
                        targetHeading.getDegrees());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Error Deg",
                        targetHeading.minus(currentHeading).getDegrees());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Rotation Correction RadPerSec",
                        targetSpeeds.omegaRadiansPerSecond);

                swerveBase.driveFieldOriented(targetSpeeds);

                double headingError = Math.abs(swerveBase.getPose().getRotation().minus(targetHeading).getDegrees());
                if (headingError < 2.5 && shooter.isAtTargetVelocity()) {
                    shooter.setFeederSpeed(Constants.ShooterConstants.FEED_SPEED);
                } else {
                    shooter.setFeederSpeed(0);
                }
                return;
            }
        } else {
            Shooter.getInstance().stop();
        }

        if (isFieldOrriented) {
            // Translation flip for Red alliance
            double finalForward = isRed ? -forward : forward;
            double finalStrafe = isRed ? -strafe : strafe;

            if (isSnapMode) {
                // Snap-to-Angle logic
                Rotation2d targetHeading;
                if (Math.abs(rotationX) < 1e-6 && Math.abs(rotationY) < 1e-6) {
                    targetHeading = swerveBase.getPose().getRotation();
                } else {
                    targetHeading = new Rotation2d(-rotationY, -rotationX);
                }
                if (isRed) {
                    targetHeading = targetHeading.plus(Rotation2d.fromDegrees(180));
                }
                Rotation2d currentHeading = swerveBase.getHeading();
                edu.wpi.first.math.kinematics.ChassisSpeeds targetSpeeds = swerveBase.getTargetSpeeds(finalForward,
                        finalStrafe, targetHeading);

                System.out.printf(
                        "Snap - Curr: %.2f deg (%.4f rad), Target: %.2f deg, Error: %.2f deg, Corr: %.2f rad/s\n",
                        currentHeading.getDegrees(), currentHeading.getRadians(),
                        targetHeading.getDegrees(),
                        targetHeading.minus(currentHeading).getDegrees(), targetSpeeds.omegaRadiansPerSecond);

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Current Heading Deg",
                        currentHeading.getDegrees());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Target Heading Deg",
                        targetHeading.getDegrees());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Error Deg",
                        targetHeading.minus(currentHeading).getDegrees());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Teleop/Rotation Correction RadPerSec",
                        targetSpeeds.omegaRadiansPerSecond);

                swerveBase.driveFieldOriented(targetSpeeds);
            } else if (isManualRotation) {
                // Manual rotation logic
                swerveBase.drive(new Translation2d(finalForward, finalStrafe), rotation, true);
            } else {
                // No rotation
                swerveBase.drive(new Translation2d(finalForward, finalStrafe), 0, true);
            }
        } else {
            swerveBase.drive(new Translation2d(forward, strafe), rotation, false);
        }
    }
}
