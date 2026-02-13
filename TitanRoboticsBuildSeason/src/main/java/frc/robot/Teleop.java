package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.PortMap;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

public class Teleop {

    Controller driverController; // object of Controller
    SwerveBase swerveBase; // object of SwerveBase
    Intake intake; // object of Intake

    private double rotationX;
    private double rotationY;
    private boolean isSnapMode = false;

    private boolean wasGlideHeld = false;

    private frc.robot.Interfaces.Actions activeAction = null;

    public Teleop() {
        driverController = new Controller(PortMap.DRIVER_CONTROLLER);
        swerveBase = SwerveBase.getInstance();
        intake = Intake.getInstance();
    }

    public void reset() {
        activeAction = null;
        isSnapMode = false;
        wasGlideHeld = false;
        intakeFeedToggle = false;
    }

    public void teleopPeriodic() {
        if (handleActiveAction())
            return;
        handleIntakeControls();
        driveBaseControl();
    }

    private boolean handleActiveAction() {
        if (activeAction == null)
            return false;

        boolean isOverride = Math.abs(driverController.getLeftY()) > 0.1 ||
                Math.abs(driverController.getLeftX()) > 0.1 ||
                Math.abs(driverController.getRightX()) > 0.1;

        boolean isGlideHeld = driverController.getLeftTriggerAxis() > 0.5;
        boolean isBallHuntHeld = driverController.getLeftBumperButton();

        if (!(isGlideHeld || isBallHuntHeld) || activeAction.isFinished() || isOverride) {
            activeAction.done();
            activeAction = null;
            return false;
        }

        activeAction.update();
        return true;
    }

    private boolean wasXButtonPressed = false;
    private boolean intakeFeedToggle = false; // false = Intake, true = Feed

    private void handleIntakeControls() {
        // Intake controls using available buttons
        // Right Bumper (toggle): Intake/Feed (Feed pulls arm in and runs hopper only
        // for safety)
        // Left Bumper: Eject (runs rollers and hopper in reverse)
        // X Button: Stop/Idle

        // Handle Right Bumper toggle logic
        boolean xButtonPressed = driverController.getXButton();
        if (xButtonPressed && !wasXButtonPressed) {
            intakeFeedToggle = !intakeFeedToggle; // Toggle state

            if (intakeFeedToggle) {
                intake.setState(Intake.IntakeState.FEEDING); // Safe mode: arm in, hopper only
            } else {
                intake.setState(Intake.IntakeState.INTAKING); // Full intake mode
            }
        }
        wasXButtonPressed = xButtonPressed;

        // Other controls
        if (driverController.getBButton()) {
            intake.setState(Intake.IntakeState.EJECTING);
        } else if (driverController.getYButton()) {
            intake.setState(Intake.IntakeState.IDLE);
            intakeFeedToggle = false; // Reset toggle when going to idle
        }
    }

    public void driveBaseControl() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        // 1. Process Inputs (Deadbands handled in Controller class)
        double speedMultiplier = Dashboard.isSlowModeEnabled() ? 0.35 : 1.0;
        double forward = -driverController.getLeftY() * Constants.MAX_SPEED * speedMultiplier;
        double strafe = -driverController.getLeftX() * Constants.MAX_SPEED * speedMultiplier;
        double manualRotation = -(driverController.getRightX() * Math.abs(driverController.getRightX()))
                * Constants.MAX_ROTATION_SPEED * speedMultiplier;

        double rightTrigger = driverController.getRightTriggerAxis();

        double leftTrigger = driverController.getLeftTriggerAxis();
        boolean isGlideHeld = leftTrigger > 0.5;
        boolean glidePressed = isGlideHeld && !wasGlideHeld;
        wasGlideHeld = isGlideHeld;

        if (Dashboard.isGlidePointsEnabled() && glidePressed && activeAction == null) {
            var nearest = swerveBase.getNearestGlidePoint();
            if (nearest != null) {
                if (nearest.isTunnelEntrance && nearest.tunnelExitPose != null) {
                    activeAction = new frc.robot.Auto.Actions.TunnelAction(nearest.pose, nearest.tunnelExitPose);
                } else {
                    activeAction = new frc.robot.Auto.Actions.DriveToPoseAction(nearest.pose);
                }
                activeAction.start();
                return;
            }
        }

        // 2. Handle Alliance Flicking (Field-Relative)
        double finalForward = isRed ? -forward : forward;
        double finalStrafe = isRed ? -strafe : strafe;

        // 3. Handle Special Modes (Shooting, Zeroing, Snapping)
        if (driverController.getAButton()) {
            swerveBase.zeroGyroWithAlliance();
            rotationX = 0;
            rotationY = isRed ? 1 : -1;
            isSnapMode = true;
        }

        if (Dashboard.isBallHuntEnabled() && driverController.getLeftBumperButton() && activeAction == null) {
            activeAction = new frc.robot.Auto.Actions.BallHuntAction();
            activeAction.start();
            return;
        }

        if (Dashboard.isAutoAimEnabled() && rightTrigger >= 0.5) {
            if (handleAutoAim(forward, strafe, finalForward, finalStrafe))
                return;
        } else {
            Shooter.getInstance().stop();
        }

        updateRotationState();
        applyDrive(finalForward, finalStrafe, manualRotation, isRed);
    }

    private void updateRotationState() {
        double rX = driverController.getRightX();
        double rY = driverController.getRightY();

        boolean snapEnabled = Dashboard.isSnapToTurnEnabled();
        isSnapMode = snapEnabled && (Math.abs(rX) >= 0.97 || Math.abs(rY) >= 0.97);
        boolean isManualRotation = !isSnapMode && Math.abs(rX) > 0; // Constants.OperatorConstants.DEADBAND already
                                                                    // applied

        if (isSnapMode) {
            rotationX = rX;
            rotationY = rY;
        } else if (!isManualRotation) {
            rotationX = 0;
            rotationY = 0;
        }
    }

    private boolean handleAutoAim(double forward, double strafe, double finalF, double finalS) {
        Shooter shooter = Shooter.getInstance();
        var solution = shooter.calculateShootingSolution(swerveBase.getPose(), swerveBase.getFieldVelocity());

        if (solution.possible()) {
            shooter.setFlywheelVelocity(solution.flywheelRPM());
            Rotation2d targetHeading = solution.turretAngle();
            edu.wpi.first.math.kinematics.ChassisSpeeds targetSpeeds = swerveBase.getTargetSpeeds(forward, strafe,
                    targetHeading);

            swerveBase.drive(new Translation2d(finalF, finalS), targetSpeeds.omegaRadiansPerSecond, true);
            logAutoAim(targetHeading, targetSpeeds.omegaRadiansPerSecond);

            shooter.setFeederSpeed(shooter.isReadyToFire(targetHeading) ? Constants.ShooterConstants.FEED_SPEED : 0);
            return true;
        }
        return false;
    }

    private void applyDrive(double finalForward, double finalStrafe, double manualRotation, boolean isRed) {
        if (isSnapMode) {
            Rotation2d targetHeading = (Math.abs(rotationX) < 1e-6 && Math.abs(rotationY) < 1e-6)
                    ? swerveBase.getPose().getRotation()
                    : new Rotation2d(-rotationY, -rotationX);

            if (isRed)
                targetHeading = targetHeading.plus(Rotation2d.fromDegrees(180));

            edu.wpi.first.math.kinematics.ChassisSpeeds targetSpeeds = swerveBase.getTargetSpeeds(finalForward,
                    finalStrafe, targetHeading);
            swerveBase.drive(new Translation2d(finalForward, finalStrafe), targetSpeeds.omegaRadiansPerSecond,
                    Dashboard.isFieldOrientedEnabled());
            logSnap(targetHeading, targetSpeeds.omegaRadiansPerSecond);
        } else {
            swerveBase.drive(new Translation2d(finalForward, finalStrafe), manualRotation,
                    Dashboard.isFieldOrientedEnabled());
        }
    }

    private void logAutoAim(Rotation2d target, double corr) {
        Rotation2d current = swerveBase.getHeading();
        SmartDashboard.putNumber("Subsystems/Swerve/Tuning/Target Heading Deg", target.getDegrees());
        SmartDashboard.putNumber("Subsystems/Swerve/Tuning/Error Deg", target.minus(current).getDegrees());
        SmartDashboard.putNumber("Subsystems/Swerve/Tuning/Rotation Correction RadPerSec", corr);
    }

    private void logSnap(Rotation2d target, double corr) {
        logAutoAim(target, corr); // Currently identical
    }
}
