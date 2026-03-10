package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Data.Constants;
import frc.robot.Data.PortMap;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

public class Teleop {

    Controller driverController;
    Controller operatorController;
    SwerveBase swerveBase;
    Intake intake;
    Shooter shooter;
    Climber climber;

    private double rotationX;
    private double rotationY;
    private boolean isSnapMode = false;
    private boolean wasGlideHeld = false;
    private boolean intakeFeedToggle = false;

    private frc.robot.Interfaces.Actions activeAction = null;

    public Teleop() {
        driverController = new Controller(PortMap.DRIVER_CONTROLLER);
        operatorController = new Controller(PortMap.OPERATOR_CONTROLLER);
        swerveBase = SwerveBase.getInstance();
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
        climber = Climber.getInstance();
    }

    public void reset() {
        activeAction = null;
        isSnapMode = false;
        wasGlideHeld = false;
        intakeFeedToggle = false;
    }

    public void teleopPeriodic() {
        if (handleEStop()) return;
        if (handleActiveAction()) return;

        handleIntakeControls();
        handleShooterControls();
        handleClimberControls();
        driveBaseControl();
    }

    private boolean handleEStop() {
        if (operatorController.getPOV() == 180) {
            swerveBase.stop();
            intake.setState(Intake.IntakeState.IDLE);
            shooter.stop();
            activeAction = null;
            return true;
        }
        return false;
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

    private void handleIntakeControls() {
        // Driver toggle (Auto)
        if (driverController.getRightBumperPressed()) {
            intakeFeedToggle = !intakeFeedToggle;
            intake.setState(intakeFeedToggle ? Intake.IntakeState.INTAKING : Intake.IntakeState.IDLE);
        }

        // Operator manual overrides
        double manualArmY = -operatorController.getLeftY();
        if (Math.abs(manualArmY) > 0.1) {
            intake.setArmVoltage(manualArmY * 6.0); // Manual voltage override
        }

        if (operatorController.getLeftTriggerAxis() > 0.5) {
            intake.setState(Intake.IntakeState.FEEDING);
        } else if (operatorController.getYButton()) {
            intake.setState(Intake.IntakeState.EJECTING);
        } else if (operatorController.getBButton()) {
            intake.setState(Intake.IntakeState.IDLE);
            intakeFeedToggle = false;
        }
    }

    private void handleShooterControls() {
        // Driver Auto-Aim
        if (Dashboard.isAutoAimEnabled() && driverController.getRightTriggerAxis() >= 0.5) {
            handleAutoAim();
        } else {
            // Operator manual overrides
            double manualShootTrigger = operatorController.getRightTriggerAxis();
            if (manualShootTrigger > 0.1) {
                shooter.setFlywheelVelocity(3000 * manualShootTrigger); // Linear mapping for troubleshooting
                if (operatorController.getRightBumperButton()) {
                    shooter.setKickerSpeed(Constants.ShooterConstants.FEED_SPEED);
                } else {
                    shooter.setKickerSpeed(0);
                }
            } else if (operatorController.getXButton()) {
                shooter.setFlywheelVelocity(-500); // Reverse slow
                shooter.setKickerSpeed(-0.3);
            } else {
                shooter.stop();
            }
        }
    }

    private void handleClimberControls() {
        Climber.ClimberState nextState = switch (operatorController.getPOV()) {
            case 0 -> Climber.ClimberState.UP;
            case 180 -> Climber.ClimberState.DOWN;
            default -> Climber.ClimberState.STATIONARY;
        };
        climber.setState(nextState);
    }

    public void driveBaseControl() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        double speedMultiplier = Dashboard.isSlowModeEnabled() ? 0.35 : 1.0;
        double forward = -driverController.getLeftY() * Constants.MAX_SPEED * speedMultiplier;
        double strafe = -driverController.getLeftX() * Constants.MAX_SPEED * speedMultiplier;
        double manualRotation = -(driverController.getRightX() * Math.abs(driverController.getRightX()))
                * Constants.MAX_ROTATION_SPEED * speedMultiplier;

        boolean isGlideHeld = driverController.getLeftTriggerAxis() > 0.5;
        if (Dashboard.isGlidePointsEnabled() && isGlideHeld && activeAction == null && !wasGlideHeld) {
            var nearest = swerveBase.getNearestGlidePoint();
            if (nearest != null) {
                if (nearest.isTunnelEntrance && nearest.tunnelExitPose != null) {
                    activeAction = new frc.robot.Auto.Actions.TrajectoryTunnelAction(nearest.pose,
                            nearest.tunnelExitPose);
                } else {
                    activeAction = new frc.robot.Auto.Actions.DriveToPoseAction(nearest.pose);
                }
                activeAction.start();
                wasGlideHeld = true;
                return;
            }
        }
        wasGlideHeld = isGlideHeld;

        double finalForward = isRed ? -forward : forward;
        double finalStrafe = isRed ? -strafe : strafe;

        if (driverController.getAButton()) {
            swerveBase.zeroGyroWithAlliance();
        }

        if (Dashboard.isBallHuntEnabled() && driverController.getLeftBumperButton() && activeAction == null) {
            activeAction = new frc.robot.Auto.Actions.BallHuntAction();
            activeAction.start();
            return;
        }

        updateRotationState();
        applyDrive(finalForward, finalStrafe, manualRotation, isRed);
    }

    private void updateRotationState() {
        double rX = driverController.getRightX();
        double rY = driverController.getRightY();
        isSnapMode = Dashboard.isSnapToTurnEnabled() && (Math.abs(rX) >= 0.97 || Math.abs(rY) >= 0.97);
        if (isSnapMode) {
            rotationX = rX;
            rotationY = rY;
        }
    }

    private void handleAutoAim() {
        double forward = -driverController.getLeftY() * Constants.MAX_SPEED;
        double strafe = -driverController.getLeftX() * Constants.MAX_SPEED;
        
        var solution = shooter.calculateShootingSolution(swerveBase.getPose(), swerveBase.getFieldVelocity());
        if (solution.possible()) {
            shooter.setFlywheelVelocity(solution.flywheelRPM());
            Rotation2d targetHeading = solution.turretAngle();
            var targetSpeeds = swerveBase.getTargetSpeeds(forward, strafe, targetHeading);
            swerveBase.drive(new Translation2d(forward, strafe), targetSpeeds.omegaRadiansPerSecond, true);
            shooter.setKickerSpeed(shooter.isReadyToFire(targetHeading) ? Constants.ShooterConstants.FEED_SPEED : 0);
        }
    }

    private void applyDrive(double finalForward, double finalStrafe, double manualRotation, boolean isRed) {
        if (isSnapMode) {
            Rotation2d targetHeading = new Rotation2d(-rotationY, -rotationX);
            if (isRed) targetHeading = targetHeading.plus(Rotation2d.fromDegrees(180));
            var targetSpeeds = swerveBase.getTargetSpeeds(finalForward, finalStrafe, targetHeading);
            swerveBase.drive(new Translation2d(finalForward, finalStrafe), targetSpeeds.omegaRadiansPerSecond, true);
        } else {
            swerveBase.drive(new Translation2d(finalForward, finalStrafe), manualRotation, true);
        }
    }
}
