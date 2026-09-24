package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Utils.AlertManager;
import frc.robot.Auto.Actions.AutoAimAction;
import frc.robot.Auto.Actions.BallHuntAction;
import frc.robot.Auto.Actions.DriveToPoseAction;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Data.PortMap;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.ShootingSolution;
import frc.robot.Subsystems.SwerveBase;

/*
 * Class: Teleop
 * Description: Teleoperated control system unified onto a single Xbox controller,
 *              combining driving, intake, auto-aim shooting, and advanced navigation features.
 * Authors: Austin, Rhea, Sarah, Trevor, Mai, InfiniteQuery
 */
public class Teleop {

    // Subsystems
    Intake intake;
    Shooter shooter;
    SwerveBase swerveBase;

    // Single Controller
    Controller controller;
    Joystick joystickController;

    public static boolean joystickEnabled = false;

    // Controller Inputs
    private double leftX;
    private double leftY;
    private double rightX;
    private double leftTrigger;
    private double rightTrigger;
    private boolean aButton;
    private boolean bButton;
    private boolean xButton;
    private boolean yButton;
    private boolean leftBumper;
    private boolean rightBumper;
    private int pov;

    // Intake Toggle States
    private String intakeToggleState = "Disabled";
    private boolean lastXButton = false;
    private boolean armOverrideActive = false;
    private boolean lastBButton = false;

    // Drive Variables
    private double driverForward;
    private double driverStrafe;

    // Shooter
    ShootingSolution shootingSolution;

    // Advanced Actions (Glide / Ball Hunt)
    private frc.robot.Interfaces.Actions activeAction = null;
    private boolean wasGlideHeld = false;

    // Haptic Feedback Tracking
    private boolean wasTargetLocked = false;
    private boolean warned30s = false;
    private boolean warned15s = false;
    private boolean wasVisionDegraded = false;
    private boolean wasHardwareError = false;

    public Teleop() {
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
        swerveBase = SwerveBase.getInstance();

        if (!joystickEnabled) {
            controller = new Controller(PortMap.DRIVER_CONTROLLER);
        } else {
            joystickController = new Joystick(PortMap.DRIVER_CONTROLLER);
        }
    }

    public void init() {
        intakeToggleState = "Disabled";
        intake.setState("Disabled");
        activeAction = null;
        wasTargetLocked = false;
        warned30s = false;
        warned15s = false;
        wasVisionDegraded = false;
        wasHardwareError = false;
    }

    public void reset() {
        init();
    }

    public void teleopPeriodic() {
        readControllers();
        updateHapticFeedback();

        // E-Stop check: Down on D-Pad
        if (pov == 180) {
            intake.setState("Disabled");
            shooter.stop();
            if (activeAction != null) {
                activeAction.done();
                activeAction = null;
            }
            return;
        }

        // Check if an advanced action is running (Glide or Ball Hunt)
        if (handleActiveAction()) {
            return;
        }

        shootingSolution = shooter.getLatestShootingSolution();
        driveBaseControl();
        intakeControl();
        shooterControl();
    }

    private void updateHapticFeedback() {
        if (!joystickEnabled && controller != null) {
            controller.updateRumble();
        }

        // Match Time Warnings (Endgame reminders)
        double matchTime = DriverStation.getMatchTime();
        if (matchTime > 0.0) {
            if (matchTime <= 30.0 && matchTime > 28.5 && !warned30s) {
                if (!joystickEnabled && controller != null) {
                    controller.triggerRumblePattern(Controller.RumblePattern.MATCH_TIME_WARNING);
                }
                warned30s = true;
            } else if (matchTime <= 15.0 && matchTime > 13.5 && !warned15s) {
                if (!joystickEnabled && controller != null) {
                    controller.triggerRumblePattern(Controller.RumblePattern.MATCH_TIME_WARNING);
                }
                warned15s = true;
            }
        }

        // Sensor / Vision Degradation Alert Pulse
        boolean visionDegraded = swerveBase.isVisionDegraded();
        if (visionDegraded && !wasVisionDegraded) {
            if (!joystickEnabled && controller != null) {
                controller.triggerRumblePattern(Controller.RumblePattern.HARDWARE_WARNING);
            }
        }
        wasVisionDegraded = visionDegraded;

        // General Hardware Error Alert Pulse
        boolean errorActive = AlertManager.hasActiveErrors();
        if (errorActive && !wasHardwareError) {
            if (!joystickEnabled && controller != null) {
                controller.triggerRumblePattern(Controller.RumblePattern.HARDWARE_WARNING);
            }
        }
        wasHardwareError = errorActive;
    }

    private void readControllers() {
        if (!joystickEnabled && controller != null) {
            leftY = controller.getLeftY();
            leftX = controller.getLeftX();
            rightX = controller.getRightX();
            leftTrigger = controller.getLeftTriggerAxis();
            rightTrigger = controller.getRightTriggerAxis();
            aButton = controller.getAButton();
            bButton = controller.getBButton();
            xButton = controller.getXButton();
            yButton = controller.getYButton();
            leftBumper = controller.getLeftBumperButton();
            rightBumper = controller.getRightBumperButton();
            pov = controller.getPOV();
        } else if (joystickController != null) {
            leftY = joystickController.getY();
            leftX = joystickController.getX();
            rightX = joystickController.getTwist();
            leftTrigger = 0.0;
            rightTrigger = joystickController.getRawButton(1) ? 1.0 : 0.0;
            aButton = joystickController.getRawButton(2);
            bButton = joystickController.getRawButton(3);
            xButton = joystickController.getRawButton(4);
            yButton = joystickController.getRawButton(5);
            leftBumper = joystickController.getRawButton(6);
            rightBumper = joystickController.getRawButton(7);
            pov = joystickController.getPOV();
        }
    }

    private boolean handleActiveAction() {
        // Advanced Modes: Glide (Right Bumper or D-Pad Up) / Ball Hunt (Left Bumper)
        boolean isGlideHeld = !joystickEnabled && (rightBumper || pov == 0) && Dashboard.isGlidePointsEnabled();
        boolean isBallHuntHeld = !joystickEnabled && leftBumper && Dashboard.isBallHuntEnabled();

        // Start Glide Action if newly triggered
        if (isGlideHeld && !wasGlideHeld && activeAction == null) {
            GlideConstants.GlidePoint nearest = swerveBase.getNearestGlidePoint();
            if (nearest != null) {
                activeAction = new DriveToPoseAction(nearest.pose());
                activeAction.start();
            }
        }
        wasGlideHeld = isGlideHeld;

        // Start Ball Hunt Action if newly triggered
        if (isBallHuntHeld && activeAction == null) {
            activeAction = new BallHuntAction();
            activeAction.start();
        }

        if (activeAction == null) return false;

        // Manual driver override cancels autonomous action
        boolean isDriverOverride = Math.abs(leftY) > 0.15 || Math.abs(leftX) > 0.15 || Math.abs(rightX) > 0.15;

        if (!(isGlideHeld || isBallHuntHeld) || activeAction.isFinished() || isDriverOverride) {
            activeAction.done();
            activeAction = null;
            return false;
        }

        activeAction.update();
        return true;
    }

    public void intakeControl() {
        // --- Arm Lock Toggle (B Button) ---
        if (bButton && !lastBButton) {
            armOverrideActive = !armOverrideActive;
        }
        lastBButton = bButton;

        // --- Intake Arm Deploy Toggle (X Button) ---
        if (xButton && !lastXButton) {
            if (!armOverrideActive) {
                if (intakeToggleState.equals("Standby")) {
                    intakeToggleState = "Down";
                } else {
                    intakeToggleState = "Standby";
                }
            }
        }
        lastXButton = xButton;

        if (armOverrideActive) {
            intakeToggleState = "Standby";
        }

        // --- Roller and Arm Mapping ---
        boolean intakeRequested = leftTrigger > 0.5;
        boolean reverseRequested = yButton && intakeRequested;
        boolean armDown = intakeToggleState.equals("Down");

        if (reverseRequested) {
            if (intakeToggleState.equals("Disabled")) intakeToggleState = "Standby";
            intake.setState(armDown ? "Reversed" : "StandbyReversed");
        } else if (intakeRequested) {
            if (intakeToggleState.equals("Disabled")) intakeToggleState = "Standby";
            intake.setState(armDown ? "Intaking" : "StandbyIntaking");
        } else if (!intakeToggleState.equals("Disabled")) {
            intake.setState(armDown ? "Down" : "Standby");
        } else {
            intake.setState("Disabled");
        }
    }

    public void driveBaseControl() {
        double rotation = 0;

        // Translation
        if (Math.abs(leftY) >= Constants.OperatorConstants.DEADBAND) {
            driverForward = leftY * Constants.MAX_SPEED;
        } else {
            driverForward = 0;
        }

        if (Math.abs(leftX) >= Constants.OperatorConstants.DEADBAND) {
            driverStrafe = leftX * Constants.MAX_SPEED;
        } else {
            driverStrafe = 0;
        }

        // Rotation (Quadratic curve for precision)
        if (Math.abs(rightX) >= Constants.OperatorConstants.DEADBAND) {
            rotation = -(Math.abs(rightX) * rightX) * Constants.MAX_ROTATION_SPEED;
        } else {
            rotation = 0;
        }

        // A Button: Zero Gyro with Alliance
        if (aButton) {
            swerveBase.zeroGyroWithAlliance();
        }

        boolean autoRequested = yButton && (leftTrigger <= 0.5);
        boolean autoAimActive = autoRequested && shootingSolution != null && shootingSolution.shotPossibility();
        if (!autoAimActive) {
            swerveBase.drive(new Translation2d(driverForward, driverStrafe), rotation, Dashboard.isFieldOriented());
        }
    }

    public void shooterControl() {
        boolean autoRequested = yButton && (leftTrigger <= 0.5);
        boolean manualRequested = rightTrigger > 0.05 && !yButton;

        if (autoRequested) {
            if (shootingSolution != null && shootingSolution.shotPossibility()) {
                // Auto-aim Swerve override (allow translation while overriding rotation to target)
                swerveBase.driveFieldOriented(swerveBase.getTargetSpeeds(driverForward, driverStrafe, shootingSolution.shootingAngle()));

                // Spool up flywheels to distance solution
                shooter.setTargetRPM(shootingSolution.flywheelRpmLeft(), shootingSolution.flywheelRpmRight());

                boolean headingAligned = Math.abs(shootingSolution.shootingAngle().minus(swerveBase.getHeading()).getDegrees()) < 3.0;
                boolean flywheelsReady = shooter.isAtTargetVelocity();
                boolean targetLocked = headingAligned && flywheelsReady;

                if (targetLocked && !wasTargetLocked) {
                    if (!joystickEnabled && controller != null) {
                        controller.triggerRumblePattern(Controller.RumblePattern.TARGET_LOCKED);
                    }
                }
                wasTargetLocked = targetLocked;

                if (headingAligned) {
                    shooter.shoot();
                } else {
                    shooter.prepareToShoot();
                }
            } else {
                shooter.stop();
                wasTargetLocked = false;
            }
        } else if (manualRequested) {
            wasTargetLocked = false;
            shooter.manualFire(rightTrigger);
        } else {
            wasTargetLocked = false;
            if (pov != 180) {
                shooter.stop();
            }
        }
    }
}
