package frc.robot;

import frc.robot.HMI.Alerts.Alert;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.HMI.CoPilot;
import frc.robot.Auto.Actions.BallHuntAction;
import frc.robot.Auto.Actions.DriveToPoseAction;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Data.PortMap;
import frc.robot.HMI.Controller;
import frc.robot.HMI.Controller.RumblePattern;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.ShootingSolution;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.HMI.Alerts.AlertManager;
import frc.robot.Utils.AllianceFlipUtil;

/*
 * Class: Teleop
 * Description: Teleoperated control system supporting dual Xbox controllers (Driver & Operator)
 *              with seamless solo-controller fallback, canonical Blue-origin field orientation,
 *              non-linear cubic input shaping, slew rate limiting, cardinal snap-to-heading,
 *              slow mode, ground intaking, and auto-aim shooting.
 * Authors: Austin, Rhea, Sarah, Trevor, Mai, InfiniteQuery
 */
public class Teleop {

    // Subsystems
    private final Intake intake;
    private final Shooter shooter;
    private final SwerveBase swerveBase;

    // Controllers
    private Controller driverController;
    private Controller operatorController;
    private Joystick joystickController;

    public static boolean joystickEnabled = false;

    // Slew Rate Limiters (True 2D vector for translation, 1D scalar for rotation)
    private frc.robot.Utils.Vector2dSlewRateLimiter translationLimiter = new frc.robot.Utils.Vector2dSlewRateLimiter(
            Constants.OperatorConstants.TRANSLATION_SLEW_RATE.get());
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.OperatorConstants.ROTATION_SLEW_RATE.get());

    // Normalized Driver Inputs (+Forward, +Left, +CCW)
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
    private boolean backButton;
    private boolean startButton;
    private boolean leftStickButton;
    private int pov;

    // State Variables
    private boolean slowModeActive = false;
    private boolean lastLeftStickButton = false;
    private boolean lastDashboardSlowMode = false;
    private boolean armDeployed = false;
    private boolean lastXButton = false;
    private Rotation2d snapTargetHeading = null;

    // Drive Speeds (in canonical Field Coordinates)
    private double driverForward;
    private double driverStrafe;
    private double driverRotation;

    // Advanced Actions (Glide / Ball Hunt)
    private final CoPilot coPilot = CoPilot.getInstance();
    private frc.robot.Interfaces.Actions activeAction = null;
    private boolean wasGlideHeld = false;

    // Haptic Feedback Tracking
    private boolean wasTargetLocked = false;
    private boolean warned30s = false;
    private boolean warned15s = false;
    private boolean wasVisionDegraded = false;
    private boolean wasHardwareError = false;
    private boolean wasCollisionDetected = false;

    // Deadbands (Circular 2D translation and 1D rotation)
    public static final double TRANSLATION_DEADBAND = 0.08;
    public static final double ROTATION_DEADBAND = 0.06;

    // Multi-tap trigger for critical gyro re-zeroing (prevents accidental single-press resets)
    private final edu.wpi.first.wpilibj2.command.button.Trigger zeroGyroTrigger;

    public Teleop() {
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
        swerveBase = SwerveBase.getInstance();

        if (!joystickEnabled) {
            driverController = new Controller(PortMap.DRIVER_CONTROLLER);
            operatorController = new Controller(PortMap.OPERATOR_CONTROLLER);
        } else {
            joystickController = new Joystick(PortMap.DRIVER_CONTROLLER);
        }

        // MultiTapTrigger: Double-tap A button within 0.4s to re-zero gyro with alliance
        zeroGyroTrigger = new edu.wpi.first.wpilibj2.command.button.Trigger(
            () -> !joystickEnabled && driverController != null && driverController.getAButton()
        ).multiPress(2, 0.4);
    }

    public void init() {
        armDeployed = false;
        slowModeActive = false;
        lastDashboardSlowMode = false;
        Dashboard.setSlowModeEnabled(false);
        snapTargetHeading = null;
        intake.setState("Disabled");
        shooter.stop();
        coPilot.stopAssist();
        if (activeAction != null) {
            activeAction.done();
            activeAction = null;
        }
        wasGlideHeld = false;
        wasTargetLocked = false;
        warned30s = false;
        warned15s = false;
        wasVisionDegraded = false;
        wasHardwareError = false;

        translationLimiter.reset(0.0, 0.0);
        rotationLimiter.reset(0.0);
    }

    public void reset() {
        init();
    }

    /**
     * Non-linear cubic input shaping function: f(x) = sign(x) * (0.7 * |x|^3 + 0.3 * |x|)
     * Provides high precision around the center while maintaining 100% full-throttle output.
     */
    public static double shapeInput(double input) {
        double abs = Math.abs(input);
        return Math.signum(input) * (0.7 * abs * abs * abs + 0.3 * abs);
    }

    /**
     * True 2D circular magnitude deadband with continuous radial scaling and cubic shaping.
     * Prevents stick drift, square corner clipping, and low-speed motor crawling.
     */
    public static Translation2d apply2DDeadbandAndShape(double x, double y, double deadband) {
        double magnitude = Math.hypot(x, y);
        if (magnitude < deadband) {
            return new Translation2d(0.0, 0.0);
        }
        // Continuous linear normalization from [deadband, 1.0] -> [0.0, 1.0]
        double scaledMagnitude = Math.min(1.0, (magnitude - deadband) / (1.0 - deadband));
        double shapedMagnitude = shapeInput(scaledMagnitude);
        // Rescale along the exact radial angle vector
        return new Translation2d((x / magnitude) * shapedMagnitude, (y / magnitude) * shapedMagnitude);
    }

    public void teleopPeriodic() {
        readControllers();
        updateHapticFeedback();

        // E-Stop / Abort: Back or Start pressed
        if (backButton || startButton) {
            intake.setState("Disabled");
            shooter.stop();
            swerveBase.stop();
            coPilot.stopAssist();
            wasGlideHeld = false;
            if (activeAction != null) {
                activeAction.done();
                activeAction = null;
            }
            snapTargetHeading = null;
            return;
        }

        // Check if an advanced action is running (Glide or Ball Hunt)
        if (handleActiveAction()) {
            return;
        }

        driveBaseControl();
        intakeControl();
        shooterControl();
    }

    public void readControllers() {
        // 1. Sync Slow Mode with Elastic Dashboard toggle switch (e.g. if toggled remotely)
        boolean dashSlow = Dashboard.isSlowModeEnabled();
        if (dashSlow != lastDashboardSlowMode) {
            slowModeActive = dashSlow;
            lastDashboardSlowMode = dashSlow;
        }

        if (!joystickEnabled && driverController != null) {
            // Normalized Driver Intent:
            // Standard WPILib XboxController has negative Y on stick forward, negative X on stick left.
            // Inverting them normalizes: +1.0 = Forward, +1.0 = Left, +1.0 = CCW rotation.
            leftY = -driverController.getLeftY();
            leftX = -driverController.getLeftX();
            rightX = -driverController.getRightX();
            pov = driverController.getPOV();
            aButton = driverController.getAButton();
            leftBumper = driverController.getLeftBumperButton();
            rightBumper = driverController.getRightBumperButton();
            leftStickButton = driverController.getLeftStickButton();

            // 2. Toggle Slow Mode on Left Stick Button press (driver override)
            if (leftStickButton && !lastLeftStickButton) {
                slowModeActive = !slowModeActive;
                lastDashboardSlowMode = slowModeActive;
                Dashboard.setSlowModeEnabled(slowModeActive);
            }
            lastLeftStickButton = leftStickButton;

            // Dual Controller / Solo fallback merging
            boolean opConnected = operatorController != null && operatorController.isConnected();

            double drvLT = driverController.getLeftTriggerAxis();
            double drvRT = driverController.getRightTriggerAxis();
            double opLT = opConnected ? operatorController.getLeftTriggerAxis() : 0.0;
            double opRT = opConnected ? operatorController.getRightTriggerAxis() : 0.0;

            leftTrigger = Math.max(drvLT, opLT);
            rightTrigger = Math.max(drvRT, opRT);

            bButton = driverController.getBButton() || (opConnected && operatorController.getBButton());
            xButton = driverController.getXButton() || (opConnected && operatorController.getXButton());
            yButton = driverController.getYButton() || (opConnected && operatorController.getYButton());
            backButton = driverController.getBackButton() || (opConnected && operatorController.getBackButton());
            startButton = driverController.getStartButton() || (opConnected && operatorController.getStartButton());

            // Operator Arm Manual Jog (if operator connected and D-pad used)
            if (opConnected) {
                int opPOV = operatorController.getPOV();
                if (opPOV == 0) {
                    intake.setArmPosition(Constants.INTAKE_UP_POSITION);
                    armDeployed = false;
                } else if (opPOV == 180) {
                    intake.setArmPosition(Constants.INTAKE_DOWN_POSITION);
                    armDeployed = true;
                }
            }

        } else if (joystickController != null) {
            leftY = -joystickController.getY();
            leftX = -joystickController.getX();
            rightX = -joystickController.getTwist();
            leftTrigger = 0.0;
            rightTrigger = joystickController.getRawButton(1) ? 1.0 : 0.0;
            aButton = joystickController.getRawButton(2);
            bButton = joystickController.getRawButton(3);
            xButton = joystickController.getRawButton(4);
            yButton = joystickController.getRawButton(5);
            leftBumper = joystickController.getRawButton(6);
            rightBumper = joystickController.getRawButton(7);
            backButton = joystickController.getRawButton(8);
            startButton = joystickController.getRawButton(9);
            pov = joystickController.getPOV();
        }
    }

    private void updateHapticFeedback() {
        if (!joystickEnabled) {
            if (driverController != null) driverController.updateRumble();
            if (operatorController != null && operatorController.isConnected()) operatorController.updateRumble();
        }

        // Match Time Warnings (Endgame reminders)
        double matchTime = DriverStation.getMatchTime();
        if (matchTime > 0.0) {
            if (matchTime <= 30.0 && matchTime > 28.5 && !warned30s) {
                triggerRumble(RumblePattern.MATCH_TIME_WARNING);
                warned30s = true;
            } else if (matchTime <= 15.0 && matchTime > 13.5 && !warned15s) {
                triggerRumble(RumblePattern.MATCH_TIME_WARNING);
                warned15s = true;
            }
        }

        // Sensor / Vision Degradation Alert Pulse
        boolean visionDegraded = swerveBase.isVisionDegraded();
        if (visionDegraded && !wasVisionDegraded) {
            triggerRumble(RumblePattern.HARDWARE_WARNING);
        }
        wasVisionDegraded = visionDegraded;

        // General Hardware Error Alert Pulse
        boolean errorActive = AlertManager.hasActiveErrors();
        if (errorActive && !wasHardwareError) {
            triggerRumble(RumblePattern.HARDWARE_WARNING);
        }
        wasHardwareError = errorActive;

        // Collision / Physical Impact Shock Alert Pulse
        boolean collisionDetected = swerveBase.isCollisionDetected();
        boolean collisionHapticEnabled = Dashboard.isHapticCollisionEnabled();
        if (collisionHapticEnabled && collisionDetected && !wasCollisionDetected) {
            triggerRumble(RumblePattern.COLLISION_IMPACT);
        }
        wasCollisionDetected = collisionDetected;
    }

    private void triggerRumble(RumblePattern pattern) {
        if (!joystickEnabled) {
            if (driverController != null) driverController.triggerRumblePattern(pattern);
            if (operatorController != null && operatorController.isConnected()) operatorController.triggerRumblePattern(pattern);
        }
    }

    private boolean handleActiveAction() {
        // Advanced Modes: Smart Assist / Glide (Right Bumper) / Ball Hunt (Left Bumper)
        boolean isGlideHeld = !joystickEnabled && rightBumper && Dashboard.isGlidePointsEnabled();
        boolean isBallHuntHeld = !joystickEnabled && leftBumper && Dashboard.isBallHuntEnabled();

        // Calculate driver inputs in field coordinates for shared authority blending with 2D circular deadband
        Translation2d shapedTrans = apply2DDeadbandAndShape(leftX, leftY, TRANSLATION_DEADBAND);
        double shapedX = shapedTrans.getX();
        double shapedY = shapedTrans.getY();
        double cleanRot = edu.wpi.first.math.MathUtil.applyDeadband(rightX, ROTATION_DEADBAND);
        double shapedRot = shapeInput(cleanRot);
        boolean isSlow = slowModeActive;
        double transScale = isSlow ? 0.35 : 1.0;
        double rotScale = isSlow ? 0.50 : 1.0;
        boolean isRed = AllianceFlipUtil.isRedAlliance();
        double driverFieldForward = (isRed ? -shapedY : shapedY) * Constants.MAX_SPEED * transScale;
        double driverFieldStrafe  = (isRed ? -shapedX : shapedX) * Constants.MAX_SPEED * transScale;
        double driverFieldRot     = shapedRot * Constants.MAX_ROTATION_SPEED * rotScale;

        // ── 1. Smart Assist (Right Bumper: Jev-Powered Co-Pilot) ─────────────
        if (isGlideHeld) {
            if (!wasGlideHeld) {
                coPilot.startSmartAssist();
                triggerRumble(RumblePattern.MODE_ENGAGED);
            }
            wasGlideHeld = true;

            boolean running = coPilot.updateSmartAssist(driverFieldForward, driverFieldStrafe, driverFieldRot);
            if (!running) {
                if (coPilot.checkAndClearBreakout()) {
                    triggerRumble(RumblePattern.OVERRIDE_DISENGAGED);
                }
                return false;
            }
            return true;
        } else if (wasGlideHeld) {
            coPilot.stopAssist();
            wasGlideHeld = false;
        }

        // ── 2. Ball Hunt Assist (Left Bumper: Vision Target Pursuit) ─────────
        if (isBallHuntHeld && activeAction == null) {
            activeAction = new BallHuntAction();
            activeAction.start();
            triggerRumble(RumblePattern.MODE_ENGAGED);
        }

        if (activeAction instanceof BallHuntAction ballHunt) {
            ballHunt.setDriverInput(driverFieldForward, driverFieldStrafe);

            if (ballHunt.checkAndClearBallAcquired()) {
                triggerRumble(RumblePattern.BALL_ACQUIRED);
                coPilot.incrementBallCount();
            }

            if (!isBallHuntHeld || ballHunt.isFinished()) {
                ballHunt.done();
                activeAction = null;
                return false;
            }

            ballHunt.update();
            return true;
        }

        if (activeAction != null) {
            if (activeAction.isFinished()) {
                activeAction.done();
                activeAction = null;
                return false;
            }
            activeAction.update();
            return true;
        }

        return false;
    }

    public void driveBaseControl() {
        // MultiTapTrigger Critical Override: Require double-tap on A button within 0.4s to re-zero gyro heading
        // Prevents accidental mid-match field-orientation loss from stray button bumps
        if (zeroGyroTrigger.getAsBoolean()) {
            swerveBase.zeroGyroWithAlliance();
            triggerRumble(RumblePattern.MODE_ENGAGED);
        }

        // Input Shaping with 2D circular magnitude deadbanding
        Translation2d shapedTrans = apply2DDeadbandAndShape(leftX, leftY, TRANSLATION_DEADBAND);
        double shapedX = shapedTrans.getX();
        double shapedY = shapedTrans.getY();
        double cleanRot = edu.wpi.first.math.MathUtil.applyDeadband(rightX, ROTATION_DEADBAND);
        double shapedRot = shapeInput(cleanRot);

        // Manual rotation input clears snap target heading
        if (Math.abs(cleanRot) > 0.01) {
            snapTargetHeading = null;
        } else if (Dashboard.isSnapToTurnEnabled() && pov >= 0) {
            // Cardinal Snap-to-Heading via D-Pad (Mapped to Driver Perspective)
            switch (pov) {
                case 0: // Forward (away from driver)
                    snapTargetHeading = AllianceFlipUtil.getDriverRelativeHeading(Rotation2d.fromDegrees(0));
                    break;
                case 90: // Right (to driver's right)
                    snapTargetHeading = AllianceFlipUtil.getDriverRelativeHeading(Rotation2d.fromDegrees(-90));
                    break;
                case 180: // Backward (towards driver)
                    snapTargetHeading = AllianceFlipUtil.getDriverRelativeHeading(Rotation2d.fromDegrees(180));
                    break;
                case 270: // Left (to driver's left)
                    snapTargetHeading = AllianceFlipUtil.getDriverRelativeHeading(Rotation2d.fromDegrees(90));
                    break;
                default:
                    break;
            }
        }

        // Speed Scaling (Slow Mode)
        boolean isSlow = slowModeActive;
        double translationScale = isSlow ? 0.35 : 1.0;
        double rotationScale = isSlow ? 0.50 : 1.0;

        // Driver Intent to Global Field Speeds (Always-Blue NWU Coordinate System):
        // Blue DS: Away = +X, Left = +Y.
        // Red DS:  Away = -X, Left = -Y (driver's left looking towards -X).
        boolean isRed = AllianceFlipUtil.isRedAlliance();
        double targetFieldForward = (isRed ? -shapedY : shapedY) * Constants.MAX_SPEED * translationScale;
        double targetFieldStrafe  = (isRed ? -shapedX : shapedX) * Constants.MAX_SPEED * translationScale;
        double targetFieldRot     = shapedRot * Constants.MAX_ROTATION_SPEED * rotationScale;

        // Dynamic live-tuning for driver slew rates
        if (Constants.OperatorConstants.TRANSLATION_SLEW_RATE.hasChanged(this.hashCode())) {
            double transRate = Constants.OperatorConstants.TRANSLATION_SLEW_RATE.get();
            translationLimiter.setRateLimit(transRate);
        }
        if (Constants.OperatorConstants.ROTATION_SLEW_RATE.hasChanged(this.hashCode())) {
            double rotRate = Constants.OperatorConstants.ROTATION_SLEW_RATE.get();
            rotationLimiter = new SlewRateLimiter(rotRate);
        }

        // Apply True 2D Vector Slew Rate Limiting in Field Coordinates
        Translation2d limitedTranslation = translationLimiter.calculate(targetFieldForward, targetFieldStrafe);
        driverForward = limitedTranslation.getX();
        driverStrafe = limitedTranslation.getY();
        driverRotation = rotationLimiter.calculate(targetFieldRot);

        // Auto-Aim overrides rotation toward target hub
        ShootingSolution solution = shooter.getLatestShootingSolution();
        boolean autoAimRequested = rightTrigger > 0.3 && Dashboard.isAutoAimEnabled();
        boolean autoAimActive = autoAimRequested && solution != null && solution.shotPossibility();

        // Publish live Driver HUD states
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Features/Slow Mode", isSlow);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Driver/Snap Active", snapTargetHeading != null);
        if (snapTargetHeading != null) {
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Driver/Snap Target Angle", snapTargetHeading.getDegrees());
        }

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("CoPilot/AssistActive", coPilot.isAssistActive());
        if (coPilot.getActiveObjective() != null) {
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("CoPilot/Objective", coPilot.getActiveObjective().name());
        }

        if (autoAimActive) {
            swerveBase.driveFieldOriented(swerveBase.getTargetSpeeds(driverForward, driverStrafe, solution.shootingAngle()));
        } else if (snapTargetHeading != null) {
            swerveBase.driveFieldOriented(swerveBase.getTargetSpeeds(driverForward, driverStrafe, snapTargetHeading));
        } else {
            swerveBase.drive(new Translation2d(driverForward, driverStrafe), driverRotation, Dashboard.isFieldOriented());
        }
    }

    public void intakeControl() {
        // Arm Deploy Toggle (X Button)
        if (xButton && !lastXButton) {
            armDeployed = !armDeployed;
        }
        lastXButton = xButton;

        boolean intakeTriggerHeld = leftTrigger > 0.3;
        boolean ejectHeld = bButton;
        boolean feedHeld = yButton;

        if (ejectHeld) {
            // Eject / unjam mode: reverse rollers & hopper
            intake.setState(armDeployed ? "Reversed" : "StandbyReversed");
        } else if (feedHeld) {
            // Standby feed / pass mode
            intake.setState("StandbyIntaking");
        } else if (intakeTriggerHeld) {
            // Ground intake: automatically deploy arm down and spin rollers & hopper
            intake.setState("Intaking");
        } else if (armDeployed) {
            // Arm deployed down but idle
            intake.setState("Down");
        } else {
            // Retracted standby idle
            intake.setState("Standby");
        }
    }

    public void shooterControl() {
        ShootingSolution solution = shooter.getLatestShootingSolution();
        boolean inAllianceZone = AllianceFlipUtil.isPoseInAllianceZone(swerveBase.getPose());
        boolean autoAimRequested = rightTrigger > 0.3 && Dashboard.isAutoAimEnabled();
        boolean autoAimActive = autoAimRequested && solution != null && solution.shotPossibility() && inAllianceZone;
        boolean manualRequested = rightTrigger > 0.3 && !autoAimActive && inAllianceZone;

        if (autoAimActive) {
            // Spool up flywheels to distance solution
            shooter.setTargetRPM(solution.flywheelRpmLeft(), solution.flywheelRpmRight());

            double headingError = Math.abs(solution.shootingAngle().minus(swerveBase.getHeading()).getDegrees());
            boolean headingAligned = headingError <= Constants.ShooterConstants.ALIGNMENT_HEADING_TOLERANCE_DEG;
            boolean flywheelsReady = shooter.isAtCorrectSpeed();
            boolean hubActive = Dashboard.getInstance().isHubActive();
            boolean targetLocked = headingAligned && flywheelsReady && hubActive;

            // Haptic notification when lock is achieved
            if (targetLocked && !wasTargetLocked) {
                triggerRumble(RumblePattern.TARGET_LOCKED);
            }
            wasTargetLocked = targetLocked;

            if (headingAligned && flywheelsReady && hubActive && inAllianceZone) {
                shooter.shoot();
            } else {
                shooter.prepareToShoot();
            }
        } else if (manualRequested) {
            wasTargetLocked = false;
            shooter.manualFire(rightTrigger);
        } else {
            // Predictive Flywheel Pre-Spooling (Hub Phase Anticipation or Co-Pilot transit)
            boolean hubActive = Dashboard.getInstance().isHubActive();
            double timeUntilSwitch = Dashboard.getInstance().getTimeUntilSwitch();
            boolean hasFuel = intake.hasFuel();
            boolean coPilotPreSpool = coPilot.isAssistActive() && coPilot.getActiveObjective() == StrategicObjective.CYCLE_SCORE_HUB;

            if ((!hubActive && timeUntilSwitch <= 2.5 && timeUntilSwitch > 0.05 && hasFuel && solution != null && solution.shotPossibility())
                || coPilotPreSpool) {
                if (solution != null && solution.shotPossibility()) {
                    shooter.setTargetRPM(solution.flywheelRpmLeft(), solution.flywheelRpmRight());
                } else {
                    shooter.setTargetRPM(3200.0, 3200.0);
                }
                shooter.prepareToShoot();
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/PreSpoolingActive", true);
            } else {
                wasTargetLocked = false;
                shooter.stop();
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/PreSpoolingActive", false);
            }
        }
    }

    // Getters for testing and telemetry
    public boolean isSlowModeActive() { return slowModeActive; }
    public void setSlowModeActive(boolean active) {
        this.slowModeActive = active;
        this.lastDashboardSlowMode = active;
        Dashboard.setSlowModeEnabled(active);
    }
    public boolean isArmDeployed() { return armDeployed; }
    public Rotation2d getSnapTargetHeading() { return snapTargetHeading; }
    public double getDriverForward() { return driverForward; }
    public double getDriverStrafe() { return driverStrafe; }
    public double getDriverRotation() { return driverRotation; }
}
