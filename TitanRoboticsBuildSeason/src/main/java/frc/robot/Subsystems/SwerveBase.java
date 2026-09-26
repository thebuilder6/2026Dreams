package frc.robot.Subsystems;

import frc.robot.Hardware.Drive.*;

import frc.robot.Telemetry.Dashboard;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.CollisionDetector;
import frc.robot.Auto.DynamicRouter;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Sim.LimelightSim;
import frc.robot.Sim.VisionSim;
import frc.robot.Hardware.Drive.DriveIO;
import frc.robot.Hardware.Drive.DriveIOInputsAutoLogged;
import frc.robot.Hardware.Drive.DriveIOSparkMax;
import frc.robot.Hardware.Drive.DriveIOSim;
import frc.robot.HMI.Alerts.Alert;
import frc.robot.HMI.Alerts.Alert.AlertType;
import frc.robot.Utils.AllianceFlipUtil;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveBase implements Subsystem {

    private static SwerveBase instance = null;

    // AdvantageKit Hardware IO Abstraction
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    // Unified field object from YAGSL
    private Field2d field;

    private final ArrayList<String> lastGlideFieldObjectNames = new ArrayList<>();
    private Boolean lastRedAllianceDrawn = null;
    private GlideConstants.GlidePoint lastHighlightedGlidePoint = null;

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    private static final Pose2d OFF_FIELD_POSE = new Pose2d(-999, -999, new Rotation2d());

    // Members for logging
    private int lastLimelightTagCount = 0;
    private double lastLimelightAvgDist = 0;
    private double lastLimelightStdDev = 0;
    private boolean lastLimelightAccepted = false;

    // Vision Watchdog & Graceful Degradation
    private final Alert visionDegradedAlert = new Alert("Vision", "Vision Degraded: Pure Odometry Active",
            AlertType.WARNING);
    private double lastVisionTimestamp = 0.0;
    private static final double VISION_TIMEOUT_SEC = 0.75;
    private boolean isVisionDegraded = false;

    private boolean isPitMode = false;

    private final CollisionDetector collisionDetector = new CollisionDetector();

    // Power Distribution & Brownout Sag Protection
    private PowerDistribution powerDistribution;
    private double brownoutSpeedScale = 1.0;
    private double simBatteryVoltage = -1.0;
    private double simTotalCurrent = -1.0;
    private final Alert brownoutAlert = new Alert("Power", "Brownout Protection Active: Throttling Drive",
            AlertType.WARNING);

    /**
     * Gets the singleton instance of SwerveBase.
     * 
     * @return The instance of SwerveBase.
     */
    public static SwerveBase getInstance() {
        if (instance == null) {
            instance = new SwerveBase();
        }
        return instance;
    }

    public SwerveBase() {
        SubsystemManager.registerSubsystem(this);
        System.out.println("SwerveBase: Simulation Mode is " + SwerveDriveTelemetry.isSimulation);
        // Dynamically determine alliance - defaults to Red if not available
        boolean blueAlliance = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Blue)
                .orElse(false);
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16),
                        Meter.of(4)),
                        Rotation2d.fromDegrees(180));
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(Constants.MAX_SPEED, startingPose);
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot
                                                // via angle.
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
                                                                              // for simulations since it causes
                                                                              // discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true,
                true,
                0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
                      // coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false,
                1); // Enable if you want to resynchronize your absolute encoders and motor encoders
                    // periodically when they are not moving.
        // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used
        // over the internal encoder and push the offsets onto it. Throws warning if not
        // possible

        field = swerveDrive.field;
        SmartDashboard.putData("Field", field);

        this.io = SwerveDriveTelemetry.isSimulation ? new DriveIOSim(swerveDrive) : new DriveIOSparkMax(swerveDrive);

        try {
            this.powerDistribution = new PowerDistribution(1, ModuleType.kRev);
        } catch (Throwable t1) {
            try {
                this.powerDistribution = new PowerDistribution();
            } catch (Throwable t2) {
                this.powerDistribution = null;
            }
        }
    }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        Translation2d scaledTranslation = translation.times(brownoutSpeedScale);
        double scaledRotation = rotation * brownoutSpeedScale;
        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(scaledTranslation.getX(), scaledTranslation.getY(),
                        scaledRotation, getHeading())
                : new ChassisSpeeds(scaledTranslation.getX(), scaledTranslation.getY(), scaledRotation);
        swerveDrive.drive(ChassisSpeeds.discretize(speeds, 0.020));
    }

    /**
     * Stop the drivebase by commanding zero velocity.
     */
    public void stop() {
        swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
    }

    /**
     * Drive according to the chassis robot oriented velocity with analytical
     * discretization and brownout protection.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        ChassisSpeeds scaledVelocity = new ChassisSpeeds(
                velocity.vxMetersPerSecond * brownoutSpeedScale,
                velocity.vyMetersPerSecond * brownoutSpeedScale,
                velocity.omegaRadiansPerSecond * brownoutSpeedScale);
        swerveDrive.drive(ChassisSpeeds.discretize(scaledVelocity, 0.020));
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this method. However, if either gyro angle or
     * module position is reset, this must be called in order for odometry to keep
     * working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        synchronized (swerveDrive) {
            swerveDrive.resetOdometry(initialHolonomicPose);
        }
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's current estimated Pose2d.
     */
    public Pose2d getPose() {
        synchronized (swerveDrive) {
            return swerveDrive.getPose();
        }
    }

    /**
     * Gets the current simulation pose of the robot.
     * 
     * @return The simulation pose
     */
    public Pose2d getSimulationPose() {
        return swerveDrive.getSimulationDriveTrainPose().orElse(getPose());
    }

    /**
     * Gets the underlying MapleSim drive train simulation if in simulation.
     */
    public java.util.Optional<swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation> getMapleSimDrive() {
        return swerveDrive.getMapleSimDrive();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        synchronized (swerveDrive) {
            swerveDrive.zeroGyro();
        }
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    private boolean isRedAlliance() {
        return AllianceFlipUtil.isRedAlliance();
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        zeroGyro();
        if (isRedAlliance()) {
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
        }
    }

    private GlideConstants.GlidePoint lastNearest = null;

    public GlideConstants.GlidePoint getNearestGlidePoint() {
        List<GlideConstants.GlidePoint> glidePoints = isRedAlliance()
                ? GlideConstants.RED_GLIDE_POINTS
                : GlideConstants.BLUE_GLIDE_POINTS;

        if (glidePoints == null || glidePoints.isEmpty()) {
            return null;
        }

        Pose2d currentPose = getPose();
        GlideConstants.GlidePoint nearest = null;
        double nearestDistance = Double.POSITIVE_INFINITY;

        // Selection Hysteresis: Give a "bonus" to the last selected point to prevent
        // flickering
        double HYSTERESIS_BONUS = 0.8; // Meters

        for (GlideConstants.GlidePoint p : glidePoints) {
            if (p == null || p.pose == null) {
                continue;
            }
            double d = currentPose.getTranslation().getDistance(p.pose.getTranslation());

            // Apply hysteresis bonus
            if (lastNearest != null && p.name.equals(lastNearest.name)) {
                d -= HYSTERESIS_BONUS;
            }

            if (d < nearestDistance) {
                nearestDistance = d;
                nearest = p;
            }

            if (p.isTunnelEntrance && p.tunnelExitPose != null) {
                double exitD = currentPose.getTranslation().getDistance(p.tunnelExitPose.getTranslation());

                // Tunnel points also need hysteresis check
                if (lastNearest != null && lastNearest.name.equals(p.name + " (Exit)")) {
                    exitD -= HYSTERESIS_BONUS;
                }

                if (exitD < nearestDistance) {
                    nearestDistance = exitD;
                    Pose2d reverseStartPose = new Pose2d(
                            p.tunnelExitPose.getTranslation(),
                            p.tunnelExitPose.getRotation().plus(Rotation2d.fromDegrees(180)));
                    Pose2d reverseEndPose = new Pose2d(
                            p.pose.getTranslation(),
                            p.pose.getRotation().plus(Rotation2d.fromDegrees(180)));

                    nearest = new GlideConstants.GlidePoint(
                            p.name + " (Exit)",
                            reverseStartPose,
                            true,
                            reverseEndPose);
                }
            }
        }

        lastNearest = nearest;
        return nearest;
    }

    public void drawGlidePointsOnField() {
        List<GlideConstants.GlidePoint> glidePoints = isRedAlliance()
                ? GlideConstants.RED_GLIDE_POINTS
                : GlideConstants.BLUE_GLIDE_POINTS;

        field.getObject("GlidePoints").setPoses(new ArrayList<>());

        HashSet<String> currentNames = new HashSet<>();
        if (glidePoints != null) {
            for (GlideConstants.GlidePoint p : glidePoints) {
                if (p == null || p.pose == null) {
                    continue;
                }

                String baseName = "GlidePoint/" + sanitizeFieldObjectName(p.name);
                currentNames.add(baseName);
                field.getObject(baseName).setPose(p.pose);

                if (p.isTunnelEntrance && p.tunnelExitPose != null) {
                    String exitName = baseName + "/Exit";
                    currentNames.add(exitName);
                    field.getObject(exitName).setPose(p.tunnelExitPose);
                }
            }
        }

        for (String old : lastGlideFieldObjectNames) {
            if (!currentNames.contains(old)) {
                field.getObject(old).setPose(OFF_FIELD_POSE);
            }
        }

        lastGlideFieldObjectNames.clear();
        lastGlideFieldObjectNames.addAll(currentNames);
    }

    private void highlightNearestGlidePointOnField(GlideConstants.GlidePoint nearest) {
        if (nearest == lastHighlightedGlidePoint) {
            return;
        }
        lastHighlightedGlidePoint = nearest;

        int segments = 12;
        double radiusMeters = 0.35;
        ArrayList<Pose2d> ringPoses = new ArrayList<>(segments);
        if (nearest != null && nearest.pose != null) {
            for (int i = 0; i < segments; i++) {
                double angleRad = (2.0 * Math.PI * i) / segments;
                double x = nearest.pose.getX() + (radiusMeters * Math.cos(angleRad));
                double y = nearest.pose.getY() + (radiusMeters * Math.sin(angleRad));
                ringPoses.add(new Pose2d(x, y, new Rotation2d()));
            }
        }

        field.getObject("NearestGlidePoint").setPoses(ringPoses);
    }

    /**
     * Sanitizes a string for use as a Field2d object name by replacing special
     * characters with underscores.
     * 
     * @param name The original name to sanitize.
     * @return A sanitized string compatible with NetworkTables/Field2d naming.
     */
    private static String sanitizeFieldObjectName(String name) {
        if (name == null) {
            return "";
        }
        return name.replaceAll("[^A-Za-z0-9_/.-]", "_");
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        if (isPitMode && DriverStation.isDisabled()) {
            swerveDrive.setMotorIdleMode(false); // Force coast in pit mode
        } else {
            swerveDrive.setMotorIdleMode(brake);
        }
    }

    /**
     * Set the robot to "Pit Mode" which disables motor brakes in disabled mode
     * for easier pushing.
     * 
     * @param pitMode True to enable pit mode.
     */
    public void setPitMode(boolean pitMode) {
        this.isPitMode = pitMode;
        setMotorBrake(true); // Re-apply current brake state logic
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                headingX,
                headingY,
                getHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        return swerveDrive.swerveController.getRawTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians());
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot.
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot.
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Gets the {@link SwerveDriveConfiguration} containing physical drive
     * dimensions and gear ratios.
     * 
     * @return The {@link SwerveDriveConfiguration} for the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * WPILib Commands v2 Subsystem.idle():
     * Returns a command that locks swerve modules in an X-pattern to resist movement
     * while the drivebase is idle.
     */
    @Override
    public Command idle() {
        return Commands.run(this::lock, this)
                .withName("SwerveBase.idle");
    }

    /**
     * Gets robot linear ground speed magnitude as a {@link LinearVelocity} measure.
     */
    public LinearVelocity getLinearVelocityMeasure() {
        ChassisSpeeds speeds = getFieldVelocity();
        return Units.MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }

    /**
     * Gets robot rotational velocity as an {@link AngularVelocity} measure.
     */
    public AngularVelocity getAngularVelocityMeasure() {
        return Units.RadiansPerSecond.of(getFieldVelocity().omegaRadiansPerSecond);
    }

    /**
     * Gets robot heading as an {@link Angle} measure.
     */
    public Angle getHeadingMeasure() {
        return Units.Degrees.of(getHeading().getDegrees());
    }

    /**
     * Gets robot pitch as an {@link Angle} measure.
     */
    public Angle getPitchMeasure() {
        return Units.Degrees.of(getPitch().getDegrees());
    }

    /**
     * Gets monitored battery voltage as a {@link Voltage} measure.
     */
    public Voltage getBatteryVoltageMeasure() {
        double v = (simBatteryVoltage >= 0) ? simBatteryVoltage : RobotController.getBatteryVoltage();
        return Units.Volts.of(v);
    }

    /**
     * Gets monitored total electrical current draw as a {@link Current} measure.
     */
    public Current getTotalCurrentMeasure() {
        double current = (simTotalCurrent >= 0)
                ? simTotalCurrent
                : ((powerDistribution != null && !edu.wpi.first.wpilibj.RobotBase.isSimulation())
                        ? powerDistribution.getTotalCurrent()
                        : getSimulationCurrentDraw());
        return Units.Amps.of(current);
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        ChassisSpeeds scaledVelocity = new ChassisSpeeds(
                velocity.vxMetersPerSecond * brownoutSpeedScale,
                velocity.vyMetersPerSecond * brownoutSpeedScale,
                velocity.omegaRadiansPerSecond * brownoutSpeedScale);
        swerveDrive.driveFieldOriented(ChassisSpeeds.discretize(scaledVelocity, 0.020));
    }

    /**
     * Fast odometry polling callback designed for addPeriodic() timeslot execution.
     * Updates swerve odometry at high frequency (e.g. 100 Hz / 10ms) between main
     * 20ms periodic loops.
     */
    public void updateOdometryFast() {
        if (swerveDrive != null) {
            synchronized (swerveDrive) {
                swerveDrive.updateOdometry();
            }
        }
    }

    public boolean isVisionDegraded() {
        return isVisionDegraded;
    }

    @Override
    public void update() {
        io.updateInputs(inputs);
        org.littletonrobotics.junction.Logger.processInputs("Drive", inputs);
        synchronized (swerveDrive) {
            swerveDrive.updateOdometry();
        }

        // Dynamic PowerDistribution & Voltage Sag Brownout Monitoring
        double batteryVoltage = (simBatteryVoltage >= 0)
                ? simBatteryVoltage
                : RobotController.getBatteryVoltage();
        double totalCurrentAmps = (simTotalCurrent >= 0)
                ? simTotalCurrent
                : ((powerDistribution != null && !edu.wpi.first.wpilibj.RobotBase.isSimulation())
                        ? powerDistribution.getTotalCurrent()
                        : getSimulationCurrentDraw());

        // Incipient brownout risk thresholds: V < 9.5V, I > 180A (Hardware cutoff at
        // 6.8V)
        double targetScale = 1.0;
        if (simBatteryVoltage < 0 && RobotController.isBrownedOut()) {
            targetScale = 0.25; // Severe hardware brownout cutoff active
        } else {
            double vScale = 1.0;
            if (batteryVoltage < 9.5) {
                // Continuous linear ramp from 1.0 at 9.5V down to 0.35 at 7.5V
                vScale = MathUtil.clamp((batteryVoltage - 7.5) / (9.5 - 7.5), 0.35, 1.0);
            }
            double iScale = 1.0;
            if (totalCurrentAmps > 180.0) {
                // Continuous ramp down from 1.0 at 180A down to 0.50 at 240A
                iScale = MathUtil.clamp(1.0 - ((totalCurrentAmps - 180.0) / (240.0 - 180.0)) * 0.5, 0.5, 1.0);
            }
            targetScale = Math.min(vScale, iScale);
        }

        // Instantaneous cut on brownout risk, smooth recovery slew back to 1.0 (+3% per
        // 20ms)
        if (targetScale < brownoutSpeedScale) {
            brownoutSpeedScale = targetScale;
        } else {
            brownoutSpeedScale = Math.min(targetScale, brownoutSpeedScale + 0.03);
        }

        boolean isBrownoutThrottling = brownoutSpeedScale < 0.95;
        brownoutAlert.set(isBrownoutThrottling);

        Pose2d estimatedPose = getPose();
        Pose2d truthPose = SwerveDriveTelemetry.isSimulation ? getSimulationPose() : estimatedPose;

        if (SwerveDriveTelemetry.isSimulation) {
            field.getObject("OdometryGhost").setPose(estimatedPose);
        } else {
            // Vision measurements and MegaTag2 gating are handled by Vision subsystem
            if (Timer.getTimestamp() - lastVisionTimestamp > VISION_TIMEOUT_SEC) {
                isVisionDegraded = true;
                lastLimelightAccepted = false;
                visionDegradedAlert.set(true);
            }
        }

        boolean currentRedAlliance = isRedAlliance();
        if (lastRedAllianceDrawn == null || lastRedAllianceDrawn != currentRedAlliance) {
            drawGlidePointsOnField();
            lastRedAllianceDrawn = currentRedAlliance;
        }
        highlightNearestGlidePointOnField(getNearestGlidePoint());

        // Explicitly update the field object with the current pose
        field.setRobotPose(truthPose);

        collisionDetector.update(
                truthPose,
                getRobotVelocity(),
                getFieldVelocity(),
                inputs.accelXG,
                inputs.accelYG,
                getAverageDriveCurrent());
    }

    public CollisionDetector getCollisionDetector() {
        return collisionDetector;
    }

    public boolean isCollisionDetected() {
        return collisionDetector.isImpactDetected();
    }

    @Override
    public void initialize() {
        zeroGyroWithAlliance();
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Subsystems/Swerve/Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Subsystems/Swerve/Pose X", getPose().getX());
        SmartDashboard.putNumber("Subsystems/Swerve/Pose Y", getPose().getY());

        // AdvantageScope Odometry & Module Telemetry
        Pose2d currentPose = getPose();
        org.littletonrobotics.junction.Logger.recordOutput("Odometry/RobotPose", currentPose);
        org.littletonrobotics.junction.Logger.recordOutput("Odometry/ModuleStates", swerveDrive.getStates());
        if (SwerveDriveTelemetry.isSimulation) {
            org.littletonrobotics.junction.Logger.recordOutput("FieldSimulation/RobotPose", getSimulationPose());
        }

        SmartDashboard.putNumber("Subsystems/Limelight/TagCount", lastLimelightTagCount);
        SmartDashboard.putNumber("Subsystems/Limelight/AvgDist", lastLimelightAvgDist);
        SmartDashboard.putNumber("Subsystems/Limelight/TrustLevel", lastLimelightStdDev);
        SmartDashboard.putBoolean("Subsystems/Limelight/IsAccepted", lastLimelightAccepted);

        // PowerDistribution & Dynamic Brownout Telemetry
        SmartDashboard.putNumber("Power/BatteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Power/TotalCurrent",
                (powerDistribution != null) ? powerDistribution.getTotalCurrent() : getSimulationCurrentDraw());
        SmartDashboard.putNumber("Power/BrownoutSpeedScale", brownoutSpeedScale);
        SmartDashboard.putBoolean("Power/IsBrownoutRisk", brownoutSpeedScale < 0.95);
    }

    /**
     * Gets the current brownout speed scale factor [0.25 to 1.0].
     */
    public double getBrownoutSpeedScale() {
        return brownoutSpeedScale;
    }

    /**
     * Gets the underlying PowerDistribution instance, or null if uninitialized.
     */
    public PowerDistribution getPowerDistribution() {
        return powerDistribution;
    }

    private double simDriveCurrent = -1.0;

    /**
     * Sets a simulated drive motor current for testing proprioceptive stall
     * detection.
     */
    public void setSimulatedDriveCurrent(double currentAmps) {
        this.simDriveCurrent = currentAmps;
    }

    /**
     * Gets average current draw across the drive motors for proprioceptive stall
     * detection.
     */
    public double getAverageDriveCurrent() {
        if (simDriveCurrent >= 0) {
            return simDriveCurrent;
        }
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return getSimulationCurrentDraw();
        }
        try {
            double totalCurrent = 0.0;
            var modules = swerveDrive.getModules();
            if (modules != null && modules.length > 0) {
                for (var mod : modules) {
                    if (mod != null && mod.getDriveMotor() != null) {
                        Object nativeMotor = mod.getDriveMotor().getMotor();
                        if (nativeMotor instanceof com.revrobotics.spark.SparkMax) {
                            totalCurrent += ((com.revrobotics.spark.SparkMax) nativeMotor).getOutputCurrent();
                        }
                    }
                }
                return totalCurrent / modules.length;
            }
        } catch (Throwable ignored) {
        }
        return getSimulationCurrentDraw();
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "SwerveBase";
    }

    public Field2d getField() {
        return field;
    }

    /**
     * Visualizes a list of waypoints on the field.
     * 
     * @param waypoints List of poses to display
     */
    public void setPathVisualization(List<Pose2d> waypoints) {
        field.getObject("CurrentPath").setPoses(waypoints);
    }

    /**
     * Visualizes a trajectory on the field.
     * 
     * @param trajectory The trajectory to display
     */
    public void setTrajectoryVisualization(Trajectory trajectory) {
        if (trajectory != null) {
            field.getObject("CurrentPath").setTrajectory(trajectory);
        } else {
            field.getObject("CurrentPath").setPoses(new ArrayList<>());
        }
    }

    /**
     * Sets the voltage to all drive motors with steering modules locked straight
     * ahead (0 deg)
     * for linear SysId characterization.
     */
    public void setSysIdDriveVoltage(double volts) {
        swervelib.SwerveModule[] modules = swerveDrive.getModules();
        for (swervelib.SwerveModule module : modules) {
            module.setAngle(0.0);
            module.getDriveMotor().setVoltage(volts);
        }
    }

    /**
     * Sets the voltage to drive motors with modules oriented tangent to the
     * rotation circle
     * for angular (rotational moment of inertia) SysId characterization.
     */
    public void setSysIdRotationVoltage(double volts) {
        // Calculate tangent module angles for pure yaw spin
        edu.wpi.first.math.kinematics.SwerveModuleState[] states = swerveDrive.kinematics
                .toSwerveModuleStates(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 1.0));
        swervelib.SwerveModule[] modules = swerveDrive.getModules();
        for (int i = 0; i < Math.min(modules.length, states.length); i++) {
            modules[i].setAngle(states[i].angle.getDegrees());
            modules[i].getDriveMotor().setVoltage(volts);
        }
    }

    /**
     * Sets the voltage to all drive motors for SysId characterization (legacy
     * alias).
     */
    public void setDriveVoltage(double volts) {
        setSysIdDriveVoltage(volts);
    }

    /**
     * Sets drive voltage on a single module by index (0=FL, 1=FR, 2=BL, 3=BR).
     */
    public void setModuleDriveVoltage(int index, double volts) {
        io.setModuleDriveVoltage(index, volts);
    }

    /**
     * Sets angle motor voltage on a single module by index (0=FL, 1=FR, 2=BL,
     * 3=BR).
     */
    public void setModuleAngleVoltage(int index, double volts) {
        io.setModuleAngleVoltage(index, volts);
    }

    public DriveIO getIO() {
        return io;
    }

    public DriveIOInputsAutoLogged getInputs() {
        return inputs;
    }

    /**
     * Gets drive motor velocity for a single module by index.
     */
    public double getModuleDriveVelocity(int index) {
        swervelib.SwerveModule[] modules = swerveDrive.getModules();
        if (index >= 0 && index < modules.length) {
            return modules[index].getDriveMotor().getVelocity();
        }
        return 0.0;
    }

    /**
     * Gets angle motor position for a single module by index (degrees).
     */
    public double getModuleAnglePosition(int index) {
        swervelib.SwerveModule[] modules = swerveDrive.getModules();
        if (index >= 0 && index < modules.length) {
            return modules[index].getAbsolutePosition();
        }
        return 0.0;
    }

    public List<Double> getDriveMotorVoltages() {
        List<Double> volts = new ArrayList<>();
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            volts.add(module.getDriveMotor().getVoltage());
        }
        return volts;
    }

    public List<Double> getDriveMotorPositions() {
        List<Double> pos = new ArrayList<>();
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            pos.add(module.getDriveMotor().getPosition());
        }
        return pos;
    }

    public List<Double> getDriveMotorVelocities() {
        List<Double> vels = new ArrayList<>();
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            vels.add(module.getDriveMotor().getVelocity());
        }
        return vels;
    }

    /**
     * Adds a vision measurement to the pose estimator.
     * 
     * @param pose      Estimated pose
     * @param timestamp Measurement timestamp
     * @param stdDevs   Standard deviations for X, Y, and Theta
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        synchronized (swerveDrive) {
            swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs);
        }
        lastVisionTimestamp = Timer.getTimestamp();
        isVisionDegraded = false;
        lastLimelightAccepted = true;
        visionDegradedAlert.set(false);
        field.getObject("LimelightGhost").setPose(pose);
    }

    /**
     * Sets the voltage to all steer motors for SysId characterization.
     */
    public void setSteerVoltage(double volts) {
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            module.getAngleMotor().setVoltage(volts);
        }
    }

    public void setSysIdSteerVoltage(double volts) {
        setSteerVoltage(volts);
    }

    /**
     * Checks if any swerve module has CANcoder absolute encoder read issues.
     */
    public boolean hasAbsoluteEncoderIssues() {
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            if (module.getAbsoluteEncoderReadIssue()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets gyro yaw rate in degrees per second from cached IO inputs.
     */
    public double getGyroYawVelocityDegPerSec() {
        return inputs.gyroYawVelocityDegPerSec;
    }

    /**
     * Gets gyro yaw rate in radians per second for rotational SysId.
     */
    public double getGyroYawRateRadsPerSec() {
        return Math.toRadians(inputs.gyroYawVelocityDegPerSec);
    }

    /**
     * Gets the voltages of all steer motors.
     */
    public List<Double> getSteerMotorVoltages() {
        List<Double> volts = new ArrayList<>();
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            volts.add(module.getAngleMotor().getVoltage());
        }
        return volts;
    }

    /**
     * Gets the positions of all steer motors.
     */
    public List<Double> getSteerMotorPositions() {
        List<Double> pos = new ArrayList<>();
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            pos.add(module.getAngleMotor().getPosition());
        }
        return pos;
    }

    /**
     * Gets the current chassis speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Set the robot pose (for testing)
     */
    public void setPose(Pose2d pose) {
        synchronized (swerveDrive) {
            swerveDrive.resetOdometry(pose);
        }
    }

    public double getCollisionJerkMagnitude() {
        return collisionDetector.getLastJerkMagnitude();
    }

    /**
     * Estimates total current draw in simulation where no real PDH is available.
     * Uses a rough heuristic based on commanded speed magnitude.
     */
    public double getSimulationCurrentDraw() {
        ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
        double speedMag = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        // Rough estimate: ~15A idle per module (4 modules), scaling up to ~40A at full
        // speed
        double perModuleCurrent = 15.0 + 25.0 * Math.min(speedMag / 4.5, 1.0);
        return perModuleCurrent * 4.0;
    }

    /**
     * Set battery voltage override for simulation brownout testing. Use -1 to
     * revert to real sensor.
     */
    public void setSimBatteryVoltage(double voltage) {
        this.simBatteryVoltage = voltage;
        if (voltage < 0 || voltage >= 12.0) {
            brownoutSpeedScale = 1.0;
            brownoutAlert.set(false);
        }
    }

    /**
     * Set total current override for simulation brownout testing. Use -1 to revert
     * to real sensor.
     */
    public void setSimTotalCurrent(double current) {
        this.simTotalCurrent = current;
        if (current < 0) {
            brownoutSpeedScale = 1.0;
            brownoutAlert.set(false);
        }
    }

}
