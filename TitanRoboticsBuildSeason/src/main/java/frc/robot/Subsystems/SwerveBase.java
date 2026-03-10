package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Sim.LimelightSim;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveBase implements Subsystem {

    private static SwerveBase instance = null;

    // Unified field object from YAGSL
    private Field2d field;

    private final ArrayList<String> lastGlideFieldObjectNames = new ArrayList<>();

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

    private boolean isPitMode = false;

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
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Stop the drivebase by commanding zero velocity.
     */
    public void stop() {
        swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // Function: resetOdometry
    //
    // Author: Austin :)
    //
    // Use: Reads position changes of robot continuosly. Usefull for autonomous and
    // corrections of drift.
    //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // Function: getPose
    //
    // Author: Austin :)
    //
    // Use: Gets position based on change from the starting point. For autonomous
    // and driver assistance.
    //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
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
        swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
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
        int segments = 12;
        double radiusMeters = 0.35;
        ArrayList<Pose2d> ringPoses = new ArrayList<>();
        if (nearest != null && nearest.pose != null) {
            for (int i = 0; i < segments; i++) {
                double angleRad = (2.0 * Math.PI * i) / segments;
                double x = nearest.pose.getX() + (radiusMeters * Math.cos(angleRad));
                double y = nearest.pose.getY() + (radiusMeters * Math.sin(angleRad));
                ringPoses.add(new Pose2d(x, y, new Rotation2d()));
            }
        }

        field.getObject("NearestGlidePoint").setPoses(ringPoses);

        for (int i = 0; i < segments; i++) {
            field.getObject("NearestGlideRing/" + i).setPose(OFF_FIELD_POSE);
        }
    }

    /**
     * Sanitizes a string for use as a Field2d object name by replacing special characters with underscores.
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
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // Function: getFieldVelocity
    //
    // Author: Austin :)
    //
    // Use: This function retrieves the robot's current motion as a Field-Relative
    // velocity vector, encapsulated in a ChassisSpeeds object. This velocity is
    // relative to the field's coordinate system (e.g., V x
    // is velocity along the field's X-axis, not the robot's nose), providing
    // crucial feedback for closed-loop control and logging.
    //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // Function: getRobotVelocity
    //
    // Author: Austin :)
    //
    // Use: This function retrieves the robot's current motion as a ChassisSpeeds
    // object, representing its velocity relative to its own frame of reference
    // (Robot-Relative). It reports the instantaneous forward/backward (V x),
    // sideways (V y), and angular (ω) speeds, providing the raw movement data used
    // for debugging and low-level control.
    //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // Function: getSwerveController
    //
    // Author: Austin :)
    //
    // Use: This function provides direct access to the robot's SwerveController
    // object, which is the high-level brain responsible for calculating the precise
    // commands for all swerve modules. By returning this controller, it allows
    // other subsystems or control loops (like an autonomous path follower) to
    // interact directly with the drive logic or retrieve its complex state
    // information.
    //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // Function: getSwerveDriveConfiguration
    //
    // Author: Austin :)
    //
    // Use: This function serves as an accessor to retrieve the
    // SwerveDriveConfiguration object, which holds all the critical, unchanging
    // physical parameters of the robot's drive base. This configuration includes
    // essential data like wheel base width, track width, gear ratios, wheel
    // circumference, and motor ID assignments, enabling all other drive logic to
    // function correctly.
    //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
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
        swerveDrive.driveFieldOriented(velocity);
    }


    @Override
    public void update() {
        swerveDrive.updateOdometry();

        Pose2d estimatedPose = getPose();
        Pose2d truthPose = SwerveDriveTelemetry.isSimulation ? getSimulationPose() : estimatedPose;

        if (SwerveDriveTelemetry.isSimulation) {
            LimelightSim.update(truthPose);
            field.getObject("OdometryGhost").setPose(estimatedPose);
        }

        drawGlidePointsOnField();
        highlightNearestGlidePointOnField(getNearestGlidePoint());

        // Update path visualization if an action is active
        // This is a bit of a hack since SwerveBase doesn't know about Teleop's
        // activeAction,
        // but we can provide a method for Teleop or Actions to push path data.

        // Explicitly update the field object with the current pose
        // In simulation, we show the Truth Pose as the main robot
        field.setRobotPose(truthPose);
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

        SmartDashboard.putNumber("Subsystems/Limelight/TagCount", lastLimelightTagCount);
        SmartDashboard.putNumber("Subsystems/Limelight/AvgDist", lastLimelightAvgDist);
        SmartDashboard.putNumber("Subsystems/Limelight/TrustLevel", lastLimelightStdDev);
        SmartDashboard.putBoolean("Subsystems/Limelight/IsAccepted", lastLimelightAccepted);
    }

    @Override
    public double getSimulationCurrentDraw() {
        // Estimate swerve current draw
        // 4 modules * (Drive Motor + Angle Motor)
        // Simple model: Base current + Speed proportion
        double driveCurrent = Math.abs(getRobotVelocity().vxMetersPerSecond) * 10.0;
        double turnCurrent = Math.abs(getRobotVelocity().omegaRadiansPerSecond) * 10.0;

        return 4.0 // Idle current
                + driveCurrent
                + turnCurrent;
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
     * Sets the voltage to all drive motors for SysId characterization.
     */
    public void setDriveVoltage(double volts) {
        swerveDrive.drive(new Translation2d(), 0, false, true); // ensure static
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            module.getDriveMotor().setVoltage(volts);
        }
    }

    /**
     * Sets drive voltage on a single module by index (0=FL, 1=FR, 2=BL, 3=BR).
     */
    public void setModuleDriveVoltage(int index, double volts) {
        swervelib.SwerveModule[] modules = swerveDrive.getModules();
        if (index >= 0 && index < modules.length) {
            modules[index].getDriveMotor().setVoltage(volts);
        }
    }

    /**
     * Sets angle motor voltage on a single module by index (0=FL, 1=FR, 2=BL, 3=BR).
     */
    public void setModuleAngleVoltage(int index, double volts) {
        swervelib.SwerveModule[] modules = swerveDrive.getModules();
        if (index >= 0 && index < modules.length) {
            modules[index].getAngleMotor().setVoltage(volts);
        }
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
     * @param pose           Estimated pose
     * @param timestamp      Measurement timestamp
     * @param stdDevs        Standard deviations for X, Y, and Theta
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    /**
     * Sets the voltage to all steer motors for SysId characterization.
     */
    public void setSteerVoltage(double volts) {
        swerveDrive.drive(new Translation2d(), 0, false, true); // ensure static
        for (swervelib.SwerveModule module : swerveDrive.getModules()) {
            module.getAngleMotor().setVoltage(volts);
        }
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
        swerveDrive.resetOdometry(pose);
    }

}
