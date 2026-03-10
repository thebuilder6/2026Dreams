package frc.robot.Sim;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.AutonConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * AIRobotSim Subsystem
 * Logic:
 * 1. If Opponent Robot is OFF -> Move to queuing position.
 * 2. If Opponent Robot is ON:
 * a. If 2 Player Defense is ON -> Control via Joystick (Port 2).
 * b. If 2 Player Defense is OFF -> Follow Choreo Path.
 */
public class AIRobotSim implements Subsystem {

    private static AIRobotSim instance;

    // Queuing positions in safe corners when opponent robot is disabled
    // Field is approximately 16.5m x 8.2m, so these positions are in the corners
    public static final Pose2d[] ROBOT_QUEUING_POSITIONS = new Pose2d[] {
            new Pose2d(1.0, -5, new Rotation2d()),
            new Pose2d(1.5, -5, new Rotation2d()),
            new Pose2d(2.0, -5, new Rotation2d())
    };

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queuingPose;

    // Path Following
    private Optional<Trajectory<SwerveSample>> trajectory = Optional.empty();
    private final Timer pathTimer = new Timer();
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;

    // 2 Player Control
    private final XboxController defenseController;

    // Track previous state to detect when opponent robot is first enabled
    private boolean wasOpponentEnabled = false;

    public static AIRobotSim getInstance() {
        if (instance == null) {
            instance = new AIRobotSim();
        }
        return instance;
    }

    private AIRobotSim() {
        this.queuingPose = ROBOT_QUEUING_POSITIONS[0];

        // Initialize simulation with a default configuration
        this.driveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default(),
                        queuingPose));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());

        // Path Controllers
        this.xController = new PIDController(AutonConstants.kAutoDriveP, AutonConstants.kAutoDriveI,
                AutonConstants.kAutoDriveD);
        this.yController = new PIDController(AutonConstants.kAutoDriveP, AutonConstants.kAutoDriveI,
                AutonConstants.kAutoDriveD);

        // Heading PID
        var config = SwerveBase.getInstance().getSwerveController().config;
        this.headingController = new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.defenseController = new XboxController(2);

        SubsystemManager.registerSubsystem(this);
    }

    public void setTrajectory(String pathName) {
        this.trajectory = Choreo.loadTrajectory(pathName);
        pathTimer.restart();
    }

    /**
     * Resets the opponent robot state. Should be called when autonomous starts.
     */
    public void reset() {
        wasOpponentEnabled = false;
        pathTimer.restart();
    }

    @Override
    public void simulationUpdate() {
        boolean opponentEnabled = Dashboard.isOpponentRobotEnabled();
        boolean defenseMode = Dashboard.is2PlayerDefenseEnabled();

        if (!opponentEnabled) {
            // "Hide" the robot
            driveSimulation.setSimulationWorldPose(queuingPose);
            driveSimulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), true, true);
            wasOpponentEnabled = false;
            return;
        }

        // When opponent robot is first enabled, position it at the mirrored trajectory
        // start
        if (!wasOpponentEnabled && trajectory.isPresent()) {
            Optional<SwerveSample> initialSample = trajectory.get().sampleAt(0, false);
            if (initialSample.isPresent()) {
                SwerveSample sample = initialSample.get();
                Pose2d startPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));

                // Mirror the pose to opposite alliance
                boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                Pose2d mirroredPose = mirrorPoseForOpponent(startPose, isRedAlliance);

                driveSimulation.setSimulationWorldPose(mirroredPose);
                pathTimer.restart();
            }
        }
        wasOpponentEnabled = true;

        ChassisSpeeds targetSpeeds;

        if (defenseMode) {
            // 2 Player manual control
            double x = -defenseController.getLeftY();
            double y = -defenseController.getLeftX();
            double rot = -defenseController.getRightX();

            // Apply deadband
            x = Math.abs(x) < 0.1 ? 0 : x;
            y = Math.abs(y) < 0.1 ? 0 : y;
            rot = Math.abs(rot) < 0.1 ? 0 : rot;

            targetSpeeds = new ChassisSpeeds(x * Constants.MAX_SPEED, y * Constants.MAX_SPEED, rot * 5.0);
        } else {
            // Choreo Path Following (if traj loaded)
            if (trajectory.isPresent()) {
                double time = pathTimer.get();
                if (time > trajectory.get().getTotalTime()) {
                    pathTimer.restart(); // Loop the path for simulation variety
                    time = 0;
                }

                Optional<SwerveSample> sampleOpt = trajectory.get().sampleAt(time, false);
                if (sampleOpt.isPresent()) {
                    SwerveSample sample = sampleOpt.get();

                    // Mirror the target pose for opponent
                    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    Pose2d targetPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
                    Pose2d mirroredTarget = mirrorPoseForOpponent(targetPose, !isRedAlliance);

                    Pose2d currentPose = driveSimulation.getActualPoseInSimulationWorld();

                    targetSpeeds = new ChassisSpeeds(
                            sample.vx + xController.calculate(currentPose.getX(), mirroredTarget.getX()),
                            sample.vy + yController.calculate(currentPose.getY(), mirroredTarget.getY()),
                            sample.omega + headingController.calculate(currentPose.getRotation().getRadians(),
                                    mirroredTarget.getRotation().getRadians()));
                } else {
                    targetSpeeds = new ChassisSpeeds();
                }
            } else {
                targetSpeeds = new ChassisSpeeds();
            }
        }

        driveSimulation.runChassisSpeeds(targetSpeeds, new Translation2d(), true, true);
    }

    @Override
    public void update() {
        // Log position to Dashboard for AdvantageScope visibility
        Pose2d pose = driveSimulation.getActualPoseInSimulationWorld();
        SmartDashboard.putNumberArray("Simulation/OpponentPose", new double[] {
                pose.getX(), pose.getY(), pose.getRotation().getDegrees()
        });
    }

    @Override
    public void initialize() {
        // Load a default path if available
        setTrajectory("OpponentPath"); // Example path
    }

    /**
     * Mirrors a pose to the opposite alliance side of the field.
     * Field is symmetric about the centerline at X = 8.27m
     */
    private Pose2d mirrorPoseForOpponent(Pose2d pose, boolean playerIsRed) {
        // If player is red, opponent should be blue (no mirror)
        // If player is blue, opponent should be red (mirror)
        if (playerIsRed) {
            return pose; // Opponent is blue, use pose as-is
        } else {
            // Mirror across field centerline (X = 8.27m for 2025 field)
            double mirroredX = 16.54 - pose.getX();
            Rotation2d mirroredRotation = pose.getRotation().plus(Rotation2d.fromDegrees(180));
            return new Pose2d(mirroredX, pose.getY(), mirroredRotation);
        }
    }

    @Override
    public void log() {
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "AIRobotSim";
    }
}
