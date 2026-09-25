package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.Subsystems.intake.IntakeIOSim;
import frc.robot.Subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.Subsystems.shooter.ShooterIOSim;
import frc.robot.Subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.Subsystems.vision.VisionIOSim;

public class HardwareIOTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testShooterIOSimInputs() {
        ShooterIOSim shooterSim = new ShooterIOSim();
        ShooterIOInputs inputs = new ShooterIOInputs();

        // Initial update
        shooterSim.updateInputs(inputs);
        assertNotNull(inputs);

        // Apply voltages
        shooterSim.setFlywheelVoltages(12.0, 12.0);
        shooterSim.updateInputs(inputs);
        assertEquals(12.0, inputs.leftAppliedVolts, 1e-3);
        assertEquals(12.0, inputs.rightAppliedVolts, 1e-3);

        shooterSim.stop();
        shooterSim.updateInputs(inputs);
        assertEquals(0.0, inputs.leftAppliedVolts, 1e-3);
        assertEquals(0.0, inputs.rightAppliedVolts, 1e-3);
    }

    @Test
    public void testIntakeIOSimInputs() {
        IntakeIOSim intakeSim = new IntakeIOSim();
        IntakeIOInputs inputs = new IntakeIOInputs();

        intakeSim.updateInputs(inputs);
        assertNotNull(inputs);
        assertTrue(inputs.encoderConnected);

        intakeSim.setRollerSpeed(0.8);
        intakeSim.setArmVoltage(4.0);
        intakeSim.updateInputs(inputs);

        assertEquals(4.0, inputs.armAppliedVolts, 1e-3);

        intakeSim.stop();
        intakeSim.updateInputs(inputs);
        assertEquals(0.0, inputs.armAppliedVolts, 1e-3);
        assertEquals(0.0, inputs.rollerAppliedVolts, 1e-3);
    }

    @Test
    public void testIntakeMapleSimAttachment() {
        IntakeIOSim intakeSim = new IntakeIOSim();
        assertNull(intakeSim.getMapleIntakeSim());

        // Create a simulated drivetrain config & simulation
        swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation driveSim = 
            new swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation(
                swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig.Default(),
                new Pose2d()
            );

        intakeSim.attachMapleSimDrivetrain(driveSim);
        assertNotNull(intakeSim.getMapleIntakeSim());
        assertFalse(intakeSim.getMapleIntakeSim().isRunning());

        intakeSim.setRollerSpeed(0.8);
        assertTrue(intakeSim.getMapleIntakeSim().isRunning());

        intakeSim.stop();
        assertFalse(intakeSim.getMapleIntakeSim().isRunning());
    }

    @Test
    public void testVisionIOSimInputs() {
        VisionIOSim visionSim = new VisionIOSim();
        VisionIOInputs inputs = new VisionIOInputs();

        // Feed robot pose and orientation
        visionSim.setSimulatedPose(new Pose2d(5.0, 4.0, new Rotation2d()));
        visionSim.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        visionSim.updateInputs(inputs);
        assertNotNull(inputs);
        assertTrue(inputs.hasTarget);
        assertEquals(1, inputs.tagCount);
        assertEquals(5.0, inputs.estimatedPose.getX(), 0.1);
        assertEquals(4.0, inputs.estimatedPose.getY(), 0.1);
    }

    @Test
    public void testDriveIOSimInputs() {
        frc.robot.Subsystems.drive.DriveIOSim driveSim = new frc.robot.Subsystems.drive.DriveIOSim();
        frc.robot.Subsystems.drive.DriveIO.DriveIOInputs inputs = new frc.robot.Subsystems.drive.DriveIO.DriveIOInputs();

        driveSim.updateInputs(inputs);
        assertNotNull(inputs);

        driveSim.setModuleDriveVoltage(0, 3.5);
        driveSim.setModuleAngleVoltage(0, 2.0);
        driveSim.setPose(new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(45.0)));

        driveSim.updateInputs(inputs);
        assertEquals(3.5, inputs.driveAppliedVolts[0], 1e-3);
        assertEquals(2.0, inputs.steerAppliedVolts[0], 1e-3);
        assertEquals(3.0, inputs.odometryPose.getX(), 1e-3);
        assertEquals(5.0, inputs.odometryPose.getY(), 1e-3);

        driveSim.stop();
        driveSim.updateInputs(inputs);
        assertEquals(0.0, inputs.driveAppliedVolts[0], 1e-3);
        assertEquals(0.0, inputs.steerAppliedVolts[0], 1e-3);
    }

    @Test
    public void testVisionIOPhotonVisionInputs() {
        frc.robot.Subsystems.vision.VisionIOPhotonVision photonIO = 
                new frc.robot.Subsystems.vision.VisionIOPhotonVision();
        VisionIOInputs inputs = new VisionIOInputs();

        photonIO.updateInputs(inputs);
        assertNotNull(inputs);
        assertEquals("rubik-pi-coprocessor", photonIO.getCameraName());
        assertFalse(inputs.hasGamePiece);
    }

    @Test
    public void testVisionGroundPlaneProjectionMath() {
        VisionIOSim primarySim = new VisionIOSim();
        VisionIOSim secondarySim = new VisionIOSim();
        Vision vision = new Vision(primarySim, secondarySim);

        // Initially no game piece
        assertFalse(vision.hasGamePiece());
        assertEquals(0.0, vision.getGamePieceDistanceMeters(), 1e-3);
        assertNull(vision.getGamePieceFieldPose());

        // Simulate target detection: e.g. pitch = 0 degrees (looking down at camera mounting angle -15 deg)
        // Camera height: 0.45m, fuel height: 0.075m, diff = 0.375m
        // totalAngle = -15 deg -> distance = 0.375 / tan(15 deg) ≈ 1.3995m
        secondarySim.setGamePieceDetected(true, 10.0, 0.0, 5.0);
        vision.update();

        assertTrue(vision.hasGamePiece());
        double dist = vision.getGamePieceDistanceMeters();
        assertTrue(dist > 1.2 && dist < 1.6, "Calculated distance should be approx 1.4m, was: " + dist);

        var robotRel = vision.getGamePieceRobotRelativeTranslation();
        assertNotNull(robotRel);
        assertTrue(robotRel.getX() > 0.5, "Forward distance should be positive");
    }

    @Test
    public void testPhotonVisionSimInitialization() {
        frc.robot.Sim.VisionSim visionSim = frc.robot.Sim.VisionSim.getInstance();
        assertNotNull(visionSim);
        assertNotNull(visionSim.getVisionSystemSim());
        assertNotNull(visionSim.getCameraSim());

        // Update with arbitrary field pose
        assertDoesNotThrow(() -> visionSim.update(new Pose2d(3.0, 3.0, new Rotation2d())));
    }

    @Test
    public void testVisionIOSimRubikPiType() {
        VisionIOSim rubikSim = new VisionIOSim(VisionIOSim.CameraType.RUBIK_PI);
        assertEquals(VisionIOSim.CameraType.RUBIK_PI, rubikSim.getCameraType());

        VisionIOInputs inputs = new VisionIOInputs();
        rubikSim.setSimulatedPose(new Pose2d(4.0, 4.0, new Rotation2d()));
        rubikSim.updateInputs(inputs);

        assertNotNull(inputs);
        // By default with no targets, hasGamePiece is false
        assertFalse(inputs.hasGamePiece);

        // With manual override
        rubikSim.setGamePieceDetected(true, -5.0, 2.0, 3.5);
        rubikSim.updateInputs(inputs);
        assertTrue(inputs.hasGamePiece);
        assertEquals(-5.0, inputs.gamePieceYaw, 1e-3);
        assertEquals(2.0, inputs.gamePiecePitch, 1e-3);
    }

    @Test
    public void testNeoSparkMaxMotorCanOptimization() {
        com.revrobotics.spark.config.SparkMaxConfig config = new com.revrobotics.spark.config.SparkMaxConfig();
        assertDoesNotThrow(() -> frc.robot.Devices.NeoSparkMaxMotor.optimizeCanBusUtilization(config, true, true));

        com.revrobotics.spark.config.SparkMaxConfig flywheelConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        assertDoesNotThrow(() -> frc.robot.Devices.NeoSparkMaxMotor.optimizeCanBusUtilization(flywheelConfig, false, true));
    }

    @Test
    public void testHardwareSparkMaxIOInstantiation() {
        // Verify that hardware IO implementations initialize cleanly in simulation/test environment
        assertDoesNotThrow(() -> {
            frc.robot.Subsystems.shooter.ShooterIOSparkMax shooterIO = new frc.robot.Subsystems.shooter.ShooterIOSparkMax();
            shooterIO.updateInputs(new ShooterIOInputs());
            shooterIO.stop();
        });

        assertDoesNotThrow(() -> {
            frc.robot.Subsystems.intake.IntakeIOSparkMax intakeIO = new frc.robot.Subsystems.intake.IntakeIOSparkMax();
            intakeIO.updateInputs(new IntakeIOInputs());
            intakeIO.stop();
        });
    }
}
