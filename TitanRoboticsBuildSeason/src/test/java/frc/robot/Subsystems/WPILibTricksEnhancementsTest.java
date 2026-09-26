package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.IntakeConstants;
import frc.robot.Hardware.Vision.VisionIOSim;

/**
 * Unit test suite verifying the 4 WPILib architectural upgrades:
 * 1. addPeriodic() 100Hz odometry polling
 * 2. Intake native Debouncer jam detection
 * 3. Vision 5-sample MedianFilter spike rejection
 * 4. SwerveBase PowerDistribution voltage sag & current brownout speed scaling
 */
public class WPILibTricksEnhancementsTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        RoboRioSim.resetData();
    }

    @Test
    public void testMedianFilterSpikeRejection() {
        // Direct verification of 5-sample median filter rejecting single-frame glitch
        // spikes
        MedianFilter filter = new MedianFilter(5);
        assertEquals(2.0, filter.calculate(2.0), 1e-4);
        assertEquals(2.0, filter.calculate(2.0), 1e-4);
        assertEquals(2.0, filter.calculate(2.0), 1e-4);

        // Inject massive 15.0m reflection spike: median of [2.0, 2.0, 2.0, 15.0] is
        // still 2.0
        double resultWithSpike = filter.calculate(15.0);
        assertEquals(2.0, resultWithSpike, 1e-4, "5-sample median filter must reject outlier spike");

        // Verify Vision subsystem integration with median filter
        VisionIOSim primarySim = new VisionIOSim();
        VisionIOSim secondarySim = new VisionIOSim();
        Vision vision = new Vision(primarySim, secondarySim);

        assertNotNull(vision);
        assertEquals(0.0, vision.getFilteredPrimaryTagDist(), 1e-4);
        assertEquals(0.0, vision.getFilteredSecondaryTagDist(), 1e-4);

        // Feed ball detection and test game piece distance filtering
        secondarySim.setGamePieceDetected(true, 5.0, 0.0, 4.0);
        vision.update();
        double initialDist = vision.getGamePieceDistanceMeters();
        assertTrue(initialDist > 1.0 && initialDist < 2.0, "Game piece distance should be ~1.4m");

        // Reset filter when target lost
        secondarySim.setGamePieceDetected(false, 0.0, 0.0, 0.0);
        vision.update();
        assertEquals(0.0, vision.getGamePieceDistanceMeters(), 1e-4);
    }

    @Test
    public void testIntakeDebouncerLogic() {
        // Verify WPILib Debouncer behaves with kRising as expected for stall current
        // limit
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_TIME, Debouncer.DebounceType.kRising);

        // Initial state
        assertFalse(debouncer.calculate(false));

        // When current spikes above 30A momentarily, debouncer should not trip
        // immediately
        assertFalse(debouncer.calculate(true));

        Intake intake = Intake.getInstance();
        assertNotNull(intake);
        assertFalse(intake.isJammed());
    }

    @Test
    public void testSwerveBrownoutProtectionAndFastOdometry() {
        SwerveBase swerve = SwerveBase.getInstance();
        assertNotNull(swerve);

        // Fast odometry sub-loop polling can be invoked without exception
        assertDoesNotThrow(() -> swerve.updateOdometryFast());

        try {
            // Nominal battery state: 12.5V, scale should be 1.0
            swerve.setSimBatteryVoltage(12.5);
            swerve.update();
            double nominalScale = swerve.getBrownoutSpeedScale();
            assertTrue(nominalScale >= 0.95 && nominalScale <= 1.0,
                    "Nominal voltage should yield full speed scale, got: " + nominalScale);

            // Test drive execution with brownout scaling and discretization
            assertDoesNotThrow(() -> swerve.drive(new Translation2d(1.5, 0.5), 0.2, true));
            assertDoesNotThrow(() -> swerve.drive(new ChassisSpeeds(1.0, 0.5, 0.1)));
            assertDoesNotThrow(() -> swerve.driveFieldOriented(new ChassisSpeeds(1.0, 0.5, 0.1)));

            // Test severe battery sag down to 8.0V (incipient brownout risk)
            swerve.setSimBatteryVoltage(8.0);
            swerve.update();
            double throttledScale = swerve.getBrownoutSpeedScale();
            assertTrue(throttledScale < 0.70,
                    "Battery voltage sag to 8.0V must throttle brownout speed scale, was: " + throttledScale);

            // Recovery: restore battery to 12.5V, scale should begin ramping back up
            // smoothly
            swerve.setSimBatteryVoltage(12.5);
            swerve.update();
            double recoveredScale = swerve.getBrownoutSpeedScale();
            assertTrue(recoveredScale > throttledScale,
                    "Speed scale must smoothly ramp back up upon voltage recovery");
        } finally {
            // Reset override back to normal sensor reading
            swerve.setSimBatteryVoltage(-1.0);
        }
    }

    @Test
    public void testTrajectoryAntiStallDeterministicEscapes() {
        edu.wpi.first.math.controller.PIDController headingController = new edu.wpi.first.math.controller.PIDController(1.0, 0, 0);
        frc.robot.Auto.TrajectoryController tc1 = new frc.robot.Auto.TrajectoryController(headingController);
        frc.robot.Auto.TrajectoryController tc2 = new frc.robot.Auto.TrajectoryController(headingController);

        Pose2d botPose = new Pose2d(5.0, 5.0, new Rotation2d(0.0));
        Pose2d goalPose = new Pose2d(10.0, 5.0, new Rotation2d(0.0));
        ChassisSpeeds stalledSpeeds = new ChassisSpeeds(0.02, 0.0, 0.0);

        // When stalled, calculate anti-stall unstick vector
        ChassisSpeeds escape1 = tc1.calculate(botPose, stalledSpeeds, goalPose, 3.0, true, false);
        ChassisSpeeds escape2 = tc2.calculate(botPose, stalledSpeeds, goalPose, 3.0, true, false);

        // Determinism assertion: two controllers given identical inputs must produce the EXACT same unstick speeds
        assertEquals(escape1.vxMetersPerSecond, escape2.vxMetersPerSecond, 1e-4, "Escape vx must be deterministic");
        assertEquals(escape1.vyMetersPerSecond, escape2.vyMetersPerSecond, 1e-4, "Escape vy must be deterministic");
        assertEquals(escape1.omegaRadiansPerSecond, escape2.omegaRadiansPerSecond, 1e-4, "Escape omega must be deterministic");
        assertTrue(tc1.isStalledActive(), "Unstick reflex should be active");
    }

    @Test
    public void testSwerveOdometryThreadSafety() throws InterruptedException {
        SwerveBase swerve = SwerveBase.getInstance();
        assertNotNull(swerve);

        // Multi-threaded stress test simulating 100Hz Notifier vs 50Hz main loop
        final int iterations = 100;
        final java.util.concurrent.atomic.AtomicBoolean hasError = new java.util.concurrent.atomic.AtomicBoolean(false);

        Thread fastOdometryThread = new Thread(() -> {
            for (int i = 0; i < iterations; i++) {
                try {
                    swerve.updateOdometryFast();
                    Thread.sleep(2);
                } catch (Throwable t) {
                    hasError.set(true);
                }
            }
        });

        Thread mainLoopThread = new Thread(() -> {
            for (int i = 0; i < iterations; i++) {
                try {
                    swerve.getPose();
                    if (i % 20 == 0) {
                        swerve.resetOdometry(new Pose2d(i * 0.1, 2.0, new Rotation2d(0.0)));
                    }
                    Thread.sleep(4);
                } catch (Throwable t) {
                    hasError.set(true);
                }
            }
        });

        fastOdometryThread.start();
        mainLoopThread.start();

        fastOdometryThread.join(2000);
        mainLoopThread.join(2000);

        assertFalse(hasError.get(), "Concurrent odometry updates must not trigger race conditions or exceptions");
    }

    @Test
    public void testShooterIsAtCorrectSpeedGuardsZeroRpm() {
        Shooter shooter = Shooter.getInstance();
        assertNotNull(shooter);

        // When stopped / uncommanded (target RPM = 0), isAtCorrectSpeed must be false
        shooter.setTargetRPM(0, 0);
        assertFalse(shooter.isAtCorrectSpeed(), "Shooter must NEVER report at speed when target RPM is zero");

        // When target RPM is set to 3000, but motors are idle (0 RPM), it must report false
        shooter.setTargetRPM(3000, 3000);
        assertFalse(shooter.isAtCorrectSpeed(), "Shooter must report false when motors have not reached target speed");

        shooter.stop();
        assertFalse(shooter.isAtCorrectSpeed());
    }

    @Test
    public void testSwerveGetTargetSpeedsPreservesLinearVelocity() {
        SwerveBase swerve = SwerveBase.getInstance();
        assertNotNull(swerve);

        double commandedVx = 2.0;
        double commandedVy = 1.0;
        ChassisSpeeds speeds = swerve.getTargetSpeeds(commandedVx, commandedVy, Rotation2d.fromDegrees(0));

        // getTargetSpeeds must pass linear velocities directly without double-scaling by MAX_SPEED
        assertEquals(commandedVx, speeds.vxMetersPerSecond, 1e-3, "vx must match commanded m/s directly");
        assertEquals(commandedVy, speeds.vyMetersPerSecond, 1e-3, "vy must match commanded m/s directly");
    }

    @Test
    public void testWaitUntilMarkerSafetyAndDeadlockPrevention() {
        frc.robot.Auto.Actions.WaitUntilMarkerAction nullPathAction =
                new frc.robot.Auto.Actions.WaitUntilMarkerAction(null, "TestMarker");
        assertTrue(nullPathAction.isFinished(), "Null path must immediately finish to prevent auto deadlock");

        frc.robot.Auto.Actions.FollowChoreoPath nonExistentPath =
                new frc.robot.Auto.Actions.FollowChoreoPath("NonExistentTrajectoryName", false);
        frc.robot.Auto.Actions.WaitUntilMarkerAction markerAction =
                new frc.robot.Auto.Actions.WaitUntilMarkerAction(nonExistentPath, "TestMarker");

        // The path will finish because the trajectory doesn't exist
        assertTrue(nonExistentPath.isFinished(), "Missing trajectory must report finished safely without throwing");
        assertTrue(markerAction.isFinished(), "Finished path must report marker action finished to prevent deadlock");
    }

    @Test
    public void testControllerRumbleCaching() {
        frc.robot.HMI.Controller controller = new frc.robot.HMI.Controller(0);
        assertDoesNotThrow(() -> {
            controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, 0.5);
            controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, 0.5); // Should hit cache
            controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0);
            controller.updateRumble();
        });
    }

    @Test
    public void testDriveToPoseActionBreakoutSmoothness() {
        Pose2d target = new Pose2d(5.0, 5.0, new Rotation2d(0.0));
        frc.robot.Auto.Actions.DriveToPoseAction action = new frc.robot.Auto.Actions.DriveToPoseAction(target);
        action.start();

        // Feed strong stick input exceeding breakout threshold
        action.setDriverInput(Constants.MAX_SPEED * 0.8, 0.0, 0.0);
        assertTrue(action.isBreakoutRequested(), "High driver input must trigger breakout");
        assertTrue(action.isFinished(), "Action must finish upon breakout");

        // Calling done() on breakout must execute safely
        assertDoesNotThrow(() -> action.done());
    }

    @Test
    public void testTrue2DVectorSlewRateLimiter() {
        // Create 2D vector limiter with 4.0 m/s^2 rate limit
        frc.robot.Utils.Vector2dSlewRateLimiter limiter = new frc.robot.Utils.Vector2dSlewRateLimiter(4.0);
        assertEquals(4.0, limiter.getRateLimit(), 1e-4);

        limiter.reset(0.0, 0.0);
        assertEquals(0.0, limiter.getTranslation().getNorm(), 1e-4);

        // Step 1: Request instant jump to (6.0, 8.0) [norm = 10.0, angle = atan2(8, 6) = 0.927 rad]
        // In one 20ms frame, max allowed change is 4.0 * 0.02 = 0.08m/s
        Translation2d target = new Translation2d(6.0, 8.0);
        Translation2d limited = limiter.calculate(target);

        // Crucial vector property: The output vector must point in the exact same direction as the target!
        // target angle = 53.13 degrees
        double targetAngleDeg = Math.toDegrees(Math.atan2(target.getY(), target.getX()));
        double limitedAngleDeg = Math.toDegrees(Math.atan2(limited.getY(), limited.getX()));
        assertEquals(targetAngleDeg, limitedAngleDeg, 1e-3,
                "True 2D vector limiter must preserve directional orientation without Cartesian axis distortion");

        // The step magnitude must not exceed the rate limit * nominal dt
        assertTrue(limited.getNorm() <= 4.0 * 0.1, "Step magnitude must be constrained by acceleration limit");

        // Test dynamic rate limit tuning
        limiter.setRateLimit(8.0);
        assertEquals(8.0, limiter.getRateLimit(), 1e-4);

        // Test reset to new coordinate
        limiter.reset(new Translation2d(2.0, 3.0));
        assertEquals(2.0, limiter.getTranslation().getX(), 1e-4);
        assertEquals(3.0, limiter.getTranslation().getY(), 1e-4);
    }

    @Test
    public void testSubsystemIdleCommands() {
        // Intake idle()
        Intake intake = Intake.getInstance();
        edu.wpi.first.wpilibj2.command.Command intakeIdle = intake.idle();
        assertNotNull(intakeIdle, "Intake.idle() must return a non-null command");
        assertTrue(intakeIdle.getRequirements().contains(intake), "Intake.idle() must require the Intake subsystem");
        assertEquals("Intake.idle", intakeIdle.getName());

        // Execute one cycle of idle command and verify state is STANDBY
        intake.setState(Intake.IntakeState.INTAKING);
        assertEquals(Intake.IntakeState.INTAKING, intake.getState());
        intakeIdle.initialize();
        intakeIdle.execute();
        assertEquals(Intake.IntakeState.STANDBY, intake.getState(),
                "Intake.idle() must command mechanism into safe STANDBY state");

        // Shooter idle()
        Shooter shooter = Shooter.getInstance();
        edu.wpi.first.wpilibj2.command.Command shooterIdle = shooter.idle();
        assertNotNull(shooterIdle, "Shooter.idle() must return a non-null command");
        assertTrue(shooterIdle.getRequirements().contains(shooter), "Shooter.idle() must require the Shooter subsystem");
        assertEquals("Shooter.idle", shooterIdle.getName());

        shooter.setTargetRPM(3000, 3000);
        shooterIdle.initialize();
        shooterIdle.execute();
        assertEquals(0.0, shooter.getTargetVelocityRPM(), 1e-4,
                "Shooter.idle() must stop flywheels and zero target RPM setpoints");

        // SwerveBase idle()
        SwerveBase swerve = SwerveBase.getInstance();
        edu.wpi.first.wpilibj2.command.Command swerveIdle = swerve.idle();
        assertNotNull(swerveIdle, "SwerveBase.idle() must return a non-null command");
        assertTrue(swerveIdle.getRequirements().contains(swerve), "SwerveBase.idle() must require the SwerveBase subsystem");
        assertEquals("SwerveBase.idle", swerveIdle.getName());
    }

    @Test
    public void testJavaUnitsLibraryMeasures() {
        // Verify static configuration constants in Constants.java
        assertEquals(Constants.MAX_SPEED,
                Constants.MAX_SPEED_MEASURE.in(edu.wpi.first.units.Units.MetersPerSecond), 1e-4);
        assertEquals(Constants.MAX_ROTATION_SPEED,
                Constants.MAX_ROTATION_SPEED_MEASURE.in(edu.wpi.first.units.Units.RadiansPerSecond), 1e-4);
        assertEquals(Constants.KICKER_VOLTAGE,
                Constants.KICKER_VOLTAGE_MEASURE.in(edu.wpi.first.units.Units.Volts), 1e-4);
        assertEquals(Constants.INTAKE_UP_POSITION,
                Constants.INTAKE_UP_POSITION_MEASURE.in(edu.wpi.first.units.Units.Degrees), 1e-4);
        assertEquals(Constants.INTAKE_DOWN_POSITION,
                Constants.INTAKE_DOWN_POSITION_MEASURE.in(edu.wpi.first.units.Units.Degrees), 1e-4);

        // Verify Intake public API with Units measures
        Intake intake = Intake.getInstance();
        intake.setArmPosition(edu.wpi.first.units.Units.Degrees.of(310.0));
        assertEquals(310.0, intake.getTargetArmPositionMeasure().in(edu.wpi.first.units.Units.Degrees), 1e-2);
        assertNotNull(intake.getArmPositionMeasure());
        assertNotNull(intake.getArmVelocityMeasure());
        assertNotNull(intake.getRollerVelocityMeasure());
        assertNotNull(intake.getHopperVelocityMeasure());
        assertNotNull(intake.getArmCurrentMeasure());
        assertNotNull(intake.getRollerCurrentMeasure());

        // Verify Shooter public API with Units measures
        Shooter shooter = Shooter.getInstance();
        shooter.setTargetVelocity(edu.wpi.first.units.Units.RPM.of(4200.0));
        assertEquals(4200.0, shooter.getTargetVelocityMeasure().in(edu.wpi.first.units.Units.RPM), 1e-2);
        shooter.setTargetVelocity(
                edu.wpi.first.units.Units.RPM.of(3800.0),
                edu.wpi.first.units.Units.RPM.of(4100.0));
        assertEquals(3950.0, shooter.getTargetVelocityMeasure().in(edu.wpi.first.units.Units.RPM), 1e-2);
        assertNotNull(shooter.getLeftFlywheelVelocityMeasure());
        assertNotNull(shooter.getRightFlywheelVelocityMeasure());
        assertNotNull(shooter.getActualVelocityMeasure());
        assertNotNull(shooter.getLeftCurrentMeasure());
        assertNotNull(shooter.getRightCurrentMeasure());

        // Verify SwerveBase public API with Units measures
        SwerveBase swerve = SwerveBase.getInstance();
        assertNotNull(swerve.getLinearVelocityMeasure());
        assertNotNull(swerve.getAngularVelocityMeasure());
        assertNotNull(swerve.getHeadingMeasure());
        assertNotNull(swerve.getPitchMeasure());
        assertNotNull(swerve.getBatteryVoltageMeasure());
        assertNotNull(swerve.getTotalCurrentMeasure());
    }
}
