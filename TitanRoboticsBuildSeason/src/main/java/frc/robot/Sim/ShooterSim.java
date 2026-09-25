package frc.robot.Sim;

import frc.robot.Intelligence.State.MatchScoreTracker;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Utils.AllianceFlipUtil;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterSim {
    private final FlywheelSim flywheelSim;
    
    // Ball simulation state
    private double lastBallSpawnTime = 0;
    private long simShotCount = 0;
    private long simScoreCount = 0;

    public ShooterSim() {
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), ShooterConstants.SIM_MOI, ShooterConstants.SIM_GEARING),
                DCMotor.getNEO(1),
                ShooterConstants.SIM_GEARING
        );
    }

    public void update(double voltage) {
        flywheelSim.setInput(voltage);
        flywheelSim.update(0.02);
    }

    public void update(double leftVoltage, double rightVoltage) {
        flywheelSim.setInput(leftVoltage);
        flywheelSim.update(0.02);
    }

    public double getVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }

    public double getLeftVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }

    public double getRightVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }

    public double getCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
    }

    public double getLeftCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
    }

    public double getRightCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
    }

    public void launchSimulatedFuel(double leftRPM, double rightRPM) {
        updateBallSimulation(1.0, (leftRPM + rightRPM) / 2.0, (leftRPM + rightRPM) / 2.0, 12.0);
    }
    
    /**
     * Updates ball simulation logic.
     * @param kickerSpeed Current kicker motor speed
     * @param flywheelVelocityRPM Current flywheel velocity in RPM
     * @param targetVelocityRPM Target flywheel velocity in RPM
     * @param loopVoltage Current loop voltage output
     */
    public void updateBallSimulation(double kickerSpeed, double flywheelVelocityRPM, double targetVelocityRPM, double loopVoltage) {
        double currentTime = Timer.getFPGATimestamp();

        // IF Kicker is running AND we have waited long enough since last ball
        if (Math.abs(kickerSpeed) > 0.1 && (currentTime - lastBallSpawnTime) > ShooterConstants.BALL_SPAWN_INTERVAL) {
            int ballsToFire = GameSim.getInstance().consumeHeldBallsForShot(2);
            if (ballsToFire > 0) {
                Pose2d robotPose = SwerveBase.getInstance().getPose();
                ChassisSpeeds robotVel = SwerveBase.getInstance().getFieldVelocity();
                double wheelSurfaceVelocity = (flywheelVelocityRPM / 60.0) * ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE;
                double exitVelocity = wheelSurfaceVelocity * ShooterConstants.BALL_LAUNCH_EFFICIENCY.get();
                Translation3d targetLoc = getGoalLocation();

                for (int i = 0; i < ballsToFire; i++) {
                    // Lateral offset for 2-wide shooter (+/- 0.12m)
                    double lateralOffset = (ballsToFire == 2) ? (i == 0 ? -0.12 : 0.12) : 0.0;
                    Translation2d shooterOffset = new Translation2d(ShooterConstants.SHOOTER_OFFSET_METERS.get(), lateralOffset);

                    // Introduce Randomness (+/- 2% velocity, +/- 0.5 deg yaw, +/- 1 deg pitch)
                    double randomExitVelocity = exitVelocity * (1.0 + (Math.random() - 0.5) * 0.04);
                    Rotation2d randomYaw = robotPose.getRotation().plus(Rotation2d.fromDegrees((Math.random() - 0.5) * 1.0));
                    double randomPitch = ShooterConstants.SHOOTER_ANGLE_RAD + (Math.random() - 0.5) * 0.035; // ~2 deg total spread

                    var fuelOnFly = new RebuiltFuelOnFly(
                            robotPose.getTranslation(),
                            shooterOffset,
                            robotVel,
                            randomYaw,
                            Meters.of(ShooterConstants.SHOOTER_HEIGHT_METERS.get()),
                            MetersPerSecond.of(randomExitVelocity),
                            Radians.of(randomPitch));

                    // Target detection aperture: center of Hub, positioned slightly inside the upper funnel (Z = 1.48m)
                    // so the ball visibly crosses through the 1.575m rim into the goal before registering the score
                    Translation3d funnelTarget = new Translation3d(targetLoc.getX(), targetLoc.getY(), 1.48);

                    fuelOnFly.withTargetPosition(() -> funnelTarget)
                            .withTargetTolerance(new Translation3d(0.35, 0.35, 0.15))
                            .withHitTargetCallBack(() -> {
                                boolean isBlueGoal = targetLoc.equals(Constants.FieldConstants.BLUE_GOAL_LOCATION);
                                boolean isRedGoal = !isBlueGoal;
                                if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt) {
                                    Arena2026Rebuilt arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
                                    if (arena.isActive(isBlueGoal)) {
                                        simScoreCount++;
                                        MatchScoreTracker.getInstance().recordPlayerScore(isRedGoal);
                                    } else {
                                        MatchScoreTracker.getInstance().recordWastedShot(isRedGoal);
                                    }
                                } else {
                                    simScoreCount++;
                                    MatchScoreTracker.getInstance().recordPlayerScore(isRedGoal);
                                }
                            })
                            .withProjectileTrajectoryDisplayCallBack(
                                (poses) -> org.littletonrobotics.junction.Logger.recordOutput("FieldSimulation/SuccessfulShotsTrajectory", poses.toArray(edu.wpi.first.math.geometry.Pose3d[]::new)),
                                (poses) -> org.littletonrobotics.junction.Logger.recordOutput("FieldSimulation/MissedShotsTrajectory", poses.toArray(edu.wpi.first.math.geometry.Pose3d[]::new))
                            );

                    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
                }

                simShotCount += ballsToFire;
                MatchScoreTracker.getInstance().recordPlayerShotAttempt(ballsToFire);
                lastBallSpawnTime = currentTime;
            }
        }
    }
    
    /**
     * Gets the goal location based on the current alliance.
     * Defaults to Blue Goal if alliance is not found.
     * 
     * @return Translation3d of the target goal.
     */
    private Translation3d getGoalLocation() {
        return AllianceFlipUtil.apply(Constants.FieldConstants.BLUE_GOAL_LOCATION);
    }
    
    // Getters for simulation counters
    public long getSimShotCount() {
        return simShotCount;
    }
    
    public long getSimScoreCount() {
        return simScoreCount;
    }
    
    /**
     * Estimates total current draw including kicker.
     * @param kickerSpeed Current kicker motor speed
     * @return Total current draw in amps
     */
    public double getTotalCurrentDraw(double kickerSpeed) {
        // Estimate kicker current (stall current is ~2.6A for NEO 550, free is ~0.4A)
        // Using a simple resistive model matching simulated voltage
        double kickerCurrent = Math.abs(kickerSpeed) * 2.0;
        return flywheelSim.getCurrentDrawAmps() + kickerCurrent;
    }
}
