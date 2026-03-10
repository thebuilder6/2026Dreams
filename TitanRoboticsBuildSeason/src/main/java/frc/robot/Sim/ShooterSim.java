package frc.robot.Sim;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Subsystems.SwerveBase;
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
                LinearSystemId.identifyVelocitySystem(ShooterConstants.kFlywheelV.get(), ShooterConstants.kFlywheelA.get()),
                DCMotor.getNEO(1),
                ShooterConstants.SIM_GEARING
        );
    }

    public void update(double voltage) {
        flywheelSim.setInput(voltage);
        flywheelSim.update(0.02);
    }

    public double getVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }

    public double getCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
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
                double exitVelocity = (flywheelVelocityRPM / 60.0) * ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE;
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

                    // Probabilistic scoring logic: Swish (tight) vs Rim (loose)
                    fuelOnFly.withTargetPosition(() -> targetLoc)
                            .withTargetTolerance(new Translation3d(0.3, 0.4, 0.2)) // Tight swish zone
                            .withHitTargetCallBack(() -> {
                                boolean isBlueGoal = targetLoc.equals(Constants.FieldConstants.BLUE_GOAL_LOCATION);
                                if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt) {
                                    Arena2026Rebuilt arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
                                    if (arena.isActive(isBlueGoal)) {
                                        simScoreCount++;
                                    }
                                } else {
                                    simScoreCount++;
                                }
                            });

                    // Secondary "Rim hit" chance (extra wide tolerance but only 40% probability)
                    if (Math.random() < 0.4) {
                        fuelOnFly.withTargetTolerance(new Translation3d(0.8, 1.0, 0.4));
                    }

                    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
                }

                simShotCount += ballsToFire;
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
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.FieldConstants.RED_GOAL_LOCATION;
        }
        return Constants.FieldConstants.BLUE_GOAL_LOCATION;
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
