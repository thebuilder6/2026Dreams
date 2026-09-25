package frc.robot.Auto.Actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.ShootingSolution;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Utils.AllianceFlipUtil;

/*
 * Class: ShootAction
 * Description: Commands the shooter to fire using either dynamic auto-aim solution
 *              or explicit RPM setpoints for a specified duration.
 * Authors: Rhea, Sarah, InfiniteQuery
 */
public class ShootAction implements Actions {
    private final double seconds;
    private final Timer timer = new Timer();
    private final Shooter shooter;
    private final SwerveBase swerveBase;

    private final boolean useAutoAim;
    private final double manualRpm;

    /** Dynamic auto-aim shooting action for the specified duration. */
    public ShootAction(double seconds) {
        this.seconds = seconds;
        this.shooter = Shooter.getInstance();
        this.swerveBase = SwerveBase.getInstance();
        this.useAutoAim = true;
        this.manualRpm = 0;
    }

    /** Explicit RPM shooting action for the specified duration. */
    public ShootAction(double rpm, double duration) {
        this.seconds = duration;
        this.shooter = Shooter.getInstance();
        this.swerveBase = SwerveBase.getInstance();
        this.useAutoAim = false;
        this.manualRpm = rpm;
    }

    @Override
    public void start() {
        timer.restart();
        if (!useAutoAim) {
            shooter.setTargetRPM(manualRpm, manualRpm);
        }
    }

    @Override
    public void update() {
        if (useAutoAim) {
            ShootingSolution solution = shooter.getLatestShootingSolution();
            shooter.setTargetRPM(solution.flywheelRpmLeft(), solution.flywheelRpmRight());

            if (solution.shotPossibility()) {
                swerveBase.driveFieldOriented(swerveBase.getTargetSpeeds(0, 0, solution.shootingAngle()));
                if (Math.abs(solution.shootingAngle().minus(swerveBase.getHeading()).getDegrees()) < 3.0) {
                    shooter.shoot();
                } else {
                    shooter.prepareToShoot();
                }
            } else {
                shooter.stop();
            }
        } else {
            boolean inAllianceZone = AllianceFlipUtil.isPoseInAllianceZone(swerveBase.getPose());
            if (shooter.isAtCorrectSpeed() && inAllianceZone) {
                shooter.shoot();
            } else {
                shooter.prepareToShoot();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void done() {
        timer.stop();
        shooter.stop();
        swerveBase.stop();
    }
}
