package frc.robot.Auto.Actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Shooter;
import frc.robot.Data.Constants.ShooterConstants;

public class ShootAction implements Actions {
    private final double rpm;
    private final double duration;
    private final Timer timer = new Timer();
    private final Shooter shooter = Shooter.getInstance();
    private boolean feeding = false;

    public ShootAction(double rpm, double duration) {
        this.rpm = rpm;
        this.duration = duration;
    }

    @Override
    public void start() {
        timer.restart();
        shooter.setFlywheelVelocity(rpm);
        feeding = false;
    }

    @Override
    public void update() {
        if (!feeding && shooter.isAtTargetVelocity()) {
            shooter.setKickerSpeed(ShooterConstants.FEED_SPEED);
            feeding = true;
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void done() {
        shooter.stop();
        timer.stop();
    }
}
