package frc.robot.Auto.Actions;

import java.util.Set;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Intake;

/**
 * Action that finishes as soon as the intake reports game piece acquisition, or when
 * a specified timeout elapses.
 * Designed for use in ParallelRaceAction alongside a collection trajectory to allow early exit.
 */
public class WaitForBallAction implements Actions {
    private final double timeoutSeconds;
    private final Timer timer = new Timer();
    private final Intake intake;

    public WaitForBallAction(double timeoutSeconds) {
        this.timeoutSeconds = timeoutSeconds;
        this.intake = Intake.getInstance();
    }

    public WaitForBallAction() {
        this(5.0); // Default 5.0 s safety timeout
    }

    @Override
    public void start() {
        timer.restart();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return (intake != null && intake.hasGamePiece()) || timer.hasElapsed(timeoutSeconds);
    }

    @Override
    public void done() {
        timer.stop();
    }

    @Override
    public Set<Class<?>> getRequirements() {
        return Set.of(Intake.class);
    }
}
