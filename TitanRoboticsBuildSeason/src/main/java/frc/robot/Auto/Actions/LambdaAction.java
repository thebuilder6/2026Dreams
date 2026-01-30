package frc.robot.Auto.Actions;

import frc.robot.Interfaces.Actions;

/**
 * A simple action that runs a Runnable once and finishes immediately.
 * Perfect for quick subsystem calls.
 */
public class LambdaAction implements Actions {
    private final Runnable runnable;

    public LambdaAction(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public void start() {
        runnable.run();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}
