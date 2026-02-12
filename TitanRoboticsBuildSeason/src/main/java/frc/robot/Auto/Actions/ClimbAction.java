package frc.robot.Auto.Actions;

import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Climber;

public class ClimbAction implements Actions {
    private final Climber climber = Climber.getInstance();
    private boolean done = false;

    @Override
    public void start() {
        climber.setState(Climber.ClimberState.UP);
        done = true;
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void done() {
    }
}
