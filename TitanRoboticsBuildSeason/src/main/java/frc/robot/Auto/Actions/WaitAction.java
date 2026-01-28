package frc.robot.Auto.Actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Interfaces.Actions;

/* This action has the robot wait for a number of seconds
 * before moving on to the next action
*/

public class WaitAction implements Actions {
    private double seconds;
    Timer timer;

    public WaitAction(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void start() {
        timer = new Timer();
        timer.start();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return timer.get() >= seconds;
    }

    @Override
    public void done() {
        timer.stop();
    }
    
}