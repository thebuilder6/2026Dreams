package frc.robot.Auto.Actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Intake;

public class IntakeAction implements Actions {
    private double seconds;
    Timer timer;
    Intake intake;
    public String state;

    /*
     * Class: Intake Action
     * Description: This sets the state of the Intake to either "Standby",
     * "Intaking", "Reverse",
     * or "Disabled"
     * Author: Mai
     */
    public IntakeAction(double seconds, String state) {
        this.seconds = seconds;
        this.state = state;
        intake = Intake.getInstance();
    }

    @Override
    public void start() {
        timer = new Timer();
        timer.start();
    }

    @Override
    public void update() {
        intake.setState(state);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= seconds;
    }

    @Override
    public void done() {
        // Do not force "Down" if the requested state was "Standby" or "Disabled"
        if ("Intaking".equalsIgnoreCase(state) || "Reversed".equalsIgnoreCase(state)) {
            intake.setState("Down");
        }
        timer.stop();
    }

}
