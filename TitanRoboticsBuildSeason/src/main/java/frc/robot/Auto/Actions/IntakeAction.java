package frc.robot.Auto.Actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Intake;

public class IntakeAction implements Actions {
    private final double seconds;
    private Timer timer;
    private final Intake intake;
    public final Intake.IntakeState intakeState;

    /**
     * Creates an IntakeAction with a duration and string state name.
     */
    public IntakeAction(double seconds, String state) {
        this(seconds, Intake.IntakeState.fromString(state));
    }

    /**
     * Creates an IntakeAction with a duration and type-safe IntakeState.
     */
    public IntakeAction(double seconds, Intake.IntakeState state) {
        this.seconds = seconds;
        this.intakeState = state;
        this.intake = Intake.getInstance();
    }

    @Override
    public void start() {
        timer = new Timer();
        timer.start();
    }

    @Override
    public void update() {
        intake.setState(intakeState);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= seconds;
    }

    @Override
    public void done() {
        if (intakeState == Intake.IntakeState.INTAKING || intakeState == Intake.IntakeState.REVERSED) {
            intake.setState(Intake.IntakeState.DOWN);
        }
        timer.stop();
    }

}
