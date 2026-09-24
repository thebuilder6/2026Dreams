package frc.robot.Auto.Actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import frc.robot.Interfaces.Actions;

public class ParallelRaceAction implements Actions {
    private final List<Actions> actions;

    public ParallelRaceAction(Actions... actions) {
        this.actions = new ArrayList<>(Arrays.asList(actions));
    }

    @Override
    public void start() {
        for (Actions action : actions) {
            action.start();
        }
    }

    @Override
    public void update() {
        for (Actions action : actions) {
            if (!action.isFinished()) {
                action.update();
            }
        }
    }

    @Override
    public boolean isFinished() {
        for (Actions action : actions) {
            if (action.isFinished()) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void done() {
        for (Actions action : actions) {
            action.done();
        }
    }
}
