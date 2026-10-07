package frc.robot.Auto.Actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Interfaces.Actions;

public class ParallelRaceAction implements Actions {
    private final List<Actions> actions;

    public ParallelRaceAction(Actions... actions) {
        this(Arrays.asList(actions));
    }

    public ParallelRaceAction(List<Actions> actions) {
        this.actions = new ArrayList<>(actions);
        validateRequirements();
    }

    private void validateRequirements() {
        Set<Class<?>> claimed = new HashSet<>();
        for (Actions action : actions) {
            for (Class<?> req : action.getRequirements()) {
                if (!claimed.add(req)) {
                    DriverStation.reportWarning(
                        "[ParallelRaceAction] Subsystem resource conflict: multiple racing actions require " + req.getSimpleName(), false);
                }
            }
        }
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

    @Override
    public Set<Class<?>> getRequirements() {
        Set<Class<?>> reqs = new HashSet<>();
        for (Actions action : actions) {
            reqs.addAll(action.getRequirements());
        }
        return reqs;
    }
}
