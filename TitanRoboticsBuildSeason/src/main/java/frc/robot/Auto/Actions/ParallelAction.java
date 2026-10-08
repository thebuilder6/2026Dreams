
package frc.robot.Auto.Actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Interfaces.Actions;

/* This action can run two or more actions on the robot concurrently
 */

public class ParallelAction implements Actions {
    private final ArrayList<Actions> actionsToExecute;

    /**
     * Takes list of actions
     */
    public ParallelAction(List<Actions> actions) {
        this.actionsToExecute = new ArrayList<>(actions);
        validateRequirements();
    }

    /**
     * Takes array of actions
     */
    public ParallelAction(Actions... actions) {
        this(Arrays.asList(actions));
    }

    private void validateRequirements() {
        Set<Class<?>> claimed = new HashSet<>();
        for (Actions action : actionsToExecute) {
            for (Class<?> req : action.getRequirements()) {
                if (!claimed.add(req)) {
                    DriverStation.reportWarning(
                        "[ParallelAction] Subsystem resource conflict: multiple parallel actions require " + req.getSimpleName(), false);
                }
            }
        }
    }

    @Override
    public void start() {
        actionsToExecute.forEach(Actions::start);
    }

    @Override
    public void update() {
        actionsToExecute.forEach(Actions::update);
    }

    @Override
    public boolean isFinished() {
        for (Actions action : actionsToExecute) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void done() {
        actionsToExecute.forEach(Actions::done);
    }

    @Override
    public Set<Class<?>> getRequirements() {
        Set<Class<?>> reqs = new HashSet<>();
        for (Actions action : actionsToExecute) {
            reqs.addAll(action.getRequirements());
        }
        return reqs;
    }
}
