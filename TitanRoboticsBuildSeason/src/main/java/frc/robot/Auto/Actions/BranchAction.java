package frc.robot.Auto.Actions;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;
import frc.robot.Interfaces.Actions;

/**
 * Conditionally executes one of two actions based on a boolean condition evaluated at start().
 * Allows reactive branching in autonomous (e.g. branch based on game piece possession).
 */
public class BranchAction implements Actions {
    private final Supplier<Boolean> condition;
    private final Actions trueAction;
    private final Actions falseAction;
    private Actions selectedAction;
    private boolean started = false;

    /**
     * Creates a BranchAction.
     *
     * @param condition   Boolean supplier evaluated when this action starts
     * @param trueAction  Action to run if condition is true (can be null for no-op)
     * @param falseAction Action to run if condition is false (can be null for no-op)
     */
    public BranchAction(Supplier<Boolean> condition, Actions trueAction, Actions falseAction) {
        this.condition = condition;
        this.trueAction = trueAction;
        this.falseAction = falseAction;
    }

    @Override
    public void start() {
        started = true;
        boolean result = (condition != null) && Boolean.TRUE.equals(condition.get());
        selectedAction = result ? trueAction : falseAction;
        if (selectedAction != null) {
            selectedAction.start();
        }
    }

    @Override
    public void update() {
        if (selectedAction != null) {
            selectedAction.update();
        }
    }

    @Override
    public boolean isFinished() {
        if (!started) {
            return false;
        }
        return selectedAction == null || selectedAction.isFinished();
    }

    @Override
    public void done() {
        if (selectedAction != null) {
            selectedAction.done();
        }
    }

    @Override
    public Set<Class<?>> getRequirements() {
        Set<Class<?>> reqs = new HashSet<>();
        if (trueAction != null) {
            reqs.addAll(trueAction.getRequirements());
        }
        if (falseAction != null) {
            reqs.addAll(falseAction.getRequirements());
        }
        return reqs;
    }
}
