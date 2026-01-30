package frc.robot.Auto.Actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import frc.robot.Interfaces.Actions;

/**
 * Executes a list of actions sequentially.
 * Moves to the next action only when the current one is finished.
 */
public class SeriesAction implements Actions {
    private final ArrayList<Actions> actions;
    private int currentIndex = 0;

    public SeriesAction(List<Actions> actions) {
        this.actions = new ArrayList<>(actions);
    }

    public SeriesAction(Actions... actions) {
        this.actions = new ArrayList<>(Arrays.asList(actions));
    }

    @Override
    public void start() {
        currentIndex = 0;
        if (!actions.isEmpty()) {
            actions.get(0).start();
        }
    }

    @Override
    public void update() {
        if (currentIndex < actions.size()) {
            Actions currentAction = actions.get(currentIndex);
            currentAction.update();

            if (currentAction.isFinished()) {
                currentAction.done();
                currentIndex++;
                if (currentIndex < actions.size()) {
                    actions.get(currentIndex).start();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return currentIndex >= actions.size();
    }

    @Override
    public void done() {
        // Any remaining cleanup if needed
    }
}
