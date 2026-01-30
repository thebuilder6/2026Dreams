package frc.robot.Devices;

import java.util.HashMap;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Data.Constants;

public class Controller extends XboxController {

    private HashMap<Integer, Boolean> debounceButtons = new HashMap<Integer, Boolean>();

    public Controller(int port) {
        super(port);
    }

    public boolean getDebouncedButton(int button) {
        if (!debounceButtons.containsKey(button)) {
            debounceButtons.put(button, false);
        }
        if (this.getRawButton(button) && debounceButtons.get(button)) {
            debounceButtons.put(button, false);
            return false;
        } else if (this.getRawButtonPressed(button)) {
            debounceButtons.put(button, true);
            return true;
        } else {
            return debounceButtons.get(button);
        }
    }

    public boolean getDebouncedButton(Button button) {
        if (!debounceButtons.containsKey(button.value)) {
            debounceButtons.put(button.value, false);
        }

        if (this.getRawButton(button.value) && debounceButtons.get(button.value)) {
            debounceButtons.put(button.value, false);
            return false;
        } else if (this.getRawButtonPressed(button.value)) {
            debounceButtons.put(button.value, true);
            return true;
        } else {
            return debounceButtons.get(button.value);
        }
    }

    private double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, Constants.OperatorConstants.DEADBAND);
    }

    @Override
    public double getLeftX() {
        return applyDeadband(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return applyDeadband(super.getLeftY());
    }

    @Override
    public double getRightX() {
        return applyDeadband(super.getRightX());
    }

    @Override
    public double getRightY() {
        return applyDeadband(super.getRightY());
    }

    public boolean isOperational() {
        return this.isConnected();
    }

    public String getName() {
        return "Controller";
    }
}