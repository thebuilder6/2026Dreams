package frc.robot.Devices;

import java.util.HashMap;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import frc.robot.Data.Constants;

public class Controller extends XboxController {

    public enum RumblePattern {
        NONE,
        TARGET_LOCKED,      // Two crisp pulses indicating vision lock
        BALL_ACQUIRED,      // Single confirmation pulse when ball is loaded
        HARDWARE_WARNING,   // Rapid triple buzz for degraded sensors or faults
        MATCH_TIME_WARNING  // Long deep rumble alerting endgame timing
    }

    private HashMap<Integer, Boolean> debounceButtons = new HashMap<Integer, Boolean>();
    private RumblePattern currentPattern = RumblePattern.NONE;
    private double patternStartTime = 0.0;
    private double pulseEndTime = 0.0;

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

    /**
     * Trigger a simple single rumble pulse.
     *
     * @param intensity Rumble magnitude [0.0, 1.0]
     * @param durationSec Duration in seconds
     */
    public void pulseRumble(double intensity, double durationSec) {
        pulseEndTime = Timer.getFPGATimestamp() + durationSec;
        setRumble(RumbleType.kBothRumble, intensity);
    }

    /**
     * Trigger a sequenced tactile feedback pattern.
     *
     * @param pattern The RumblePattern to execute
     */
    public void triggerRumblePattern(RumblePattern pattern) {
        currentPattern = pattern;
        patternStartTime = Timer.getFPGATimestamp();
    }

    /**
     * Non-blocking periodic update for active rumble patterns. Call once per robot loop.
     */
    public void updateRumble() {
        double now = Timer.getFPGATimestamp();

        // Check if an explicit pulse is active
        if (now < pulseEndTime) {
            return;
        }

        if (currentPattern == RumblePattern.NONE) {
            setRumble(RumbleType.kBothRumble, 0.0);
            return;
        }

        double elapsed = now - patternStartTime;

        switch (currentPattern) {
            case TARGET_LOCKED:
                // Double crisp tap on right motor
                if (elapsed < 0.10) {
                    setRumble(RumbleType.kRightRumble, 0.85);
                    setRumble(RumbleType.kLeftRumble, 0.0);
                } else if (elapsed < 0.18) {
                    setRumble(RumbleType.kBothRumble, 0.0);
                } else if (elapsed < 0.28) {
                    setRumble(RumbleType.kRightRumble, 0.85);
                    setRumble(RumbleType.kLeftRumble, 0.0);
                } else {
                    currentPattern = RumblePattern.NONE;
                    setRumble(RumbleType.kBothRumble, 0.0);
                }
                break;

            case BALL_ACQUIRED:
                // Solid medium confirmation buzz
                if (elapsed < 0.22) {
                    setRumble(RumbleType.kBothRumble, 0.65);
                } else {
                    currentPattern = RumblePattern.NONE;
                    setRumble(RumbleType.kBothRumble, 0.0);
                }
                break;

            case HARDWARE_WARNING:
                // Rapid triple warning pulse
                if (elapsed < 0.08) {
                    setRumble(RumbleType.kBothRumble, 1.0);
                } else if (elapsed < 0.15) {
                    setRumble(RumbleType.kBothRumble, 0.0);
                } else if (elapsed < 0.23) {
                    setRumble(RumbleType.kBothRumble, 1.0);
                } else if (elapsed < 0.30) {
                    setRumble(RumbleType.kBothRumble, 0.0);
                } else if (elapsed < 0.38) {
                    setRumble(RumbleType.kBothRumble, 1.0);
                } else {
                    currentPattern = RumblePattern.NONE;
                    setRumble(RumbleType.kBothRumble, 0.0);
                }
                break;

            case MATCH_TIME_WARNING:
                // Sustained 0.5s rumble
                if (elapsed < 0.50) {
                    setRumble(RumbleType.kBothRumble, 0.9);
                } else {
                    currentPattern = RumblePattern.NONE;
                    setRumble(RumbleType.kBothRumble, 0.0);
                }
                break;

            default:
                currentPattern = RumblePattern.NONE;
                setRumble(RumbleType.kBothRumble, 0.0);
                break;
        }
    }

    public boolean isOperational() {
        return this.isConnected();
    }

    public String getName() {
        return "Controller";
    }
}