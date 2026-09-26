package frc.robot.HMI.Alerts;

import edu.wpi.first.wpilibj.Timer;

/**
 * Class representing a persistent alert displayed on driver dashboards and used
 * for haptic and LED state indication without relying on terminal/console logging.
 */
public class Alert {

    public enum AlertType {
        INFO,
        WARNING,
        ERROR
    }

    private final String group;
    private String text;
    private final AlertType type;
    private boolean active = false;
    private double activeStartTime = 0.0;

    public Alert(String group, String text, AlertType type) {
        this.group = group;
        this.text = text;
        this.type = type;
        AlertManager.register(this);
    }

    public Alert(String text, AlertType type) {
        this("General", text, type);
    }

    public void set(boolean active) {
        if (this.active != active) {
            this.active = active;
            if (active) {
                this.activeStartTime = Timer.getTimestamp();
            }
        }
    }

    public void setText(String text) {
        this.text = text;
    }

    public String getText() {
        return text;
    }

    public String getGroup() {
        return group;
    }

    public AlertType getType() {
        return type;
    }

    public boolean isActive() {
        return active;
    }

    public double getActiveStartTime() {
        return activeStartTime;
    }
}
