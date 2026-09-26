package frc.robot.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Telemetry.Alert.AlertType;

/**
 * Manages all robot alerts and publishes them to NetworkTables for dashboards
 * (like Elastic) without cluttering terminal/console logs.
 */
public class AlertManager {

    private static final List<Alert> alerts = new ArrayList<>();

    public static synchronized void register(Alert alert) {
        if (!alerts.contains(alert)) {
            alerts.add(alert);
        }
    }

    /**
     * Deactivates all currently registered alerts. Useful for test isolation and clean state resets.
     */
    public static synchronized void resetAll() {
        for (Alert a : alerts) {
            a.set(false);
        }
    }

    public static synchronized List<Alert> getActiveAlerts() {
        List<Alert> active = new ArrayList<>();
        for (Alert a : alerts) {
            if (a.isActive()) {
                active.add(a);
            }
        }
        return active;
    }

    public static synchronized AlertType getHighestSeverity() {
        AlertType highest = null;
        for (Alert a : alerts) {
            if (a.isActive()) {
                if (a.getType() == AlertType.ERROR) {
                    return AlertType.ERROR; // Error is maximum severity
                } else if (a.getType() == AlertType.WARNING) {
                    highest = AlertType.WARNING;
                } else if (highest == null && a.getType() == AlertType.INFO) {
                    highest = AlertType.INFO;
                }
            }
        }
        return highest;
    }

    public static synchronized boolean hasActiveErrors() {
        for (Alert a : alerts) {
            if (a.isActive() && a.getType() == AlertType.ERROR) {
                return true;
            }
        }
        return false;
    }

    public static synchronized boolean hasActiveWarnings() {
        for (Alert a : alerts) {
            if (a.isActive() && a.getType() == AlertType.WARNING) {
                return true;
            }
        }
        return false;
    }

    private static String lastBanner = "";
    private static int lastErrorCount = -1;
    private static int lastWarningCount = -1;
    private static int lastInfoCount = -1;

    public static synchronized void update() {
        List<String> errors = new ArrayList<>();
        List<String> warnings = new ArrayList<>();
        List<String> infos = new ArrayList<>();

        for (Alert a : alerts) {
            if (a.isActive()) {
                String formatted = "[" + a.getGroup() + "] " + a.getText();
                switch (a.getType()) {
                    case ERROR:
                        errors.add(formatted);
                        break;
                    case WARNING:
                        warnings.add(formatted);
                        break;
                    case INFO:
                        infos.add(formatted);
                        break;
                }
            }
        }

        // Construct high-visibility Driver Alert Banner
        String banner;
        if (!errors.isEmpty()) {
            banner = "[ERROR (" + errors.size() + ")] " + errors.get(0);
        } else if (!warnings.isEmpty()) {
            banner = "[WARN (" + warnings.size() + ")] " + warnings.get(0);
        } else if (!infos.isEmpty()) {
            banner = "[INFO] " + infos.get(0);
        } else {
            banner = "[NOMINAL] Systems Operational";
        }

        // Only publish to NetworkTables if alert state actually changed
        if (errors.size() != lastErrorCount || warnings.size() != lastWarningCount || infos.size() != lastInfoCount || !banner.equals(lastBanner)) {
            SmartDashboard.putStringArray("Alerts/Errors", errors.toArray(new String[0]));
            SmartDashboard.putStringArray("Alerts/Warnings", warnings.toArray(new String[0]));
            SmartDashboard.putStringArray("Alerts/Infos", infos.toArray(new String[0]));
            SmartDashboard.putString("Driver/AlertBanner", banner);
            AlertType highest = getHighestSeverity();
            SmartDashboard.putString("Alerts/HighestSeverity", highest != null ? highest.name() : "NONE");
            SmartDashboard.putBoolean("Alerts/HasErrors", !errors.isEmpty());
            SmartDashboard.putBoolean("Alerts/HasWarnings", !warnings.isEmpty());

            lastErrorCount = errors.size();
            lastWarningCount = warnings.size();
            lastInfoCount = infos.size();
            lastBanner = banner;
        }
    }
}
