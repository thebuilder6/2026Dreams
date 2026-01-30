package frc.robot.Subsystems;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Subsystem;

public class Dashboard implements Subsystem {
    
    private static Dashboard instance = null;

    private final ShuffleboardTab driverTab;
    private final frc.robot.Auto.AutoMissionChooser autoMissionChooser;

    // 2026 Game Data Variables
    private String gameData = "";
    private boolean isMyHubActive = true; // Default to active until told otherwise

    public static Dashboard getInstance() {
        if (instance == null) {
            instance = new Dashboard();
        }
        return instance;
    }

    private Dashboard() {
        driverTab = Shuffleboard.getTab("Driver");
        autoMissionChooser = new frc.robot.Auto.AutoMissionChooser();

        setupLayout();

        SubsystemManager.registerSubsystem(this);
    }

    private void setupLayout() {
        // Add Auto Routine from the chooser
        driverTab.add("Auto Routine", autoMissionChooser.getRawChooser())
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
        
        // Match Timer & Phase
        driverTab.addString("Match Phase", () -> {
            if (DriverStation.isAutonomous()) {
                return "AUTO (" + String.format("%.0f", DriverStation.getMatchTime()) + ")";
            } else if (DriverStation.isTeleop()) {
                double time = DriverStation.getMatchTime();
                if (time < 30) { // ENDGAME starts at 30s remaining per 2026 rules
                    return "ENDGAME (" + String.format("%.0f", time) + ")";
                }
                return "TELEOP (" + String.format("%.0f", time) + ")";
            } else {
                return "DISABLED";
            }
        })
                .withPosition(2, 0)
                .withSize(2, 1);

        // 2026 HUB STATUS (New Addition)
        // Green = Active (Score!), Red = Inactive (Hold Fuel)
        driverTab.addBoolean("HUB ACTIVE", () -> isMyHubActive)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FF0000"))
                .withPosition(4, 0)
                .withSize(2, 2);

        // Shooter Status
        driverTab.addBoolean("Shooter Ready", () -> Shooter.getInstance().isAtTargetVelocity())
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FF0000"))
                .withPosition(6, 0) // Moved slightly to accommodate Hub Status
                .withSize(2, 2);

        // Intake Status
        driverTab.addString("Intake State", () -> Intake.getInstance().getState().toString())
                .withPosition(0, 1)
                .withSize(2, 1);

        driverTab.addString("Nearest Glide Point", () -> {
            var nearest = SwerveBase.getInstance().getNearestGlidePoint();
            return nearest == null ? "" : nearest.name;
        })
                .withPosition(2, 1)
                .withSize(2, 1);

        // Field
        driverTab.add("Field", SwerveBase.getInstance().getField())
                .withWidget(BuiltInWidgets.kField)
                .withPosition(0, 2)
                .withSize(6, 4);
    }

    @Override
    public void update() {
        double timeRemainingSec = DriverStation.getMatchTime();
        boolean isTimeValid = timeRemainingSec >= 0.0;
        SmartDashboard.putBoolean("Match/TimeRemainingValid", isTimeValid);
        SmartDashboard.putNumber("Match/TimeRemainingSec", Math.max(0.0, timeRemainingSec));

        // --- 2026 Game Data Logic Start ---
        updateHubStatus(timeRemainingSec);
        SmartDashboard.putBoolean("Match/HubActive", isMyHubActive);
        SmartDashboard.putString("Match/GameData", gameData);
        // --- 2026 Game Data Logic End ---

        String phase;
        if (DriverStation.isAutonomous()) {
            phase = "Auto";
        } else if (DriverStation.isTeleop()) {
            phase = "Teleop";
        } else if (DriverStation.isTest()) {
            phase = "Test";
        } else {
            phase = "Disabled";
        }

        SmartDashboard.putString("Match/Phase", phase);
        SmartDashboard.putBoolean("Match/IsEnabled", DriverStation.isEnabled());
        SmartDashboard.putBoolean("Match/IsDSAttached", DriverStation.isDSAttached());
        SmartDashboard.putBoolean("Match/IsFMSAttached", DriverStation.isFMSAttached());
        SmartDashboard.putBoolean("Match/IsEStopped", DriverStation.isEStopped());

        SmartDashboard.putString("Match/EventName", DriverStation.getEventName());
        SmartDashboard.putString("Match/MatchType", DriverStation.getMatchType().toString());
        SmartDashboard.putNumber("Match/MatchNumber", DriverStation.getMatchNumber());
        SmartDashboard.putNumber("Match/ReplayNumber", DriverStation.getReplayNumber());

        SmartDashboard.putString("Match/Alliance", DriverStation.getAlliance().map(Enum::toString).orElse(""));
        SmartDashboard.putNumber("Match/Location", DriverStation.getLocation().orElse(-1));
    }

    /**
     * Calculates if the Alliance Hub is active based on 2026 Shift Rules.
     * @param matchTime The time remaining in the match (seconds)
     */
    private void updateHubStatus(double matchTime) {
        // 1. Get Game Data if we don't have it
        if (gameData == null || gameData.isEmpty()) {
            gameData = DriverStation.getGameSpecificMessage();
        }

        // 2. Default to true (Active) if Auto, Transition, End Game, or no data yet
        if (DriverStation.isAutonomous() || gameData.isEmpty()) {
            isMyHubActive = true;
            return;
        }

        // 3. Logic for Teleop Shifts
        // 'R' = Red Hub Inactive first. 'B' = Blue Hub Inactive first.
        char targetChar = gameData.charAt(0);
        boolean redStartsInactive = (targetChar == 'R');
        boolean blueStartsInactive = (targetChar == 'B');

        Optional<Alliance> myAlliance = DriverStation.getAlliance();
        
        // If we don't know our alliance, assume active to be safe
        if(myAlliance.isEmpty()) {
            isMyHubActive = true;
            return;
        }

        // Match Time counts DOWN. Teleop starts at 2:20 (140s).
        // Transition: 140 -> 130
        // Shift 1:    130 -> 105
        // Shift 2:    105 -> 80
        // Shift 3:    80 -> 55
        // Shift 4:    55 -> 30
        // End Game:   30 -> 0

        if (matchTime > 130 || matchTime <= 30) {
            // Transition Period or End Game
            isMyHubActive = true;
        } 
        else if ((matchTime <= 130 && matchTime > 105) || (matchTime <= 80 && matchTime > 55)) {
            // SHIFT 1 or SHIFT 3
            if (myAlliance.get() == Alliance.Red) {
                isMyHubActive = !redStartsInactive; 
            } else {
                isMyHubActive = !blueStartsInactive;
            }
        } 
        else if ((matchTime <= 105 && matchTime > 80) || (matchTime <= 55 && matchTime > 30)) {
            // SHIFT 2 or SHIFT 4 (Statuses flip)
            if (myAlliance.get() == Alliance.Red) {
                isMyHubActive = redStartsInactive; 
            } else {
                isMyHubActive = blueStartsInactive;
            }
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void log() {
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "Dashboard";
    }

    public frc.robot.Auto.AutoMissionChooser getAutoChooser() {
        return autoMissionChooser;
    }
    
    // Getter for other subsystems (e.g., Shooter) to check before firing
    public boolean isHubActive() {
        return isMyHubActive;
    }
}
