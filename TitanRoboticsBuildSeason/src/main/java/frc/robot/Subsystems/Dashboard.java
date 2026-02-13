package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Subsystem;

public class Dashboard implements Subsystem {

    private static Dashboard instance = null;

    private final frc.robot.Auto.AutoMissionChooser autoMissionChooser;

    // 2026 Game Data Variables
    private String gameData = "";
    private boolean isMyHubActive = true;
    private double hubSwitchProgress = 0.0;
    private double timeUntilSwitch = 0.0;

    public static Dashboard getInstance() {
        if (instance == null) {
            instance = new Dashboard();
        }
        return instance;
    }

    private Dashboard() {
        autoMissionChooser = new frc.robot.Auto.AutoMissionChooser();

        setupLayout();

        SubsystemManager.registerSubsystem(this);
    }

    private void setupLayout() {
        // Auto Routine selection is published to SmartDashboard/Auto Routine by the
        // chooser automatically
        // No Shuffleboard specific layout needed for Elastic

        // --- Feature Toggles ---
        // Initialize with default values if not present
        if (!SmartDashboard.containsKey("Features/Snap to Turn"))
            SmartDashboard.putBoolean("Features/Snap to Turn", true);
        if (!SmartDashboard.containsKey("Features/Ball Hunt"))
            SmartDashboard.putBoolean("Features/Ball Hunt", true);
        if (!SmartDashboard.containsKey("Features/Glide Points"))
            SmartDashboard.putBoolean("Features/Glide Points", true);
        if (!SmartDashboard.containsKey("Features/Field Oriented"))
            SmartDashboard.putBoolean("Features/Field Oriented", true);
        if (!SmartDashboard.containsKey("Features/Slow Mode"))
            SmartDashboard.putBoolean("Features/Slow Mode", false);
        if (!SmartDashboard.containsKey("Features/Auto Aim"))
            SmartDashboard.putBoolean("Features/Auto Aim", true);
        if (!SmartDashboard.containsKey("Features/Opponent Robot"))
            SmartDashboard.putBoolean("Features/Opponent Robot", false);
        if (!SmartDashboard.containsKey("Features/2 Player Defense"))
            SmartDashboard.putBoolean("Features/2 Player Defense", false);

    }

    @Override
    public void update() {
        double timeRemainingSec = DriverStation.getMatchTime();
        updateHubStatus(timeRemainingSec);
        updateFieldVisuals();
    }

    @Override
    public void log() {
        double timeRemainingSec = DriverStation.getMatchTime();
        boolean isTimeValid = timeRemainingSec >= 0.0;

        // --- 2026 Match Info ---
        SmartDashboard.putBoolean("Match/TimeRemainingValid", isTimeValid);
        SmartDashboard.putNumber("Match/TimeRemainingSec", Math.max(0.0, timeRemainingSec));
        SmartDashboard.putBoolean("Match/HubActive", isMyHubActive);
        SmartDashboard.putNumber("Match/Hub Switch Progress", hubSwitchProgress);
        SmartDashboard.putNumber("Match/Time Until Switch", timeUntilSwitch);
        SmartDashboard.putString("Match/GameData", gameData);

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
        SmartDashboard.putString("Match/Alliance", DriverStation.getAlliance().map(Enum::toString).orElse(""));

        // --- Driver Aggregation (For Elastic) ---
        SmartDashboard.putBoolean("Driver/Shooter Ready", Shooter.getInstance().isAtTargetVelocity());
        SmartDashboard.putBoolean("Driver/Hub Active", isMyHubActive);
        SmartDashboard.putString("Driver/Hub Status", isMyHubActive ? "ACTIVE" : "INACTIVE");

        boolean canShoot = isMyHubActive && Shooter.getInstance().isAtTargetVelocity()
                && Shooter.getInstance().isLinedUp();
        SmartDashboard.putBoolean("Driver/Shoot Alert", canShoot);
        SmartDashboard.putString("Driver/Shoot Message",
                canShoot ? "READY TO FIRE" : (isMyHubActive ? "WAITING FOR FLYWHEEL/ALIGN" : "HUB INACTIVE"));

        SmartDashboard.putString("Driver/Intake State", Intake.getInstance().getState().toString());

        var nearest = SwerveBase.getInstance().getNearestGlidePoint();
        SmartDashboard.putString("Driver/Nearest Glide", nearest == null ? "---" : nearest.name);

        SmartDashboard.putBoolean("Driver/Lined Up", Shooter.getInstance().isLinedUp());
    }

    /**
     * Calculates if the Alliance Hub is active based on 2026 Shift Rules.
     * 
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
        if (myAlliance.isEmpty()) {
            isMyHubActive = true;
            return;
        }

        // Match Time counts DOWN. Teleop starts at 2:20 (140s).
        // Transition: 140 -> 130
        // Shift 1: 130 -> 105
        // Shift 2: 105 -> 80
        // Shift 3: 80 -> 55
        // Shift 4: 55 -> 30
        // End Game: 30 -> 0

        if (matchTime > 130 || matchTime <= 30) {
            // Transition Period or End Game
            isMyHubActive = true;
            hubSwitchProgress = 1.0; // Fully active
            timeUntilSwitch = (matchTime > 130) ? (matchTime - 130) : matchTime;
        } else {
            // SHIFTS (25s intervals)
            double shiftStartTime = 0;
            if (matchTime > 105)
                shiftStartTime = 130;
            else if (matchTime > 80)
                shiftStartTime = 105;
            else if (matchTime > 55)
                shiftStartTime = 80;
            else
                shiftStartTime = 55;

            double elapsedInShift = shiftStartTime - matchTime;
            hubSwitchProgress = Math.min(1.0, elapsedInShift / 25.0);
            timeUntilSwitch = Math.max(0.0, 25.0 - elapsedInShift);

            if ((matchTime <= 130 && matchTime > 105) || (matchTime <= 80 && matchTime > 55)) {
                // SHIFT 1 or SHIFT 3
                if (myAlliance.get() == Alliance.Red) {
                    isMyHubActive = !redStartsInactive;
                } else {
                    isMyHubActive = !blueStartsInactive;
                }
            } else if ((matchTime <= 105 && matchTime > 80) || (matchTime <= 55 && matchTime > 30)) {
                // SHIFT 2 or SHIFT 4 (Statuses flip)
                if (myAlliance.get() == Alliance.Red) {
                    isMyHubActive = redStartsInactive;
                } else {
                    isMyHubActive = blueStartsInactive;
                }
            }
        }
    }

    /**
     * Updates the Field2d visual representation of the hub timing.
     */
    private void updateFieldVisuals() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Translation2d goalPos;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            goalPos = frc.robot.Data.Constants.FieldConstants.RED_GOAL_LOCATION.toTranslation2d();
        } else {
            goalPos = frc.robot.Data.Constants.FieldConstants.BLUE_GOAL_LOCATION.toTranslation2d();
        }

        // Create a circular "timing ring" around the goal
        // We use a series of Pose2d objects to represent the progress
        int maxPoints = 32;
        int activePoints = (int) (hubSwitchProgress * maxPoints);
        List<Pose2d> ringPoints = new ArrayList<>();
        double radius = 1.0; // 1 meter radius around the hub

        for (int i = 0; i < activePoints; i++) {
            double angleRad = (i / (double) maxPoints) * 2.0 * Math.PI;
            ringPoints.add(new Pose2d(
                    goalPos.getX() + radius * Math.cos(angleRad),
                    goalPos.getY() + radius * Math.sin(angleRad),
                    Rotation2d.fromRadians(angleRad)));
        }

        // Publish to Field2d
        SwerveBase.getInstance().getField().getObject("Driver/Hub Timing Ring").setPoses(ringPoints);
    }

    @Override
    public void initialize() {
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

    // --- Feature Toggle Getters ---
    public static boolean isSnapToTurnEnabled() {
        return SmartDashboard.getBoolean("Features/Snap to Turn", true);
    }

    public static boolean isBallHuntEnabled() {
        return SmartDashboard.getBoolean("Features/Ball Hunt", true);
    }

    public static boolean isGlidePointsEnabled() {
        return SmartDashboard.getBoolean("Features/Glide Points", true);
    }

    public static boolean isFieldOrientedEnabled() {
        return SmartDashboard.getBoolean("Features/Field Oriented", true);
    }

    public static boolean isSlowModeEnabled() {
        return SmartDashboard.getBoolean("Features/Slow Mode", false);
    }

    public static boolean isAutoAimEnabled() {
        return SmartDashboard.getBoolean("Features/Auto Aim", true);
    }

    public static boolean isOpponentRobotEnabled() {
        return SmartDashboard.getBoolean("Features/Opponent Robot", false);
    }

    public static boolean is2PlayerDefenseEnabled() {
        return SmartDashboard.getBoolean("Features/2 Player Defense", false);
    }
}
