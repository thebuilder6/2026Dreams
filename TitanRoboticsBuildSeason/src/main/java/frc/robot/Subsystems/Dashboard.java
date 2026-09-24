package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Utils.AllianceFlipUtil;

/**
 * Subsystem responsible for publishing aggregated driver telemetry, match state,
 * and caching NT4 feature toggles for high-frequency low-latency access.
 */
public class Dashboard implements Subsystem {

    private static Dashboard instance = null;

    private final frc.robot.Auto.AutoMissionChooser autoMissionChooser;

    // NT4 Cached Subscribers and Table Handle
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    private static final BooleanSubscriber snapToTurnSub = table.getBooleanTopic("Features/Snap to Turn").subscribe(true);
    private static final BooleanSubscriber ballHuntSub = table.getBooleanTopic("Features/Ball Hunt").subscribe(true);
    private static final BooleanSubscriber glidePointsSub = table.getBooleanTopic("Features/Glide Points").subscribe(true);
    private static final BooleanSubscriber fieldOrientedSub = table.getBooleanTopic("Features/Field Oriented").subscribe(true);
    private static final BooleanSubscriber slowModeSub = table.getBooleanTopic("Features/Slow Mode").subscribe(false);
    private static final BooleanSubscriber autoAimSub = table.getBooleanTopic("Features/Auto Aim").subscribe(true);
    private static final BooleanSubscriber opponentRobotSub = table.getBooleanTopic("Features/Opponent Robot").subscribe(false);
    private static final BooleanSubscriber twoPlayerDefenseSub = table.getBooleanTopic("Features/2 Player Defense").subscribe(false);
    private static final BooleanSubscriber pitModeSub = table.getBooleanTopic("Features/Pit Mode").subscribe(false);
    private static final BooleanSubscriber hapticCollisionSub = table.getBooleanTopic("Operator/HapticCollisionEnabled")
            .subscribe(!edu.wpi.first.wpilibj.RobotBase.isSimulation());

    // 2026 Game Data Variables
    private String gameData = "";
    private boolean isMyHubActive = true;
    private double hubSwitchProgress = 0.0;
    private double timeUntilSwitch = 0.0;
    private int lastActivePoints = -1;
    private Translation2d lastGoalPos = null;

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
        // Publish default toggle states if not already present on NetworkTables
        ensureTopicDefault("Features/Snap to Turn", true);
        ensureTopicDefault("Features/Ball Hunt", true);
        ensureTopicDefault("Features/Glide Points", true);
        ensureTopicDefault("Features/Field Oriented", true);
        ensureTopicDefault("Features/Slow Mode", false);
        ensureTopicDefault("Features/Auto Aim", true);
        ensureTopicDefault("Features/Opponent Robot", false);
        ensureTopicDefault("Features/2 Player Defense", false);
        ensureTopicDefault("Features/Pit Mode", false);
        ensureTopicDefault("Operator/HapticCollisionEnabled", !edu.wpi.first.wpilibj.RobotBase.isSimulation());
    }

    public static boolean isHapticCollisionEnabled() {
        return hapticCollisionSub.get();
    }

    private void ensureTopicDefault(String topicPath, boolean defaultVal) {
        if (!table.containsKey(topicPath)) {
            table.getBooleanTopic(topicPath).publish().set(defaultVal);
        }
    }

    @Override
    public void update() {
        double timeRemainingSec = DriverStation.getMatchTime();
        updateHubStatus(timeRemainingSec);
        updateFieldVisuals();

        // Update the auto mission chooser and delay
        autoMissionChooser.updateMissionCreator();

        // Sync Pit Mode
        SwerveBase.getInstance().setPitMode(isPitModeEnabled());
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
        boolean shooterAtSpeed = Shooter.getInstance().isAtTargetVelocity();
        boolean shooterLinedUp = Shooter.getInstance().isLinedUp();
        boolean canShoot = isMyHubActive && shooterAtSpeed && shooterLinedUp;

        SmartDashboard.putBoolean("Driver/Shooter Ready", shooterAtSpeed);
        SmartDashboard.putBoolean("Driver/Hub Active", isMyHubActive);
        SmartDashboard.putString("Driver/Hub Status", isMyHubActive ? "ACTIVE" : "INACTIVE");
        SmartDashboard.putNumber("Driver/Hub Shift Time Remaining", timeUntilSwitch);
        SmartDashboard.putNumber("Driver/Hub Shift Progress", hubSwitchProgress);

        SmartDashboard.putBoolean("Driver/Shoot Alert", canShoot);
        SmartDashboard.putString("Driver/Shoot Message",
                canShoot ? "READY TO FIRE" : (!isMyHubActive ? "HUB INACTIVE" : (!shooterAtSpeed ? "SPINNING UP" : "ALIGNING")));

        // Dual Flywheel RPM breakdown
        double avgActualRPM = (Shooter.getInstance().getFlywheelLeftVelocityRPM() + Shooter.getInstance().getFlywheelRightVelocityRPM()) / 2.0;
        var solution = Shooter.getInstance().getLatestShootingSolution();
        double targetRPM = solution != null ? solution.flywheelRPM() : 0.0;
        SmartDashboard.putNumber("Driver/Flywheel Actual RPM", avgActualRPM);
        SmartDashboard.putNumber("Driver/Flywheel Target RPM", targetRPM);

        SmartDashboard.putString("Driver/Intake State", Intake.getInstance().getStateString());

        var nearest = SwerveBase.getInstance().getNearestGlidePoint();
        SmartDashboard.putString("Driver/Nearest Glide", nearest == null ? "---" : nearest.name);

        SmartDashboard.putBoolean("Driver/Lined Up", shooterLinedUp);
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
        Translation2d goalPos = AllianceFlipUtil.apply(frc.robot.Data.Constants.FieldConstants.BLUE_GOAL_LOCATION).toTranslation2d();

        int maxPoints = 32;
        int activePoints = (int) (hubSwitchProgress * maxPoints);

        if (activePoints == lastActivePoints && goalPos.equals(lastGoalPos)) {
            return;
        }
        lastActivePoints = activePoints;
        lastGoalPos = goalPos;

        // Create a circular "timing ring" around the goal
        List<Pose2d> ringPoints = new ArrayList<>(activePoints);
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

    public double getTimeUntilSwitch() {
        return timeUntilSwitch;
    }

    // --- Fast NT4 Feature Toggle Getters (Zero String Lookup Overhead) ---
    public static boolean isSnapToTurnEnabled() {
        return snapToTurnSub.get();
    }

    public static boolean isBallHuntEnabled() {
        return ballHuntSub.get();
    }

    public static boolean isGlidePointsEnabled() {
        return glidePointsSub.get();
    }

    public static boolean isFieldOrientedEnabled() {
        return fieldOrientedSub.get();
    }

    public static boolean isFieldOriented() {
        return isFieldOrientedEnabled();
    }

    public static boolean isSlowModeEnabled() {
        return slowModeSub.get();
    }

    public static boolean isAutoAimEnabled() {
        return autoAimSub.get();
    }

    public static boolean isOpponentRobotEnabled() {
        return opponentRobotSub.get();
    }

    public static boolean is2PlayerDefenseEnabled() {
        return twoPlayerDefenseSub.get();
    }

    public static boolean isPitModeEnabled() {
        return pitModeSub.get();
    }
}
