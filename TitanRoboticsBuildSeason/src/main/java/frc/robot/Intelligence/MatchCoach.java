package frc.robot.Intelligence;

import frc.robot.Subsystems.SubsystemManager;

import frc.robot.Subsystems.Intake;

import frc.robot.Subsystems.Shooter;

import frc.robot.Subsystems.SwerveBase;

import frc.robot.Telemetry.Dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.DynamicRouter;
import frc.robot.HMI.Watchdogs.LegalPinningWatchdog;
import frc.robot.Auto.SmartTunnelRouter;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Intelligence.Sparring.AIRobotSim;
import frc.robot.Intelligence.Sparring.AIRobotSim.AIMode;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Sim.GameSim;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.State.WorldState;
import frc.robot.Intelligence.State.WorldStateBuilder;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * MatchCoach: Real-Time AI Coaching & Practice Proving Ground System.
 * 
 * Capabilities:
 * 1. Cycle & Shot Analytics: Real-time calculation of cycle duration, shooting accuracy, and wasted shots.
 * 2. Driver Grading: Dynamic evaluation (A+ down to D) based on cycle speed and alignment discipline.
 * 3. Contextual HUD Coaching: Instantaneous tactical prompts and telegraph warnings during practice and matches.
 * 4. Practice Drill Engine: Selectable training routines with 1-click arena reset and automated AI sparring configuration.
 * 5. Structured Telemetry: Emits TypeSafe Jev-compliant coaching summaries for external review and LLM coaching bridges.
 */
public class MatchCoach implements Subsystem {

    public enum DrillMode {
        FREE_PLAY("Free Play Match"),
        RAPID_CYCLING("Rapid Cycling Sprint"),
        TRENCH_DEFENSE("Trench Defense & Pirouette Drill"),
        ANTI_DEFENSE_SHOOTING("Anti-Defense SOTF Drill");

        public final String displayName;

        DrillMode(String name) {
            this.displayName = name;
        }

        public static DrillMode fromString(String name) {
            for (DrillMode m : values()) {
                if (m.name().equalsIgnoreCase(name) || m.displayName.equalsIgnoreCase(name)) {
                    return m;
                }
            }
            return FREE_PLAY;
        }
    }

    private static MatchCoach instance = null;

    // Subsystem references
    private final SwerveBase swerve = SwerveBase.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();

    // Practice / Drill State
    private DrillMode currentDrill = DrillMode.FREE_PLAY;
    private final SendableChooser<DrillMode> drillChooser = new SendableChooser<>();
    private double drillStartTime = 0.0;
    private boolean isDrillActive = false;

    // Cycle Metrics
    private double currentCycleStartTime = -1.0;
    private double totalCycleDurationSec = 0.0;
    private int completedCyclesCount = 0;
    private double lastCycleDurationSec = 0.0;
    private double fastestCycleSec = Double.MAX_VALUE;
    private boolean wasHoldingFuel = false;

    // Shot Quality Metrics
    private int totalShotsAttempted = 0;
    private int goodShotsOnTarget = 0;
    private int wastedShotsInactiveHub = 0;
    private int misalignedShots = 0;
    private boolean wasShooting = false;

    // Tactical & Navigation Metrics
    private int pinWarningsCount = 0;
    private boolean wasPinned = false;
    private int trenchPassagesCount = 0;
    private int trenchDiversionsCount = 0;
    private boolean wasInTrench = false;

    // Live Coach Banner
    private String activeCoachingTip = "Ready. Practice session initialized.";
    private String driverGrade = "A";

    public static MatchCoach getInstance() {
        if (instance == null) {
            instance = new MatchCoach();
        }
        return instance;
    }

    private MatchCoach() {
        SubsystemManager.registerSubsystem(this);
        for (DrillMode mode : DrillMode.values()) {
            if (mode == DrillMode.FREE_PLAY) {
                drillChooser.setDefaultOption(mode.displayName, mode);
            } else {
                drillChooser.addOption(mode.displayName, mode);
            }
        }
        SmartDashboard.putData("Coaching/DrillModeChooser", drillChooser);
    }

    @Override
    public void initialize() {
        resetSessionStats();
    }

    /**
     * Resets all coaching session analytics and cycle timers.
     */
    public void resetSessionStats() {
        currentCycleStartTime = -1.0;
        totalCycleDurationSec = 0.0;
        completedCyclesCount = 0;
        lastCycleDurationSec = 0.0;
        fastestCycleSec = Double.MAX_VALUE;
        wasHoldingFuel = false;

        totalShotsAttempted = 0;
        goodShotsOnTarget = 0;
        wastedShotsInactiveHub = 0;
        misalignedShots = 0;
        wasShooting = false;

        pinWarningsCount = 0;
        wasPinned = false;
        trenchPassagesCount = 0;
        trenchDiversionsCount = 0;
        wasInTrench = false;

        drillStartTime = Timer.getTimestamp();
        isDrillActive = true;
        driverGrade = "A";
        activeCoachingTip = "Session reset. Execute cycles to begin coaching.";
    }

    @Override
    public void update() {
        double now = Timer.getTimestamp();

        // 1. Check for Dashboard Practice Reset Trigger
        if (SmartDashboard.getBoolean("Coaching/ResetPractice", false)) {
            SmartDashboard.putBoolean("Coaching/ResetPractice", false);
            executePracticeReset();
        }

        // 2. Sync Selected Drill Mode
        DrillMode chosen = drillChooser.getSelected();
        DrillMode selectedDrill = (chosen != null)
                ? chosen
                : DrillMode.fromString(SmartDashboard.getString("Coaching/DrillMode", currentDrill.displayName));
        if (selectedDrill != currentDrill) {
            currentDrill = selectedDrill;
            configureDrillEnvironment(currentDrill);
        }

        // 3. Track Fuel Ingestion & Cycle Timers
        boolean currentlyHoldingFuel = intake.hasFuel();
        if (!wasHoldingFuel && currentlyHoldingFuel) {
            // New cycle started: fuel ingested
            if (currentCycleStartTime < 0.0) {
                currentCycleStartTime = now;
            }
        }
        wasHoldingFuel = currentlyHoldingFuel;

        // 4. Track Shot Execution Quality
        boolean isShooting = shooter.isShooting();
        if (isShooting && !wasShooting) {
            totalShotsAttempted++;

            boolean hubActive = dashboard.isHubActive();
            boolean flywheelsAtSpeed = shooter.isAtCorrectSpeed();
            boolean headingLinedUp = shooter.isLinedUp();

            if (!hubActive) {
                wastedShotsInactiveHub++;
                activeCoachingTip = "[WASTED SHOT] Hub is inactive! Hold fire and cycle inventory.";
            } else if (!headingLinedUp) {
                misalignedShots++;
                activeCoachingTip = "[ALIGNMENT] Heading error! Hold Right Trigger for Auto-Aim lock.";
            } else if (flywheelsAtSpeed) {
                goodShotsOnTarget++;
                activeCoachingTip = "[GOOD SHOT] Locked heading & full target RPM!";
            }

            // If a cycle was in progress, complete it
            if (currentCycleStartTime > 0.0) {
                double cycleDuration = now - currentCycleStartTime;
                if (cycleDuration > 1.0) {
                    completedCyclesCount++;
                    totalCycleDurationSec += cycleDuration;
                    lastCycleDurationSec = cycleDuration;
                    if (cycleDuration < fastestCycleSec) {
                        fastestCycleSec = cycleDuration;
                    }
                }
                currentCycleStartTime = -1.0;
            }
        }
        wasShooting = isShooting;

        // 5. Track Opponent Pinning Duration
        double pinDuration = LegalPinningWatchdog.getInstance().getPinDuration();
        boolean warningActive = LegalPinningWatchdog.getInstance().isWarningActive();
        if (warningActive && !wasPinned) {
            pinWarningsCount++;
            activeCoachingTip = String.format("[PIN WARNING] Contact duration %.1fs! Pirouette or back off to avoid foul.", pinDuration);
            wasPinned = true;
        } else if (!warningActive) {
            wasPinned = false;
        }

        // 6. Track Trench Navigation
        Pose2d robotPose = swerve.getPose();
        boolean inTrench = Intake.isPoseInTrenchLowClearanceZone(robotPose);
        if (inTrench && !wasInTrench) {
            trenchPassagesCount++;
        }
        wasInTrench = inTrench;

        // 7. Update Real-Time Coaching Tip if no urgent event is overriding
        updateTacticalCoachingTip(robotPose, now);

        // 8. Compute Dynamic Driver Grade
        computeDriverGrade();
    }

    private void updateTacticalCoachingTip(Pose2d robotPose, double now) {
        double timeUntilSwitch = dashboard.getTimeUntilSwitch();
        if (timeUntilSwitch <= 3.5 && timeUntilSwitch > 0.1) {
            activeCoachingTip = String.format("[HUB SHIFT] Hub shifting in %.1fs! Disengage lane and rotate to Depot.", timeUntilSwitch);
            return;
        }

        try {
            int heldFuelEstimate = intake.hasFuel() ? 6 : 0;
            WorldState world = WorldStateBuilder.buildForPlayerRobot(heldFuelEstimate);
            AIActionIntent intent = JevDecisionEngine.getInstance().evaluatePolicy(world, Archetype.CO_PILOT);

            switch (intent.objective()) {
                case RUSH_CLIMB:
                    activeCoachingTip = "[ENDGAME] Match ending! Hold Right Bumper to Glide to Alliance Parking now!";
                    break;
                case CYCLE_SCORE_HUB:
                    activeCoachingTip = "[SCORING READY] " + intent.rationale() + " Hold Right Bumper to Glide to Hub.";
                    break;
                case STAGE_STANDOFF:
                    activeCoachingTip = "[STAGE STANDOFF] " + intent.rationale() + " Wait at standoff arc.";
                    break;
                case STOCKPILE_DEPOT:
                    activeCoachingTip = "[RELOAD] " + intent.rationale() + " Restock inventory at Alliance Depot.";
                    break;
                case VACUUM_MIDFIELD:
                default:
                    activeCoachingTip = "[HUNTING] " + intent.rationale() + " Tap Left Bumper for Auto Ball Hunt.";
                    break;
            }
        } catch (Exception e) {
            boolean hubActive = dashboard.isHubActive();
            boolean hasFuel = intake.hasFuel();
            if (hasFuel && hubActive) {
                activeCoachingTip = "[SCORING READY] Fuel loaded & Hub active! Hold Right Bumper to Glide to Hub.";
            } else if (!hasFuel && hubActive && !wasShooting) {
                activeCoachingTip = "[HUNTING] Empty hopper! Tap Left Bumper for Auto Ball Hunt.";
            } else if (!hubActive && !hasFuel) {
                activeCoachingTip = "[RELOAD] Hub inactive. Restock inventory at Alliance Depot loading station.";
            }
        }
    }

    private void computeDriverGrade() {
        double accuracy = getShootingAccuracyPercent();
        double avgCycle = getAverageCycleTimeSec();

        if (totalShotsAttempted == 0 && completedCyclesCount == 0) {
            driverGrade = "A";
            return;
        }

        if (accuracy >= 85.0 && (avgCycle <= 11.0 || completedCyclesCount == 0) && wastedShotsInactiveHub == 0) {
            driverGrade = "A+";
        } else if (accuracy >= 75.0 && (avgCycle <= 14.0 || completedCyclesCount == 0)) {
            driverGrade = "A";
        } else if (accuracy >= 60.0 && (avgCycle <= 18.0 || completedCyclesCount == 0)) {
            driverGrade = "B";
        } else if (accuracy >= 45.0) {
            driverGrade = "C";
        } else {
            driverGrade = "D";
        }
    }

    /**
     * Executes a complete arena & practice proving ground reset:
     * - Resets coaching performance statistics
     * - Respawns all field game pieces in GameSim
     * - Teleports robot to alliance starting zone
     * - Re-initializes opponent AI based on selected drill mode
     */
    public void executePracticeReset() {
        resetSessionStats();

        // 1. Reset Game Simulation & Respawn Field Fuel
        SmartDashboard.putBoolean("Simulation/Reset", true);
        SmartDashboard.putBoolean("Simulation/RespawnBalls", true);

        // 2. Reset Robot Pose to Blue / Red starting line
        boolean isRed = AllianceFlipUtil.isRedAlliance();
        Pose2d startPose = isRed
                ? new Pose2d(14.50, 4.035, Rotation2d.fromDegrees(180))
                : new Pose2d(2.00, 4.035, Rotation2d.fromDegrees(0));
        swerve.resetOdometry(startPose);

        // 3. Reset Opponent AI to opposite side of field and configure for Drill
        AIRobotSim.getInstance().reset();
        configureDrillEnvironment(currentDrill);
    }

    private void configureDrillEnvironment(DrillMode drill) {
        if (!RobotBase.isSimulation()) return;

        switch (drill) {
            case RAPID_CYCLING:
                // Disable opponent to allow pure time-trial cycling
                SmartDashboard.putBoolean("Features/Opponent Robot", false);
                SmartDashboard.putBoolean("Simulation/RespawnBalls", true);
                break;

            case TRENCH_DEFENSE:
                // Set opponent to tactical defense patrol
                SmartDashboard.putBoolean("Features/Opponent Robot", true);
                SmartDashboard.putString("Simulation/AIMode", AIMode.TACTICAL_DEFENSE.name());
                SmartDashboard.putNumber("Simulation/OpponentSpeedPercent", 80.0);
                break;

            case ANTI_DEFENSE_SHOOTING:
                // Set opponent to aggressive lead-pursuit interceptor
                SmartDashboard.putBoolean("Features/Opponent Robot", true);
                SmartDashboard.putString("Simulation/AIMode", AIMode.LEAD_PURSUIT_INTERCEPT.name());
                SmartDashboard.putNumber("Simulation/OpponentSpeedPercent", 85.0);
                break;

            case FREE_PLAY:
            default:
                // Standard match competitor cycling
                SmartDashboard.putBoolean("Features/Opponent Robot", true);
                SmartDashboard.putString("Simulation/AIMode", AIMode.AUTONOMOUS_CYCLER.name());
                SmartDashboard.putNumber("Simulation/OpponentSpeedPercent", 75.0);
                break;
        }
    }

    // Metric Getters
    public double getShootingAccuracyPercent() {
        if (totalShotsAttempted == 0) return 100.0;
        return (goodShotsOnTarget * 100.0) / totalShotsAttempted;
    }

    public double getAverageCycleTimeSec() {
        if (completedCyclesCount == 0) return 0.0;
        return totalCycleDurationSec / completedCyclesCount;
    }

    public double getLastCycleDurationSec() {
        return lastCycleDurationSec;
    }

    public double getFastestCycleSec() {
        return fastestCycleSec == Double.MAX_VALUE ? 0.0 : fastestCycleSec;
    }

    public int getCompletedCyclesCount() {
        return completedCyclesCount;
    }

    public int getTotalShotsAttempted() {
        return totalShotsAttempted;
    }

    public int getGoodShotsOnTarget() {
        return goodShotsOnTarget;
    }

    public int getWastedShotsInactiveHub() {
        return wastedShotsInactiveHub;
    }

    public int getMisalignedShots() {
        return misalignedShots;
    }

    public int getPinWarningsCount() {
        return pinWarningsCount;
    }

    public String getDriverGrade() {
        return driverGrade;
    }

    public String getActiveCoachingTip() {
        return activeCoachingTip;
    }

    public DrillMode getCurrentDrill() {
        return currentDrill;
    }

    @Override
    public void log() {
        SmartDashboard.putString("Coaching/Recommendation", activeCoachingTip);
        SmartDashboard.putString("Coaching/DriverGrade", driverGrade);
        SmartDashboard.putNumber("Coaching/ShootingAccuracyPercent", getShootingAccuracyPercent());
        SmartDashboard.putNumber("Coaching/AverageCycleTimeSec", getAverageCycleTimeSec());
        SmartDashboard.putNumber("Coaching/LastCycleTimeSec", lastCycleDurationSec);
        SmartDashboard.putNumber("Coaching/FastestCycleTimeSec", getFastestCycleSec());
        SmartDashboard.putNumber("Coaching/CompletedCyclesCount", completedCyclesCount);
        SmartDashboard.putNumber("Coaching/TotalShotsAttempted", totalShotsAttempted);
        SmartDashboard.putNumber("Coaching/GoodShotsOnTarget", goodShotsOnTarget);
        SmartDashboard.putNumber("Coaching/WastedShotsInactiveHub", wastedShotsInactiveHub);
        SmartDashboard.putNumber("Coaching/MisalignedShots", misalignedShots);
        SmartDashboard.putNumber("Coaching/PinWarningsCount", pinWarningsCount);
        SmartDashboard.putString("Coaching/DrillMode", currentDrill.displayName);

        Logger.recordOutput("Coaching/DriverGrade", driverGrade);
        Logger.recordOutput("Coaching/AccuracyPercent", getShootingAccuracyPercent());
        Logger.recordOutput("Coaching/AvgCycleTime", getAverageCycleTimeSec());
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "MatchCoach";
    }
}
