package frc.robot.Telemetry;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * TelemetryKeys: Single source of truth for robot NetworkTables topics
 * backing driver dashboard tabs, Elastic widgets, Co-Pilot telemetry,
 * match coaching, diagnostics, and subsystem controls.
 *
 * <p>Complementary to {@link frc.robot.Sim.SimDashboardKeys} (which defines
 * {@code Simulation/*} topics for sim sparring bots), this class defines
 * all production robot telemetry topics.
 */
public final class TelemetryKeys {

    private TelemetryKeys() {
    }

    // =========================================================================
    // 1. FEATURE TOGGLES (Features/*)
    // =========================================================================
    public static final class Features {
        public static final String SNAP_TO_TURN = "Features/Snap to Turn";
        public static final String BALL_HUNT = "Features/Ball Hunt";
        public static final String GLIDE_POINTS = "Features/Glide Points";
        public static final String FIELD_ORIENTED = "Features/Field Oriented";
        public static final String SLOW_MODE = "Features/Slow Mode";
        public static final String AUTO_AIM = "Features/Auto Aim";
        public static final String OPPONENT_ROBOT = "Features/Opponent Robot";
        public static final String ALLY_BOTS = "Features/Ally Bots";
        public static final String TWO_PLAYER_DEFENSE = "Features/2 Player Defense";
        public static final String PIT_MODE = "Features/Pit Mode";
        public static final String USE_TYPESAFE_JEV = "Features/Use TypeSafe Jev AI";

        private Features() {}
    }

    // =========================================================================
    // 2. DRIVER & OPERATOR (Driver/*, Operator/*)
    // =========================================================================
    public static final class Driver {
        public static final String HUB_ACTIVE = "Driver/Hub Active";
        public static final String HUB_STATUS = "Driver/Hub Status";
        public static final String HUB_SHIFT_TIME_REMAINING = "Driver/Hub Shift Time Remaining";
        public static final String HUB_SHIFT_PROGRESS = "Driver/Hub Shift Progress";
        public static final String SHOOT_ALERT = "Driver/Shoot Alert";
        public static final String SHOOT_MESSAGE = "Driver/Shoot Message";
        public static final String SHOOTER_READY = "Driver/Shooter Ready";
        public static final String FLYWHEEL_ACTUAL_RPM = "Driver/Flywheel Actual RPM";
        public static final String FLYWHEEL_TARGET_RPM = "Driver/Flywheel Target RPM";
        public static final String INTAKE_STATE = "Driver/Intake State";
        public static final String NEAREST_GLIDE = "Driver/Nearest Glide";
        public static final String ALERT_BANNER = "Driver/AlertBanner";
        public static final String SNAP_ACTIVE = "Driver/Snap Active";
        public static final String SNAP_TARGET_ANGLE = "Driver/Snap Target Angle";
        public static final String LINED_UP = "Driver/Lined Up";

        private Driver() {}
    }

    public static final class Operator {
        public static final String HAPTIC_COLLISION_ENABLED = "Operator/HapticCollisionEnabled";

        private Operator() {}
    }

    // =========================================================================
    // 3. CO-PILOT ASSIST (CoPilot/*)
    // =========================================================================
    public static final class CoPilot {
        public static final String ASSIST_ACTIVE = "CoPilot/AssistActive";
        public static final String OBJECTIVE = "CoPilot/Objective";
        public static final String CURRENT_OBJECTIVE = "CoPilot/CurrentObjective";
        public static final String NEXT_OBJECTIVE = "CoPilot/NextObjective";
        public static final String TIME_TO_TRANSITION_SEC = "CoPilot/TimeToTransitionSec";
        public static final String AUTO_FEED_ACTIVE = "CoPilot/AutoFeedActive";

        private CoPilot() {}
    }

    // =========================================================================
    // 4. BUILD & RUNTIME METADATA (Build/*)
    // =========================================================================
    public static final class Build {
        public static final String ROBOT_NAME = "Build/RobotName";
        public static final String GIT_SHA = "Build/GitSHA";
        public static final String GIT_BRANCH = "Build/GitBranch";
        public static final String GIT_DATE = "Build/GitDate";
        public static final String COMPILE_DATE = "Build/CompileDate";
        public static final String IS_DIRTY = "Build/IsDirty";
        public static final String SUMMARY = "Build/Summary";

        private Build() {}
    }

    // =========================================================================
    // 5. AUTONOMOUS SELECTION & STATUS
    // =========================================================================
    public static final class Auto {
        public static final String DELAY_SECONDS = "Auto Delay (seconds)";
        public static final String MISSION = "Auto Mission";
        public static final String MISSION_SELECTED = "AutoMissionSelected";
        public static final String CURRENT_ACTION_SYSTEM = "Current Action System";

        private Auto() {}
    }

    // =========================================================================
    // 6. MATCH TIMING & STATUS (Match/*)
    // =========================================================================
    public static final class Match {
        public static final String TIME_REMAINING_SEC = "Match/TimeRemainingSec";
        public static final String ALLIANCE = "Match/Alliance";
        public static final String PHASE = "Match/Phase";

        private Match() {}
    }

    // =========================================================================
    // 7. MATCH COACHING (Coaching/*)
    // =========================================================================
    public static final class Coaching {
        public static final String DRILL_MODE_CHOOSER = "Coaching/DrillModeChooser";
        public static final String RESET_PRACTICE = "Coaching/ResetPractice";
        public static final String RECOMMENDATION = "Coaching/Recommendation";
        public static final String DRIVER_GRADE = "Coaching/DriverGrade";
        public static final String SHOOTING_ACCURACY_PERCENT = "Coaching/ShootingAccuracyPercent";
        public static final String AVERAGE_CYCLE_TIME_SEC = "Coaching/AverageCycleTimeSec";
        public static final String LAST_CYCLE_TIME_SEC = "Coaching/LastCycleTimeSec";
        public static final String FASTEST_CYCLE_TIME_SEC = "Coaching/FastestCycleTimeSec";
        public static final String COMPLETED_CYCLES_COUNT = "Coaching/CompletedCyclesCount";
        public static final String TOTAL_SHOTS_ATTEMPTED = "Coaching/TotalShotsAttempted";
        public static final String GOOD_SHOTS_ON_TARGET = "Coaching/GoodShotsOnTarget";
        public static final String WASTED_SHOTS_INACTIVE_HUB = "Coaching/WastedShotsInactiveHub";
        public static final String MISALIGNED_SHOTS = "Coaching/MisalignedShots";
        public static final String PIN_WARNINGS_COUNT = "Coaching/PinWarningsCount";
        public static final String DRILL_MODE = "Coaching/DrillMode";

        private Coaching() {}
    }

    // =========================================================================
    // 8. JEV AI & STRATEGY (JevAI/*, Strategy/*)
    // =========================================================================
    public static final class JevAI {
        public static final String DECISION_MODE = "JevAI/DecisionMode";
        public static final String SELECTED_ACTION = "JevAI/SelectedAction";
        public static final String CONFIDENCE = "JevAI/Confidence";
        public static final String LATENCY_MS = "JevAI/LatencyMs";
        public static final String RATIONALE = "JevAI/Rationale";
        public static final String TARGET_POSE = "JevAI/TargetPose";
        public static final String TELEMETRY_JSON = "JevAI/TelemetryJson";
        public static final String GLIDE_ARBITRATION_MODE = "JevAI/GlideArbitrationMode";
        public static final String GLIDE_TARGET = "JevAI/GlideTarget";
        public static final String TRENCH_CORRIDOR_SELECTED = "JevAI/TrenchCorridorSelected";

        private JevAI() {}
    }

    public static final class Strategy {
        public static final String RECOMMENDATION = "Strategy/Recommendation";
        public static final String ADVICE_TEXT = "Strategy/AdviceText";
        public static final String UTILITY = "Strategy/Utility";

        private Strategy() {}
    }

    // =========================================================================
    // 9. SUB-SYSTEMS & HARDWARE (AutoAim/*, Shooter/*, Power/*, PinWatchdog/*)
    // =========================================================================
    public static final class AutoAim {
        public static final String POSSIBLE = "AutoAim/Possible";
        public static final String TARGET_RPM = "AutoAim/TargetRPM";
        public static final String TARGET_YAW = "AutoAim/TargetYaw";
        public static final String HEADING_ERROR = "AutoAim/HeadingError";
        public static final String FLYWHEEL_ERROR = "AutoAim/FlywheelError";
        public static final String STATUS = "AutoAim/Status";

        private AutoAim() {}
    }

    public static final class Shooter {
        public static final String PRE_SPOOLING_ACTIVE = "Shooter/PreSpoolingActive";

        private Shooter() {}
    }

    public static final class Power {
        public static final String DRIVE_SPEED_SCALE = "Power/DriveSpeedScale";
        public static final String BREAKER_DAMAGE_FRACTION = "Power/BreakerDamageFraction";
        public static final String IS_BREAKER_TRIPPED = "Power/IsBreakerTripped";

        private Power() {}
    }

    public static final class PinWatchdog {
        public static final String IS_WARNING = "PinWatchdog/IsWarning";
        public static final String FORCED_BACKOFF = "PinWatchdog/ForcedBackoff";
        public static final String PIN_DURATION_SEC = "PinWatchdog/PinDurationSec";
        public static final String BACKOFF_REMAINING_SEC = "PinWatchdog/BackoffRemainingSec";

        private PinWatchdog() {}
    }

    public static final class SmartTunnel {
        public static final String CLEARANCE_SAFE = "SmartTunnel/ClearanceSafe";

        private SmartTunnel() {}
    }

    public static final class Diagnostics {
        public static final String SCORECARD_OVERALL = "Diagnostics/Scorecard/Overall";
        public static final String RUN_PREFLIGHT = "Diagnostics/Run Pre-Flight Check";
        public static final String PREFLIGHT_STEP = "Diagnostics/PreFlight/Step";
        public static final String PREFLIGHT_PROGRESS = "Diagnostics/PreFlight/Progress";
        public static final String SCORECARD_CAN_BUS = "Diagnostics/Scorecard/CAN_Bus";
        public static final String SCORECARD_SWERVE_DRIVE = "Diagnostics/Scorecard/Swerve_Drive";
        public static final String SCORECARD_STEER_ALIGNMENT = "Diagnostics/Scorecard/Steer_Alignment";
        public static final String SCORECARD_INTAKE = "Diagnostics/Scorecard/Intake";
        public static final String SCORECARD_SHOOTER = "Diagnostics/Scorecard/Shooter";
        public static final String SCORECARD_VISION = "Diagnostics/Scorecard/Vision";

        private Diagnostics() {}
    }

    // =========================================================================
    // 10. SCOREBOARD (Scoreboard/*)
    // =========================================================================
    public static final class Scoreboard {
        public static final String RED_TOTAL_SCORE = "Scoreboard/Red/TotalScore";
        public static final String BLUE_TOTAL_SCORE = "Scoreboard/Blue/TotalScore";
        public static final String MATCH_PHASE = "Scoreboard/Match/Phase";
        public static final String SHIFT_SEED = "Scoreboard/Match/ShiftSeed";
        public static final String RED_FUEL_TOTAL = "Scoreboard/Red/FuelTotal";
        public static final String BLUE_FUEL_TOTAL = "Scoreboard/Blue/FuelTotal";
        public static final String RED_CLIMB_SCORE = "Scoreboard/Red/ClimbScore";
        public static final String BLUE_CLIMB_SCORE = "Scoreboard/Blue/ClimbScore";

        private Scoreboard() {}
    }

    // =========================================================================
    // REGISTRY & VALIDATION
    // =========================================================================
    private static final Set<String> ALL_KEYS;

    static {
        Set<String> keys = new HashSet<>();
        // Features
        keys.add(Features.SNAP_TO_TURN);
        keys.add(Features.BALL_HUNT);
        keys.add(Features.GLIDE_POINTS);
        keys.add(Features.FIELD_ORIENTED);
        keys.add(Features.SLOW_MODE);
        keys.add(Features.AUTO_AIM);
        keys.add(Features.OPPONENT_ROBOT);
        keys.add(Features.ALLY_BOTS);
        keys.add(Features.TWO_PLAYER_DEFENSE);
        keys.add(Features.PIT_MODE);
        keys.add(Features.USE_TYPESAFE_JEV);

        // Driver / Operator
        keys.add(Driver.HUB_ACTIVE);
        keys.add(Driver.HUB_STATUS);
        keys.add(Driver.HUB_SHIFT_TIME_REMAINING);
        keys.add(Driver.HUB_SHIFT_PROGRESS);
        keys.add(Driver.SHOOT_ALERT);
        keys.add(Driver.SHOOT_MESSAGE);
        keys.add(Driver.SHOOTER_READY);
        keys.add(Driver.FLYWHEEL_ACTUAL_RPM);
        keys.add(Driver.FLYWHEEL_TARGET_RPM);
        keys.add(Driver.INTAKE_STATE);
        keys.add(Driver.NEAREST_GLIDE);
        keys.add(Driver.ALERT_BANNER);
        keys.add(Driver.SNAP_ACTIVE);
        keys.add(Driver.SNAP_TARGET_ANGLE);
        keys.add(Driver.LINED_UP);
        keys.add(Operator.HAPTIC_COLLISION_ENABLED);

        // CoPilot
        keys.add(CoPilot.ASSIST_ACTIVE);
        keys.add(CoPilot.OBJECTIVE);
        keys.add(CoPilot.CURRENT_OBJECTIVE);
        keys.add(CoPilot.NEXT_OBJECTIVE);
        keys.add(CoPilot.TIME_TO_TRANSITION_SEC);
        keys.add(CoPilot.AUTO_FEED_ACTIVE);

        // Build
        keys.add(Build.ROBOT_NAME);
        keys.add(Build.GIT_SHA);
        keys.add(Build.GIT_BRANCH);
        keys.add(Build.GIT_DATE);
        keys.add(Build.COMPILE_DATE);
        keys.add(Build.IS_DIRTY);
        keys.add(Build.SUMMARY);

        // Auto
        keys.add(Auto.DELAY_SECONDS);
        keys.add(Auto.MISSION);
        keys.add(Auto.MISSION_SELECTED);
        keys.add(Auto.CURRENT_ACTION_SYSTEM);

        // Match
        keys.add(Match.TIME_REMAINING_SEC);
        keys.add(Match.ALLIANCE);
        keys.add(Match.PHASE);

        // Coaching
        keys.add(Coaching.DRILL_MODE_CHOOSER);
        keys.add(Coaching.RESET_PRACTICE);
        keys.add(Coaching.RECOMMENDATION);
        keys.add(Coaching.DRIVER_GRADE);
        keys.add(Coaching.SHOOTING_ACCURACY_PERCENT);
        keys.add(Coaching.AVERAGE_CYCLE_TIME_SEC);
        keys.add(Coaching.LAST_CYCLE_TIME_SEC);
        keys.add(Coaching.FASTEST_CYCLE_TIME_SEC);
        keys.add(Coaching.COMPLETED_CYCLES_COUNT);
        keys.add(Coaching.TOTAL_SHOTS_ATTEMPTED);
        keys.add(Coaching.GOOD_SHOTS_ON_TARGET);
        keys.add(Coaching.WASTED_SHOTS_INACTIVE_HUB);
        keys.add(Coaching.MISALIGNED_SHOTS);
        keys.add(Coaching.PIN_WARNINGS_COUNT);
        keys.add(Coaching.DRILL_MODE);

        // JevAI & Strategy
        keys.add(JevAI.DECISION_MODE);
        keys.add(JevAI.SELECTED_ACTION);
        keys.add(JevAI.CONFIDENCE);
        keys.add(JevAI.LATENCY_MS);
        keys.add(JevAI.RATIONALE);
        keys.add(JevAI.TARGET_POSE);
        keys.add(JevAI.TELEMETRY_JSON);
        keys.add(JevAI.GLIDE_ARBITRATION_MODE);
        keys.add(JevAI.GLIDE_TARGET);
        keys.add(JevAI.TRENCH_CORRIDOR_SELECTED);
        keys.add(Strategy.RECOMMENDATION);
        keys.add(Strategy.ADVICE_TEXT);
        keys.add(Strategy.UTILITY);

        // Subsystems & Diagnostics
        keys.add(AutoAim.POSSIBLE);
        keys.add(AutoAim.TARGET_RPM);
        keys.add(AutoAim.TARGET_YAW);
        keys.add(AutoAim.HEADING_ERROR);
        keys.add(AutoAim.FLYWHEEL_ERROR);
        keys.add(AutoAim.STATUS);
        keys.add(Shooter.PRE_SPOOLING_ACTIVE);
        keys.add(Power.DRIVE_SPEED_SCALE);
        keys.add(Power.BREAKER_DAMAGE_FRACTION);
        keys.add(Power.IS_BREAKER_TRIPPED);
        keys.add(PinWatchdog.IS_WARNING);
        keys.add(PinWatchdog.FORCED_BACKOFF);
        keys.add(PinWatchdog.PIN_DURATION_SEC);
        keys.add(PinWatchdog.BACKOFF_REMAINING_SEC);
        keys.add(SmartTunnel.CLEARANCE_SAFE);
        keys.add(Diagnostics.SCORECARD_OVERALL);
        keys.add(Diagnostics.RUN_PREFLIGHT);
        keys.add(Diagnostics.PREFLIGHT_STEP);
        keys.add(Diagnostics.PREFLIGHT_PROGRESS);
        keys.add(Diagnostics.SCORECARD_CAN_BUS);
        keys.add(Diagnostics.SCORECARD_SWERVE_DRIVE);
        keys.add(Diagnostics.SCORECARD_STEER_ALIGNMENT);
        keys.add(Diagnostics.SCORECARD_INTAKE);
        keys.add(Diagnostics.SCORECARD_SHOOTER);
        keys.add(Diagnostics.SCORECARD_VISION);

        // Scoreboard
        keys.add(Scoreboard.RED_TOTAL_SCORE);
        keys.add(Scoreboard.BLUE_TOTAL_SCORE);
        keys.add(Scoreboard.MATCH_PHASE);
        keys.add(Scoreboard.SHIFT_SEED);
        keys.add(Scoreboard.RED_FUEL_TOTAL);
        keys.add(Scoreboard.BLUE_FUEL_TOTAL);
        keys.add(Scoreboard.RED_CLIMB_SCORE);
        keys.add(Scoreboard.BLUE_CLIMB_SCORE);

        ALL_KEYS = Collections.unmodifiableSet(keys);
    }

    /** Returns all registered production telemetry topics. */
    public static Set<String> allKeys() {
        return ALL_KEYS;
    }
}
