package frc.robot.Sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.FieldMap;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Utils.Alert;
import frc.robot.Utils.Alert.AlertType;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * RefereeSim: Real-Time FRC Autonomous & Rules Referee for Simulation.
 *
 * Enforces 2026 Rebuilt rules and tallies penalties into {@link MatchScoreTracker}
 * (MINOR FOUL = 5 pts, MAJOR FOUL = 15 pts to the opponent's total):
 * - AUTO Centerline Contact: in AUTO, a ROBOT whose BUMPERS are completely across
 *   the CENTER LINE may not contact an opponent ROBOT. Violation: MAJOR FOUL.
 * - G407 Alliance-Zone Shooting: a ROBOT may not launch a SCORING ELEMENT into
 *   their HUB unless its BUMPERS are partially/fully within its ALLIANCE ZONE.
 *   Checked at every simulated launch site via {@link #checkShotLegality}.
 *   Violation: MAJOR FOUL.
 * - G418 Pinning: a ROBOT may not PIN an opponent for more than 3 seconds.
 *   Violation: MINOR FOUL, plus a MAJOR FOUL for every additional 3 seconds
 *   the situation is not corrected.
 * - G420 Tower Protection: during the last 30 s, a ROBOT may not contact an
 *   opponent ROBOT that is in contact with its own TOWER. Violation: MAJOR FOUL.
 */
public class RefereeSim implements Subsystem {

    public static final double PIN_MAX_TIME_SEC = 3.00;
    public static final double PIN_ESCALATION_INTERVAL_SEC = 3.00;
    public static final double PIN_SEPARATION_METERS = 1.83; // 72 in: PIN count ends
    public static final double CONTACT_DIST_METERS = 1.10;
    public static final double CENTERLINE_ACROSS_MARGIN_METERS = 0.50; // bumpers fully across
    public static final double TOWER_PROXIMITY_METERS = 1.50;
    public static final double FOUL_DEBOUNCE_SEC = 5.00;

    private static RefereeSim instance;

    public static synchronized RefereeSim getInstance() {
        if (instance == null) {
            instance = new RefereeSim();
        }
        return instance;
    }

    private final Alert refereeAlert = new Alert("Referee Infraction", "No active infractions", AlertType.WARNING);

    // Pinning state tracking (player vs Bot 0 pair)
    private double playerOpponentPinTime = 0.0;
    private int pinViolationCount = 0;
    private boolean playerPinningActive = false;
    private boolean opponentPinningActive = false;

    // Per-violator debounce so one sustained infraction isn't re-flagged every tick
    private final java.util.Map<String, Double> lastFoulTimeByViolator = new java.util.HashMap<>();

    private RefereeSim() {
        SubsystemManager.registerSubsystem(this);
    }

    @Override
    public boolean isEnabled() {
        return RobotBase.isSimulation();
    }

    @Override
    public void initialize() {
        reset();
    }

    public synchronized void reset() {
        playerOpponentPinTime = 0.0;
        pinViolationCount = 0;
        playerPinningActive = false;
        opponentPinningActive = false;
        lastFoulTimeByViolator.clear();
        refereeAlert.set(false);
    }

    @Override
    public void simulationUpdate() {
        double now = Timer.getFPGATimestamp();

        // ── 1. AUTO Centerline Contact ─────────────────────────────────────
        if (DriverStation.isAutonomous()) {
            evaluateAutonomousContact(now);
        }

        // ── 2. G418 Pinning Rule Evaluation ────────────────────────────────
        evaluatePinningRule(now);

        // ── 3. G420 Tower Protection (last 30 s) ───────────────────────────
        evaluateTowerProtection(now);

        // ── 4. Publish Telemetry ───────────────────────────────────────────
        publishTelemetry();
    }

    /** Lightweight robot snapshot used for cross-alliance pair checks. */
    private static final class RobotState {
        final String label;
        final Pose2d pose;
        final boolean isRed;

        RobotState(String label, Pose2d pose, boolean isRed) {
            this.label = label;
            this.pose = pose;
            this.isRed = isRed;
        }
    }

    /**
     * Collects all active robot poses, tagged by alliance. Allies skate with
     * the player; Bot 0 and additional bots skate against the player.
     */
    private List<RobotState> collectRobotStates(boolean playerIsRed) {
        List<RobotState> states = new ArrayList<>();
        Pose2d playerPose = SwerveBase.getInstance().getPose();
        if (playerPose != null && playerPose.getY() > 0.0) {
            states.add(new RobotState("Player", playerPose, playerIsRed));
        }
        try {
            AIRobotSim sim = AIRobotSim.getInstance();
            if (sim != null) {
                if (sim.getDriveSimulation() != null) {
                    Pose2d bot0 = sim.getDriveSimulation().getActualPoseInSimulationWorld();
                    if (bot0 != null && bot0.getY() > 0.0) {
                        states.add(new RobotState("Bot 0", bot0, !playerIsRed));
                    }
                }
                for (AIRobotInstance bot : sim.getAdditionalBots()) {
                    Pose2d p = bot.getActualPose();
                    if (p != null && p.getY() > 0.0) {
                        states.add(new RobotState("Opponent Bot " + bot.getBotId(), p, !playerIsRed));
                    }
                }
                for (AIRobotInstance ally : sim.getAllyBots()) {
                    Pose2d p = ally.getActualPose();
                    if (p != null && p.getY() > 0.0) {
                        states.add(new RobotState("Ally " + (ally.getBotId() - 100), p, playerIsRed));
                    }
                }
            }
        } catch (Exception ignored) {}
        return states;
    }

    /** True when the robot's BUMPERS are completely past the CENTER LINE. */
    private boolean isFullyAcrossCenterline(Pose2d pose, boolean isRed) {
        if (pose == null) return false;
        double centerX = FieldMap.CENTERLINE_X;
        if (isRed) {
            return pose.getX() < (centerX - CENTERLINE_ACROSS_MARGIN_METERS);
        } else {
            return pose.getX() > (centerX + CENTERLINE_ACROSS_MARGIN_METERS);
        }
    }

    private boolean debounced(String violatorKey, double now) {
        Double last = lastFoulTimeByViolator.get(violatorKey);
        if (last != null && (now - last) < FOUL_DEBOUNCE_SEC) {
            return true;
        }
        lastFoulTimeByViolator.put(violatorKey, now);
        return false;
    }

    /**
     * AUTO Centerline Contact: in AUTO, a ROBOT fully across the CENTER LINE
     * may not contact an opponent ROBOT. Violation: MAJOR FOUL (15 pts).
     */
    public synchronized void evaluateAutonomousContact(double now) {
        boolean playerIsRed = AllianceFlipUtil.isRedAlliance();
        List<RobotState> states = collectRobotStates(playerIsRed);

        for (int i = 0; i < states.size(); i++) {
            for (int j = i + 1; j < states.size(); j++) {
                RobotState a = states.get(i);
                RobotState b = states.get(j);
                if (a.isRed == b.isRed) continue;
                double dist = a.pose.getTranslation().getDistance(b.pose.getTranslation());
                if (dist >= CONTACT_DIST_METERS) continue;

                RobotState violator = null;
                if (isFullyAcrossCenterline(a.pose, a.isRed)) {
                    violator = a;
                } else if (isFullyAcrossCenterline(b.pose, b.isRed)) {
                    violator = b;
                }
                if (violator == null) continue;

                String key = "AUTO_CONTACT_" + violator.label;
                if (debounced(key, now)) continue;
                MatchScoreTracker.getInstance().recordMajorFoul(violator.isRed,
                        "AUTO Centerline Contact by " + violator.label);
                triggerFoulAlert("[AUTO CONTACT] " + violator.label + " across centerline in contact (+15 pts)");
            }
        }
    }

    /**
     * Backwards-compatible alias for {@link #evaluateAutonomousContact(double)}.
     */
    public synchronized void evaluateAutonomousBoundaries(double now) {
        evaluateAutonomousContact(now);
    }

    /**
     * G418 Pinning: a ROBOT may not PIN an opponent for more than 3 seconds.
     * First violation is a MINOR FOUL (5 pts); every additional 3 seconds the
     * situation is not corrected draws a MAJOR FOUL (15 pts). Separation of
     * >= 1.83 m ends the PIN count.
     */
    public synchronized void evaluatePinningRule(double now) {
        Pose2d playerPose = SwerveBase.getInstance().getPose();
        Pose2d opponentPose = null;
        try {
            AIRobotSim sim = AIRobotSim.getInstance();
            if (sim != null && sim.getDriveSimulation() != null) {
                opponentPose = sim.getDriveSimulation().getActualPoseInSimulationWorld();
            }
        } catch (Exception ignored) {}

        if (playerPose == null || opponentPose == null) return;

        double distance = playerPose.getTranslation().getDistance(opponentPose.getTranslation());
        boolean isCloseContact = distance < CONTACT_DIST_METERS;

        if (isCloseContact) {
            playerOpponentPinTime += 0.02;

            if (playerOpponentPinTime >= PIN_MAX_TIME_SEC) {
                int deserved = 1 + (int) ((playerOpponentPinTime - PIN_MAX_TIME_SEC) / PIN_ESCALATION_INTERVAL_SEC);
                while (pinViolationCount < deserved) {
                    pinViolationCount++;
                    boolean isMajor = pinViolationCount > 1;
                    boolean playerIsRed = AllianceFlipUtil.isRedAlliance();
                    // Determine aggressor by speed vector directed toward opponent
                    var playerSpeeds = SwerveBase.getInstance().getFieldVelocity();
                    double pDot = (opponentPose.getX() - playerPose.getX()) * playerSpeeds.vxMetersPerSecond
                            + (opponentPose.getY() - playerPose.getY()) * playerSpeeds.vyMetersPerSecond;

                    boolean playerIsAggressor = pDot > 0.15;
                    boolean foulOnRed = playerIsAggressor ? playerIsRed : !playerIsRed;

                    String violator = playerIsAggressor ? "Player" : "Opponent Bot";
                    if (isMajor) {
                        MatchScoreTracker.getInstance().recordMajorFoul(foulOnRed,
                                "G418 Uncorrected Pin by " + violator + " (+" + PIN_ESCALATION_INTERVAL_SEC + "s)");
                        triggerFoulAlert("[G418 PIN] " + violator + " pin uncorrected (+15 pts MAJOR)");
                    } else {
                        MatchScoreTracker.getInstance().recordMinorFoul(foulOnRed,
                                "G418 Pinning Violation by " + violator);
                        triggerFoulAlert("[G418 PIN] " + violator + " exceeded 3s pin limit (+5 pts MINOR)");
                    }
                }
            }
        } else if (distance >= PIN_SEPARATION_METERS) {
            // PIN count ends once separated by 72 in
            playerOpponentPinTime = 0.0;
            pinViolationCount = 0;
        } else {
            // Closing back in: hold the count without accruing
            playerOpponentPinTime = Math.max(0.0, playerOpponentPinTime - 0.04);
        }
    }

    /**
     * G407 Alliance-Zone Shooting: records a MAJOR FOUL when a ROBOT launches
     * a SCORING ELEMENT while outside its own ALLIANCE ZONE. Called from every
     * simulated launch site (player + AI bots). The AI's targeting guards
     * already confine it to legal zones, so AI fouls here should be rare.
     *
     * @param shooterPose Launch pose of the shooting robot
     * @param shooterIsRedAlliance Alliance of the shooting robot
     * @param shooterLabel Human-readable robot label for the foul report
     */
    public static void checkShotLegality(Pose2d shooterPose, boolean shooterIsRedAlliance, String shooterLabel) {
        if (shooterPose == null) return;
        if (!RobotBase.isSimulation()) return;
        if (!FieldMap.AllianceZones.isInAllianceZone(shooterPose, shooterIsRedAlliance)) {
            MatchScoreTracker.getInstance().recordMajorFoul(shooterIsRedAlliance,
                    "G407 Shot Outside Alliance Zone by " + shooterLabel);
        }
    }

    /**
     * G420 Tower Protection: during the last 30 s, a ROBOT may not contact an
     * opponent ROBOT that is in contact with its own TOWER.
     * Violation: MAJOR FOUL (15 pts). Simulated robots never leave the ground,
     * so no LEVEL 3 TOWER points are awarded here.
     */
    public synchronized void evaluateTowerProtection(double now) {
        if (DriverStation.isAutonomous()) return;
        double remaining = getMatchTimeRemainingSec();
        if (remaining > 30.0 || remaining <= 0.0) return;

        boolean playerIsRed = AllianceFlipUtil.isRedAlliance();
        List<RobotState> states = collectRobotStates(playerIsRed);

        for (int i = 0; i < states.size(); i++) {
            for (int j = i + 1; j < states.size(); j++) {
                RobotState a = states.get(i);
                RobotState b = states.get(j);
                if (a.isRed == b.isRed) continue;
                double dist = a.pose.getTranslation().getDistance(b.pose.getTranslation());
                if (dist >= CONTACT_DIST_METERS) continue;

                RobotState protectedBot = null;
                if (isNearOwnTower(a)) {
                    protectedBot = a;
                } else if (isNearOwnTower(b)) {
                    protectedBot = b;
                }
                if (protectedBot == null) continue;
                RobotState violator = (protectedBot == a) ? b : a;

                String key = "G420_" + violator.label;
                if (debounced(key, now)) continue;
                MatchScoreTracker.getInstance().recordMajorFoul(violator.isRed,
                        "G420 Tower Protection Contact by " + violator.label
                                + " on " + protectedBot.label);
                triggerFoulAlert("[G420] " + violator.label + " contacted tower-side "
                        + protectedBot.label + " (+15 pts MAJOR)");
            }
        }
    }

    private boolean isNearOwnTower(RobotState robot) {
        Translation2d tower = FieldMap.ClimbingTowers.getTowerPole(robot.isRed);
        return robot.pose.getTranslation().getDistance(tower) <= TOWER_PROXIMITY_METERS;
    }

    private double getMatchTimeRemainingSec() {
        double matchTime = Timer.getMatchTime();
        if (matchTime < 0.0) {
            try {
                matchTime = GameSim.getInstance().getSimTimeRemainingSec();
            } catch (Exception ignored) {}
        }
        return matchTime;
    }

    private void triggerFoulAlert(String message) {
        refereeAlert.setText(message);
        refereeAlert.set(true);
    }

    public synchronized double getPinTimer() {
        return playerOpponentPinTime;
    }

    public synchronized void setSimPinTimer(double seconds) {
        this.playerOpponentPinTime = seconds;
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber("Scoreboard/Referee/ActivePinTimerSec", playerOpponentPinTime);
        SmartDashboard.putBoolean("Scoreboard/Referee/PinWarningActive", playerOpponentPinTime >= 1.8);
        Logger.recordOutput("Scoreboard/Referee/ActivePinTimerSec", playerOpponentPinTime);
    }

    @Override
    public void update() {}

    @Override
    public void log() {}

    @Override
    public String getName() {
        return "RefereeSim";
    }
}
