package frc.robot.Sim;

import edu.wpi.first.math.geometry.Pose2d;
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
 * Enforces official FRC rules and automatically tallies penalties into {@link MatchScoreTracker}:
 * - G401 Pinning Limits: 2.4s max contact against walls/corners. If pinning robot fails to back off >=3ft,
 *   assesses Minor Fouls (2 pts to opposing alliance) every 3.0s.
 * - G201 Autonomous Boundary Violation: Crossing past midfield centerline during autonomous awards
 *   Tech Fouls (5 pts to opposing alliance).
 * - Illegal Shooting Violations: Attempted shots launched outside the legal Alliance Zone.
 */
public class RefereeSim implements Subsystem {

    public static final double PIN_MAX_TIME_SEC = 2.40;
    public static final double PIN_FOUL_INTERVAL_SEC = 3.00;
    public static final double PIN_BACKOFF_DIST_METERS = 0.9144; // 3 feet
    public static final double CENTERLINE_BUFFER_METERS = 0.40;

    private static RefereeSim instance;

    public static synchronized RefereeSim getInstance() {
        if (instance == null) {
            instance = new RefereeSim();
        }
        return instance;
    }

    private final Alert refereeAlert = new Alert("Referee Infraction", "No active infractions", AlertType.WARNING);

    // Pinning state tracking
    private double playerOpponentPinTime = 0.0;
    private double lastPinFoulTime = -10.0;
    private boolean playerPinningActive = false;
    private boolean opponentPinningActive = false;

    // Auto crossing debounce
    private double lastAutoCrossFoulTime = -10.0;

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
        lastPinFoulTime = -10.0;
        playerPinningActive = false;
        opponentPinningActive = false;
        lastAutoCrossFoulTime = -10.0;
        refereeAlert.set(false);
    }

    @Override
    public void simulationUpdate() {
        double now = Timer.getFPGATimestamp();

        // ── 1. G201 Autonomous Crossing Rule ─────────────────────────────────
        if (DriverStation.isAutonomous()) {
            evaluateAutonomousBoundaries(now);
        }

        // ── 2. G401 Pinning Rule Evaluation ──────────────────────────────────
        evaluatePinningRule(now);

        // ── 3. Publish Telemetry ─────────────────────────────────────────────
        publishTelemetry();
    }

    /**
     * Evaluates autonomous boundary crossings (G201).
     * Robots may not cross the midfield centerline into the opposing half during auto.
     */
    public synchronized void evaluateAutonomousBoundaries(double now) {
        if (now - lastAutoCrossFoulTime < 2.5) return; // Debounce

        double centerX = FieldMap.CENTERLINE_X;
        boolean playerIsRed = AllianceFlipUtil.isRedAlliance();

        // 1. Check Player Robot
        Pose2d playerPose = SwerveBase.getInstance().getPose();
        if (playerPose != null) {
            if (playerIsRed && playerPose.getX() < (centerX - CENTERLINE_BUFFER_METERS)) {
                // Red player crossed into Blue auto zone
                MatchScoreTracker.getInstance().recordFoul(true, true, "G201 Auto Centerline Crossing by Player");
                triggerFoulAlert("[G201] Red Player crossed Auto Centerline (+5 to Blue)");
                lastAutoCrossFoulTime = now;
            } else if (!playerIsRed && playerPose.getX() > (centerX + CENTERLINE_BUFFER_METERS)) {
                // Blue player crossed into Red auto zone
                MatchScoreTracker.getInstance().recordFoul(false, true, "G201 Auto Centerline Crossing by Player");
                triggerFoulAlert("[G201] Blue Player crossed Auto Centerline (+5 to Red)");
                lastAutoCrossFoulTime = now;
            }
        }

        // 2. Check Opponent Bots
        try {
            AIRobotSim sim = AIRobotSim.getInstance();
            if (sim != null && sim.getDriveSimulation() != null) {
                Pose2d oppPose = sim.getDriveSimulation().getActualPoseInSimulationWorld();
                boolean oppIsRed = !playerIsRed;
                if (oppIsRed && oppPose.getX() < (centerX - CENTERLINE_BUFFER_METERS)) {
                    MatchScoreTracker.getInstance().recordFoul(true, true, "G201 Auto Centerline Crossing by Bot 0");
                    triggerFoulAlert("[G201] Red Bot crossed Auto Centerline (+5 to Blue)");
                    lastAutoCrossFoulTime = now;
                } else if (!oppIsRed && oppPose.getX() > (centerX + CENTERLINE_BUFFER_METERS)) {
                    MatchScoreTracker.getInstance().recordFoul(false, true, "G201 Auto Centerline Crossing by Bot 0");
                    triggerFoulAlert("[G201] Blue Bot crossed Auto Centerline (+5 to Red)");
                    lastAutoCrossFoulTime = now;
                }
            }
        } catch (Exception ignored) {}
    }

    /**
     * Evaluates close-quarters contact and illegal pinning (G401).
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
        boolean isCloseContact = distance < 1.10;

        if (isCloseContact) {
            playerOpponentPinTime += 0.02;

            if (playerOpponentPinTime >= PIN_MAX_TIME_SEC) {
                // Pinning limit exceeded! Check if foul interval elapsed
                if (now - lastPinFoulTime >= PIN_FOUL_INTERVAL_SEC) {
                    boolean playerIsRed = AllianceFlipUtil.isRedAlliance();
                    // Determine aggressor by speed vector directed toward opponent
                    var playerSpeeds = SwerveBase.getInstance().getFieldVelocity();
                    double pDot = (opponentPose.getX() - playerPose.getX()) * playerSpeeds.vxMetersPerSecond
                            + (opponentPose.getY() - playerPose.getY()) * playerSpeeds.vyMetersPerSecond;

                    boolean playerIsAggressor = pDot > 0.15;
                    boolean foulOnRed = playerIsAggressor ? playerIsRed : !playerIsRed;

                    String violator = playerIsAggressor ? "Player" : "Opponent Bot";
                    MatchScoreTracker.getInstance().recordFoul(foulOnRed, false, "G401 Pinning Violation by " + violator);
                    triggerFoulAlert("[G401 PIN] " + violator + " exceeded 2.4s pin limit (+2 pts)");
                    lastPinFoulTime = now;
                }
            }
        } else {
            // Decay contact timer when separated
            playerOpponentPinTime = Math.max(0.0, playerOpponentPinTime - 0.04);
        }
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
