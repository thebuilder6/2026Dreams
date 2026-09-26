package frc.robot.Sim;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Navigation.FieldMap;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * MatchScoreTracker: Unified FRC 2026 Rebuilt Real-Time Match Scoring & Scoreboard System.
 * 
 * Provides comprehensive real-time score tracking for both alliances and all robots:
 * - Red Alliance vs. Blue Alliance live scoreboard.
 * - Dynamic Player Alliance vs. Opponent Alliance mapping.
 * - Fuel scoring breakdowns (Active Hub Fuel = 1 pt, Inactive Wasted Shots = 0 pts),
 *   split by AUTO vs TELEOP period.
 * - Tower climb & endgame parking detection for all active robots (10 pts per climbed robot).
 * - Fouls: MINOR FOUL = 5 pts, MAJOR FOUL = 15 pts credited to the opponent's total.
 * - FRC Ranking Point (RP) evaluation:
 *     * Win = 2 RP, Tie = 1 RP
 *     * Energized / Fuel RP: >= 40 total fuel scored (+1 RP)
 *     * Supercharged / Tower Climb RP: >= 2 robots climbed (+1 RP)
 * - Per-robot shot attribution (Player, Bot 0, Bot 1, Bot 2, Ally 1, Ally 2).
 * - Live AdvantageKit and SmartDashboard telemetry publishing.
 */
public class MatchScoreTracker implements Subsystem {

    public static final int POINTS_PER_FUEL = 1;
    public static final int POINTS_PER_CLIMB = 10;
    public static final int POINTS_PER_MINOR_FOUL = 5;
    public static final int POINTS_PER_MAJOR_FOUL = 15;
    public static final int FUEL_RP_THRESHOLD = 40;
    public static final int CLIMB_RP_ROBOT_COUNT = 2;
    public static final double TOWER_CLIMB_RADIUS_METERS = 1.20;

    private static MatchScoreTracker instance;

    public static synchronized MatchScoreTracker getInstance() {
        if (instance == null) {
            instance = new MatchScoreTracker();
        }
        return instance;
    }

    // Fuel scores
    private int redFuelCount = 0;
    private int blueFuelCount = 0;
    private int redAutoFuelCount = 0;
    private int blueAutoFuelCount = 0;
    private int redTeleopFuelCount = 0;
    private int blueTeleopFuelCount = 0;
    private int redWastedFuelCount = 0;
    private int blueWastedFuelCount = 0;

    // Fouls committed & penalty points awarded (MINOR = 5 pts, MAJOR = 15 pts to opponent)
    private int redFoulCount = 0;
    private int blueFoulCount = 0;
    private int redMajorFoulCount = 0;
    private int blueMajorFoulCount = 0;
    private int redPenaltyPoints = 0; // Points given to Red (because Blue committed fouls)
    private int bluePenaltyPoints = 0; // Points given to Blue (because Red committed fouls)
    private String lastFoulDescription = "None";

    // Player specific shot tracking
    private int playerShotsAttempted = 0;
    private int playerShotsScored = 0;

    // Per-bot scoring tracking (botId -> count)
    private int bot0FuelScored = 0;
    private int bot1FuelScored = 0;
    private int bot2FuelScored = 0;
    private int ally1FuelScored = 0;
    private int ally2FuelScored = 0;

    // Climb tracking
    private boolean playerClimbed = false;
    private boolean bot0Climbed = false;
    private boolean bot1Climbed = false;
    private boolean bot2Climbed = false;
    private boolean ally1Climbed = false;
    private boolean ally2Climbed = false;

    private int redClimbCount = 0;
    private int blueClimbCount = 0;

    private MatchScoreTracker() {
        SubsystemManager.registerSubsystem(this);
    }

    /**
     * Records a fuel score for an alliance, attributing it to the current
     * match period (AUTO vs TELEOP).
     *
     * @param isRedAlliance True if scored into the Red Alliance Hub
     */
    public synchronized void recordFuelScore(boolean isRedAlliance) {
        boolean isAuto = DriverStation.isAutonomous();
        if (isRedAlliance) {
            redFuelCount++;
            if (isAuto) {
                redAutoFuelCount++;
            } else {
                redTeleopFuelCount++;
            }
        } else {
            blueFuelCount++;
            if (isAuto) {
                blueAutoFuelCount++;
            } else {
                blueTeleopFuelCount++;
            }
        }
    }

    /**
     * Records a player shot attempt.
     *
     * @param count Number of balls launched
     */
    public synchronized void recordPlayerShotAttempt(int count) {
        playerShotsAttempted += count;
    }

    /**
     * Records a player score into the target hub.
     *
     * @param isRedHub True if scored into Red Hub
     */
    public synchronized void recordPlayerScore(boolean isRedHub) {
        playerShotsScored++;
        recordFuelScore(isRedHub);
    }

    /**
     * Records a wasted shot into an inactive hub.
     *
     * @param isRedHub True if shot was aimed at Red Hub
     */
    public synchronized void recordWastedShot(boolean isRedHub) {
        if (isRedHub) {
            redWastedFuelCount++;
        } else {
            blueWastedFuelCount++;
        }
    }

    /**
     * Records a score attributed to a specific bot.
     *
     * @param botId Bot identifier (0 = Bot 0, 1 = Bot 1, 2 = Bot 2, 101 = Ally 1, 102 = Ally 2)
     * @param isRedAlliance True if bot belongs to Red Alliance
     */
    public synchronized void recordBotScore(int botId, boolean isRedAlliance) {
        switch (botId) {
            case 0: bot0FuelScored++; break;
            case 1: bot1FuelScored++; break;
            case 2: bot2FuelScored++; break;
            case 101: ally1FuelScored++; break;
            case 102: ally2FuelScored++; break;
            default: break;
        }
        recordFuelScore(isRedAlliance);
    }

    /**
     * Records a foul committed by an alliance, awarding penalty points to the opposing alliance.
     * MINOR FOUL = 5 pts, MAJOR FOUL = 15 pts.
     *
     * @param committedByRed True if Red Alliance committed the infraction
     * @param isMajorFoul True for MAJOR FOUL (15 pts), false for MINOR FOUL (5 pts)
     * @param reason Rule citation or description of the infraction
     */
    public synchronized void recordFoul(boolean committedByRed, boolean isMajorFoul, String reason) {
        int pts = isMajorFoul ? POINTS_PER_MAJOR_FOUL : POINTS_PER_MINOR_FOUL;
        if (committedByRed) {
            redFoulCount++;
            if (isMajorFoul) redMajorFoulCount++;
            bluePenaltyPoints += pts;
            lastFoulDescription = "[RED " + (isMajorFoul ? "MAJOR FOUL" : "MINOR FOUL") + "] " + reason + " (+" + pts + " pts to Blue)";
        } else {
            blueFoulCount++;
            if (isMajorFoul) blueMajorFoulCount++;
            redPenaltyPoints += pts;
            lastFoulDescription = "[BLUE " + (isMajorFoul ? "MAJOR FOUL" : "MINOR FOUL") + "] " + reason + " (+" + pts + " pts to Red)";
        }
        Logger.recordOutput("Scoreboard/LastFoul", lastFoulDescription);
    }

    /** Convenience wrapper for a MINOR FOUL (5 pts to the opponent). */
    public synchronized void recordMinorFoul(boolean committedByRed, String reason) {
        recordFoul(committedByRed, false, reason);
    }

    /** Convenience wrapper for a MAJOR FOUL (15 pts to the opponent). */
    public synchronized void recordMajorFoul(boolean committedByRed, String reason) {
        recordFoul(committedByRed, true, reason);
    }

    // ── Score Totals & Calculations ──────────────────────────────────────────

    public synchronized int getRedFuelScore() {
        return redFuelCount * POINTS_PER_FUEL;
    }

    public synchronized int getBlueFuelScore() {
        return blueFuelCount * POINTS_PER_FUEL;
    }

    public synchronized int getRedClimbScore() {
        return redClimbCount * POINTS_PER_CLIMB;
    }

    public synchronized int getBlueClimbScore() {
        return blueClimbCount * POINTS_PER_CLIMB;
    }

    public synchronized int getRedPenaltyScore() {
        return redPenaltyPoints;
    }

    public synchronized int getBluePenaltyScore() {
        return bluePenaltyPoints;
    }

    public synchronized int getRedFoulCount() {
        return redFoulCount;
    }

    public synchronized int getBlueFoulCount() {
        return blueFoulCount;
    }

    /**
     * @deprecated Renamed: the rulebook calls these MAJOR fouls.
     *             Use {@link #getRedMajorFoulCount()} instead.
     */
    @Deprecated
    public synchronized int getRedTechFoulCount() {
        return redMajorFoulCount;
    }

    /**
     * @deprecated Renamed: the rulebook calls these MAJOR fouls.
     *             Use {@link #getBlueMajorFoulCount()} instead.
     */
    @Deprecated
    public synchronized int getBlueTechFoulCount() {
        return blueMajorFoulCount;
    }

    public synchronized int getRedMajorFoulCount() {
        return redMajorFoulCount;
    }

    public synchronized int getBlueMajorFoulCount() {
        return blueMajorFoulCount;
    }

    public synchronized int getRedAutoFuelCount() { return redAutoFuelCount; }
    public synchronized int getBlueAutoFuelCount() { return blueAutoFuelCount; }
    public synchronized int getRedTeleopFuelCount() { return redTeleopFuelCount; }
    public synchronized int getBlueTeleopFuelCount() { return blueTeleopFuelCount; }

    public synchronized String getLastFoulDescription() {
        return lastFoulDescription;
    }

    public synchronized int getRedTotalScore() {
        return getRedFuelScore() + getRedClimbScore() + redPenaltyPoints;
    }

    public synchronized int getBlueTotalScore() {
        return getBlueFuelScore() + getBlueClimbScore() + bluePenaltyPoints;
    }

    public synchronized int getPlayerScore() {
        return AllianceFlipUtil.isRedAlliance() ? getRedTotalScore() : getBlueTotalScore();
    }

    public synchronized int getOpponentScore() {
        return AllianceFlipUtil.isRedAlliance() ? getBlueTotalScore() : getRedTotalScore();
    }

    public synchronized int getLeadMargin() {
        return Math.abs(getRedTotalScore() - getBlueTotalScore());
    }

    public synchronized String getLeader() {
        int red = getRedTotalScore();
        int blue = getBlueTotalScore();
        if (red > blue) {
            return "RED (+" + (red - blue) + ")";
        } else if (blue > red) {
            return "BLUE (+" + (blue - red) + ")";
        } else {
            return "TIED (" + red + ")";
        }
    }

    public synchronized double getPlayerAccuracyPercent() {
        if (playerShotsAttempted <= 0) return 0.0;
        return (double) playerShotsScored / playerShotsAttempted * 100.0;
    }

    // ── FRC Ranking Points (RP) ──────────────────────────────────────────────

    public synchronized boolean isRedFuelRpAchieved() {
        return redFuelCount >= FUEL_RP_THRESHOLD;
    }

    public synchronized boolean isBlueFuelRpAchieved() {
        return blueFuelCount >= FUEL_RP_THRESHOLD;
    }

    public synchronized boolean isRedClimbRpAchieved() {
        return redClimbCount >= CLIMB_RP_ROBOT_COUNT;
    }

    public synchronized boolean isBlueClimbRpAchieved() {
        return blueClimbCount >= CLIMB_RP_ROBOT_COUNT;
    }

    public synchronized int getRedRankingPoints() {
        int rp = 0;
        int red = getRedTotalScore();
        int blue = getBlueTotalScore();
        if (red > blue) rp += 2;
        else if (red == blue && red > 0) rp += 1;
        if (isRedFuelRpAchieved()) rp += 1;
        if (isRedClimbRpAchieved()) rp += 1;
        return rp;
    }

    public synchronized int getBlueRankingPoints() {
        int rp = 0;
        int red = getRedTotalScore();
        int blue = getBlueTotalScore();
        if (blue > red) rp += 2;
        else if (blue == red && blue > 0) rp += 1;
        if (isBlueFuelRpAchieved()) rp += 1;
        if (isBlueClimbRpAchieved()) rp += 1;
        return rp;
    }

    // ── Endgame Tower Climb Verification ─────────────────────────────────────

    /**
     * Checks all robot positions during endgame (t <= 20.0s) and awards climb points
     * to robots positioned within the alliance tower climbing zone.
     */
    public synchronized void updateClimbEvaluation() {
        double matchTime = Timer.getMatchTime();
        if (matchTime < 0.0) matchTime = GameSim.getInstance().getSimTimeRemainingSec();

        // Climb evaluation is only valid in the final 20 seconds of teleop / endgame (never in autonomous)
        if (DriverStation.isAutonomous() || (matchTime > 20.0 && matchTime <= 150.0)) {
            playerClimbed = false;
            bot0Climbed = false;
            bot1Climbed = false;
            bot2Climbed = false;
            ally1Climbed = false;
            ally2Climbed = false;
            redClimbCount = 0;
            blueClimbCount = 0;
            return;
        }

        boolean playerIsRed = AllianceFlipUtil.isRedAlliance();
        boolean opponentIsRed = !playerIsRed;

        Translation2d redTower = FieldMap.ClimbingTowers.RED_TOWER_POLE;
        Translation2d blueTower = FieldMap.ClimbingTowers.BLUE_TOWER_POLE;

        // 1. Evaluate Player Robot
        Pose2d playerPose = SwerveBase.getInstance().getPose();
        Translation2d playerTower = playerIsRed ? redTower : blueTower;
        playerClimbed = (playerPose != null && playerPose.getTranslation().getDistance(playerTower) <= TOWER_CLIMB_RADIUS_METERS);

        // 2. Evaluate Opponent Bots (Bot 0, 1, 2)
        AIRobotSim opponentSim = AIRobotSim.getInstance();
        Translation2d oppTower = opponentIsRed ? redTower : blueTower;

        bot0Climbed = false;
        bot1Climbed = false;
        bot2Climbed = false;

        if (opponentSim != null && opponentSim.getDriveSimulation() != null) {
            Pose2d bot0Pose = opponentSim.getDriveSimulation().getActualPoseInSimulationWorld();
            if (bot0Pose != null && bot0Pose.getY() > 0.0) {
                bot0Climbed = bot0Pose.getTranslation().getDistance(oppTower) <= TOWER_CLIMB_RADIUS_METERS;
            }

            List<AIRobotInstance> additionalBots = opponentSim.getAdditionalBots();
            if (additionalBots.size() >= 1) {
                Pose2d bot1Pose = additionalBots.get(0).getActualPose();
                if (bot1Pose != null && bot1Pose.getY() > 0.0) {
                    bot1Climbed = bot1Pose.getTranslation().getDistance(oppTower) <= TOWER_CLIMB_RADIUS_METERS;
                }
            }
            if (additionalBots.size() >= 2) {
                Pose2d bot2Pose = additionalBots.get(1).getActualPose();
                if (bot2Pose != null && bot2Pose.getY() > 0.0) {
                    bot2Climbed = bot2Pose.getTranslation().getDistance(oppTower) <= TOWER_CLIMB_RADIUS_METERS;
                }
            }

            // 3. Evaluate Ally Bots (Ally 1, 2)
            List<AIRobotInstance> allyBots = opponentSim.getAllyBots();
            if (allyBots.size() >= 1) {
                Pose2d ally1Pose = allyBots.get(0).getActualPose();
                if (ally1Pose != null && ally1Pose.getY() > 0.0) {
                    ally1Climbed = ally1Pose.getTranslation().getDistance(playerTower) <= TOWER_CLIMB_RADIUS_METERS;
                }
            }
            if (allyBots.size() >= 2) {
                Pose2d ally2Pose = allyBots.get(1).getActualPose();
                if (ally2Pose != null && ally2Pose.getY() > 0.0) {
                    ally2Climbed = ally2Pose.getTranslation().getDistance(playerTower) <= TOWER_CLIMB_RADIUS_METERS;
                }
            }
        }

        // Aggregate climb counts per alliance
        int redClimbs = 0;
        int blueClimbs = 0;

        if (playerIsRed) {
            if (playerClimbed) redClimbs++;
            if (ally1Climbed) redClimbs++;
            if (ally2Climbed) redClimbs++;
            if (bot0Climbed) blueClimbs++;
            if (bot1Climbed) blueClimbs++;
            if (bot2Climbed) blueClimbs++;
        } else {
            if (playerClimbed) blueClimbs++;
            if (ally1Climbed) blueClimbs++;
            if (ally2Climbed) blueClimbs++;
            if (bot0Climbed) redClimbs++;
            if (bot1Climbed) redClimbs++;
            if (bot2Climbed) redClimbs++;
        }

        this.redClimbCount = redClimbs;
        this.blueClimbCount = blueClimbs;
    }

    @Override
    public void simulationUpdate() {
        updateClimbEvaluation();
        ShotTracker.resolve();
        publishTelemetry();
    }

    @Override
    public void update() {
    }

    /**
     * Publishes full match scoreboard and analytics to SmartDashboard and AdvantageKit.
     */
    public synchronized void publishTelemetry() {
        boolean playerIsRed = AllianceFlipUtil.isRedAlliance();

        // ── Main Match Scoreboard ────────────────────────────────────────────
        SmartDashboard.putNumber("Scoreboard/Match/RedScore", getRedTotalScore());
        SmartDashboard.putNumber("Scoreboard/Match/BlueScore", getBlueTotalScore());
        SmartDashboard.putNumber("Scoreboard/Match/PlayerScore", getPlayerScore());
        SmartDashboard.putNumber("Scoreboard/Match/OpponentScore", getOpponentScore());
        SmartDashboard.putNumber("Scoreboard/Match/LeadMargin", getLeadMargin());
        SmartDashboard.putString("Scoreboard/Match/Leader", getLeader());

        // ── Red Alliance Breakdown ───────────────────────────────────────────
        SmartDashboard.putNumber("Scoreboard/Red/FuelPoints", getRedFuelScore());
        SmartDashboard.putNumber("Scoreboard/Red/FuelCount", redFuelCount);
        SmartDashboard.putNumber("Scoreboard/Red/AutoFuelCount", redAutoFuelCount);
        SmartDashboard.putNumber("Scoreboard/Red/TeleopFuelCount", redTeleopFuelCount);
        SmartDashboard.putNumber("Scoreboard/Red/ClimbPoints", getRedClimbScore());
        SmartDashboard.putNumber("Scoreboard/Red/ClimbedRobots", redClimbCount);
        SmartDashboard.putNumber("Scoreboard/Red/PenaltyPoints", redPenaltyPoints);
        SmartDashboard.putNumber("Scoreboard/Red/MinorFouls", redFoulCount - redMajorFoulCount);
        SmartDashboard.putNumber("Scoreboard/Red/MajorFouls", redMajorFoulCount);
        SmartDashboard.putNumber("Scoreboard/Red/TotalScore", getRedTotalScore());
        SmartDashboard.putNumber("Scoreboard/Red/RankingPoints", getRedRankingPoints());
        SmartDashboard.putBoolean("Scoreboard/Red/FuelRPAchieved", isRedFuelRpAchieved());
        SmartDashboard.putBoolean("Scoreboard/Red/ClimbRPAchieved", isRedClimbRpAchieved());

        // ── Blue Alliance Breakdown ──────────────────────────────────────────
        SmartDashboard.putNumber("Scoreboard/Blue/FuelPoints", getBlueFuelScore());
        SmartDashboard.putNumber("Scoreboard/Blue/FuelCount", blueFuelCount);
        SmartDashboard.putNumber("Scoreboard/Blue/AutoFuelCount", blueAutoFuelCount);
        SmartDashboard.putNumber("Scoreboard/Blue/TeleopFuelCount", blueTeleopFuelCount);
        SmartDashboard.putNumber("Scoreboard/Blue/ClimbPoints", getBlueClimbScore());
        SmartDashboard.putNumber("Scoreboard/Blue/ClimbedRobots", blueClimbCount);
        SmartDashboard.putNumber("Scoreboard/Blue/PenaltyPoints", bluePenaltyPoints);
        SmartDashboard.putNumber("Scoreboard/Blue/MinorFouls", blueFoulCount - blueMajorFoulCount);
        SmartDashboard.putNumber("Scoreboard/Blue/MajorFouls", blueMajorFoulCount);
        SmartDashboard.putNumber("Scoreboard/Blue/TotalScore", getBlueTotalScore());
        SmartDashboard.putNumber("Scoreboard/Blue/RankingPoints", getBlueRankingPoints());
        SmartDashboard.putBoolean("Scoreboard/Blue/FuelRPAchieved", isBlueFuelRpAchieved());
        SmartDashboard.putBoolean("Scoreboard/Blue/ClimbRPAchieved", isBlueClimbRpAchieved());

        // ── Player Robot Analytics ───────────────────────────────────────────
        SmartDashboard.putNumber("Scoreboard/Player/ShotsAttempted", playerShotsAttempted);
        SmartDashboard.putNumber("Scoreboard/Player/ShotsScored", playerShotsScored);
        SmartDashboard.putNumber("Scoreboard/Player/FuelScored", playerShotsScored);
        SmartDashboard.putNumber("Scoreboard/Player/AccuracyPercent", getPlayerAccuracyPercent());
        SmartDashboard.putBoolean("Scoreboard/Player/Climbed", playerClimbed);

        // ── Opponent Robot Contributions ─────────────────────────────────────
        SmartDashboard.putNumber("Scoreboard/Opponents/Bot0_FuelScored", bot0FuelScored);
        SmartDashboard.putNumber("Scoreboard/Opponents/Bot1_FuelScored", bot1FuelScored);
        SmartDashboard.putNumber("Scoreboard/Opponents/Bot2_FuelScored", bot2FuelScored);
        SmartDashboard.putNumber("Scoreboard/Opponents/TotalFuelScored", bot0FuelScored + bot1FuelScored + bot2FuelScored);
        SmartDashboard.putBoolean("Scoreboard/Opponents/Bot0_Climbed", bot0Climbed);
        SmartDashboard.putBoolean("Scoreboard/Opponents/Bot1_Climbed", bot1Climbed);
        SmartDashboard.putBoolean("Scoreboard/Opponents/Bot2_Climbed", bot2Climbed);

        // ── Ally Robot Contributions ─────────────────────────────────────────
        SmartDashboard.putNumber("Scoreboard/Allies/Ally1_FuelScored", ally1FuelScored);
        SmartDashboard.putNumber("Scoreboard/Allies/Ally2_FuelScored", ally2FuelScored);
        SmartDashboard.putNumber("Scoreboard/Allies/TotalFuelScored", ally1FuelScored + ally2FuelScored);
        SmartDashboard.putBoolean("Scoreboard/Allies/Ally1_Climbed", ally1Climbed);
        SmartDashboard.putBoolean("Scoreboard/Allies/Ally2_Climbed", ally2Climbed);

        // ── Referee & Penalty Breakdown ──────────────────────────────────────
        SmartDashboard.putNumber("Scoreboard/Red/PenaltyPoints", redPenaltyPoints);
        SmartDashboard.putNumber("Scoreboard/Red/FoulsCommitted", redFoulCount);
        SmartDashboard.putNumber("Scoreboard/Blue/PenaltyPoints", bluePenaltyPoints);
        SmartDashboard.putNumber("Scoreboard/Blue/FoulsCommitted", blueFoulCount);
        SmartDashboard.putString("Scoreboard/Referee/LastFoul", lastFoulDescription);

        Logger.recordOutput("Scoreboard/RedPenaltyPoints", redPenaltyPoints);
        Logger.recordOutput("Scoreboard/BluePenaltyPoints", bluePenaltyPoints);
        Logger.recordOutput("Scoreboard/Referee/LastFoul", lastFoulDescription);
    }

    /**
     * Resets all match scores and robot tracking state.
     */
    public synchronized void reset() {
        redFuelCount = 0;        blueFuelCount = 0;
        redAutoFuelCount = 0;
        blueAutoFuelCount = 0;
        redTeleopFuelCount = 0;
        blueTeleopFuelCount = 0;
        redWastedFuelCount = 0;
        blueWastedFuelCount = 0;

        redFoulCount = 0;
        blueFoulCount = 0;
        redMajorFoulCount = 0;
        blueMajorFoulCount = 0;
        redPenaltyPoints = 0;
        bluePenaltyPoints = 0;
        lastFoulDescription = "None";

        playerShotsAttempted = 0;
        playerShotsScored = 0;

        bot0FuelScored = 0;
        bot1FuelScored = 0;
        bot2FuelScored = 0;
        ally1FuelScored = 0;
        ally2FuelScored = 0;

        playerClimbed = false;
        bot0Climbed = false;
        bot1Climbed = false;
        bot2Climbed = false;
        ally1Climbed = false;
        ally2Climbed = false;

        redClimbCount = 0;
        blueClimbCount = 0;

        ShotTracker.clear();
    }

    public synchronized int getRedFuelCount() { return redFuelCount; }
    public synchronized int getBlueFuelCount() { return blueFuelCount; }
    public synchronized int getRedWastedFuelCount() { return redWastedFuelCount; }
    public synchronized int getBlueWastedFuelCount() { return blueWastedFuelCount; }
    public synchronized int getPlayerShotsAttempted() { return playerShotsAttempted; }
    public synchronized int getPlayerShotsScored() { return playerShotsScored; }
    public synchronized int getBot0FuelScored() { return bot0FuelScored; }
    public synchronized int getBot1FuelScored() { return bot1FuelScored; }
    public synchronized int getBot2FuelScored() { return bot2FuelScored; }
    public synchronized int getAlly1FuelScored() { return ally1FuelScored; }
    public synchronized int getAlly2FuelScored() { return ally2FuelScored; }
    public synchronized boolean isPlayerClimbed() { return playerClimbed; }
    public synchronized boolean isAlly1Climbed() { return ally1Climbed; }
    public synchronized boolean isAlly2Climbed() { return ally2Climbed; }
    public synchronized int getRedClimbCount() { return redClimbCount; }
    public synchronized int getBlueClimbCount() { return blueClimbCount; }
    public synchronized int getPlayerAllianceScore() { return getPlayerScore(); }
    public synchronized int getPlayerAllianceClimbScore() { return AllianceFlipUtil.isRedAlliance() ? getRedClimbScore() : getBlueClimbScore(); }

    @Override
    public boolean isEnabled() {
        return RobotBase.isSimulation();
    }

    @Override
    public void initialize() {
        reset();
    }

    @Override
    public void log() {
    }

    @Override
    public String getName() {
        return "MatchScoreTracker";
    }
}
