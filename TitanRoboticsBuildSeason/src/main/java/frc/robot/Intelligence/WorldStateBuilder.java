package frc.robot.Intelligence;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Sim.AIRobotInstance;
import frc.robot.Sim.AIRobotSim;
import frc.robot.Sim.GameSim;
import frc.robot.Sim.MatchScoreTracker;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Utils.AllianceFlipUtil;

/**
 * Factory utilities for building immutable WorldState snapshots from
 * simulation and real-robot hardware contexts.
 */
public final class WorldStateBuilder {

    private WorldStateBuilder() {}

    /**
     * Builds a WorldState snapshot for the primary player robot (e.g., AutonomousTeleopAgent or MatchCoach).
     *
     * <p>Driver-assist tier: the robot knows only what it could theoretically
     * perceive itself. Opponent robots are <i>not</i> known (no vision tracker
     * feeds them today), so the mark is a neutral placeholder and consumers
     * must evaluate with {@link MatchKnowledge#unknown()}.
     *
     * @param heldFuelCount Estimated or sensor-confirmed fuel/game pieces held in hopper
     * @return Immutable WorldState snapshot
     */
    public static WorldState buildForPlayerRobot(int heldFuelCount) {
        Pose2d playerPose = SwerveBase.getInstance().getPose();
        ChassisSpeeds playerVel = SwerveBase.getInstance().getFieldVelocity();

        // Unobserved mark: inert placeholder, never to be acted on. The policy
        // gates every opponent read on MatchKnowledge.opponentObserved().
        Pose2d oppPose = new Pose2d();
        ChassisSpeeds oppVel = new ChassisSpeeds();

        double matchTime = Timer.getMatchTime();
        if (matchTime < 0.0) matchTime = 135.0;

        boolean isPlayerRed = AllianceFlipUtil.isRedAlliance();
        boolean playerHubActive = Dashboard.getInstance().isHubActive();
        boolean oppHubActive = !playerHubActive;
        double timeUntilShift = Dashboard.getInstance().getTimeUntilSwitch();

        return new WorldState(
                playerPose,
                playerVel,
                heldFuelCount,
                oppPose,
                oppVel,
                matchTime,
                playerHubActive,
                oppHubActive,
                timeUntilShift,
                isPlayerRed,
                DriverStation.isAutonomous()
        );
    }

    /**
     * Builds a WorldState snapshot for a simulated AI sparring robot.
     *
     * @param selfPose Current pose of the AI robot in simulation world
     * @param selfVelocity Current field-relative velocity of the AI robot
     * @param heldFuelCount Current fuel pieces in AI robot's intake simulation
     * @param isOpponentRedAlliance True if this AI robot is on the Red Alliance
     * @param isSelfHubActive Whether this AI robot's scoring hub is currently active
     * @return Immutable WorldState snapshot from the perspective of this AI robot
     */
    public static WorldState buildForSimBot(
            Pose2d selfPose,
            ChassisSpeeds selfVelocity,
            int heldFuelCount,
            boolean isOpponentRedAlliance,
            boolean isSelfHubActive) {
        return buildForSimBot(selfPose, selfVelocity, heldFuelCount, isOpponentRedAlliance,
                isSelfHubActive, SwerveBase.getInstance().getPose(),
                SwerveBase.getInstance().getFieldVelocity());
    }

    /**
     * Builds a WorldState snapshot for a simulated AI sparring robot tracking an
     * explicit opponent mark (defensive assignments) instead of the player.
     */
    public static WorldState buildForSimBot(
            Pose2d selfPose,
            ChassisSpeeds selfVelocity,
            int heldFuelCount,
            boolean isOpponentRedAlliance,
            boolean isSelfHubActive,
            Pose2d markPose,
            ChassisSpeeds markVelocity) {

        double matchTime = Timer.getMatchTime();
        if (matchTime < 0.0) matchTime = 135.0;

        boolean playerHubActive = Dashboard.getInstance().isHubActive();
        double timeUntilShift = Dashboard.getInstance().getTimeUntilSwitch();

        return new WorldState(
                selfPose,
                selfVelocity,
                heldFuelCount,
                markPose,
                markVelocity,
                matchTime,
                isSelfHubActive,
                playerHubActive,
                timeUntilShift,
                isOpponentRedAlliance,
                DriverStation.isAutonomous()
        );
    }

    /**
     * Builds the sim-sparring tier {@link MatchKnowledge} for one AI bot: what
     * its robot would know plus what its human player would know — live score
     * differential, both sides' field picture (poses/velocities), and roughly
     * how many balls each side holds and has scored.
     *
     * @param botIsRed True if the bot skates for Red
     * @return Populated, immutable match knowledge
     */
    public static MatchKnowledge buildMatchKnowledgeForSimBot(boolean botIsRed) {
        MatchScoreTracker tracker;
        try {
            tracker = MatchScoreTracker.getInstance();
        } catch (Exception e) {
            return MatchKnowledge.unknown();
        }

        int redTotal;
        int blueTotal;
        int playerHeld = 0;
        int playerScored = 0;
        try {
            redTotal = tracker.getRedTotalScore();
            blueTotal = tracker.getBlueTotalScore();
            playerHeld = GameSim.getInstance().getHeldBalls();
            playerScored = tracker.getPlayerShotsScored();
        } catch (Exception e) {
            redTotal = 0;
            blueTotal = 0;
        }
        int scoreDifferential = botIsRed ? (redTotal - blueTotal) : (blueTotal - redTotal);

        boolean playerIsRed;
        Pose2d playerPose = new Pose2d();
        ChassisSpeeds playerVel = new ChassisSpeeds();
        try {
            playerIsRed = AllianceFlipUtil.isRedAlliance();
            AIRobotSim sim = AIRobotSim.getInstance();
            AIRobotInstance trainingPrimary = sim != null && sim.isTrainingScenarioActive()
                    ? sim.getTrainingBluePrimaryBot() : null;
            if (trainingPrimary != null) {
                playerPose = trainingPrimary.getActualPose();
                playerVel = trainingPrimary.getFieldVelocity();
                playerHeld = trainingPrimary.getFuelCount();
                playerScored = trainingPrimary.getScoreCount();
            } else {
                playerPose = SwerveBase.getInstance().getPose();
                playerVel = SwerveBase.getInstance().getFieldVelocity();
            }
        } catch (Exception e) {
            playerIsRed = !botIsRed;
        }
        boolean playerIsAlly = (playerIsRed == botIsRed);

        List<Pose2d> allyPoses = new ArrayList<>();
        List<Pose2d> opponentPoses = new ArrayList<>();
        List<ChassisSpeeds> allyVels = new ArrayList<>();
        List<ChassisSpeeds> opponentVels = new ArrayList<>();
        int alliesHeld = 0;
        int opponentsHeld = 0;
        int alliesScored = 0;
        int opponentsScored = 0;

        if (playerIsAlly) {
            allyPoses.add(playerPose);
            allyVels.add(playerVel);
            alliesHeld += playerHeld;
            alliesScored += playerScored;
        } else {
            opponentPoses.add(playerPose);
            opponentVels.add(playerVel);
            opponentsHeld += playerHeld;
            opponentsScored += playerScored;
        }

        try {
            AIRobotSim sim = AIRobotSim.getInstance();
            if (sim != null) {
                // Bot 0 and the additional pool always skate against the player.
                boolean poolIsAlly = (!playerIsRed == botIsRed);
                if (sim.getDriveSimulation() != null) {
                    Pose2d bot0 = sim.getDriveSimulation().getActualPoseInSimulationWorld();
                    if (bot0 != null && bot0.getY() > 0.0) {
                        if (poolIsAlly) {
                            allyPoses.add(bot0);
                            allyVels.add(new ChassisSpeeds());
                            alliesHeld += sim.getFuelCount();
                            alliesScored += tracker.getBot0FuelScored();
                        } else {
                            opponentPoses.add(bot0);
                            opponentVels.add(new ChassisSpeeds());
                            opponentsHeld += sim.getFuelCount();
                            opponentsScored += tracker.getBot0FuelScored();
                        }
                    }
                }
                for (AIRobotInstance bot : sim.getAdditionalBots()) {
                    if (bot == null) continue;
                    Pose2d p = bot.getActualPose();
                    if (p == null || p.getY() <= 0.0) continue;
                    int scored = bot.getBotId() == 1 ? tracker.getBot1FuelScored()
                            : bot.getBotId() == 2 ? tracker.getBot2FuelScored() : 0;
                    if (poolIsAlly) {
                        allyPoses.add(p);
                        allyVels.add(bot.getFieldVelocity());
                        alliesHeld += bot.getFuelCount();
                        alliesScored += scored;
                    } else {
                        opponentPoses.add(p);
                        opponentVels.add(bot.getFieldVelocity());
                        opponentsHeld += bot.getFuelCount();
                        opponentsScored += scored;
                    }
                }
                // Ally bots always skate with the player.
                boolean allyPoolIsAlly = (playerIsRed == botIsRed);
                for (AIRobotInstance ally : sim.getAllyBots()) {
                    if (ally == null) continue;
                    Pose2d p = ally.getActualPose();
                    if (p == null || p.getY() <= 0.0) continue;
                    int allyIndex = ally.getBotId() - 100;
                    int scored = allyIndex == 1 ? tracker.getAlly1FuelScored()
                            : allyIndex == 2 ? tracker.getAlly2FuelScored() : 0;
                    if (allyPoolIsAlly) {
                        allyPoses.add(p);
                        allyVels.add(ally.getFieldVelocity());
                        alliesHeld += ally.getFuelCount();
                        alliesScored += scored;
                    } else {
                        opponentPoses.add(p);
                        opponentVels.add(ally.getFieldVelocity());
                        opponentsHeld += ally.getFuelCount();
                        opponentsScored += scored;
                    }
                }
            }
        } catch (Exception ignored) {
        }

        return new MatchKnowledge(true, scoreDifferential,
                alliesHeld, opponentsHeld, alliesScored, opponentsScored,
                List.copyOf(allyPoses), List.copyOf(opponentPoses),
                List.copyOf(allyVels), List.copyOf(opponentVels));
    }
}
