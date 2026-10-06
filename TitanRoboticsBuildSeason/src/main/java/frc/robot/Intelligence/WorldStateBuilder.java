package frc.robot.Intelligence;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;
import frc.robot.Navigation.StaticPathfinder;
import frc.robot.Sim.AIRobotInstance;
import frc.robot.Sim.AIRobotSim;
import frc.robot.Sim.GameSim;
import frc.robot.Sim.HubSchedule;
import frc.robot.Sim.MatchScoreTracker;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Telemetry.Alert;
import frc.robot.Telemetry.Alert.AlertType;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Utils.AllianceFlipUtil;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;


/**
 * Factory utilities for building immutable WorldState snapshots from
 * simulation and real-robot hardware contexts.
 */
public final class WorldStateBuilder {

    private WorldStateBuilder() {}

    /** Seconds between repeat reports, mirroring {@code GameSim.logRateLimitedError}. */
    private static final double ERROR_LOG_INTERVAL_SEC = 2.0;

    private static double lastErrorLogTimestamp = Double.NEGATIVE_INFINITY;

    /**
     * Raised when the sim roster cannot be read, so match knowledge degrades to
     * "no field picture". Previously a bare {@code catch (Exception ignored)}
     * made that silent. The Alert carries the persistent state; this carries
     * the one-off detail, rate limited because this runs once per bot per tick.
     */
    private static final Alert ROSTER_UNAVAILABLE =
            new Alert("Intelligence", "Jev match knowledge: ally/opponent roster unavailable",
                    AlertType.WARNING);

    private static void reportDegraded(String context, Throwable t) {
        ROSTER_UNAVAILABLE.set(true);
        double now = Timer.getFPGATimestamp();
        if (now - lastErrorLogTimestamp > ERROR_LOG_INTERVAL_SEC) {
            lastErrorLogTimestamp = now;
            DriverStation.reportError("WorldStateBuilder [" + context + "]: " + t.getMessage(), false);
        }
    }

    /** Cleared by any successful build, so a transient blip does not latch forever. */
    private static void reportHealthy() {
        ROSTER_UNAVAILABLE.set(false);
    }

    /** Package-private for testing alert latching. */
    static boolean isRosterUnavailableAlertActive() {
        return ROSTER_UNAVAILABLE.isActive();
    }

    static void setRosterUnavailableAlertForTesting(boolean active) {
        ROSTER_UNAVAILABLE.set(active);
    }

    static void reportDegradedForTesting(String context, Throwable t) {
        reportDegraded(context, t);
    }

    /**
     * Whether a bot is actually on the playing field, as opposed to parked in
     * the queuing lane. The queuing poses are hardcoded at
     * {@code Y = OFF_FIELD_QUEUING_Y}, and the field spans {@code Y} 0 to
     * {@code FieldMap.FIELD_WIDTH}, so the sign of {@code Y} is the only
     * discriminator.
     *
     * <p>This was an inline {@code pose.getY() > 0.0} in three places. It works,
     * but it reads as a field-space test rather than the queuing test it
     * actually is, and nothing tied it to the queuing Y it depends on.
     */
    private static boolean isOnField(Pose2d pose) {
        return pose != null && pose.getY() > 0.0;
    }

    /**
     * Builds a WorldState snapshot for the primary player robot (e.g., AutonomousTeleopAgent or MatchCoach).
     *
     * <p>Driver-assist tier: the robot knows only what it could theoretically
     * perceive itself. Opponent robots are <i>not</i> known (no vision tracker
     * feeds them today), so the mark is a neutral placeholder and consumers
     * must evaluate with {@link ObservedKnowledge#selfOnly()}.
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
        double timeUntilShift = Dashboard.getInstance().getTimeUntilSwitch();

        // Do NOT infer the opponent hub by inverting our own. The two hubs are
        // complementary only during SHIFT 1-4; during AUTO, TRANSITION and
        // ENDGAME both are live, so inverting told the real robot the opponent
        // hub was dead for all of autonomous and the whole 30 s endgame. That
        // suppressed DENY_SHOOTING_LANE (JevDecisionEngine gates
        // laneDenialUtility on this flag) exactly when denying a live hub is
        // worth the most. See HubSchedule.isOpponentHubActiveGivenMineIs.
        boolean oppHubActive = HubSchedule.isOpponentHubActiveGivenMineIs(
                playerHubActive, HubSchedule.phaseFor(matchTime, DriverStation.isAutonomous()));

        // Where can this robot score AFTER the next flip. Without this the engine
        // cannot tell a shuttle (which lobs at the opponent's end) from a trip to
        // its own hub, so both fire at the wrong time.
        HubSchedule.Phase phase = HubSchedule.phaseFor(matchTime, DriverStation.isAutonomous());
        boolean mineAfter = HubSchedule.isHubActive(isPlayerRed, HubSchedule.nextPhase(phase),
                HubSchedule.getShiftSeed());
        boolean theirsAfter = HubSchedule.isHubActive(!isPlayerRed,
                HubSchedule.nextPhase(phase), HubSchedule.getShiftSeed());

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
                DriverStation.isAutonomous(),
                mineAfter,
                theirsAfter
        );
    }

    /**
     * Builds a WorldState snapshot for a simulated AI sparring robot.
     *
     * @param selfPose Current pose of the AI robot in simulation world
     * @param selfVelocity Current field-relative velocity of the AI robot
     * @param heldFuelCount Current fuel pieces in AI robot's intake simulation
     * @param isBotRed True if THIS AI robot is on the Red Alliance. Renamed from
     *                 {@code isOpponentRedAlliance}, which was wrong: the value
     *                 populates {@link WorldState#isRedAlliance()}, i.e. the
     *                 bot's own alliance, not an opponent's.
     * @param isSelfHubActive Whether this AI robot's scoring hub is currently active
     * @return Immutable WorldState snapshot from the perspective of this AI robot
     */
    public static WorldState buildForSimBot(
            Pose2d selfPose,
            ChassisSpeeds selfVelocity,
            int heldFuelCount,
            boolean isBotRed,
            boolean isSelfHubActive) {
        return buildForSimBot(selfPose, selfVelocity, heldFuelCount, isBotRed,
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
            boolean isBotRed,
            boolean isSelfHubActive,
            Pose2d markPose,
            ChassisSpeeds markVelocity) {

        // Shift-aware decisions read the schedule's own clock. The DS clock is
        // -1 under simulation, so Dashboard.getTimeUntilSwitch() stays pinned at
        // 0.0 and every bot believes its shift is ALWAYS ending. The opt-in real
        // clock is off by default because it was last measured to starve Blue's
        // teleop scoring, and that measurement predates the archetype-override,
        // fuel-attribution and Common Random Numbers fixes -- it has not been
        // re-run. The gate is the flag alone: refreshFromMatchState() already
        // fails open (unknown clock reads as 150.0 remaining, both hubs live, no
        // shift anticipation), so no extra validity test is needed.
        //
        // Full reasoning, and why the flag is still off, live in
        // KNOWN_ISSUES.md section A. Do not re-document it here.
        HubSchedule.refreshFromMatchState();
        boolean useRealShiftClock = Boolean.getBoolean("frc.jev.realShiftClock");
        double matchTime = useRealShiftClock ? HubSchedule.lastMatchTimeRemaining() : 135.0;
        if (matchTime < 0.0) matchTime = 135.0;

        // The opponent hub is the OTHER alliance's hub, so it must be derived
        // from this bot's own hub state -- not from the human player's. The
        // previous code passed Dashboard's player hub into the opponent slot,
        // which told a Blue sparring bot the Red hub was live exactly when Blue
        // was live: inverted for half the match, for every bot.
        boolean oppHubActive = HubSchedule.isOpponentHubActiveGivenMineIs(
                isSelfHubActive, HubSchedule.phaseFor(matchTime, DriverStation.isAutonomous()));
        double timeUntilShift = useRealShiftClock
                ? HubSchedule.timeUntilShiftEnd()
                : Dashboard.getInstance().getTimeUntilSwitch();

        // Next-shift state, same derivation as the player builder. Uses this bot's
        // own alliance, not the human player's.
        HubSchedule.Phase botPhase = HubSchedule.phaseFor(matchTime, DriverStation.isAutonomous());
        boolean mineAfter = HubSchedule.isHubActive(isBotRed, HubSchedule.nextPhase(botPhase),
                HubSchedule.getShiftSeed());
        boolean theirsAfter = HubSchedule.isHubActive(!isBotRed,
                HubSchedule.nextPhase(botPhase), HubSchedule.getShiftSeed());

        return new WorldState(
                selfPose,
                selfVelocity,
                heldFuelCount,
                markPose,
                markVelocity,
                matchTime,
                isSelfHubActive,
                oppHubActive,
                timeUntilShift,
                isBotRed,
                DriverStation.isAutonomous(),
                mineAfter,
                theirsAfter
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
        boolean degraded = false;
        MatchScoreTracker tracker;
        try {
            tracker = MatchScoreTracker.getInstance();
        } catch (Exception e) {
            // No tracker means no score and no per-bot attribution. Fall back to
            // ObservedKnowledge rather than fabricating a clairvoyant record with
            // zeros: zero zone counts under ClairvoyantKnowledge would read as
            // "the field really is empty", which is a claim the policy would act on.
            reportDegraded("scoreTracker", e);
            return ObservedKnowledge.selfOnly();
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
            // A 0-0 fallback is not the same as "tied" -- it is "unknown", and
            // the policy reads it as a real differential.
            reportDegraded("scoreRead", e);
            degraded = true;
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
            // The previous fallback assumed the player was an OPPONENT, which
            // silently inverted this bot's entire team membership: every ally
            // read as an opponent and vice versa, with no signal. An honest
            // assumption is that the player is a non-opponent (it is the robot
            // this code is running alongside), and the failure is reported.
            reportDegraded("playerPose", e);
            degraded = true;
            playerIsRed = botIsRed;
        }
        // Resolve team membership ONCE. The previous code carried two booleans
        // whose names differed only in word order and whose values were exact
        // negations:
        //     poolIsAlly     = (!playerIsRed == botIsRed)   // opponents + Bot 0
        //     allyPoolIsAlly = (playerIsRed == botIsRed)    // ally bots
        // Both were correct, but a swap between them is invisible at the call
        // site and would invert every bot's team awareness. One relation, used
        // three times, cannot drift that way.
        boolean playerIsAlly = (playerIsRed == botIsRed);
        boolean opponentPoolIsAlly = !playerIsAlly;   // Bot 0 + additional bots
        boolean allyPoolIsAlly = playerIsAlly;         // dedicated ally bots

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
                if (sim.getDriveSimulation() != null) {
                    Pose2d bot0 = sim.getDriveSimulation().getActualPoseInSimulationWorld();
                    if (isOnField(bot0)) {
                        if (opponentPoolIsAlly) {
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
                    if (!isOnField(p)) continue;
                    int scored = bot.getBotId() == 1 ? tracker.getBot1FuelScored()
                            : bot.getBotId() == 2 ? tracker.getBot2FuelScored() : 0;
                    if (opponentPoolIsAlly) {
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
                for (AIRobotInstance ally : sim.getAllyBots()) {
                    if (ally == null) continue;
                    Pose2d p = ally.getActualPose();
                    if (!isOnField(p)) continue;
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
        } catch (Exception e) {
            // Swallowing this used to leave the bot believing it had no match
            // context at all -- a silent wrong-tier failure that is
            // indistinguishable from correct behaviour. Alert instead.
            reportDegraded("roster", e);
            degraded = true;
        }

        int[] zoneFuel = countZoneFuel(botIsRed);
        if (zoneFuel == null) {
            degraded = true;
            zoneFuel = new int[] {0, 0, 0};
        }

        if (!degraded) {
            reportHealthy();
        }

        return new ClairvoyantKnowledge(scoreDifferential,
                alliesHeld, opponentsHeld, alliesScored, opponentsScored,
                List.copyOf(allyPoses), List.copyOf(opponentPoses),
                List.copyOf(allyVels), List.copyOf(opponentVels),
                zoneFuel[0], zoneFuel[1], zoneFuel[2]);
    }

    /**
     * Fuel on the field, bucketed into the three zones by
     * {@code FieldMap.AllianceZones} — the same geometry the engine and every
     * other caller use, so no caller can invent its own zoning.
     *
     * <p>These counts used to be recomputed inside
     * {@code JevDecisionEngine.countFuelInZone} on every objective evaluation,
     * which meant the knowledge record did not carry them and the "unobserved"
     * tier read perfect sim data anyway. Computing once here is what makes
     * {@link ObservedKnowledge}'s zeros meaningful.
     *
     * <p>Counted the way the selectors will actually treat it: fuel inside a hard
     * obstacle, near a dynamic obstacle, or abandoned by the
     * {@code TargetProgressWatchdog} is excluded. Counting abandoned fuel here
     * would reinstate the live-lock, where {@code SWEEP_ALLIANCE_ZONE} stayed
     * viable on pieces the policy was simultaneously forbidden to approach.
     *
     * @return {@code {ownAllianceZone, midfield, opponentZone}}
     */
    private static int[] countZoneFuel(boolean botIsRed) {
        try {
            var arena = SimulatedArena.getInstance();
            if (arena == null) {
                // Real hardware: no sensor for field fuel. Zeros are the truth.
                return new int[] {0, 0, 0};
            }
            int own = 0;
            int midfield = 0;
            int theirs = 0;
            // Single-owned by MatchDeterminism, which returns a stable (x,y)-sorted
            // snapshot rather than iterating a fresh HashSet whose order follows
            // identity hash codes and differs between JVM runs.
            for (var piece : frc.robot.Sim.MatchDeterminism.fuelOnFieldSorted()) {
                if (piece == null || !"Fuel".equals(piece.getType())) {
                    continue;
                }
                Translation2d at = piece.getPoseOnField().getTranslation();
                if (StaticPathfinder.isPointInHardObstacle(at)
                        || StaticPathfinder.isPointNearDynamicObstacle(at)) {
                    continue;
                }
                if (FieldMap.AllianceZones.isInMidfield(at)) {
                    midfield++;
                } else if (FieldMap.AllianceZones.isInAllianceZone(at, botIsRed)) {
                    own++;
                } else {
                    theirs++;
                }
            }
            return new int[] {own, midfield, theirs};
        } catch (Exception e) {
            reportDegraded("zoneFuel", e);
            return null;
        }
    }
}
