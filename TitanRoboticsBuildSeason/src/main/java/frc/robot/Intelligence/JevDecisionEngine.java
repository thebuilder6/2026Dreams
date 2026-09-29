package frc.robot.Intelligence;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Navigation.DynamicObstacle;
import frc.robot.Navigation.DynamicRouter;
import frc.robot.Navigation.StaticPathfinder;
import frc.robot.Navigation.TargetProgressWatchdog;
import frc.robot.Data.Constants;
import frc.robot.Navigation.FieldMap;
import frc.robot.Navigation.GlidePoints;
import frc.robot.Navigation.GlidePoints.GlidePoint;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Jev AI Decision Engine (TypeSafe AI)
 * 
 * Cognitive Architecture:
 * - System 2: Strategic utility model evaluating match time, Hub active phase,
 * and fuel inventory.
 * - System 1: High-frequency (<1ms) tactical policy generating concrete
 * mechanism and drive intents.
 */
public class JevDecisionEngine {

    public enum DecisionMode {
        LOCAL_HEURISTIC,
        TYPESAFE_CLOUD,
        AUTO_FALLBACK
    }

    private static final double CLOUD_DECISION_FRESHNESS_SEC = 1.2;
    private static final double MIN_CLOUD_CONFIDENCE = 0.60;
    private static final double LOCAL_PRIORITY_OVERRIDE_THRESHOLD = 0.95;

    public enum AIArchetype {
        AUTONOMOUS_CYCLER,
        TACTICAL_DEFENDER,
        LEAD_PURSUIT_INTERCEPTOR,
        PINNING_BULLY,
        ADAPTIVE_COMPETITOR
    }

    public enum TacticalAction {
        BLOCK_SHOOTING_LANE,
        CONTEST_DEPOT,
        SHADOW_PLAYER,
        RETREAT_DEFENSE
    }

    public static class DecisionResult {
        public final TacticalAction action;
        public final Pose2d targetPose;
        public final double confidence;
        public final Map<TacticalAction, Double> utilityScores;
        public final String rationale;
        public final double latencyMs;
        public final double timestamp;

        public DecisionResult(
                TacticalAction action,
                Pose2d targetPose,
                double confidence,
                Map<TacticalAction, Double> utilityScores,
                String rationale,
                double latencyMs,
                double timestamp) {
            this.action = action;
            this.targetPose = targetPose;
            this.confidence = confidence;
            this.utilityScores = Collections.unmodifiableMap(utilityScores);
            this.rationale = rationale;
            this.latencyMs = latencyMs;
            this.timestamp = timestamp;
        }

        public String toSchemaJson() {
            StringBuilder sb = new StringBuilder();
            sb.append("{");
            sb.append("\"timestamp\":").append(String.format("%.3f", timestamp)).append(",");
            sb.append("\"selected_action\":\"").append(action.name()).append("\",");
            sb.append("\"confidence\":").append(String.format("%.2f", confidence)).append(",");
            sb.append("\"latency_ms\":").append(String.format("%.2f", latencyMs)).append(",");
            sb.append("\"rationale\":\"").append(rationale.replace("\"", "\\\"")).append("\",");
            sb.append("\"target_pose\":{");
            sb.append("\"x\":").append(String.format("%.3f", targetPose.getX())).append(",");
            sb.append("\"y\":").append(String.format("%.3f", targetPose.getY())).append(",");
            sb.append("\"rot_deg\":").append(String.format("%.1f", targetPose.getRotation().getDegrees()));
            sb.append("},");
            sb.append("\"utility_scores\":{");
            int i = 0;
            for (Map.Entry<TacticalAction, Double> entry : utilityScores.entrySet()) {
                if (i++ > 0)
                    sb.append(",");
                sb.append("\"").append(entry.getKey().name()).append("\":")
                        .append(String.format("%.3f", entry.getValue()));
            }
            sb.append("}}");
            return sb.toString();
        }
    }

    public static final Translation2d BLUE_HUB_POS = FieldMap.Hubs.BLUE_HUB_2D;
    public static final Translation2d BLUE_DEPOT_POS = FieldMap.Depots.BLUE_DEPOT_CONTEST;
    public static final double CENTERLINE_X = FieldMap.CENTERLINE_X;
    public static final double RETREAT_X = 12.0;

    /**
     * Minimum hopper load before an auto bot commits to a scoring trip
     * (fill-then-volley).
     */
    public static final int AUTO_BATCH_MIN_FUEL = 8;
    /**
     * Auto clock (s remaining) below which bots dump whatever they hold instead of
     * harvesting.
     */
    public static final double AUTO_DUMP_SECONDS_LEFT = 4.0;

    private static JevDecisionEngine instance;

    public static synchronized JevDecisionEngine getInstance() {
        if (instance == null) {
            instance = new JevDecisionEngine();
        }
        return instance;
    }

    private DecisionResult lastDecision = null;
    private volatile DecisionMode decisionMode = DecisionMode.AUTO_FALLBACK;
    private final Map<String, String> lastPublishedCloudErrors = new ConcurrentHashMap<>();

    // ── Objective commitment thresholds ────────────────────────────────────
    // The utility matrix is re-evaluated every cycle, and several objectives sit
    // within ~0.02 of each other (SWEEP 0.96 vs CYCLE 0.72-0.98, VACUUM 0.35 at
    // batch vs 0.82+ when a piece drifts out of reach). Without a latch a
    // loaded bot re-routes every 20 ms: the observed symptom was "constantly
    // moving before trying to shoot".
    //
    // The engine itself stays STATELESS. Commitment is per-agent state owned by
    // the caller (AIRobotInstance, AutonomousTeleopAgent) and passed in, so two
    // agents can never read each other's decision. These thresholds are shared
    // constants only.
    /** Challenger must beat the incumbent by this much to steal commitment. */
    public static final double COMMITMENT_MARGIN = 0.06;

    /**
     * Diagnostic: forces every agent onto one objective, bypassing the commitment
     * latch, when {@code -Dfrc.jev.freezeObjective=<NAME>} is set. Read once so a typo
     * cannot fail silently mid-match. Exists to isolate whether headless
     * non-reproducibility originates in the AI or in the unseeded MapleSim physics
     * beneath it — see {@code KNOWN_ISSUES.md} §A. Returns {@code null} when unset.
     */
    private static volatile StrategicObjective frozenObjectiveCache;
    private static volatile boolean frozenObjectiveResolved;

    static StrategicObjective frozenObjective() {
        if (!frozenObjectiveResolved) {
            frozenObjectiveResolved = true;
            String raw = System.getProperty("frc.jev.freezeObjective", "").trim();
            if (raw.isEmpty()) {
                frozenObjectiveCache = null;
            } else {
                try {
                    frozenObjectiveCache = StrategicObjective.valueOf(raw.toUpperCase());
                } catch (IllegalArgumentException e) {
                    System.err.println("[Jev] frc.jev.freezeObjective unknown objective: " + raw
                            + " -- leaving the AI unfrozen");
                    frozenObjectiveCache = null;
                }
            }
        }
        return frozenObjectiveCache;
    }

    /**
     * A challenger this far ahead switches immediately, ignoring the minimum
     * hold. A decisive gap means the match state materially changed (hub
     * deactivating, inventory filling), and delaying it would be wrong. Small
     * gaps are the thrash we are damping, so they must also clear
     * {@link #COMMITMENT_MIN_HOLD_SEC}.
     */
    public static final double COMMITMENT_DECISIVE_MARGIN = 0.20;

    /** Minimum time on an objective before a non-decisive challenger may take it. */
    public static final double COMMITMENT_MIN_HOLD_SEC = 1.5;

    /**
     * Resolves the committed objective for one agent.
     *
     * <p>Pure: it reads and returns the caller's own latch fields without
     * mutating engine state. The caller writes the result back, which keeps
     * commitment ownership with the agent that owns the robot.
     *
     * @param candidate fresh local utility winner
     * @param utilities this cycle's utility scores
     * @param held currently committed objective, or null if none
     * @param heldSinceSeconds when the incumbent was adopted
     * @param nowSeconds current time (monotonic seconds)
     * @return the objective the agent should pursue this cycle
     */
    public static StrategicObjective resolveCommittedObjective(
            StrategicObjective candidate,
            Map<StrategicObjective, Double> utilities,
            StrategicObjective held,
            double heldSinceSeconds,
            double nowSeconds) {
        if (candidate == null) {
            return held;
        }
        if (held == null) {
            return candidate;
        }
        if (candidate == held) {
            return held;
        }

        // The incumbent is no longer viable (its utility collapsed, e.g. its
        // hub just went inactive): release immediately rather than honor the
        // minimum hold.
        double incumbentUtility = utilities.getOrDefault(held, 0.0);
        if (incumbentUtility <= 0.0) {
            return candidate;
        }

        double gain = utilities.getOrDefault(candidate, 0.0) - incumbentUtility;
        boolean decisive = gain >= COMMITMENT_DECISIVE_MARGIN;
        boolean matured = (nowSeconds - heldSinceSeconds) >= COMMITMENT_MIN_HOLD_SEC;
        return (decisive || (matured && gain >= COMMITMENT_MARGIN)) ? candidate : held;
    }

    public DecisionMode getDecisionMode() {
        return decisionMode;
    }

    public void setDecisionMode(DecisionMode mode) {
        if (mode == null)
            return;
        decisionMode = mode;
        Dashboard.setJevDecisionModeName(mode.name());
    }

    // =========================================================================
    // JEV TYPE-SAFE POLICY ENGINE (Option A & Option B)
    // =========================================================================

    /**
     * Evaluates WorldState through the Jev Macro Utility Matrix and outputs a
     * concrete AIActionIntent.
     * Guaranteed deterministic, stateless (re-entrant for multi-bot simulations),
     * and sub-millisecond latency.
     *
     * <p>
     * Defaults to {@link ObservedKnowledge#selfOnly()} — the honest answer for a
     * robot with no opponent tracker and no field-fuel sensor. This overload
     * previously defaulted to {@code MatchKnowledge.legacyObserved()}, which
     * claimed {@code opponentObserved = true} with four empty lists, so the
     * *default* path asserted an observation it did not have. Pass an explicit
     * {@link ClairvoyantKnowledge} for a sim sparring bot; pass
     * {@link ObservedKnowledge} for the real robot.
     *
     * @param world     State snapshot of the match and robots
     * @param archetype AI persona / behavior profile
     * @return Fully specified AIActionIntent
     */
    public AIActionIntent evaluatePolicy(WorldState world, Archetype archetype) {
        return evaluatePolicy(world, ObservedKnowledge.selfOnly(), archetype);
    }

    /**
     * Tier-aware policy evaluation.
     *
     * @param world     robot-knowable snapshot (own state, hub phase, one mark)
     * @param knowledge player-knowable match context (score, sides' balls,
     *                  field picture) or {@link MatchKnowledge#unknown()} for
     *                  the driver-assist tier
     * @param archetype behavior archetype selecting utility weights
     */
    public AIActionIntent evaluatePolicy(WorldState world, MatchKnowledge knowledge, Archetype archetype) {
        return evaluatePolicy(world, knowledge, archetype, null);
    }

    /**
     * Evaluation overload that isolates cloud state and telemetry for a simulator
     * bot.
     */
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype, String cloudContext) {
        return evaluatePolicy(world, knowledge, archetype, cloudContext, null);
    }

    /**
     * Evaluation overload for executors that own transient navigation state
     * (sim bots via {@code TargetProgressWatchdog}). {@code blockedFuel} holds
     * points the caller has abandoned this match - unreachable behind a hard
     * footprint, or a standoff with no legal approach - so fuel selectors skip
     * them instead of re-picking the same piece every cycle. The engine stays
     * stateless; the caller owns the lifetime.
     *
     * @param blockedFuel abandoned fuel points, or {@code null} for none
     */
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype, String cloudContext,
            Set<Translation2d> blockedFuel) {
        return evaluatePolicy(world, knowledge, archetype, cloudContext, blockedFuel, null);
    }

    /**
     * Full evaluation overload, including the caller's objective commitment.
     *
     * <p>{@code commitmentIn} is this agent's latch (one per robot, owned by
     * the caller). Passing null selects the raw utility winner, which is the
     * correct behavior for callers that have no persistent identity - the Match
     * Coach, and unit tests.
     *
     * @param blockedFuel  abandoned fuel points, or {@code null}
     * @param commitmentIn this agent's objective latch, or {@code null}
     */
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype, String cloudContext,
            Set<Translation2d> blockedFuel, ObjectiveCommitment commitmentIn) {
        if (knowledge == null) {
            // Null used to become legacyObserved(), i.e. "opponents seen". Default to
            // the tier that claims the least, so a caller that forgets to pass
            // knowledge degrades to sensor-only rather than acting on nothing.
            knowledge = ObservedKnowledge.selfOnly();
        }
        boolean opponentObserved = knowledge.opponentObserved();
        long startNanos = System.nanoTime();

        DecisionMode activeDecisionMode = resolveDecisionMode();
        TypeSafeJevClient cloudClient = TypeSafeJevClient.getInstance();
        boolean cloudConfigured = activeDecisionMode != DecisionMode.LOCAL_HEURISTIC
                && (archetype == Archetype.CO_PILOT || cloudContext != null)
                && cloudClient.hasValidKey();
        if (cloudConfigured) {
            cloudClient.evaluateAsync(cloudContext, world, knowledge, archetype);
        }

        Translation2d selfHub = FieldMap.Hubs.getHubLocation2d(world.isRedAlliance());
        double distToSelfHub = world.selfPose().getTranslation().getDistance(selfHub);
        double transitTimeToHub = distToSelfHub / 3.2;
        double timeLeftToHarvest = world.timeUntilHubShift() - transitTimeToHub;

        // ── 1. Evaluate Utility Scores Across Objectives ─────────────────────
        Map<StrategicObjective, Double> utilities = new LinkedHashMap<>();

        // Rush Climb (Endgame priority) — player robot only. Simulated bots are
        // assumed to have no climber fitted, so they never select RUSH_CLIMB and
        // instead keep playing (cycle / stage / defend) through endgame.
        double climbUtility = 0.0;
        if (archetype == Archetype.CO_PILOT
                && !world.isAutonomous()
                && world.matchTimeRemaining() > 0.0 && world.matchTimeRemaining() <= 20.0) {
            if (world.matchTimeRemaining() <= 15.0) {
                // Dominant endgame priority: strictly overrides cycling in final 15 seconds
                climbUtility = 0.99 + (0.01 * (1.0 - world.matchTimeRemaining() / 15.0));
            } else {
                climbUtility = 0.85 + (0.13 * (1.0 - (world.matchTimeRemaining() - 15.0) / 5.0));
            }
        }
        utilities.put(StrategicObjective.RUSH_CLIMB, climbUtility);

        // Cycle Score Hub
        // If already in shooting range (dist <= 4.0m) or Co-Pilot assist, keep firing
        // down to the very last ball!
        // If out in midfield, require a solid batch (>=16) unless the active shift is
        // about to end (<=4.5s)
        boolean inShootingRange = distToSelfHub <= 4.0;
        boolean shiftEndingSoon = world.timeUntilHubShift() <= 4.5 && world.timeUntilHubShift() > 0.0;
        int minFuelToScore = (archetype == Archetype.CO_PILOT || inShootingRange) ? 1 : (shiftEndingSoon ? 4 : 16);

        double scoreUtility = 0.0;
        if (world.isAllianceHubActive() && world.heldFuelCount() >= minFuelToScore) {
            double loadRatio = (archetype == Archetype.CO_PILOT || inShootingRange)
                    ? 1.0
                    : Math.min(1.0, world.heldFuelCount() / 20.0);
            scoreUtility = 0.72 + (0.26 * loadRatio); // 0.72 to 0.98
            // Tier-2 score awareness: chase when behind on the scoreboard.
            // Bounded (+0.03) so it biases close calls without overriding
            // geometry (range, batch, hub phase).
            if (knowledge.scoreDifferential() < 0) {
                scoreUtility = Math.min(0.99, scoreUtility + 0.03);
            }
        }
        utilities.put(StrategicObjective.CYCLE_SCORE_HUB, scoreUtility);

        // Stage Standoff (Hub is inactive; wait at standoff arc once hopper is well
        // stocked)
        double stageUtility = 0.0;
        int minFuelToStage = (archetype == Archetype.CO_PILOT) ? 1 : 18;
        if (!world.isAllianceHubActive() && world.heldFuelCount() >= minFuelToStage) {
            stageUtility = 0.80;
            if (world.timeUntilHubShift() <= 3.5 && world.timeUntilHubShift() > 0.0) {
                stageUtility = (archetype == Archetype.CO_PILOT) ? 0.99 : 0.95; // Anticipate imminent shift
            }
        }
        utilities.put(StrategicObjective.STAGE_STANDOFF, stageUtility);

        // Vacuum Midfield (Field fuel harvest: fill hopper with large payloads)
        double vacuumUtility = 0.0;
        if (!world.isInventoryFull()) {
            if (!world.isAllianceHubActive()) {
                // Stockpile full 20-30 ball capacity while hub is locked
                vacuumUtility = 0.88 + (0.10 * (1.0 - world.heldFuelCount() / 30.0));
            } else {
                int targetBatch = shiftEndingSoon ? 6 : 18;
                if (world.heldFuelCount() < targetBatch) {
                    vacuumUtility = 0.82 + (0.14 * (1.0 - (double) world.heldFuelCount() / targetBatch));
                } else {
                    vacuumUtility = 0.35;
                }
            }
        }
        utilities.put(StrategicObjective.VACUUM_MIDFIELD, vacuumUtility);

        // Stockpile Depot (Feeder station restock: collect full payloads)
        double stockpileUtility = 0.0;
        if (!world.isInventoryFull() && !world.isAllianceHubActive()) {
            stockpileUtility = 0.86 + (0.10 * (1.0 - world.heldFuelCount() / 30.0));
        }
        utilities.put(StrategicObjective.STOCKPILE_DEPOT, stockpileUtility);

        int homeFuelCount = countFuelInZone(world.isRedAlliance(), false, blockedFuel, knowledge);
        double sweepUtility = 0.0;
        if (homeFuelCount > 0 && !world.isInventoryFull()) {
            sweepUtility = world.isAllianceHubActive()
                    ? 0.96
                    : 0.90 + 0.08 * (1.0 - world.heldFuelCount() / 30.0);
        }
        utilities.put(StrategicObjective.SWEEP_ALLIANCE_ZONE, sweepUtility);

        // Poach only while a shift flip is genuinely approaching.
        //
        // The `> 0.0` floor is load-bearing and was missing. `HubSchedule
        // .timeUntilShiftEnd()` returns 0.0 by design during AUTO, TRANSITION,
        // ENDGAME and DONE, where no flip is coming - so a bare `<= 6.0` test
        // stayed true for the entire endgame. It was masked while the sim ran on
        // the frozen shift clock, because that clock is pinned at exactly 0.0
        // and the guard was therefore true everywhere; fixing the clock without
        // this bound would have shifted poach behaviour during shift phases and
        // read as "fixed" while the endgame case stayed wrong.
        double opponentZoneUtility = 0.0;
        double timeUntilShiftForPoach = world.timeUntilHubShift();
        if (!world.isAutonomous() && !world.isInventoryFull()
                && timeUntilShiftForPoach > 0.0 && timeUntilShiftForPoach <= 6.0
                // Only poach when the OPPONENT's hub is the one coming back on. In
                // SHIFT2/SHIFT4 our own hub is about to open and theirs is about to
                // close; sprinting across for their loose fuel then abandons our own
                // scoring window. This is a behaviour change on a strategy judgement,
                // not a proven defect -- same class as the STAGE_STANDOFF question.
                && world.isOpponentHubActiveAfterShift()
                && world.heldFuelCount() < 20
                && countFuelInZone(!world.isRedAlliance(), true, blockedFuel, knowledge) > 0) {
            opponentZoneUtility = 0.78;
        }
        utilities.put(StrategicObjective.POACH_OPPONENT_ZONE, opponentZoneUtility);

        // G407 and the robot's shooter safety policy allow launches only from our
        // alliance zone. Do not turn a midfield pass into an illegal launch.
        //
        // NOTE: the explicit opponent-hub check below is a NO-OP today, and is kept
        // deliberately rather than as a claimed fix. Both hubs are never dark at the
        // same time -- SHIFT1-4 have exactly one live, and AUTO/TRANSITION/ENDGAME
        // have both -- so `!isAllianceHubActive()` already implies the opponent's hub
        // is scoring. An earlier version of this comment claimed it prevented
        // "lobbing into a dead hub"; that scenario is unreachable. It becomes
        // load-bearing only if the schedule ever allows two dark hubs (DONE), so it
        // documents the invariant at the point where the aim target is chosen.
        double shuttleUtility = !world.isAutonomous() && !world.isAllianceHubActive()
                && (world.isOpponentHubActive() || world.isOpponentHubActiveAfterShift())
                && FieldMap.AllianceZones.isInAllianceZone(world.selfPose(), world.isRedAlliance())
                && !FieldMap.Trenches.isLowClearance(world.selfPose()) && distToSelfHub > 6.0
                && world.heldFuelCount() >= 16 ? 0.87 : 0.0;
        utilities.put(StrategicObjective.SHUTTLE_PASS, shuttleUtility);

        double longRangeUtility = 0.0;
        if (world.isAllianceHubActive() && world.heldFuelCount() >= 6
                && FieldMap.AllianceZones.isInAllianceZone(world.selfPose(), world.isRedAlliance())
                && distToSelfHub >= 3.6 && distToSelfHub <= FieldMap.Hubs.SHOOTING_MAX_DISTANCE) {
            longRangeUtility = opponentObserved
                    && world.opponentPose().getTranslation().getDistance(selfHub) <= 2.4 ? 0.94 : 0.86;
        }
        utilities.put(StrategicObjective.LONG_RANGE_SNIPE, longRangeUtility);

        // Defense Objectives
        double laneDenialUtility = 0.0;
        double shadowUtility = 0.0;
        double interceptUtility = 0.0;

        if (archetype == Archetype.TACTICAL_DEFENDER || archetype == Archetype.DEFENSE_BULLY
                || archetype == Archetype.ADAPTIVE_COMPETITOR) {
            Translation2d oppHub = FieldMap.Hubs.getHubLocation2d(!world.isRedAlliance());
            double oppDistToHub = world.opponentPose().getTranslation().getDistance(oppHub);

            if (world.isOpponentHubActive() && oppDistToHub < 6.5) {
                laneDenialUtility = 0.86;
            }
            shadowUtility = 0.65;
            interceptUtility = 0.75;
        }

        boolean defensiveArchetype = archetype == Archetype.TACTICAL_DEFENDER
                || archetype == Archetype.DEFENSE_BULLY || archetype == Archetype.ADAPTIVE_COMPETITOR;
        double chokeUtility = 0.0;
        // Gate on the occupied trench itself (X band AND low-clearance Y), not
        // the X band alone — an opponent driving midfield at trench X must not
        // pull a defender to a trench mouth.
        if (defensiveArchetype && opponentObserved && !world.isAutonomous()
                && FieldMap.Trenches.isLowClearance(world.opponentPose())) {
            chokeUtility = 0.91;
        }
        utilities.put(StrategicObjective.CHOKE_TRENCH, chokeUtility);

        double screenUtility = 0.0;
        if (opponentObserved && knowledge.alliesHeldFuel() >= 12 && world.isAllianceHubActive()) {
            outer: for (Pose2d ally : knowledge.allyPoses()) {
                for (Pose2d opponent : knowledge.opponentPoses()) {
                    if (ally.getTranslation().getDistance(opponent.getTranslation()) <= 2.2) {
                        screenUtility = 0.89;
                        break outer;
                    }
                }
            }
        }
        utilities.put(StrategicObjective.SCREEN_FOR_ALLY, screenUtility);

        // Pin duration is not present in WorldState/MatchKnowledge yet; keep this
        // objective unavailable until the simulator supplies an explicit referee
        // signal.
        utilities.put(StrategicObjective.BAIT_PIN_FOUL, 0.0);

        // Tier-1 driver assist: with no vision-tracked opponent, opponent-
        // chasing objectives are unavailable regardless of archetype.
        if (!opponentObserved) {
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
        }

        // Apply Archetype Multipliers
        if (world.isAutonomous()) {
            // FRC G201 centerline rule: in autonomous mode, robots must stay on their
            // alliance half.
            // Opponent interception, lane denial, and cross-field pinning are illegal in
            // auto.
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
            // Fill-then-volley: harvest until a full batch before committing to a
            // scoring trip. (Prevents one-ball-at-a-time auto: previously any held
            // fuel > 0 sent the bot straight to the hub.) Exceptions: already in
            // shooting range (finish the volley instead of driving away), or auto
            // clock nearly out (dump the hopper rather than carrying balls home).
            boolean batchReady = world.heldFuelCount() >= AUTO_BATCH_MIN_FUEL;
            boolean autoClockLow = world.matchTimeRemaining() >= 0.0
                    && world.matchTimeRemaining() <= AUTO_DUMP_SECONDS_LEFT;
            if (world.heldFuelCount() > 0 && (batchReady || inShootingRange || autoClockLow)) {
                scoreUtility = 0.95;
                vacuumUtility = 0.0;
                // Do not let the newly added home-zone sweep outrank the
                // established auto batch/clock dump decision.
                sweepUtility = 0.0;
            } else {
                scoreUtility = 0.0;
                vacuumUtility = 0.85;
            }
        } else if (archetype == Archetype.AUTONOMOUS_CYCLER) {
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
        } else if (archetype == Archetype.TACTICAL_DEFENDER) {
            scoreUtility = 0.0;
            stageUtility = 0.0;
            vacuumUtility *= 0.20;
            stockpileUtility *= 0.20;
            laneDenialUtility *= 1.15;
            shadowUtility *= 1.10;
        } else if (archetype == Archetype.LEAD_PURSUIT_INTERCEPTOR) {
            interceptUtility = 0.99;
            scoreUtility = 0.0;
            vacuumUtility = 0.0;
            stockpileUtility = 0.0;
        } else if (archetype == Archetype.DEFENSE_BULLY) {
            interceptUtility = 0.99;
            scoreUtility = 0.0;
        } else if (archetype == Archetype.CO_PILOT) {
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
            if (world.heldFuelCount() > 0 && world.isAllianceHubActive()) {
                scoreUtility = 0.98;
                vacuumUtility = 0.0;
            }
        }

        if (world.isAllianceHubActive() && timeLeftToHarvest <= 0.0 && world.heldFuelCount() >= 8) {
            scoreUtility = 0.98;
            vacuumUtility = 0.0;
            sweepUtility = 0.0;
        }

        utilities.put(StrategicObjective.CYCLE_SCORE_HUB, scoreUtility);
        utilities.put(StrategicObjective.STAGE_STANDOFF, stageUtility);
        utilities.put(StrategicObjective.VACUUM_MIDFIELD, vacuumUtility);
        utilities.put(StrategicObjective.STOCKPILE_DEPOT, stockpileUtility);
        utilities.put(StrategicObjective.DENY_SHOOTING_LANE, laneDenialUtility);
        utilities.put(StrategicObjective.SHADOW_MIDLINE, shadowUtility);
        utilities.put(StrategicObjective.LEAD_INTERCEPT, interceptUtility);
        utilities.put(StrategicObjective.SWEEP_ALLIANCE_ZONE, sweepUtility);
        utilities.put(StrategicObjective.POACH_OPPONENT_ZONE, opponentZoneUtility);
        utilities.put(StrategicObjective.SHUTTLE_PASS, shuttleUtility);
        utilities.put(StrategicObjective.LONG_RANGE_SNIPE, longRangeUtility);
        utilities.put(StrategicObjective.CHOKE_TRENCH, chokeUtility);
        utilities.put(StrategicObjective.SCREEN_FOR_ALLY, screenUtility);

        // ── 2. Select Highest Utility Objective ──────────────────────────────
        StrategicObjective bestObjective = evaluateLocalUtilityMatrix(utilities);

        // Diagnostic pin: frc.jev.freezeObjective=<NAME> forces every agent onto one
        // objective and bypasses the commitment latch entirely. Built to isolate
        // whether headless non-reproducibility came from the AI or from the physics
        // below it (see KNOWN_ISSUES.md §A). The test exonerated the AI: a
        // frozen-AI seed still varied run to run. Superseded as a reproducibility
        // fix by the Common Random Numbers work in Sim/MatchDeterminism, which cut
        // residual variance substantially; the pin is retained because it isolates
        // decision-layer behaviour specifically. Off unless the property is set;
        // never enable alongside frc.jev.realShiftClock.
        StrategicObjective frozen = frozenObjective();
        if (frozen != null) {
            bestObjective = frozen;
            Logger.recordOutput("JevAI/FrozenObjective", frozen.name());
        }

        // Commitment is applied by the caller (it owns the latch for this
        // agent) and handed back in via commitmentIn, so the engine keeps its
        // stateless contract and no two agents can share a decision.
        if (commitmentIn != null && frozen == null) {
            bestObjective = commitmentIn.apply(bestObjective, utilities);
        }
        double maxUtility = utilities.getOrDefault(bestObjective, 0.0);

        // Tier-1 safety net: opponent-chasing objectives require a tracked
        // opponent. If one ever wins without observation (e.g. a future
        // utility change), fall back to harvesting instead of acting on a
        // placeholder mark.
        if (!opponentObserved && (bestObjective == StrategicObjective.LEAD_INTERCEPT
                || bestObjective == StrategicObjective.DENY_SHOOTING_LANE
                || bestObjective == StrategicObjective.SHADOW_MIDLINE
                || bestObjective == StrategicObjective.CHOKE_TRENCH
                || bestObjective == StrategicObjective.SCREEN_FOR_ALLY)) {
            bestObjective = StrategicObjective.VACUUM_MIDFIELD;
            maxUtility = utilities.getOrDefault(bestObjective, 0.0);
        }

        TypeSafeJevClient.JevDecision cloudDecision = cloudConfigured
                ? cloudClient.getLatestDecision(cloudContext)
                : null;
        boolean usedCloud = false;
        double nowSeconds = System.nanoTime() / 1_000_000_000.0;
        if (cloudDecision != null
                && nowSeconds - cloudDecision.timestamp() >= 0.0
                && nowSeconds - cloudDecision.timestamp() <= CLOUD_DECISION_FRESHNESS_SEC
                && cloudDecision.confidence() >= MIN_CLOUD_CONFIDENCE
                && isCloudObjectiveAdmissible(
                        cloudDecision.objective(), utilities, world, knowledge, bestObjective, maxUtility)) {
            bestObjective = cloudDecision.objective();
            maxUtility = cloudDecision.confidence();
            usedCloud = true;
        }

        // ── 3. Resolve Concrete Tactical Action Intent ───────────────────────
        Pose2d navTarget;
        Rotation2d aimOverride = null;
        IntakeState intakeCmd = IntakeState.STANDBY;
        ShooterState shooterCmd = ShooterState.STOPPED;
        double targetRPM = 0.0;
        boolean triggerKicker = false;
        String rationale;
        StrategicObjective nextObjective = resolveNextObjective(bestObjective, world);
        double timeToTransitionSec = world.isAllianceHubActive() && world.heldFuelCount() >= 8
                ? Math.max(0.0, timeLeftToHarvest)
                : Math.max(0.0, world.timeUntilHubShift());

        switch (bestObjective) {
            case CYCLE_SCORE_HUB:
                navTarget = calculatePolarStandoffPose(world.selfPose(), world.isRedAlliance());
                intakeCmd = IntakeState.STANDBY;

                // Pre-spool flywheels during transit
                shooterCmd = ShooterState.PREPARING;
                targetRPM = 3200.0;

                // Fire evaluation if within shooting range
                boolean inRange = distToSelfHub <= 3.60 && distToSelfHub >= 1.40;
                boolean inAllianceZone = FieldMap.AllianceZones.isInAllianceZone(world.selfPose(),
                        world.isRedAlliance());
                boolean openCeiling = !FieldMap.Trenches.isLowClearance(world.selfPose());
                // Tier 1: without a tracked opponent the human judges the lane;
                // assume clear rather than gating the driver's shots on a guess.
                boolean laneClear = !opponentObserved
                        || !isShootingLaneBlocked(world.selfPose(), selfHub, world.opponentPose());

                if (inRange && inAllianceZone && openCeiling && laneClear) {
                    Rotation2d faceHub = selfHub.minus(world.selfPose().getTranslation()).getAngle();
                    aimOverride = faceHub;
                    double headingErr = Math.abs(world.selfPose().getRotation().minus(faceHub).getDegrees());
                    if (headingErr < 8.0) {
                        shooterCmd = ShooterState.SHOOTING;
                        triggerKicker = true;
                    }
                }
                rationale = String.format("Hub Active. Cycling %d fuel to Hub.", world.heldFuelCount());
                break;

            case STAGE_STANDOFF:
                navTarget = calculatePolarStandoffPose(world.selfPose(), world.isRedAlliance());
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = String.format("Hub Inactive (shifts in %.1fs). Staging with %d fuel.",
                        world.timeUntilHubShift(), world.heldFuelCount());
                break;

            case STOCKPILE_DEPOT:
                Translation2d depotApproach = FieldMap.Depots.getDepotApproach(world.isRedAlliance());
                Rotation2d faceWall = Rotation2d.fromDegrees(world.isRedAlliance() ? 0.0 : 180.0);
                navTarget = new Pose2d(depotApproach, faceWall);
                intakeCmd = IntakeState.INTAKING;
                shooterCmd = ShooterState.STOPPED;
                rationale = String.format("Stockpiling at Alliance Depot (%d/30).", world.heldFuelCount());
                break;

            case VACUUM_MIDFIELD:
                navTarget = findClusterWeightedFuelTarget(world.selfPose(), world.isRedAlliance(),
                        world.isAutonomous(), blockedFuel);
                intakeCmd = IntakeState.INTAKING;
                shooterCmd = ShooterState.STOPPED;
                rationale = String.format("Hunting fuel (%d/30). Hopper capacity available.", world.heldFuelCount());
                break;

            case SWEEP_ALLIANCE_ZONE:
                navTarget = findAllianceZoneFuelTarget(world.selfPose(), world.isRedAlliance(),
                        blockedFuel);
                intakeCmd = IntakeState.INTAKING;
                if (world.isAllianceHubActive()) {
                    shooterCmd = ShooterState.PREPARING;
                    targetRPM = 3200.0;
                }
                rationale = String.format("Sweeping %d loose fuel pieces in the alliance zone.", homeFuelCount);
                break;

            case LONG_RANGE_SNIPE:
                navTarget = world.selfPose();
                aimOverride = selfHub.minus(world.selfPose().getTranslation()).getAngle();
                shooterCmd = ShooterState.SHOOTING;
                targetRPM = 4000.0;
                triggerKicker = Math.abs(world.selfPose().getRotation().minus(aimOverride).getDegrees()) < 4.0;
                rationale = "Taking an outer-perimeter shot while the alliance Hub is active.";
                break;

            case SHUTTLE_PASS:
                navTarget = world.selfPose();
                Translation2d homeZoneCenter = new Translation2d(world.isRedAlliance() ? 14.2 : 2.3,
                        FieldMap.FIELD_WIDTH / 2.0);
                aimOverride = homeZoneCenter.minus(world.selfPose().getTranslation()).getAngle();
                shooterCmd = ShooterState.SHOOTING;
                targetRPM = 2400.0;
                triggerKicker = true;
                rationale = "Shuttling held fuel toward the home zone while the Hub is inactive.";
                break;

            case POACH_OPPONENT_ZONE:
                navTarget = findOpponentZoneFuelTarget(world.selfPose(), world.isRedAlliance(),
                        blockedFuel);
                intakeCmd = IntakeState.INTAKING;
                rationale = "Harvesting opponent-zone fuel before the next Hub shift.";
                break;

            case CHOKE_TRENCH:
                boolean opponentAtTop = world.opponentPose().getY() >= FieldMap.FIELD_WIDTH / 2.0;
                // Identify whether opponent is in the Red or Blue half:
                boolean opponentInRedTrench = world.opponentPose().getX() > FieldMap.CENTERLINE_X;
                // Stage at the occupied trench's midfield exit (east exit for
                // Blue, west exit for Red). Staging at the opponent's own X
                // would sit inside the Hub/ramp footprint — the whole
                // (4.5, 5.2–6.5) mouth band near the trench is hard obstacle.
                double exitX = opponentInRedTrench
                        ? FieldMap.Trenches.RED_TRENCH_MIN_X - 0.9
                        : FieldMap.Trenches.BLUE_TRENCH_MAX_X + 0.9;
                double corridorY = opponentAtTop
                        ? FieldMap.Trenches.TOP_CORRIDOR_Y
                        : FieldMap.Trenches.BOT_CORRIDOR_Y;
                Translation2d exitPos = new Translation2d(exitX, corridorY);
                Rotation2d faceOpponent = world.opponentPose().getTranslation().minus(exitPos).getAngle();
                navTarget = StaticPathfinder.ensurePoseOutsideObstacles(new Pose2d(exitPos, faceOpponent));
                rationale = "Contesting the occupied trench approach.";
                break;

            case SCREEN_FOR_ALLY:
                Pose2d screeningAlly = knowledge.allyPoses().get(0);
                Pose2d nearestDefender = knowledge.opponentPoses().stream()
                        .min((a, b) -> Double.compare(
                                a.getTranslation().getDistance(screeningAlly.getTranslation()),
                                b.getTranslation().getDistance(screeningAlly.getTranslation())))
                        .orElse(world.opponentPose());
                Translation2d screenMidpoint = screeningAlly.getTranslation()
                        .plus(nearestDefender.getTranslation()).div(2.0);
                navTarget = new Pose2d(screenMidpoint,
                        nearestDefender.getTranslation().minus(screenMidpoint).getAngle());
                rationale = "Screening the nearest defender away from a loaded ally.";
                break;

            case BAIT_PIN_FOUL:
                navTarget = world.selfPose();
                rationale = "Holding position during an observed pin.";
                break;

            case LEAD_INTERCEPT:
                navTarget = solveLeadPursuitIntercept(
                        world.selfPose(), world.opponentPose(),
                        new Translation2d(world.opponentVelocity().vxMetersPerSecond,
                                world.opponentVelocity().vyMetersPerSecond),
                        Constants.MAX_SPEED);
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Pursuing and intercepting opponent trajectory.";
                break;

            case DENY_SHOOTING_LANE:
                Translation2d oppGoal = FieldMap.Hubs.getHubLocation2d(!world.isRedAlliance());
                Translation2d toGoal = oppGoal.minus(world.opponentPose().getTranslation());
                double distToGoal = toGoal.getNorm();
                Translation2d dir = (distToGoal > 1e-3) ? toGoal.div(distToGoal) : new Translation2d(1, 0);

                // Block from the shooter-to-Hub line, but never stage inside the
                // Hub/ramp safety shell (1.6 m) plus bumper margin — pressing
                // into the footprint is what wedged defenders against it.
                double minHubClearance = 1.60 + 0.45;
                double maxBlockDist = distToGoal - minHubClearance;
                Translation2d blockPos;
                if (maxBlockDist >= 0.5) {
                    double blockDist = Math.min(1.5, Math.max(0.5, maxBlockDist));
                    blockPos = world.opponentPose().getTranslation().plus(dir.times(blockDist));
                } else {
                    // Shooter is already inside the shell: stage off to the side
                    // of the shooter (toward field center) instead of between
                    // shooter and Hub.
                    Translation2d perp = new Translation2d(-dir.getY(), dir.getX());
                    double side = world.opponentPose().getY() >= FieldMap.FIELD_WIDTH / 2.0 ? -1.0 : 1.0;
                    blockPos = world.opponentPose().getTranslation().plus(perp.times(1.5 * side));
                }
                Rotation2d faceOpp = world.opponentPose().getTranslation().minus(blockPos).getAngle();
                navTarget = StaticPathfinder.ensurePoseOutsideObstacles(new Pose2d(blockPos, faceOpp));
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Blocking opponent shooting corridor.";
                break;

            case SHADOW_MIDLINE:
                double shadowX = world.isRedAlliance() ? (CENTERLINE_X + 0.8) : (CENTERLINE_X - 0.8);
                double clampedY = Math.max(1.0, Math.min(FieldMap.FIELD_WIDTH - 1.0, world.opponentPose().getY()));
                Rotation2d face = world.opponentPose().getTranslation().minus(new Translation2d(shadowX, clampedY))
                        .getAngle();
                navTarget = new Pose2d(shadowX, clampedY, face);
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Shadowing opponent across field midline.";
                break;

            case RUSH_CLIMB:
                if (archetype != Archetype.CO_PILOT) {
                    // Belt-and-suspenders: RUSH_CLIMB is inserted first in the utility
                    // map, so a strict-greater max-selection would hand it an all-zero
                    // tie. Bots must never navigate to the tower (they would score
                    // phantom climb points), so hold position instead.
                    navTarget = world.selfPose();
                    intakeCmd = IntakeState.STANDBY;
                    shooterCmd = ShooterState.STOPPED;
                    rationale = "No climber fitted. Holding position instead of climbing.";
                    break;
                }
                Translation2d pole = FieldMap.ClimbingTowers.getTowerPole(world.isRedAlliance());
                Pose2d poleStandoff = new Pose2d(
                        pole.plus(new Translation2d(world.isRedAlliance() ? -0.8 : 0.8, 0.0)),
                        Rotation2d.fromDegrees(world.isRedAlliance() ? 0 : 180));
                navTarget = glidePose("Blue Right Side Climb", world.isRedAlliance(), poleStandoff);
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = String.format("Endgame (%.1fs remaining). Navigating to Alliance Parking.",
                        world.matchTimeRemaining());
                break;

            case IDLE:
            default:
                navTarget = world.selfPose();
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Idle.";
                break;
        }

        boolean isLowClearance = FieldMap.Trenches.isLowClearance(world.selfPose());
        boolean hasHopperSpace = world.heldFuelCount() < WorldState.DEFAULT_MAX_CAPACITY;
        if ((bestObjective.isDefensive() || bestObjective == StrategicObjective.STAGE_STANDOFF)
                && !isLowClearance && hasHopperSpace) {
            intakeCmd = IntakeState.INTAKING;
        }

        if (usedCloud) {
            rationale = "TypeSafe Jev selected " + bestObjective.name() + " (confidence "
                    + String.format("%.2f", maxUtility) + "). " + rationale;
        }

        double latencyMs = (System.nanoTime() - startNanos) / 1_000_000.0;
        Logger.recordOutput("JevAI/ActiveObjective", bestObjective.name());
        Logger.recordOutput("JevAI/Confidence", maxUtility);
        Logger.recordOutput("JevAI/Rationale", rationale);
        Logger.recordOutput("JevAI/PolicyLatencyMs", latencyMs);
        String telemetryPrefix = cloudContext == null ? "JevAI" : "JevAI/" + cloudContext;
        Logger.recordOutput(telemetryPrefix + "/UsingCloudAI", usedCloud);
        Logger.recordOutput(telemetryPrefix + "/ActiveDecisionMode", activeDecisionMode.name());
        SmartDashboard.putBoolean(telemetryPrefix + "/UsingCloudAI", usedCloud);
        SmartDashboard.putString(telemetryPrefix + "/ActiveDecisionMode", activeDecisionMode.name());
        if (cloudDecision != null) {
            Logger.recordOutput(telemetryPrefix + "/CloudConfidence", cloudDecision.confidence());
            Logger.recordOutput(telemetryPrefix + "/CloudLatencyMs", cloudDecision.latencyMs());
            Logger.recordOutput(telemetryPrefix + "/CloudProbabilities",
                    cloudDecision.probabilityDistribution().toString());
            SmartDashboard.putNumber(telemetryPrefix + "/CloudConfidence", cloudDecision.confidence());
            SmartDashboard.putNumber(telemetryPrefix + "/CloudLatencyMs", cloudDecision.latencyMs());
            SmartDashboard.putString(telemetryPrefix + "/CloudProbabilities",
                    cloudDecision.probabilityDistribution().toString());
        }
        double opponentThreat = cloudDecision == null ? 0.0 : cloudDecision.opponentThreatProbability();
        Logger.recordOutput(telemetryPrefix + "/OpponentThreatProbability", opponentThreat);
        SmartDashboard.putNumber(telemetryPrefix + "/OpponentThreatProbability", opponentThreat);
        String cloudError = cloudClient.getLastError(cloudContext);
        if (!cloudError.equals(lastPublishedCloudErrors.get(telemetryPrefix))) {
            lastPublishedCloudErrors.put(telemetryPrefix, cloudError);
            Logger.recordOutput(telemetryPrefix + "/CloudError", cloudError);
            SmartDashboard.putString(telemetryPrefix + "/CloudError", cloudError);
        }

        StrategicPlan plan = new StrategicPlan(bestObjective, nextObjective, timeToTransitionSec);
        return new AIActionIntent(
                bestObjective, navTarget, aimOverride, intakeCmd, shooterCmd, targetRPM, triggerKicker, maxUtility,
                rationale, plan);
    }

    private DecisionMode resolveDecisionMode() {
        if (!Dashboard.isUseTypeSafeJevEnabled()) {
            decisionMode = DecisionMode.LOCAL_HEURISTIC;
            return DecisionMode.LOCAL_HEURISTIC;
        }
        String requestedMode = Dashboard.getJevDecisionModeName();
        try {
            decisionMode = DecisionMode.valueOf(requestedMode);
        } catch (IllegalArgumentException | NullPointerException exception) {
            Dashboard.setJevDecisionModeName(decisionMode.name());
        }
        return decisionMode;
    }

    private static StrategicObjective evaluateLocalUtilityMatrix(Map<StrategicObjective, Double> utilities) {
        StrategicObjective bestObjective = StrategicObjective.VACUUM_MIDFIELD;
        double bestUtility = -1.0;
        for (Map.Entry<StrategicObjective, Double> entry : utilities.entrySet()) {
            if (entry.getValue() > bestUtility) {
                bestUtility = entry.getValue();
                bestObjective = entry.getKey();
            }
        }
        return bestObjective;
    }

    static boolean isCloudObjectiveAdmissible(
            StrategicObjective objective,
            Map<StrategicObjective, Double> utilities,
            WorldState world,
            MatchKnowledge knowledge,
            StrategicObjective localObjective,
            double localUtility) {
        if (objective == null || !isTypeSafeChoice(objective)
                || utilities.getOrDefault(objective, 0.0) <= 0.0) {
            return false;
        }
        if (localUtility >= LOCAL_PRIORITY_OVERRIDE_THRESHOLD && objective != localObjective) {
            return false;
        }
        if (objective.isDefensive() && !knowledge.opponentObserved()) {
            return false;
        }
        if (world.isAutonomous()) {
            // Keep all remotely selected autonomous targets on the robot's own
            // half. Defensive and opponent-zone objectives are never accepted.
            return objective == StrategicObjective.CYCLE_SCORE_HUB
                    || objective == StrategicObjective.STAGE_STANDOFF
                    || objective == StrategicObjective.VACUUM_MIDFIELD
                    || objective == StrategicObjective.STOCKPILE_DEPOT
                    || objective == StrategicObjective.SWEEP_ALLIANCE_ZONE;
        }
        return true;
    }

    private static boolean isTypeSafeChoice(StrategicObjective objective) {
        return objective == StrategicObjective.CYCLE_SCORE_HUB
                || objective == StrategicObjective.STAGE_STANDOFF
                || objective == StrategicObjective.VACUUM_MIDFIELD
                || objective == StrategicObjective.STOCKPILE_DEPOT
                || objective == StrategicObjective.SWEEP_ALLIANCE_ZONE
                || objective == StrategicObjective.DENY_SHOOTING_LANE
                || objective == StrategicObjective.LEAD_INTERCEPT
                || objective == StrategicObjective.RUSH_CLIMB;
    }

    /**
     * Backward-compatible overload accepting legacy AIArchetype.
     */
    public AIActionIntent evaluatePolicy(WorldState world, AIArchetype archetype) {
        return evaluatePolicy(world, Archetype.fromString(archetype.name()));
    }

    /**
     * Spatial cluster-density piece scent algorithm (Option B).
     * Replaces single closest ball search with a Gaussian density field evaluator
     * that targets
     * dense rows/clusters of 3-6 pieces (depot lines, centerline grid) in
     * continuous sweeps.
     *
     * @param robotPose     Current pose of the robot seeking fuel
     * @param isRedAlliance True if robot is on Red Alliance
     * @return Target Pose2d on carpet facing the highest-density fuel cluster
     */
    public Pose2d findClusterWeightedFuelTarget(Pose2d robotPose, boolean isRedAlliance) {
        return findClusterWeightedFuelTarget(robotPose, isRedAlliance, false, null);
    }

    public Pose2d findClusterWeightedFuelTarget(Pose2d robotPose, boolean isRedAlliance, boolean isAutonomous) {
        return findClusterWeightedFuelTarget(robotPose, isRedAlliance, isAutonomous, null);
    }

    /**
     * Cluster-weighted fuel selection that skips caller-abandoned points.
     *
     * @param blockedFuel points to skip (see
     *        {@link #evaluatePolicy(WorldState, MatchKnowledge, Archetype, String, Set)}),
     *        or {@code null}
     */
    public Pose2d findClusterWeightedFuelTarget(Pose2d robotPose, boolean isRedAlliance,
            boolean isAutonomous, Set<Translation2d> blockedFuel) {
        SimulatedArena arena = SimulatedArena.getInstance();
        Translation2d bestTarget = null;
        double highestScent = -1.0;

        List<Translation2d> candidates = new ArrayList<>();

        if (arena != null) {
            try {
                // Sorted snapshot: the arena returns a HashSet whose order is
                // identity-hash based, and equal-scoring candidates resolve by
                // strict '>', so unsorted iteration picked a different target
                // every JVM run.
                var pieces = frc.robot.Sim.MatchDeterminism.fuelOnFieldSorted();
                if (pieces != null && !pieces.isEmpty()) {
                    for (var piece : pieces) {
                        if (piece == null || !"Fuel".equals(piece.getType()))
                            continue;
                        Translation2d pos = piece.getPoseOnField().getTranslation();

                        // Field boundaries & obstacle avoidance (wall-band balls stay
                        // eligible: the wall-normal approach below reaches them)
                        if (pos.getX() < 0.05 || pos.getX() > 16.48 || pos.getY() < 0.05 || pos.getY() > 8.00)
                            continue;
                        if (StaticPathfinder.isPointInHardObstacle(pos)
                                || StaticPathfinder.isPointNearDynamicObstacle(pos))
                            continue;
                        if (isBlocked(blockedFuel, pos))
                            continue;

                        // Restrict opposing driver wall zone (and centerline in autonomous under FRC
                        // G201)
                        if (isAutonomous) {
                            if (isRedAlliance && pos.getX() < FieldMap.CENTERLINE_X + 0.15)
                                continue;
                            if (!isRedAlliance && pos.getX() > FieldMap.CENTERLINE_X - 0.15)
                                continue;
                        } else {
                            if (isRedAlliance && pos.getX() < 3.5)
                                continue;
                            if (!isRedAlliance && pos.getX() > 13.0)
                                continue;
                        }

                        candidates.add(pos);
                    }
                }
            } catch (Exception ignored) {
            }
        }

        if (!candidates.isEmpty()) {
            final double clusterRadius = 1.30;
            final double twoSigmaSq = 2.0 * 0.50 * 0.50; // sigma = 0.50m

            for (int i = 0; i < candidates.size(); i++) {
                Translation2d cand = candidates.get(i);
                double density = 1.0;

                for (int j = 0; j < candidates.size(); j++) {
                    if (i == j)
                        continue;
                    double d = cand.getDistance(candidates.get(j));
                    if (d <= clusterRadius) {
                        density += Math.exp(-(d * d) / twoSigmaSq);
                    }
                }

                double dist = robotPose.getTranslation().getDistance(cand);
                Translation2d delta = cand.minus(robotPose.getTranslation());
                double angleDiff = Math.abs(robotPose.getRotation().minus(delta.getAngle()).getRadians());
                double alignBonus = 0.70 + 0.30 * Math.max(0.0, Math.cos(angleDiff));
                Translation2d homeCenter = new Translation2d(isRedAlliance ? 14.2 : 2.3,
                        FieldMap.FIELD_WIDTH / 2.0);
                Translation2d toHome = homeCenter.minus(cand);
                Translation2d travel = delta;
                double directionBonus = 0.0;
                if (toHome.getNorm() > 1e-9 && travel.getNorm() > 1e-9) {
                    directionBonus = 0.35 * Math.max(0.0,
                            (travel.div(travel.getNorm())).dot(toHome.div(toHome.getNorm())));
                }

                double scent = (Math.pow(density, 1.5) / (dist + 0.40)) * (alignBonus + directionBonus);

                if (scent > highestScent) {
                    highestScent = scent;
                    bestTarget = cand;
                }
            }
        }

        if (bestTarget != null) {
            return StaticPathfinder.wallStandoffApproach(bestTarget, robotPose.getTranslation());
        }

        // Fallback: Midline patrol (buffered away from centerline during autonomous
        // under FRC G201)
        double midX = isAutonomous
                ? (isRedAlliance ? CENTERLINE_X + 0.60 : CENTERLINE_X - 0.60)
                : CENTERLINE_X;
        double midY = (robotPose.getY() > 4.0) ? 5.80 : 2.40;
        return StaticPathfinder.ensurePoseOutsideObstacles(
                new Pose2d(midX, midY, Rotation2d.fromDegrees(isRedAlliance ? 180 : 0)), robotPose.getTranslation());
    }

    /**
     * Selects the densest reachable Fuel cluster strictly inside our alliance zone.
     */
    public Pose2d findAllianceZoneFuelTarget(Pose2d robotPose, boolean isRedAlliance) {
        return findFuelTargetInZone(robotPose, isRedAlliance, false, null);
    }

    /** Zone-limited variant that skips caller-abandoned points. */
    public Pose2d findAllianceZoneFuelTarget(
            Pose2d robotPose, boolean isRedAlliance, Set<Translation2d> blockedFuel) {
        return findFuelTargetInZone(robotPose, isRedAlliance, false, blockedFuel);
    }

    /**
     * Selects the densest reachable Fuel cluster strictly inside the opponent zone.
     */
    public Pose2d findOpponentZoneFuelTarget(Pose2d robotPose, boolean isRedAlliance) {
        return findFuelTargetInZone(robotPose, !isRedAlliance, true, null);
    }

    /** Opponent-zone variant that skips caller-abandoned points. */
    public Pose2d findOpponentZoneFuelTarget(
            Pose2d robotPose, boolean isRedAlliance, Set<Translation2d> blockedFuel) {
        return findFuelTargetInZone(robotPose, !isRedAlliance, true, blockedFuel);
    }

    /** True when {@code point} sits within the blocked radius of any entry. */
    private static boolean isBlocked(Set<Translation2d> blockedFuel, Translation2d point) {
        if (blockedFuel == null || blockedFuel.isEmpty()) {
            return false;
        }
        for (Translation2d blocked : blockedFuel) {
            if (blocked != null
                    && blocked.getDistance(point) <= TargetProgressWatchdog.BLACKLIST_RADIUS_M) {
                return true;
            }
        }
        return false;
    }

    private Pose2d findFuelTargetInZone(Pose2d robotPose, boolean zoneIsRed, boolean strictOpponentZone,
            Set<Translation2d> blockedFuel) {
        SimulatedArena arena = SimulatedArena.getInstance();
        Translation2d best = null;
        double bestScore = -1.0;
        if (arena != null) {
            try {
                List<Translation2d> candidates = new ArrayList<>();
                // Sorted snapshot: equal-scoring candidates resolve by strict
                // '>', so the arena's HashSet order decided the target.
                var pieces = frc.robot.Sim.MatchDeterminism.fuelOnFieldSorted();
                if (pieces != null) {
                    for (GamePieceOnFieldSimulation piece : pieces) {
                        if (piece == null || !"Fuel".equals(piece.getType()))
                            continue;
                        Translation2d point = piece.getPoseOnField().getTranslation();
                        if (!FieldMap.AllianceZones.isInAllianceZone(point, zoneIsRed))
                            continue;
                        if (strictOpponentZone && (zoneIsRed ? point.getX() < 12.0 : point.getX() > 4.5))
                            continue;
                        if (StaticPathfinder.isPointInHardObstacle(point)
                                || StaticPathfinder.isPointNearDynamicObstacle(point))
                            continue;
                        if (isBlocked(blockedFuel, point))
                            continue;
                        candidates.add(point);
                    }
                }
                for (Translation2d candidate : candidates) {
                    double density = 1.0;
                    for (Translation2d neighbor : candidates) {
                        double distance = candidate.getDistance(neighbor);
                        if (distance > 1e-9 && distance <= 1.3) {
                            density += Math.exp(-(distance * distance) / 0.5);
                        }
                    }
                    double distance = robotPose.getTranslation().getDistance(candidate);
                    double score = Math.pow(density, 1.5) / (distance + 0.4);
                    if (score > bestScore) {
                        bestScore = score;
                        best = candidate;
                    }
                }
            } catch (Exception ignored) {
                // The real robot has no SimulatedArena; callers receive the safe fallback
                // below.
            }
        }
        if (best != null) {
            return StaticPathfinder.wallStandoffApproach(best, robotPose.getTranslation());
        }
        Translation2d fallback = FieldMap.AllianceZones.isInAllianceZone(robotPose, zoneIsRed)
                ? robotPose.getTranslation()
                : new Translation2d(zoneIsRed ? FieldMap.FIELD_LENGTH - 2.3 : 2.3,
                        FieldMap.FIELD_WIDTH / 2.0);
        return StaticPathfinder.ensurePoseOutsideObstacles(
                new Pose2d(fallback, new Rotation2d()), robotPose.getTranslation());
    }

    /**
     * Fuel available in a zone, as reported by the knowledge type rather than by
     * reaching into the simulation.
     *
     * <p>This used to iterate {@code SimulatedArena} directly behind a
     * {@code catch (Exception) { return 0; }}, which broke the knowledge contract
     * three ways: the old "unobserved" tier still read perfect sim data behind a
     * record claiming ignorance; the real robot always took the {@code return 0}
     * fallback with no way to distinguish "no fuel there" from "I cannot see any";
     * and a silent 0 is a real number the policy will act on. Zone counts are now
     * computed once per tick by {@code WorldStateBuilder} and carried on
     * {@link MatchKnowledge}, so {@link ObservedKnowledge} reports 0 because that
     * is genuinely what a real robot can know.
     *
     * <p>Zone selection is by <b>role relative to this bot</b>, not by alliance:
     * {@code knowledge} is already the bot's own field picture, so
     * {@code allianceZoneFuel()} is our own zone whichever alliance we skate for.
     * An earlier version keyed this on {@code isRedZone}, which is correct for
     * {@code isInAllianceZone(translation, isRedZone)} but wrong here &mdash; it
     * sent a Blue bot's home-zone count to the midfield bucket. The parameter is
     * retained only because {@link #countBlockedInZone} still needs it to place an
     * abandoned piece in the same zone the raw count covers.
     *
     * @param isRedZone         which alliance the bot skates for, for blocked-piece
     *                          zone placement only
     * @param strictOpponentZone count the opponent's zone rather than our own
     * @param blockedFuel       fuel the watchdog abandoned, excluded from the count
     */
    private int countFuelInZone(boolean isRedZone, boolean strictOpponentZone,
            Set<Translation2d> blockedFuel, MatchKnowledge knowledge) {
        int available = strictOpponentZone
                ? knowledge.opponentZoneFuel()
                : knowledge.allianceZoneFuel();
        if (available <= 0) {
            return 0;
        }
        // Abandoned fuel must not keep an objective viable: the watchdog blacklists
        // a point it can no longer make progress toward, and the selectors skip it
        // via isBlocked. Counting it here would leave SWEEP_ALLIANCE_ZONE at
        // 0.90-0.98 on pieces the policy is simultaneously forbidden to approach,
        // so sweepUtility never collapses, ObjectiveCommitment's release rule
        // (incumbentUtility <= 0) never fires, and the latch holds all match. That
        // was the root cause of the seed-dependent teleop collapse.
        return Math.max(0, available - countBlockedInZone(isRedZone, strictOpponentZone, blockedFuel));
    }

    /**
     * How many pieces the watchdog has abandoned in the zone being counted.
     *
     * <p>Still reads the arena, deliberately: the blocked set is a list of points
     * and the total is a count, but the only place that knows a piece's zone is the
     * piece list itself. Returns 0 when no arena is available (real hardware), which
     * is correct &mdash; an {@link ObservedKnowledge} reports 0 total fuel anyway, so
     * the subtraction is never reached there.
     */
    private int countBlockedInZone(boolean isRedZone, boolean strictOpponentZone,
            Set<Translation2d> blockedFuel) {
        if (blockedFuel == null || blockedFuel.isEmpty()) {
            return 0;
        }
        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena == null) {
            return 0;
        }
        try {
            int blocked = 0;
            for (GamePieceOnFieldSimulation piece :
                    frc.robot.Sim.MatchDeterminism.fuelOnFieldSorted()) {
                if (piece == null || blockedFuel.isEmpty() || !"Fuel".equals(piece.getType())) {
                    continue;
                }
                var at = piece.getPoseOnField().getTranslation();
                // Same zone the raw count covered: our own alliance zone normally,
                // the deep opponent band when counting for POACH.
                boolean inZone = strictOpponentZone
                        ? (isRedZone ? at.getX() >= 12.0 : at.getX() <= 4.5)
                        : FieldMap.AllianceZones.isInAllianceZone(at, isRedZone);
                if (inZone && isBlocked(blockedFuel, at)) {
                    blocked++;
                }
            }
            return blocked;
        } catch (Exception e) {
            // Over-reporting blocked fuel would suppress a viable objective, so on
            // failure subtract nothing and keep the raw count.
            return 0;
        }
    }

    private StrategicObjective resolveNextObjective(StrategicObjective current, WorldState world) {
        return switch (current) {
            case SWEEP_ALLIANCE_ZONE -> world.isAllianceHubActive()
                    ? StrategicObjective.CYCLE_SCORE_HUB
                    : StrategicObjective.STAGE_STANDOFF;
            case VACUUM_MIDFIELD -> StrategicObjective.CYCLE_SCORE_HUB;
            case STOCKPILE_DEPOT, STAGE_STANDOFF -> StrategicObjective.CYCLE_SCORE_HUB;
            case POACH_OPPONENT_ZONE -> StrategicObjective.CYCLE_SCORE_HUB;
            case LONG_RANGE_SNIPE, SHUTTLE_PASS -> StrategicObjective.CYCLE_SCORE_HUB;
            case CHOKE_TRENCH, SCREEN_FOR_ALLY, BAIT_PIN_FOUL -> StrategicObjective.LEAD_INTERCEPT;
            default -> current;
        };
    }

    private boolean isShootingLaneBlocked(Pose2d shooterPose, Translation2d targetHub, Pose2d opponentPose) {
        Translation2d botPos = shooterPose.getTranslation();
        Translation2d toHub = targetHub.minus(botPos);
        Translation2d toOpp = opponentPose.getTranslation().minus(botPos);

        double dist = toOpp.getNorm();
        if (dist < 1.60 && dist > 0.05) {
            double angleDiff = Math.abs(toHub.getAngle().minus(toOpp.getAngle()).getRadians());
            return angleDiff < Math.toRadians(25.0);
        }
        return false;
    }

    // =========================================================================
    // LEGACY & SPECIALIZED UTILITY METHODS (Full Backwards Compatibility)
    // =========================================================================

    public DecisionResult evaluate(
            Pose2d playerPose,
            Pose2d opponentPose,
            double matchTimeRemaining,
            boolean isHubActive,
            boolean isRedAlliance) {

        long startNanos = System.nanoTime();
        double now = Timer.getFPGATimestamp();

        Translation2d targetHub = isRedAlliance
                ? new Translation2d(Constants.RED_HUB_LOCATION.getX(), Constants.RED_HUB_LOCATION.getY())
                : BLUE_HUB_POS;
        Translation2d targetDepot = AllianceFlipUtil.apply(BLUE_DEPOT_POS, isRedAlliance);

        double distToHub = playerPose.getTranslation().getDistance(targetHub);
        double distToDepot = playerPose.getTranslation().getDistance(targetDepot);

        Map<TacticalAction, Double> scores = new LinkedHashMap<>();

        double blockScore = 0.20;
        if (isHubActive && distToHub < 6.5) {
            double proximityWeight = Math.max(0.0, 1.0 - (distToHub / 6.5));
            blockScore = 0.65 + (0.30 * proximityWeight);
        } else if (!isHubActive) {
            blockScore = 0.10;
        }
        scores.put(TacticalAction.BLOCK_SHOOTING_LANE, blockScore);

        double depotScore = 0.25;
        if (distToDepot < 4.0) {
            double depotWeight = Math.max(0.0, 1.0 - (distToDepot / 4.0));
            depotScore = 0.60 + (0.35 * depotWeight);
        }
        scores.put(TacticalAction.CONTEST_DEPOT, depotScore);

        double shadowScore = isHubActive ? 0.50 : 0.20;
        double midfieldDist = Math.abs(playerPose.getX() - CENTERLINE_X);
        if (isHubActive && midfieldDist < 3.0) {
            shadowScore = 0.75 + (0.15 * (1.0 - midfieldDist / 3.0));
        }
        scores.put(TacticalAction.SHADOW_PLAYER, shadowScore);

        double retreatScore = 0.15;
        if (!isHubActive) {
            retreatScore = 0.85;
        }
        if (!DriverStation.isAutonomous() && matchTimeRemaining < 15.0) {
            retreatScore = Math.max(retreatScore, 0.80);
        }
        scores.put(TacticalAction.RETREAT_DEFENSE, retreatScore);

        TacticalAction selected = TacticalAction.SHADOW_PLAYER;
        double maxScore = -1.0;
        for (Map.Entry<TacticalAction, Double> entry : scores.entrySet()) {
            if (entry.getValue() > maxScore) {
                maxScore = entry.getValue();
                selected = entry.getKey();
            }
        }

        Pose2d targetPose;
        String rationale;

        switch (selected) {
            case BLOCK_SHOOTING_LANE:
                Translation2d dirToHub = targetHub.minus(playerPose.getTranslation());
                double norm = dirToHub.getNorm();
                Translation2d unitDir = norm > 1e-4 ? dirToHub.times(1.0 / norm) : new Translation2d(1, 0);
                Translation2d blockPos = playerPose.getTranslation().plus(unitDir.times(1.5));
                Rotation2d angleToPlayer = playerPose.getTranslation().minus(blockPos).getAngle();
                targetPose = new Pose2d(blockPos, angleToPlayer);
                rationale = String.format("Player is %.2fm from active Hub; blocking shooting corridor.", distToHub);
                break;

            case CONTEST_DEPOT:
                Translation2d contestPos = targetDepot.plus(new Translation2d(0.8, -0.5));
                Rotation2d angleFacingDepot = targetDepot.minus(contestPos).getAngle();
                targetPose = new Pose2d(contestPos, angleFacingDepot);
                rationale = String.format("Player approaching Depot (%.2fm away); contesting game piece loading.",
                        distToDepot);
                break;

            case RETREAT_DEFENSE:
                boolean opponentIsRed = !isRedAlliance;
                double retreatX = opponentIsRed ? (AllianceFlipUtil.FIELD_LENGTH - RETREAT_X) : RETREAT_X;
                targetPose = new Pose2d(retreatX, 4.0, Rotation2d.fromDegrees(opponentIsRed ? 180 : 0));
                rationale = isHubActive ? "Falling back to alliance defense perimeter."
                        : "Hub inactive; holding defensive position.";
                break;

            case SHADOW_PLAYER:
            default:
                boolean oppIsRed = !isRedAlliance;
                double sX = oppIsRed ? (CENTERLINE_X + 0.8) : (CENTERLINE_X - 0.8);
                double clampedY = Math.max(1.0, Math.min(AllianceFlipUtil.FIELD_WIDTH - 1.0, playerPose.getY()));
                Rotation2d facePlayer = playerPose.getTranslation().minus(new Translation2d(sX, clampedY)).getAngle();
                targetPose = new Pose2d(sX, clampedY, facePlayer);
                rationale = String.format("Shadowing player transition at midline (Y=%.2fm).", clampedY);
                break;
        }

        double latencyMs = (System.nanoTime() - startNanos) / 1_000_000.0;
        DecisionResult result = new DecisionResult(
                selected, targetPose, maxScore, scores, rationale, latencyMs, now);

        this.lastDecision = result;
        publishTelemetry(result);
        return result;
    }

    private void publishTelemetry(DecisionResult result) {
        SmartDashboard.putString("JevAI/SelectedAction", result.action.name());
        SmartDashboard.putNumber("JevAI/Confidence", result.confidence);
        SmartDashboard.putNumber("JevAI/LatencyMs", result.latencyMs);
        SmartDashboard.putString("JevAI/Rationale", result.rationale);
        SmartDashboard.putNumberArray("JevAI/TargetPose", new double[] {
                result.targetPose.getX(),
                result.targetPose.getY(),
                result.targetPose.getRotation().getDegrees()
        });
        SmartDashboard.putString("JevAI/TelemetryJson", result.toSchemaJson());
    }

    public DecisionResult getLastDecision() {
        return lastDecision;
    }

    public enum OffensiveStrategy {
        SCORE_HUB_HIGH("Score in Active Hub"),
        FEED_DEPOT("Intake at Alliance Depot"),
        BALL_HUNT_MIDFIELD("Hunt Neutral Game Pieces"),
        DEFEND_TRANSITION("Guard Midfield During Hub Shift");

        public final String description;

        OffensiveStrategy(String desc) {
            this.description = desc;
        }
    }

    public static class StrategicAdvice {
        public final OffensiveStrategy strategy;
        public final double utility;
        public final String adviceText;
        public final Pose2d suggestedWaypoint;

        public StrategicAdvice(
                OffensiveStrategy strategy,
                double utility,
                String adviceText,
                Pose2d suggestedWaypoint) {
            this.strategy = strategy;
            this.utility = utility;
            this.adviceText = adviceText;
            this.suggestedWaypoint = suggestedWaypoint;
        }
    }

    public StrategicAdvice evaluateOffensiveStrategy(
            Pose2d robotPose,
            double matchTimeRemaining,
            boolean isHubActive,
            boolean isRedAlliance) {
        Translation2d targetHub = isRedAlliance
                ? new Translation2d(Constants.RED_HUB_LOCATION.getX(), Constants.RED_HUB_LOCATION.getY())
                : BLUE_HUB_POS;
        Translation2d targetDepot = AllianceFlipUtil.apply(BLUE_DEPOT_POS, isRedAlliance);

        double distToHub = robotPose.getTranslation().getDistance(targetHub);
        double distToDepot = robotPose.getTranslation().getDistance(targetDepot);

        OffensiveStrategy strategy;
        double utility;
        String advice;
        Pose2d waypoint;

        if (isHubActive && distToHub < 6.5) {
            strategy = OffensiveStrategy.SCORE_HUB_HIGH;
            utility = 0.90;
            advice = String.format("Hub active (%.1fm). Execute high scoring cycle.", distToHub);
            waypoint = new Pose2d(targetHub.plus(new Translation2d(isRedAlliance ? 2.5 : -2.5, 0.0)),
                    Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
        } else if (!isHubActive) {
            if (distToDepot < 5.0) {
                strategy = OffensiveStrategy.FEED_DEPOT;
                utility = 0.85;
                advice = "Hub shifted inactive. Cycle inventory at Depot.";
                waypoint = new Pose2d(targetDepot, Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
            } else {
                strategy = OffensiveStrategy.DEFEND_TRANSITION;
                utility = 0.75;
                advice = "Hub shifted inactive. Guard midfield transition lane.";
                double midX = isRedAlliance ? (AllianceFlipUtil.FIELD_LENGTH - CENTERLINE_X) : CENTERLINE_X;
                waypoint = new Pose2d(midX, robotPose.getY(), Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
            }
        } else {
            strategy = OffensiveStrategy.BALL_HUNT_MIDFIELD;
            utility = 0.70;
            advice = "Acquiring neutral field game pieces.";
            waypoint = new Pose2d(CENTERLINE_X, 4.0, Rotation2d.fromDegrees(0));
        }

        SmartDashboard.putString("Strategy/Recommendation", strategy.description);
        SmartDashboard.putString("Strategy/AdviceText", advice);
        SmartDashboard.putNumber("Strategy/Utility", utility);

        return new StrategicAdvice(strategy, utility, advice, waypoint);
    }

    /**
     * Resolves a GlidePoint by its canonical Blue name and mirrors it for Red.
     *
     * <p>Blue-origin only: there is deliberately no parallel hardcoded Red fallback pose
     * here. The old {@code isRedAlliance ? 15.48 : 1.05} style literals were a second,
     * hand-maintained copy of data that already lives in {@link GlidePoints}, and they
     * had already drifted (the park literal still said Y=2.88 after the waypoint moved).
     *
     * <p>Neutral waypoints ({@link GlidePoints.GlidePoint#neutral}) are returned
     * unmirrored because they are already field-symmetric.
     *
     * @return the resolved pose, or {@code fallback} if the waypoint is missing from the
     *         map
     */
    private static Pose2d glidePose(String blueName, boolean isRedAlliance, Pose2d fallback) {
        GlidePoint point = GlidePoints.GLIDE_POINTS.get(blueName);
        if (point == null) {
            return fallback;
        }
        return point.neutral ? point.pose() : AllianceFlipUtil.apply(point.pose(), isRedAlliance);
    }

    public Pose2d getSmartGlideTarget(
            Pose2d robotPose,
            boolean hasFuel,
            boolean isHubActive,
            boolean isRedAlliance) {

        Pose2d targetPose;
        String mode;

        double matchTime = Timer.getMatchTime();
        if (matchTime > 0.0 && matchTime <= 20.0) {
            // No route to a missing waypoint: hold position rather than invent a pose.
            targetPose = glidePose("Blue Right Side Climb", isRedAlliance, robotPose);
            mode = "ENDGAME_PARK (" + (isRedAlliance ? "Red" : "Blue") + ")";
        } else if (hasFuel && isHubActive) {
            Pose2d frontPose = glidePose("Blue Hub Front", isRedAlliance, robotPose);
            Pose2d backPose = glidePose("Blue Hub Back", isRedAlliance, robotPose);

            double distFront = robotPose.getTranslation().getDistance(frontPose.getTranslation());
            double distBack = robotPose.getTranslation().getDistance(backPose.getTranslation());

            targetPose = (distFront <= distBack) ? frontPose : backPose;
            mode = "SCORE_HUB (" + (distFront <= distBack ? "Front" : "Back") + ")";
        } else if (!hasFuel && isHubActive) {
            Pose2d topMid = glidePose("Midfield Top", isRedAlliance, robotPose);
            Pose2d botMid = glidePose("Midfield Bottom", isRedAlliance, robotPose);

            double distTop = Math.abs(robotPose.getY() - topMid.getY());
            double distBot = Math.abs(robotPose.getY() - botMid.getY());

            targetPose = (distTop <= distBot) ? topMid : botMid;
            mode = "BALL_HUNT_MIDFIELD (" + (distTop <= distBot ? "Top" : "Bottom") + ")";
        } else {
            Pose2d topFeeder = glidePose("Blue Feeder Top", isRedAlliance, robotPose);
            Pose2d botFeeder = glidePose("Blue Feeder Bottom", isRedAlliance, robotPose);

            double distTop = robotPose.getTranslation().getDistance(topFeeder.getTranslation());
            double distBot = robotPose.getTranslation().getDistance(botFeeder.getTranslation());

            targetPose = (distTop <= distBot) ? topFeeder : botFeeder;
            mode = "RELOAD_DEPOT (" + (distTop <= distBot ? "Top" : "Bottom") + ")";
        }

        SmartDashboard.putString("JevAI/GlideArbitrationMode", mode);
        SmartDashboard.putNumberArray("JevAI/GlideTarget", new double[] {
                targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()
        });
        Logger.recordOutput("JevAI/SmartGlideTarget", targetPose);

        return targetPose;
    }

    public Pose2d solveLeadPursuitIntercept(
            Pose2d robotPose,
            Pose2d opponentPose,
            Translation2d opponentVel,
            double maxRobotSpeed) {

        if (opponentPose == null)
            return robotPose;
        if (opponentVel == null)
            opponentVel = new Translation2d();
        if (maxRobotSpeed <= 0.1)
            maxRobotSpeed = Constants.MAX_SPEED;

        Translation2d delta = opponentPose.getTranslation().minus(robotPose.getTranslation());
        double dx = delta.getX();
        double dy = delta.getY();
        double vx = opponentVel.getX();
        double vy = opponentVel.getY();

        double a = (vx * vx + vy * vy) - (maxRobotSpeed * maxRobotSpeed);
        double b = 2.0 * (dx * vx + dy * vy);
        double c = dx * dx + dy * dy;

        double interceptTime = -1.0;

        if (Math.abs(a) < 1e-5) {
            if (Math.abs(b) > 1e-4) {
                double tLinear = -c / b;
                if (tLinear > 0.05) {
                    interceptTime = tLinear;
                }
            }
        } else {
            double discriminant = (b * b) - (4.0 * a * c);
            if (discriminant >= 0) {
                double sqrtD = Math.sqrt(discriminant);
                double t1 = (-b - sqrtD) / (2.0 * a);
                double t2 = (-b + sqrtD) / (2.0 * a);

                if (t1 > 0.05 && t2 > 0.05) {
                    interceptTime = Math.min(t1, t2);
                } else if (t1 > 0.05) {
                    interceptTime = t1;
                } else if (t2 > 0.05) {
                    interceptTime = t2;
                }
            }
        }

        Translation2d interceptPos;
        if (interceptTime > 0.05 && interceptTime < 5.0) {
            interceptPos = opponentPose.getTranslation().plus(opponentVel.times(interceptTime));
        } else {
            interceptPos = opponentPose.getTranslation().plus(opponentVel.times(0.25));
        }

        double clampedX = Math.max(0.60, Math.min(15.94, interceptPos.getX()));
        double clampedY = Math.max(0.60, Math.min(7.65, interceptPos.getY()));

        Translation2d diff = opponentPose.getTranslation().minus(new Translation2d(clampedX, clampedY));
        Rotation2d faceOpponent = diff.getNorm() > 0.01 ? diff.getAngle() : opponentPose.getRotation();
        Pose2d target = new Pose2d(clampedX, clampedY, faceOpponent);

        Logger.recordOutput("JevAI/LeadPursuitIntercept", target);
        return target;
    }

    public Pose2d calculatePolarStandoffPose(Pose2d robotPose, boolean isRedAlliance) {
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(isRedAlliance);
        Translation2d toRobot = robotPose.getTranslation().minus(hub);

        double clampedAngleRad;
        if (isRedAlliance) {
            double angleFromEast = Math.atan2(toRobot.getY(), Math.abs(toRobot.getX()));
            clampedAngleRad = Math.max(-Math.PI / 4.0, Math.min(Math.PI / 4.0, angleFromEast));
        } else {
            double angleFromWest = Math.atan2(toRobot.getY(), -Math.abs(toRobot.getX()));
            double angleDiff = Math.IEEEremainder(angleFromWest - Math.PI, 2 * Math.PI);
            double clampedFromWest = Math.max(-Math.PI / 4.0, Math.min(Math.PI / 4.0, angleDiff));
            clampedAngleRad = Math.PI + clampedFromWest;
        }

        double targetX = hub.getX() + FieldMap.Hubs.OPTIMAL_STANDOFF_DISTANCE * Math.cos(clampedAngleRad);
        double targetY = hub.getY() + FieldMap.Hubs.OPTIMAL_STANDOFF_DISTANCE * Math.sin(clampedAngleRad);

        targetY = Math.max(2.20, Math.min(5.80, targetY));

        if (isRedAlliance) {
            targetX = Math.max(12.60, Math.min(14.80, targetX));
        } else {
            targetX = Math.max(1.80, Math.min(3.90, targetX));
        }

        Translation2d standoffPos = new Translation2d(targetX, targetY);
        Rotation2d faceHubAngle = hub.minus(standoffPos).getAngle();

        return new Pose2d(standoffPos, faceHubAngle);
    }

    public GlidePoint selectOptimalTrenchCorridor(
            Pose2d robotPose,
            boolean outbound,
            boolean isRedAlliance) {

        String topKey = isRedAlliance ? "Red Top Trench" : "Blue Top Trench";
        String botKey = isRedAlliance ? "Red Bottom Trench" : "Blue Bottom Trench";

        GlidePoint topPoint = GlidePoints.GLIDE_POINTS.get(topKey);
        GlidePoint botPoint = GlidePoints.GLIDE_POINTS.get(botKey);

        double minX = isRedAlliance ? 10.3 : 3.0;
        double maxX = isRedAlliance ? 13.6 : 6.3;

        int topObstacleCount = 0;
        int botObstacleCount = 0;

        for (DynamicObstacle obs : DynamicRouter.getActiveObstacles()) {
            double ox = obs.position.getX();
            double oy = obs.position.getY();

            if (ox >= minX && ox <= maxX) {
                if (oy > 5.5) {
                    topObstacleCount++;
                } else if (oy < 2.5) {
                    botObstacleCount++;
                }
            }
        }

        GlidePoint chosen;
        if (topObstacleCount < botObstacleCount) {
            chosen = topPoint;
        } else if (botObstacleCount < topObstacleCount) {
            chosen = botPoint;
        } else {
            double distTop = Math.abs(robotPose.getY() - (topPoint != null ? topPoint.pose.getY() : 7.4));
            double distBot = Math.abs(robotPose.getY() - (botPoint != null ? botPoint.pose.getY() : 0.65));
            chosen = (distTop <= distBot) ? topPoint : botPoint;
        }

        SmartDashboard.putString("JevAI/TrenchCorridorSelected", chosen != null ? chosen.name : "None");
        return chosen;
    }
}
