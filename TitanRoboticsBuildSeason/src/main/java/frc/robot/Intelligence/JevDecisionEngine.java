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
import frc.robot.Intelligence.utility.Consideration;
import frc.robot.Intelligence.utility.ResponseCurve;
import frc.robot.Intelligence.utility.UtilityAction;
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
    /** A challenger this far ahead switches immediately, ignoring action inertia. */
    public static final double COMMITMENT_DECISIVE_MARGIN = 0.20;

    /** Minimum time on an objective before a matured non-decisive challenger may take it. */
    public static final double COMMITMENT_MIN_HOLD_SEC = 1.5;

    /** Initial dynamic action inertia boost applied to the active objective. */
    public static final double INERTIA_INITIAL_BOOST = 0.20;

    /** Time constant tau for exponential action inertia decay (seconds). */
    public static final double INERTIA_TIME_CONSTANT_SEC = 1.0;

    /** Residual baseline hysteresis margin preventing chattering after inertia decays. */
    public static final double INERTIA_RESIDUAL_MARGIN = 0.04;

    /**
     * Resolves the committed objective for one agent using Dave Mark's Dynamic Action Inertia.
     *
     * <p>Pure: it reads and returns the caller's own latch fields without mutating engine state.
     * The incumbent receives an exponential momentum bonus:
     * <pre>
     *   Inertia(t) = I_0 * exp(-(now - heldSince) / tau)
     * </pre>
     * If the incumbent's raw utility collapses to <= 0.0 (e.g. active hub turns off), it is released
     * immediately via natural veto. A challenger exceeding incumbent utility plus inertia breaks out at once.
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
        return resolveCommittedObjective(candidate, utilities, held, heldSinceSeconds, nowSeconds, PolicyWeights.getActive());
    }

    /**
     * Resolves the committed objective for one agent using explicit {@link PolicyWeights}.
     */
    public static StrategicObjective resolveCommittedObjective(
            StrategicObjective candidate,
            Map<StrategicObjective, Double> utilities,
            StrategicObjective held,
            double heldSinceSeconds,
            double nowSeconds,
            PolicyWeights weights) {
        if (candidate == null) {
            return held;
        }
        if (held == null) {
            return candidate;
        }
        if (candidate == held) {
            return held;
        }

        PolicyWeights pw = (weights != null) ? weights : PolicyWeights.getActive();

        // The incumbent is no longer viable (its utility collapsed, e.g. its
        // hub just went inactive): natural veto releases immediately rather than
        // honoring momentum.
        double incumbentUtility = utilities.getOrDefault(held, 0.0);
        if (incumbentUtility <= 0.0) {
            return candidate;
        }

        double candidateUtility = utilities.getOrDefault(candidate, 0.0);
        double gain = candidateUtility - incumbentUtility;

        // Decisive margin allows immediate switch on massive match-state changes
        if (gain >= pw.commitmentDecisiveMargin()) {
            return candidate;
        }

        // Continuous Action Inertia:
        // Inertia boost decays exponentially over time constant tau.
        double elapsedSec = Math.max(0.0, nowSeconds - heldSinceSeconds);
        double dynamicInertia = pw.inertiaInitialBoost() * Math.exp(-elapsedSec / pw.inertiaTimeConstantSec());
        double requiredGain = dynamicInertia + pw.inertiaResidualMargin();

        if (gain >= requiredGain) {
            return candidate;
        }

        // Matured hold rule check: after commitmentMinHoldSec, gain >= commitmentMargin is enough
        boolean matured = elapsedSec >= pw.commitmentMinHoldSec();
        return (matured && gain >= pw.commitmentMargin()) ? candidate : held;
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
        return evaluatePolicy(world, knowledge, archetype, (String) null);
    }

    /**
     * Evaluation overload that accepts explicit PolicyWeights.
     */
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype, PolicyWeights weights) {
        return evaluatePolicy(world, knowledge, archetype, null, null, null, null, weights);
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
        return evaluatePolicy(world, knowledge, archetype, cloudContext, blockedFuel,
                commitmentIn, null);
    }

    public Map<StrategicObjective, Double> evaluateUtilityScores(WorldState world, Archetype archetype) {
        return evaluateUtilityScores(world, Collections.emptySet(), ObservedKnowledge.selfOnly(), archetype);
    }

    public Map<StrategicObjective, Double> evaluateUtilityScores(
            WorldState world, MatchKnowledge knowledge, Archetype archetype) {
        return evaluateUtilityScores(world, Collections.emptySet(), knowledge, archetype);
    }

    /**
     * Evaluates the complete raw and adjusted utility map for all strategic objectives.
     * Useful for diagnostic inspection, decision cards, and verifying policy calculations.
     *
     * @param world       the observable or simulated world state
     * @param blockedFuel dynamic set of fuel positions currently contested or blocked
     * @param knowledge   match tier knowledge (clairvoyant sim vs observed real robot)
     * @param archetype   tactical behavior archetype
     * @return map of every StrategicObjective to its computed utility value [0.0, 1.0]
     */
    public Map<StrategicObjective, Double> evaluateUtilityScores(
            WorldState world, Set<Translation2d> blockedFuel, MatchKnowledge knowledge, Archetype archetype) {
        return evaluateUtilityScores(world, blockedFuel, knowledge, archetype, PolicyWeights.getActive());
    }

    /**
     * Evaluates the complete raw and adjusted utility map for all strategic objectives
     * using explicit {@link PolicyWeights}.
     *
     * @param world       the observable or simulated world state
     * @param blockedFuel dynamic set of fuel positions currently contested or blocked
     * @param knowledge   match tier knowledge (clairvoyant sim vs observed real robot)
     * @param archetype   tactical behavior archetype
     * @param weights     parameterized utility weights and thresholds
     * @return map of every StrategicObjective to its computed utility value [0.0, 1.0]
     */
    public Map<StrategicObjective, Double> evaluateUtilityScores(
            WorldState world, Set<Translation2d> blockedFuel, MatchKnowledge knowledge, Archetype archetype,
            PolicyWeights weights) {
        if (weights == null) {
            weights = PolicyWeights.getActive();
        }
        if (knowledge == null) {
            knowledge = ObservedKnowledge.selfOnly();
        }
        if (blockedFuel == null) {
            blockedFuel = Collections.emptySet();
        }
        boolean opponentObserved = knowledge.opponentObserved();

        Translation2d selfHub = FieldMap.Hubs.getHubLocation2d(world.isRedAlliance());
        double distToSelfHub = world.selfPose().getTranslation().getDistance(selfHub);
        double transitTimeToHub = distToSelfHub / 3.2;
        double timeLeftToHarvest = world.timeUntilHubShift() - transitTimeToHub;

        // ── 1. Evaluate Utility Scores Across Objectives ─────────────────────
        Map<StrategicObjective, Double> utilities = new LinkedHashMap<>();

        // ── 1. RUSH_CLIMB (IAUS Continuous Endgame Ramp & Natural Role Veto) ─
        double climbRoleFactor = (world.hasClimber() && !world.isAutonomous()) ? 1.0 : 0.0;
        double climbTimeFactor = (world.matchTimeRemaining() > 0.0 && world.matchTimeRemaining() <= 20.0) ? 1.0 : 0.0;
        double climbBase = (world.matchTimeRemaining() <= 15.0) ? weights.climbBase15() : weights.climbBase20();
        double climbScale = (world.matchTimeRemaining() <= 15.0) ? weights.climbScale15() : weights.climbScale20();
        double climbProgression = (world.matchTimeRemaining() <= 15.0)
                ? Math.max(0.0, Math.min(1.0, 1.0 - world.matchTimeRemaining() / 15.0))
                : Math.max(0.0, Math.min(1.0, 1.0 - (world.matchTimeRemaining() - 15.0) / 5.0));
        ResponseCurve climbRampCurve = ResponseCurve.linear(climbScale, 0.0, climbBase);
        double rawClimbScore = climbRampCurve.calculate(climbProgression);
        double climbUtility = UtilityAction.evaluateProduct(rawClimbScore, climbRoleFactor, climbTimeFactor);
        utilities.put(StrategicObjective.RUSH_CLIMB, climbUtility);

        // ── 2. CYCLE_SCORE_HUB (IAUS Continuous Payload & Hub Veto) ──────────
        boolean inShootingRange = distToSelfHub <= weights.scoreHubShootingRangeMeters();
        boolean shiftEndingSoon = world.timeUntilHubShift() <= weights.scoreHubShiftEndingWindowSec() && world.timeUntilHubShift() > 0.0;
        int minFuelToScore = (archetype == Archetype.CO_PILOT || inShootingRange) ? 1 : (shiftEndingSoon ? weights.scoreHubMinFuelShiftEnding() : weights.scoreHubMinFuelNormal());

        double shooterFactor = world.hasShooter() ? 1.0 : 0.0;
        double hubActiveFactor = world.isAllianceHubActive() ? 1.0 : 0.0;
        double fuelThresholdFactor = (world.heldFuelCount() >= minFuelToScore) ? 1.0 : 0.0;

        double capacityDivisor = Math.min(weights.scoreHubCapacityDivisor(), Math.max(1, world.ballCapacity()));
        double loadRatio = (archetype == Archetype.CO_PILOT || inShootingRange)
                ? 1.0
                : Math.min(1.0, (double) world.heldFuelCount() / capacityDivisor);
        // IAUS polynomial payload consideration: exponent 1.0 reproduces the legacy linear load exactly.
        double shapedLoad = Math.pow(loadRatio, weights.scoreHubPayloadExponent());
        ResponseCurve scoreLoadCurve = ResponseCurve.linear(weights.scoreHubScale(), 0.0, weights.scoreHubBase());
        double baseScore = scoreLoadCurve.calculate(shapedLoad);

        if (knowledge.scoreDifferential() < 0) {
            baseScore = Math.min(weights.scoreHubBehindMax(), baseScore + weights.scoreHubBehindBonus());
        }
        double scoreUtility = UtilityAction.evaluateProduct(baseScore, shooterFactor, hubActiveFactor, fuelThresholdFactor);
        utilities.put(StrategicObjective.CYCLE_SCORE_HUB, scoreUtility);

        // ── 3. STAGE_STANDOFF (IAUS Shift Anticipation & Stockpile Gating) ───
        int minFuelToStage = (archetype == Archetype.CO_PILOT) ? 1 : weights.stageMinFuelNormal();
        double stageShooterFactor = world.hasShooter() ? 1.0 : 0.0;
        double hubInactiveFactor = (!world.isAllianceHubActive()) ? 1.0 : 0.0;
        double stageFuelFactor = (world.heldFuelCount() >= minFuelToStage) ? 1.0 : 0.0;

        double targetStage = (archetype == Archetype.CO_PILOT)
                ? weights.stageShiftImminentCoPilot()
                : weights.stageShiftImminentNormal();
        // IAUS logistic shift-urgency consideration over raw seconds: rises to 1 as the flip
        // approaches, 0 when no flip is coming (timeUntilHubShift 0.0 is the no-flip sentinel,
        // never "imminent"). Defaults (k=8, mid=3.5s) reproduce the legacy 3.5 s step to <1e-3
        // everywhere except a ~0.75 s smoothing band around the midpoint — the plan's §4 tunable.
        double shiftUrgency = 0.0;
        if (world.timeUntilHubShift() > 0.0) {
            shiftUrgency = 1.0 / (1.0 + Math.exp(-weights.shiftUrgencySigmoidSteepness()
                    * (weights.shiftUrgencyMidpointSec() - world.timeUntilHubShift())));
        }
        double stageBase = weights.stageStandoffBase()
                + (targetStage - weights.stageStandoffBase()) * shiftUrgency;
        ResponseCurve stageCurve = ResponseCurve.linear(0.0, 0.0, stageBase);
        double stageUtility = UtilityAction.evaluateProduct(stageCurve.calculate(1.0), stageShooterFactor, hubInactiveFactor, stageFuelFactor);
        utilities.put(StrategicObjective.STAGE_STANDOFF, stageUtility);

        // ── 4. VACUUM_MIDFIELD (IAUS Continuous Harvest Response Curves) ─────
        double harvestCapacityFactor = (world.ballCapacity() > 0) ? 1.0 : 0.0;
        double headroomFactor = (!world.isInventoryFull()) ? 1.0 : 0.0;
        double effectiveCapacity = Math.max(1, world.ballCapacity());
        double vacuumRaw;
        if (!world.isAllianceHubActive()) {
            ResponseCurve inactiveVacuumCurve = ResponseCurve.linear(
                    -weights.vacuumInactiveScale(), 0.0, weights.vacuumInactiveBase() + weights.vacuumInactiveScale());
            double loadProgress = Math.min(1.0, (double) world.heldFuelCount() / effectiveCapacity);
            vacuumRaw = inactiveVacuumCurve.calculate(loadProgress);
        } else {
            int targetBatch = shiftEndingSoon ? weights.vacuumActiveShiftEndingBatch() : weights.vacuumActiveNormalBatch();
            targetBatch = Math.min(targetBatch, world.ballCapacity());
            if (world.heldFuelCount() < targetBatch) {
                ResponseCurve activeVacuumCurve = ResponseCurve.linear(
                        -weights.vacuumActiveScale(), 0.0, weights.vacuumActiveBase() + weights.vacuumActiveScale());
                double batchProgress = (targetBatch > 0) ? (double) world.heldFuelCount() / targetBatch : 1.0;
                vacuumRaw = activeVacuumCurve.calculate(batchProgress);
            } else {
                vacuumRaw = weights.vacuumActiveFullCap();
            }
        }
        double vacuumUtility = UtilityAction.evaluateProduct(vacuumRaw, headroomFactor, harvestCapacityFactor);
        utilities.put(StrategicObjective.VACUUM_MIDFIELD, vacuumUtility);

        // ── 5. STOCKPILE_DEPOT (IAUS Feeder Restock Response Curve) ──────────
        double depotHeadroomFactor = (!world.isInventoryFull()) ? 1.0 : 0.0;
        double depotHubInactiveFactor = (!world.isAllianceHubActive()) ? 1.0 : 0.0;
        ResponseCurve stockpileCurve = ResponseCurve.linear(
                -weights.stockpileDepotScale(), 0.0, weights.stockpileDepotBase() + weights.stockpileDepotScale());
        double rawStockpile = stockpileCurve.calculate(Math.min(1.0, (double) world.heldFuelCount() / effectiveCapacity));
        double stockpileUtility = UtilityAction.evaluateProduct(rawStockpile, depotHeadroomFactor, depotHubInactiveFactor, harvestCapacityFactor);
        utilities.put(StrategicObjective.STOCKPILE_DEPOT, stockpileUtility);

        // ── 6. SWEEP_ALLIANCE_ZONE (IAUS Home Zone Scavenge Response Curve) ──
        int homeFuelCount = countFuelInZone(world.isRedAlliance(), false, blockedFuel, knowledge);
        double sweepFuelPresentFactor = (homeFuelCount > 0) ? 1.0 : 0.0;
        double sweepHeadroomFactor = (!world.isInventoryFull()) ? 1.0 : 0.0;
        double rawSweep;
        if (world.isAllianceHubActive()) {
            rawSweep = weights.sweepAllianceZoneActive();
        } else {
            ResponseCurve sweepInactiveCurve = ResponseCurve.linear(
                -weights.sweepAllianceZoneInactiveScale(), 0.0,
                weights.sweepAllianceZoneInactiveBase() + weights.sweepAllianceZoneInactiveScale());
            rawSweep = sweepInactiveCurve.calculate(Math.min(1.0, (double) world.heldFuelCount() / effectiveCapacity));
        }
        double sweepUtility = UtilityAction.evaluateProduct(rawSweep, sweepFuelPresentFactor, sweepHeadroomFactor, harvestCapacityFactor);
        utilities.put(StrategicObjective.SWEEP_ALLIANCE_ZONE, sweepUtility);

        // ── 7. POACH_OPPONENT_ZONE (IAUS Shift Boundary Poaching) ────────────
        double poachTeleopFactor = (!world.isAutonomous()) ? 1.0 : 0.0;
        double poachHeadroomFactor = (!world.isInventoryFull()) ? 1.0 : 0.0;
        double timeUntilShiftForPoach = world.timeUntilHubShift();
        double poachWindowFactor = (timeUntilShiftForPoach > 0.0 && timeUntilShiftForPoach <= weights.poachOpponentZoneWindowSec()) ? 1.0 : 0.0;
        double poachOppHubNextFactor = world.isOpponentHubActiveAfterShift() ? 1.0 : 0.0;
        double poachMaxHeldFactor = (world.heldFuelCount() < weights.poachOpponentZoneMaxHeld()) ? 1.0 : 0.0;
        int oppFuelCount = countFuelInZone(!world.isRedAlliance(), true, blockedFuel, knowledge);
        double poachFuelPresentFactor = (oppFuelCount > 0) ? 1.0 : 0.0;

        double opponentZoneUtility = UtilityAction.evaluateProduct(
                weights.poachOpponentZoneUtility(),
                poachTeleopFactor,
                poachHeadroomFactor,
                poachWindowFactor,
                poachOppHubNextFactor,
                poachMaxHeldFactor,
                poachFuelPresentFactor,
                harvestCapacityFactor);
        utilities.put(StrategicObjective.POACH_OPPONENT_ZONE, opponentZoneUtility);

        // ── 8. SHUTTLE_PASS (IAUS Tactical Fuel Transit / Lobbing) ───────────
        double shuttleShooterFactor = world.hasShooter() ? 1.0 : 0.0;
        double shuttleTeleopFactor = (!world.isAutonomous()) ? 1.0 : 0.0;
        double shuttleHubInactiveFactor = (!world.isAllianceHubActive()) ? 1.0 : 0.0;
        double shuttleOppHubFactor = (world.isOpponentHubActive() || world.isOpponentHubActiveAfterShift()) ? 1.0 : 0.0;
        double shuttleZoneFactor = FieldMap.AllianceZones.isInAllianceZone(world.selfPose(), world.isRedAlliance()) ? 1.0 : 0.0;
        double shuttleCeilingFactor = (!FieldMap.Trenches.isLowClearance(world.selfPose())) ? 1.0 : 0.0;
        double shuttleDistFactor = (distToSelfHub > weights.shuttlePassMinDistMeters()) ? 1.0 : 0.0;
        double shuttleFuelFactor = (world.heldFuelCount() >= weights.shuttlePassMinHeld()) ? 1.0 : 0.0;

        double shuttleUtility = UtilityAction.evaluateProduct(
                weights.shuttlePassUtility(),
                shuttleShooterFactor,
                shuttleTeleopFactor,
                shuttleHubInactiveFactor,
                shuttleOppHubFactor,
                shuttleZoneFactor,
                shuttleCeilingFactor,
                shuttleDistFactor,
                shuttleFuelFactor);
        utilities.put(StrategicObjective.SHUTTLE_PASS, shuttleUtility);

        // ── 9. LONG_RANGE_SNIPE (IAUS Outer Perimeter Launch) ────────────────
        double snipeShooterFactor = world.hasShooter() ? 1.0 : 0.0;
        double snipeHubFactor = world.isAllianceHubActive() ? 1.0 : 0.0;
        double snipeFuelFactor = (world.heldFuelCount() >= weights.snipeMinHeld()) ? 1.0 : 0.0;
        double snipeZoneFactor = FieldMap.AllianceZones.isInAllianceZone(world.selfPose(), world.isRedAlliance()) ? 1.0 : 0.0;
        double snipeDistFactor = (distToSelfHub >= weights.snipeMinDistMeters() && distToSelfHub <= FieldMap.Hubs.SHOOTING_MAX_DISTANCE) ? 1.0 : 0.0;

        double snipeBase = (opponentObserved && world.opponentPose().getTranslation().getDistance(selfHub) <= 2.4)
                ? weights.snipeCloseUtility()
                : weights.snipeFarUtility();
        double longRangeUtility = UtilityAction.evaluateProduct(
                snipeBase,
                snipeShooterFactor,
                snipeHubFactor,
                snipeFuelFactor,
                snipeZoneFactor,
                snipeDistFactor);
        utilities.put(StrategicObjective.LONG_RANGE_SNIPE, longRangeUtility);

        // ── Defense Objectives (IAUS Multiplicative Defensive Postures) ──────
        boolean roleCanDefend = (archetype == Archetype.TACTICAL_DEFENDER || archetype == Archetype.DEFENSE_BULLY || archetype == Archetype.ADAPTIVE_COMPETITOR);
        double defRoleFactor = roleCanDefend ? 1.0 : 0.0;
        double defObservedFactor = opponentObserved ? 1.0 : 0.0;

        Translation2d oppHub = FieldMap.Hubs.getHubLocation2d(!world.isRedAlliance());
        double oppDistToHub = world.opponentPose().getTranslation().getDistance(oppHub);
        double laneDistFactor = (oppDistToHub < weights.laneDenialMaxDistMeters()) ? 1.0 : 0.0;
        double laneOppHubActiveFactor = world.isOpponentHubActive() ? 1.0 : 0.0;

        // ── 10. DENY_SHOOTING_LANE ───────────────────────────────────────────
        double laneDenialUtility = UtilityAction.evaluateProduct(
                weights.laneDenialActiveUtility(),
                defRoleFactor,
                defObservedFactor,
                laneOppHubActiveFactor,
                laneDistFactor);

        // ── 11. SHADOW_MIDLINE ───────────────────────────────────────────────
        double shadowUtility = UtilityAction.evaluateProduct(
                weights.shadowMidlineBaseUtility(),
                defRoleFactor,
                defObservedFactor);

        // ── 12. LEAD_INTERCEPT ───────────────────────────────────────────────
        double interceptUtility = UtilityAction.evaluateProduct(
                weights.interceptBaseUtility(),
                defRoleFactor,
                defObservedFactor);

        // ── 13. CHOKE_TRENCH ─────────────────────────────────────────────────
        boolean defensiveArchetype = archetype == Archetype.TACTICAL_DEFENDER
                || archetype == Archetype.DEFENSE_BULLY || archetype == Archetype.ADAPTIVE_COMPETITOR;
        double chokeRoleFactor = defensiveArchetype ? 1.0 : 0.0;
        double chokeObservedFactor = opponentObserved ? 1.0 : 0.0;
        double chokeTeleopFactor = (!world.isAutonomous()) ? 1.0 : 0.0;
        double chokeTrenchFactor = FieldMap.Trenches.isLowClearance(world.opponentPose()) ? 1.0 : 0.0;

        double chokeUtility = UtilityAction.evaluateProduct(
                weights.chokeTrenchUtility(),
                chokeRoleFactor,
                chokeObservedFactor,
                chokeTeleopFactor,
                chokeTrenchFactor);
        utilities.put(StrategicObjective.CHOKE_TRENCH, chokeUtility);

        // ── 14. SCREEN_FOR_ALLY ──────────────────────────────────────────────
        double screenObservedFactor = opponentObserved ? 1.0 : 0.0;
        double screenAllyFuelFactor = (knowledge.alliesHeldFuel() >= 12) ? 1.0 : 0.0;
        double screenHubFactor = world.isAllianceHubActive() ? 1.0 : 0.0;
        boolean allyInDistress = false;
        if (opponentObserved && knowledge.alliesHeldFuel() >= 12 && world.isAllianceHubActive()) {
            outer: for (Pose2d ally : knowledge.allyPoses()) {
                for (Pose2d opponent : knowledge.opponentPoses()) {
                    if (ally.getTranslation().getDistance(opponent.getTranslation()) <= 2.2) {
                        allyInDistress = true;
                        break outer;
                    }
                }
            }
        }
        double screenDistressFactor = allyInDistress ? 1.0 : 0.0;
        double screenUtility = UtilityAction.evaluateProduct(
                weights.screenForAllyUtility(),
                screenObservedFactor,
                screenAllyFuelFactor,
                screenHubFactor,
                screenDistressFactor);
        utilities.put(StrategicObjective.SCREEN_FOR_ALLY, screenUtility);

        // Pin duration is not present in WorldState/MatchKnowledge yet
        utilities.put(StrategicObjective.BAIT_PIN_FOUL, 0.0);

        // Tier-1 driver assist: with no vision-tracked opponent, opponent-chasing objectives are unavailable
        if (!opponentObserved) {
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
        }

        // Apply Archetype Multipliers
        if (world.isAutonomous()) {
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
            boolean batchReady = world.heldFuelCount() >= AUTO_BATCH_MIN_FUEL;
            boolean autoClockLow = world.matchTimeRemaining() >= 0.0
                    && world.matchTimeRemaining() <= AUTO_DUMP_SECONDS_LEFT;
            if (world.heldFuelCount() > 0 && (batchReady || inShootingRange || autoClockLow)) {
                scoreUtility = world.hasShooter() ? weights.autoBatchDumpScoreUtility() : 0.0;
                vacuumUtility = 0.0;
                sweepUtility = 0.0;
            } else {
                scoreUtility = 0.0;
                vacuumUtility = (world.ballCapacity() > 0 && !world.isInventoryFull())
                        ? weights.autoHarvestVacuumUtility()
                        : 0.0;
            }
        } else if (archetype == Archetype.AUTONOMOUS_CYCLER) {
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
        } else if (archetype == Archetype.TACTICAL_DEFENDER) {
            scoreUtility = 0.0;
            stageUtility = 0.0;
            sweepUtility = 0.0;
            vacuumUtility = 0.0;
            stockpileUtility = 0.0;
            opponentZoneUtility = 0.0;
            shuttleUtility = 0.0;
            longRangeUtility = 0.0;
            laneDenialUtility *= weights.tacticalDefenderLaneMultiplier();
            shadowUtility *= weights.tacticalDefenderShadowMultiplier();
        } else if (archetype == Archetype.LEAD_PURSUIT_INTERCEPTOR) {
            interceptUtility = weights.bullyInterceptUtility();
            scoreUtility = 0.0;
            stageUtility = 0.0;
            sweepUtility = 0.0;
            vacuumUtility = 0.0;
            stockpileUtility = 0.0;
            opponentZoneUtility = 0.0;
            shuttleUtility = 0.0;
            longRangeUtility = 0.0;
        } else if (archetype == Archetype.DEFENSE_BULLY) {
            interceptUtility = weights.bullyInterceptUtility();
            scoreUtility = 0.0;
            stageUtility = 0.0;
            sweepUtility = 0.0;
            vacuumUtility = 0.0;
            stockpileUtility = 0.0;
            opponentZoneUtility = 0.0;
            shuttleUtility = 0.0;
            longRangeUtility = 0.0;
        } else if (archetype == Archetype.CO_PILOT) {
            laneDenialUtility = 0.0;
            shadowUtility = 0.0;
            interceptUtility = 0.0;
            if (world.heldFuelCount() > 0 && world.isAllianceHubActive() && world.hasShooter()) {
                scoreUtility = weights.coPilotActiveScoreUtility();
                vacuumUtility = 0.0;
            }
        }

        if ((archetype == null || !archetype.isDefensive())
                && world.isAllianceHubActive()
                && timeLeftToHarvest <= 0.0
                && world.heldFuelCount() >= 8
                && world.hasShooter()) {
            scoreUtility = weights.harvestDeadlineForceUtility();
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

        return utilities;
    }

    /**
     * Full evaluation including both of the caller's per-agent latches.
     *
     * <p>{@code fuelTargetMemory} holds the fuel piece this agent is committed to
     * collecting, so a heading swing cannot make the selector oscillate between
     * two comparable pieces. Like {@code commitmentIn} it belongs to the agent,
     * not to this engine; pass {@code null} for the legacy behaviour.
     *
     * @param commitmentIn     this agent's objective latch, or {@code null}
     * @param fuelTargetMemory this agent's fuel-target latch, or {@code null}
     */
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype, String cloudContext,
            Set<Translation2d> blockedFuel, ObjectiveCommitment commitmentIn,
            FuelTargetMemory fuelTargetMemory) {
        return evaluatePolicy(world, knowledge, archetype, cloudContext, blockedFuel,
                commitmentIn, fuelTargetMemory, PolicyWeights.getActive());
    }

    /**
     * Full evaluation overload including per-agent latches and explicit {@link PolicyWeights}.
     */
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype, String cloudContext,
            Set<Translation2d> blockedFuel, ObjectiveCommitment commitmentIn,
            FuelTargetMemory fuelTargetMemory, PolicyWeights weights) {
        return evaluatePolicy(world, knowledge, archetype, cloudContext, blockedFuel,
                commitmentIn, fuelTargetMemory, weights, Timer.getFPGATimestamp());
    }

    /**
     * Sim-time evaluation for fast-forward runners, whose wall clock does not
     * advance between ticks. Identical to the full overload except the
     * commitment latch and inertia telemetry read {@code nowSeconds} instead of
     * the FPGA clock. Production callers must keep the wall-clock overload.
     *
     * @param nowSeconds caller-owned time base (sim elapsed, seconds)
     */
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype, String cloudContext,
            Set<Translation2d> blockedFuel, ObjectiveCommitment commitmentIn,
            FuelTargetMemory fuelTargetMemory, PolicyWeights weights, double nowSeconds) {
        if (weights == null) {
            weights = PolicyWeights.getActive();
        }
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
        Map<StrategicObjective, Double> utilities = evaluateUtilityScores(world, blockedFuel, knowledge, archetype, weights);

        // ── 2. Select Highest Utility Objective ──────────────────────────────
        StrategicObjective bestObjective = evaluateLocalUtilityMatrix(utilities, world, archetype);

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
            bestObjective = commitmentIn.applyAt(bestObjective, utilities, weights, nowSeconds);
        }
        double maxUtility = utilities.getOrDefault(bestObjective, 0.0);

        // Tier-1 safety net: opponent-chasing objectives require a tracked
        // opponent. If one ever wins without observation (e.g. a future
        // utility change), fall back to safe zone defense (SHADOW_MIDLINE)
        // for defenders, or harvesting/staging for offensive bots.
        if (!opponentObserved && (bestObjective == StrategicObjective.LEAD_INTERCEPT
                || bestObjective == StrategicObjective.DENY_SHOOTING_LANE
                || bestObjective == StrategicObjective.CHOKE_TRENCH
                || bestObjective == StrategicObjective.SCREEN_FOR_ALLY)) {
            bestObjective = (archetype != null && archetype.isDefensive())
                    ? StrategicObjective.SHADOW_MIDLINE
                    : (world.isInventoryFull()
                            ? (world.hasShooter() ? StrategicObjective.STAGE_STANDOFF : StrategicObjective.SHADOW_MIDLINE)
                            : StrategicObjective.VACUUM_MIDFIELD);
            maxUtility = utilities.getOrDefault(bestObjective, 0.0);
        }

        TypeSafeJevClient.JevDecision cloudDecision = cloudConfigured
                ? cloudClient.getLatestDecision(cloudContext)
                : null;
        boolean usedCloud = false;
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
        FuelTourOptimizer.TourResult tourResult = FuelTourOptimizer.TourResult.EMPTY;
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
                rationale = String.format("Stockpiling at Alliance Depot (%d/%d).", world.heldFuelCount(), world.ballCapacity());
                break;

            case VACUUM_MIDFIELD:
                int piecesNeeded = Math.min(5, Math.max(0, world.ballCapacity() - world.heldFuelCount()));
                FuelTourOptimizer.TourResult tour = (piecesNeeded > 1)
                        ? planFuelHarvestTour(world.selfPose(), world.isRedAlliance(), world.isAutonomous(),
                                piecesNeeded, blockedFuel)
                        : FuelTourOptimizer.TourResult.EMPTY;

                if (tour.isValid() && tour.pieceCount() > 1) {
                    Translation2d immediateTarget = tour.immediateTargetPose().getTranslation();
                    if (fuelTargetMemory != null && fuelTargetMemory.latched() != null) {
                        boolean latchedStillPresent = tour.waypoints().stream()
                                .anyMatch(p -> p.getDistance(fuelTargetMemory.latched()) <= FuelTargetMemory.STICK_RADIUS_M);
                        if (!latchedStillPresent) {
                            fuelTargetMemory.resolve(List.of(new FuelTargetMemory.ScoredTarget(immediateTarget, 100.0)));
                        }
                    } else if (fuelTargetMemory != null) {
                        fuelTargetMemory.resolve(List.of(new FuelTargetMemory.ScoredTarget(immediateTarget, 100.0)));
                    }
                    navTarget = tour.immediateTargetPose();
                    tourResult = tour;
                    rationale = String.format("Harvesting multi-piece tour (%d pieces, %.1fm, %d/%d held).",
                            tour.pieceCount(), tour.totalDistanceMeters(), world.heldFuelCount(), world.ballCapacity());
                } else {
                    navTarget = findClusterWeightedFuelTarget(world.selfPose(), world.isRedAlliance(),
                            world.isAutonomous(), blockedFuel, fuelTargetMemory, knowledge.fieldFuel());
                    rationale = String.format("Hunting fuel (%d/%d). Hopper capacity available.", world.heldFuelCount(), world.ballCapacity());
                }
                intakeCmd = IntakeState.INTAKING;
                shooterCmd = ShooterState.STOPPED;
                break;

            case SWEEP_ALLIANCE_ZONE:
                int piecesNeededZone = Math.min(5, Math.max(0, world.ballCapacity() - world.heldFuelCount()));
                List<Translation2d> zoneCandidates = findAllianceZoneFuelCandidates(world.isRedAlliance(), blockedFuel, knowledge.fieldFuel());
                Translation2d selfHubPos = FieldMap.Hubs.getHubLocation2d(world.isRedAlliance());
                FuelTourOptimizer.TourResult zoneTour = (zoneCandidates.size() >= 2 && piecesNeededZone > 1)
                        ? FuelTourOptimizer.optimizeTour(world.selfPose(), zoneCandidates, piecesNeededZone, selfHubPos)
                        : FuelTourOptimizer.TourResult.EMPTY;

                if (zoneTour.isValid() && zoneTour.pieceCount() > 1) {
                    navTarget = zoneTour.immediateTargetPose();
                    tourResult = zoneTour;
                    int currentHomeFuel = countFuelInZone(world.isRedAlliance(), false, blockedFuel, knowledge);
                    rationale = String.format("Executing alliance zone tour (%d pieces, %.1fm, %d loose total).",
                            zoneTour.pieceCount(), zoneTour.totalDistanceMeters(), currentHomeFuel);
                } else {
                    navTarget = findAllianceZoneFuelTarget(world.selfPose(), world.isRedAlliance(), blockedFuel, knowledge.fieldFuel());
                    int currentHomeFuel = countFuelInZone(world.isRedAlliance(), false, blockedFuel, knowledge);
                    rationale = String.format("Sweeping %d loose fuel pieces in the alliance zone.", currentHomeFuel);
                }
                intakeCmd = IntakeState.INTAKING;
                if (world.isAllianceHubActive() && world.hasShooter()) {
                    shooterCmd = ShooterState.PREPARING;
                    targetRPM = 3200.0;
                }
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
                        blockedFuel, knowledge.fieldFuel());
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
                double clampedY;
                Rotation2d face;
                if (opponentObserved) {
                    clampedY = Math.max(1.0, Math.min(FieldMap.FIELD_WIDTH - 1.0, world.opponentPose().getY()));
                    face = world.opponentPose().getTranslation().minus(new Translation2d(shadowX, clampedY))
                            .getAngle();
                } else {
                    clampedY = FieldMap.FIELD_WIDTH / 2.0;
                    face = Rotation2d.fromDegrees(world.isRedAlliance() ? 180.0 : 0.0);
                }
                navTarget = new Pose2d(shadowX, clampedY, face);
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = opponentObserved
                        ? "Shadowing opponent across field midline."
                        : "Defending field midline; opponent unobserved.";
                break;

            case RUSH_CLIMB:
                if (!world.hasClimber()) {
                    // Belt-and-suspenders: RUSH_CLIMB is inserted first in the utility
                    // map, so a strict-greater max-selection would hand it an all-zero
                    // tie. Bots without a climber must never navigate to the tower (they would score
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
        boolean hasHopperSpace = world.ballCapacity() > 0 && world.heldFuelCount() < world.ballCapacity();
        if ((bestObjective.isDefensive() || bestObjective == StrategicObjective.STAGE_STANDOFF)
                && !isLowClearance && hasHopperSpace) {
            intakeCmd = IntakeState.INTAKING;
        }

        if (usedCloud) {
            rationale = "TypeSafe Jev selected " + bestObjective.name() + " (confidence "
                    + String.format("%.2f", maxUtility) + "). " + rationale;
        }

        double latencyMs = (System.nanoTime() - startNanos) / 1_000_000.0;
        double activeInertia = (commitmentIn != null) ? commitmentIn.activeInertia(nowSeconds) : 0.0;
        Logger.recordOutput("JevAI/ActiveObjective", bestObjective.name());
        Logger.recordOutput("JevAI/Confidence", maxUtility);
        Logger.recordOutput("JevAI/Rationale", rationale);
        Logger.recordOutput("JevAI/PolicyLatencyMs", latencyMs);
        Logger.recordOutput("JevAI/ActiveInertia", activeInertia);
        SmartDashboard.putNumber("JevAI/ActiveInertia", activeInertia);
        String telemetryPrefix = cloudContext == null ? "JevAI" : "JevAI/" + cloudContext;
        Logger.recordOutput(telemetryPrefix + "/UsingCloudAI", usedCloud);
        Logger.recordOutput(telemetryPrefix + "/ActiveDecisionMode", activeDecisionMode.name());
        Logger.recordOutput(telemetryPrefix + "/ActiveInertia", activeInertia);
        SmartDashboard.putBoolean(telemetryPrefix + "/UsingCloudAI", usedCloud);
        SmartDashboard.putString(telemetryPrefix + "/ActiveDecisionMode", activeDecisionMode.name());
        SmartDashboard.putNumber(telemetryPrefix + "/ActiveInertia", activeInertia);
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
                rationale, plan, tourResult);
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

    private static StrategicObjective evaluateLocalUtilityMatrix(
            Map<StrategicObjective, Double> utilities,
            WorldState world,
            Archetype archetype) {
        StrategicObjective bestObjective = null;
        double bestUtility = 0.0;
        for (Map.Entry<StrategicObjective, Double> entry : utilities.entrySet()) {
            if (entry.getValue() > bestUtility) {
                bestUtility = entry.getValue();
                bestObjective = entry.getKey();
            }
        }
        if (bestObjective != null) {
            return bestObjective;
        }

        // Explicit no-information / zero-utility degradation path:
        // When no objective has positive utility (> 0.0), degrade safely
        // according to archetype and inventory state. Never fall through to
        // RUSH_CLIMB outside endgame.
        if (archetype != null && archetype.isDefensive()) {
            return StrategicObjective.SHADOW_MIDLINE;
        }
        if (world != null && world.heldFuelCount() > 0 && world.hasShooter()) {
            return world.isAllianceHubActive()
                    ? StrategicObjective.CYCLE_SCORE_HUB
                    : StrategicObjective.STAGE_STANDOFF;
        }
        return (world != null && world.ballCapacity() > 0 && !world.isInventoryFull())
                ? StrategicObjective.VACUUM_MIDFIELD
                : StrategicObjective.SHADOW_MIDLINE;
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
        return findClusterWeightedFuelTarget(robotPose, isRedAlliance, isAutonomous, blockedFuel, null);
    }

    /**
     * Cluster-weighted fuel selection with optional per-agent target hysteresis.
     *
     * <p>{@code memory} is this agent's {@link FuelTargetMemory} latch. It must be
     * owned by the caller, never by this engine: a latch held on the shared
     * singleton leaks one bot's target into every other bot's decisions and into
     * the Co-Pilot (see {@link ObjectiveCommitment} for the incident). Pass
     * {@code null} for the legacy winner-take-all behaviour.
     */
    public Pose2d findClusterWeightedFuelTarget(Pose2d robotPose, boolean isRedAlliance,
            boolean isAutonomous, Set<Translation2d> blockedFuel,
            FuelTargetMemory memory) {
        return findClusterWeightedFuelTarget(robotPose, isRedAlliance, isAutonomous, blockedFuel, memory, null);
    }

    /**
     * Parameterized cluster-weighted fuel selector accepting an explicit candidate list.
     * When {@code candidatePieces} is null, falls back to {@link WorldStateBuilder#getFieldFuel()}.
     */
    public Pose2d findClusterWeightedFuelTarget(Pose2d robotPose, boolean isRedAlliance,
            boolean isAutonomous, Set<Translation2d> blockedFuel,
            FuelTargetMemory memory, List<Translation2d> candidatePieces) {
        return findClusterWeightedFuelTarget(robotPose, isRedAlliance, isAutonomous, blockedFuel, memory,
                candidatePieces, PolicyWeights.getActive());
    }

    /**
     * Parameterized cluster-weighted fuel selector accepting an explicit candidate list
     * and explicit {@link PolicyWeights}.
     */
    public Pose2d findClusterWeightedFuelTarget(Pose2d robotPose, boolean isRedAlliance,
            boolean isAutonomous, Set<Translation2d> blockedFuel,
            FuelTargetMemory memory, List<Translation2d> candidatePieces,
            PolicyWeights weights) {
        Translation2d bestTarget = null;
        double highestScent = -1.0;

        List<Translation2d> candidates = new ArrayList<>();
        List<FuelTargetMemory.ScoredTarget> scored = new ArrayList<>();

        List<Translation2d> pieces = (candidatePieces != null) ? candidatePieces : WorldStateBuilder.getFieldFuel();
        if (pieces != null && !pieces.isEmpty()) {
            for (Translation2d pos : pieces) {
                if (pos == null)
                    continue;

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

        PolicyWeights pw = (weights != null) ? weights : PolicyWeights.getActive();
        if (!candidates.isEmpty()) {
            final double clusterRadius = pw.clusterNeighborhoodRadius();
            final double sigma = pw.clusterKernelSigma();
            final double twoSigmaSq = 2.0 * sigma * sigma;

            int n = candidates.size();
            double[] densities = new double[n];
            for (int i = 0; i < n; i++) {
                densities[i] = 1.0;
            }

            for (int i = 0; i < n; i++) {
                Translation2d candI = candidates.get(i);
                for (int j = i + 1; j < n; j++) {
                    Translation2d candJ = candidates.get(j);
                    double d = candI.getDistance(candJ);
                    if (d <= clusterRadius) {
                        double addedDensity = Math.exp(-(d * d) / twoSigmaSq);
                        densities[i] += addedDensity;
                        densities[j] += addedDensity;
                    }
                }
            }

            for (int i = 0; i < n; i++) {
                Translation2d cand = candidates.get(i);
                double density = densities[i];

                double dist = robotPose.getTranslation().getDistance(cand);
                Translation2d delta = cand.minus(robotPose.getTranslation());
                double angleDiff = Math.abs(robotPose.getRotation().minus(delta.getAngle()).getRadians());
                double alignBonus = (1.0 - pw.harvestHeadingAlignScale())
                        + pw.harvestHeadingAlignScale() * Math.max(0.0, Math.cos(angleDiff));
                Translation2d homeCenter = new Translation2d(isRedAlliance ? 14.2 : 2.3,
                        FieldMap.FIELD_WIDTH / 2.0);
                Translation2d toHome = homeCenter.minus(cand);
                Translation2d travel = delta;
                double directionBonus = 0.0;
                if (toHome.getNorm() > 1e-9 && travel.getNorm() > 1e-9) {
                    directionBonus = pw.harvestReturnVectorBonus() * Math.max(0.0,
                            (travel.div(travel.getNorm())).dot(toHome.div(toHome.getNorm())));
                }

                double scent = (Math.pow(density, pw.clusterDensityExponent()) / (dist + pw.clusterDistanceFloor()))
                        * (alignBonus + directionBonus);
                scored.add(new FuelTargetMemory.ScoredTarget(cand, scent));

                if (scent > highestScent) {
                    highestScent = scent;
                    bestTarget = cand;
                }
            }
        }

        // Hysteresis, when the caller supplied per-agent memory. The raw
        // winner above is a function of the robot's instantaneous heading (the
        // alignBonus term), so comparable pieces trade places whenever the robot
        // turns slightly -- which resets TargetProgressWatchdog's no-progress
        // window every few ticks and produces STALLED_CHURN. A null memory
        // (Match Coach, unit tests) keeps the legacy winner-take-all behaviour.
        if (memory != null && !scored.isEmpty()) {
            Translation2d held = memory.resolve(scored);
            if (held != null) {
                bestTarget = held;
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
     * Plans an optimized sequential multi-piece fuel harvesting tour using
     * {@link FuelTourOptimizer}.
     *
     * @param robotPose        current robot pose
     * @param isRedAlliance    alliance orientation
     * @param isAutonomous     autonomous state (enforces G201)
     * @param maxPieces        maximum number of pieces to gather in this tour
     * @param blockedFuel      watchdog-abandoned pieces to skip
     * @return optimized TourResult with waypoints and immediate approach pose
     */
    public FuelTourOptimizer.TourResult planFuelHarvestTour(
            Pose2d robotPose,
            boolean isRedAlliance,
            boolean isAutonomous,
            int maxPieces,
            Set<Translation2d> blockedFuel) {
        Translation2d hubTarget = FieldMap.Hubs.getHubLocation2d(isRedAlliance);
        List<Translation2d> candidates = FuelTourOptimizer.findFieldFuelCandidates(
                robotPose, isRedAlliance, isAutonomous, blockedFuel);
        return FuelTourOptimizer.optimizeTour(robotPose, candidates, maxPieces, hubTarget);
    }

    /**
     * Selects the densest reachable Fuel cluster strictly inside our alliance zone.
     */
    public Pose2d findAllianceZoneFuelTarget(Pose2d robotPose, boolean isRedAlliance) {
        return findFuelTargetInZone(robotPose, isRedAlliance, null, null);
    }

    /** Zone-limited variant that skips caller-abandoned points. */
    public Pose2d findAllianceZoneFuelTarget(
            Pose2d robotPose, boolean isRedAlliance, Set<Translation2d> blockedFuel) {
        return findFuelTargetInZone(robotPose, isRedAlliance, blockedFuel, null);
    }

    public Pose2d findAllianceZoneFuelTarget(
            Pose2d robotPose, boolean isRedAlliance, Set<Translation2d> blockedFuel, List<Translation2d> fieldFuel) {
        return findFuelTargetInZone(robotPose, isRedAlliance, blockedFuel, fieldFuel);
    }

    /**
     * Collects reachable fuel pieces located strictly within the specified alliance zone.
     */
    public List<Translation2d> findAllianceZoneFuelCandidates(
            boolean isRedAlliance, Set<Translation2d> blockedFuel) {
        return findAllianceZoneFuelCandidates(isRedAlliance, blockedFuel, null);
    }

    public List<Translation2d> findAllianceZoneFuelCandidates(
            boolean isRedAlliance, Set<Translation2d> blockedFuel, List<Translation2d> fieldFuel) {
        List<Translation2d> candidates = new ArrayList<>();
        List<Translation2d> pieces = (fieldFuel != null) ? fieldFuel : WorldStateBuilder.getFieldFuel();
        if (pieces != null) {
            for (Translation2d point : pieces) {
                if (point == null) {
                    continue;
                }
                if (!FieldMap.AllianceZones.isInAllianceZone(point, isRedAlliance)) {
                    continue;
                }
                if (StaticPathfinder.isPointInHardObstacle(point)
                        || StaticPathfinder.isPointNearDynamicObstacle(point)) {
                    continue;
                }
                if (isBlocked(blockedFuel, point)) {
                    continue;
                }
                candidates.add(point);
            }
        }
        return candidates;
    }

    /**
     * Selects the densest reachable Fuel cluster strictly inside the opponent zone.
     */
    public Pose2d findOpponentZoneFuelTarget(Pose2d robotPose, boolean isRedAlliance) {
        return findFuelTargetInZone(robotPose, !isRedAlliance, null, null);
    }

    /** Opponent-zone variant that skips caller-abandoned points. */
    public Pose2d findOpponentZoneFuelTarget(
            Pose2d robotPose, boolean isRedAlliance, Set<Translation2d> blockedFuel) {
        return findFuelTargetInZone(robotPose, !isRedAlliance, blockedFuel, null);
    }

    public Pose2d findOpponentZoneFuelTarget(
            Pose2d robotPose, boolean isRedAlliance, Set<Translation2d> blockedFuel, List<Translation2d> fieldFuel) {
        return findFuelTargetInZone(robotPose, !isRedAlliance, blockedFuel, fieldFuel);
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

    private Pose2d findFuelTargetInZone(Pose2d robotPose, boolean zoneIsRed,
            Set<Translation2d> blockedFuel, List<Translation2d> fieldFuel) {
        Translation2d best = null;
        double bestScore = -1.0;
        List<Translation2d> pieces = (fieldFuel != null) ? fieldFuel : WorldStateBuilder.getFieldFuel();
        if (pieces != null && !pieces.isEmpty()) {
            List<Translation2d> candidates = new ArrayList<>();
            for (Translation2d point : pieces) {
                if (point == null)
                    continue;
                if (!FieldMap.AllianceZones.isInAllianceZone(point, zoneIsRed))
                    continue;
                if (StaticPathfinder.isPointInHardObstacle(point)
                        || StaticPathfinder.isPointNearDynamicObstacle(point))
                    continue;
                if (isBlocked(blockedFuel, point))
                    continue;
                candidates.add(point);
            }
            int n = candidates.size();
            double[] densities = new double[n];
            for (int i = 0; i < n; i++) {
                densities[i] = 1.0;
            }
            for (int i = 0; i < n; i++) {
                Translation2d candI = candidates.get(i);
                for (int j = i + 1; j < n; j++) {
                    Translation2d candJ = candidates.get(j);
                    double distance = candI.getDistance(candJ);
                    if (distance > 1e-9 && distance <= 1.3) {
                        double addedDensity = Math.exp(-(distance * distance) / 0.5);
                        densities[i] += addedDensity;
                        densities[j] += addedDensity;
                    }
                }
            }
            for (int i = 0; i < n; i++) {
                Translation2d candidate = candidates.get(i);
                double density = densities[i];
                double distance = robotPose.getTranslation().getDistance(candidate);
                double score = Math.pow(density, 1.5) / (distance + 0.4);
                if (score > bestScore) {
                    bestScore = score;
                    best = candidate;
                }
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
        return Math.max(0, available - countBlockedInZone(isRedZone, blockedFuel, knowledge.fieldFuel()));
    }

    /**
     * How many pieces the watchdog has abandoned in the zone being counted.
     */
    int countBlockedInZone(boolean isRedZone, Set<Translation2d> blockedFuel) {
        return countBlockedInZone(isRedZone, blockedFuel, null);
    }

    int countBlockedInZone(boolean isRedZone, Set<Translation2d> blockedFuel, List<Translation2d> fieldFuel) {
        if (blockedFuel == null || blockedFuel.isEmpty()) {
            return 0;
        }
        List<Translation2d> pieces = (fieldFuel != null) ? fieldFuel : WorldStateBuilder.getFieldFuel();
        if (pieces == null || pieces.isEmpty()) {
            return 0;
        }
        int blocked = 0;
        for (Translation2d at : pieces) {
            if (at == null) {
                continue;
            }
            // Same zone the raw count covered: single-owned by FieldMap.AllianceZones
            boolean inZone = FieldMap.AllianceZones.isInAllianceZone(at, isRedZone);
            if (inZone && isBlocked(blockedFuel, at)) {
                blocked++;
            }
        }
        return blocked;
    }

    private StrategicObjective resolveNextObjective(StrategicObjective current, WorldState world) {
        if (world != null && !world.hasShooter()) {
            return switch (current) {
                case SWEEP_ALLIANCE_ZONE, STOCKPILE_DEPOT, POACH_OPPONENT_ZONE ->
                    (world.ballCapacity() > 0 && !world.isInventoryFull())
                            ? StrategicObjective.VACUUM_MIDFIELD
                            : StrategicObjective.SHADOW_MIDLINE;
                case CHOKE_TRENCH, SCREEN_FOR_ALLY, BAIT_PIN_FOUL -> StrategicObjective.LEAD_INTERCEPT;
                default -> current;
            };
        }
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
