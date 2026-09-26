package frc.robot.Intelligence;

import frc.robot.Subsystems.Intake;

import frc.robot.Subsystems.Shooter;

import frc.robot.Telemetry.Dashboard;

import frc.robot.Intelligence.State.WorldState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.DynamicObstacle;
import frc.robot.Auto.DynamicRouter;
import frc.robot.Auto.StaticPathfinder;
import frc.robot.Data.Constants;
import frc.robot.Data.FieldMap;
import frc.robot.Data.GlideConstants;
import frc.robot.Data.GlideConstants.GlidePoint;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Jev AI Decision Engine (TypeSafe AI)
 * 
 * Cognitive Architecture:
 * - System 2: Strategic utility model evaluating match time, Hub active phase, and fuel inventory.
 * - System 1: High-frequency (<1ms) tactical policy generating concrete mechanism and drive intents.
 */
public class JevDecisionEngine {

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
                if (i++ > 0) sb.append(",");
                sb.append("\"").append(entry.getKey().name()).append("\":").append(String.format("%.3f", entry.getValue()));
            }
            sb.append("}}");
            return sb.toString();
        }
    }

    public static final Translation2d BLUE_HUB_POS = FieldMap.Hubs.BLUE_HUB_2D;
    public static final Translation2d BLUE_DEPOT_POS = FieldMap.Depots.BLUE_DEPOT_CONTEST;
    public static final double CENTERLINE_X = FieldMap.CENTERLINE_X;
    public static final double RETREAT_X = 12.0;

    private static JevDecisionEngine instance;

    public static JevDecisionEngine getInstance() {
        if (instance == null) {
            instance = new JevDecisionEngine();
        }
        return instance;
    }

    private DecisionResult lastDecision = null;

    // =========================================================================
    // JEV TYPE-SAFE POLICY ENGINE (Option A & Option B)
    // =========================================================================

    /**
     * Evaluates WorldState through the Jev Macro Utility Matrix and outputs a concrete AIActionIntent.
     * Guaranteed deterministic, stateless (re-entrant for multi-bot simulations), and sub-millisecond latency.
     *
     * @param world State snapshot of the match and robots
     * @param archetype AI persona / behavior profile
     * @return Fully specified AIActionIntent
     */
    public AIActionIntent evaluatePolicy(WorldState world, Archetype archetype) {
        long startNanos = System.nanoTime();

        Translation2d selfHub = FieldMap.Hubs.getHubLocation2d(world.isRedAlliance());
        double distToSelfHub = world.selfPose().getTranslation().getDistance(selfHub);

        // ── 1. Evaluate Utility Scores Across Objectives ─────────────────────
        Map<StrategicObjective, Double> utilities = new LinkedHashMap<>();

        // Rush Climb (Endgame priority)
        double climbUtility = 0.0;
        if (world.matchTimeRemaining() > 0.0 && world.matchTimeRemaining() <= 20.0) {
            if (world.matchTimeRemaining() <= 15.0) {
                // Dominant endgame priority: strictly overrides cycling in final 15 seconds
                climbUtility = 0.99 + (0.01 * (1.0 - world.matchTimeRemaining() / 15.0));
            } else {
                climbUtility = 0.85 + (0.13 * (1.0 - (world.matchTimeRemaining() - 15.0) / 5.0));
            }
        }
        utilities.put(StrategicObjective.RUSH_CLIMB, climbUtility);

        // Cycle Score Hub
        // If already in shooting range (dist <= 4.0m) or Co-Pilot assist, keep firing down to the very last ball!
        // If out in midfield, require a solid batch (>=16) unless the active shift is about to end (<=4.5s)
        boolean inShootingRange = distToSelfHub <= 4.0;
        boolean shiftEndingSoon = world.timeUntilHubShift() <= 4.5 && world.timeUntilHubShift() > 0.0;
        int minFuelToScore = (archetype == Archetype.CO_PILOT || inShootingRange) ? 1 : (shiftEndingSoon ? 4 : 16);

        double scoreUtility = 0.0;
        if (world.isAllianceHubActive() && world.heldFuelCount() >= minFuelToScore) {
            double loadRatio = (archetype == Archetype.CO_PILOT || inShootingRange) 
                    ? 1.0 
                    : Math.min(1.0, world.heldFuelCount() / 20.0);
            scoreUtility = 0.72 + (0.26 * loadRatio); // 0.72 to 0.98
        }
        utilities.put(StrategicObjective.CYCLE_SCORE_HUB, scoreUtility);

        // Stage Standoff (Hub is inactive; wait at standoff arc once hopper is well stocked)
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

        // Defense Objectives
        double laneDenialUtility = 0.0;
        double shadowUtility = 0.0;
        double interceptUtility = 0.0;

        if (archetype == Archetype.TACTICAL_DEFENDER || archetype == Archetype.DEFENSE_BULLY || archetype == Archetype.ADAPTIVE_COMPETITOR) {
            Translation2d oppHub = FieldMap.Hubs.getHubLocation2d(!world.isRedAlliance());
            double oppDistToHub = world.opponentPose().getTranslation().getDistance(oppHub);

            if (world.isOpponentHubActive() && oppDistToHub < 6.5) {
                laneDenialUtility = 0.86;
            }
            shadowUtility = 0.65;
            interceptUtility = 0.75;
        }

        // Apply Archetype Multipliers
        if (archetype == Archetype.AUTONOMOUS_CYCLER) {
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

        utilities.put(StrategicObjective.CYCLE_SCORE_HUB, scoreUtility);
        utilities.put(StrategicObjective.STAGE_STANDOFF, stageUtility);
        utilities.put(StrategicObjective.VACUUM_MIDFIELD, vacuumUtility);
        utilities.put(StrategicObjective.STOCKPILE_DEPOT, stockpileUtility);
        utilities.put(StrategicObjective.DENY_SHOOTING_LANE, laneDenialUtility);
        utilities.put(StrategicObjective.SHADOW_MIDLINE, shadowUtility);
        utilities.put(StrategicObjective.LEAD_INTERCEPT, interceptUtility);

        // ── 2. Select Highest Utility Objective ──────────────────────────────
        StrategicObjective bestObjective = StrategicObjective.VACUUM_MIDFIELD;
        double maxUtility = -1.0;
        for (Map.Entry<StrategicObjective, Double> entry : utilities.entrySet()) {
            if (entry.getValue() > maxUtility) {
                maxUtility = entry.getValue();
                bestObjective = entry.getKey();
            }
        }

        // ── 3. Resolve Concrete Tactical Action Intent ───────────────────────
        Pose2d navTarget;
        Rotation2d aimOverride = null;
        IntakeState intakeCmd = IntakeState.STANDBY;
        ShooterState shooterCmd = ShooterState.STOPPED;
        double targetRPM = 0.0;
        boolean triggerKicker = false;
        String rationale;

        switch (bestObjective) {
            case CYCLE_SCORE_HUB:
                navTarget = calculatePolarStandoffPose(world.selfPose(), world.isRedAlliance());
                intakeCmd = IntakeState.STANDBY;

                // Pre-spool flywheels during transit
                shooterCmd = ShooterState.PREPARING;
                targetRPM = 3200.0;

                // Fire evaluation if within shooting range
                boolean inRange = distToSelfHub <= 3.60 && distToSelfHub >= 1.40;
                boolean openCeiling = !FieldMap.Trenches.isLowClearance(world.selfPose());
                boolean laneClear = !isShootingLaneBlocked(world.selfPose(), selfHub, world.opponentPose());

                if (inRange && openCeiling && laneClear) {
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
                navTarget = findClusterWeightedFuelTarget(world.selfPose(), world.isRedAlliance());
                intakeCmd = IntakeState.INTAKING;
                shooterCmd = ShooterState.STOPPED;
                rationale = String.format("Hunting fuel (%d/30). Hopper capacity available.", world.heldFuelCount());
                break;

            case LEAD_INTERCEPT:
                navTarget = solveLeadPursuitIntercept(
                        world.selfPose(), world.opponentPose(),
                        new Translation2d(world.opponentVelocity().vxMetersPerSecond, world.opponentVelocity().vyMetersPerSecond),
                        Constants.MAX_SPEED);
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Pursuing and intercepting opponent trajectory.";
                break;

            case DENY_SHOOTING_LANE:
                Translation2d oppGoal = FieldMap.Hubs.getHubLocation2d(!world.isRedAlliance());
                Translation2d dir = oppGoal.minus(world.opponentPose().getTranslation());
                if (dir.getNorm() > 1e-3) dir = dir.div(dir.getNorm());
                Translation2d blockPos = world.opponentPose().getTranslation().plus(dir.times(1.5));
                Rotation2d faceOpp = world.opponentPose().getTranslation().minus(blockPos).getAngle();
                navTarget = new Pose2d(blockPos, faceOpp);
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Blocking opponent shooting corridor.";
                break;

            case SHADOW_MIDLINE:
                double shadowX = world.isRedAlliance() ? (CENTERLINE_X + 0.8) : (CENTERLINE_X - 0.8);
                double clampedY = Math.max(1.0, Math.min(FieldMap.FIELD_WIDTH - 1.0, world.opponentPose().getY()));
                Rotation2d face = world.opponentPose().getTranslation().minus(new Translation2d(shadowX, clampedY)).getAngle();
                navTarget = new Pose2d(shadowX, clampedY, face);
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Shadowing opponent across field midline.";
                break;

            case RUSH_CLIMB:
                String parkKey = world.isRedAlliance() ? "Red Right Side Climb" : "Blue Right Side Climb";
                if (GlideConstants.GLIDE_POINTS.containsKey(parkKey)) {
                    navTarget = GlideConstants.GLIDE_POINTS.get(parkKey).pose();
                } else {
                    Translation2d pole = FieldMap.ClimbingTowers.getTowerPole(world.isRedAlliance());
                    navTarget = new Pose2d(pole.plus(new Translation2d(world.isRedAlliance() ? -0.8 : 0.8, 0.0)), 
                            Rotation2d.fromDegrees(world.isRedAlliance() ? 0 : 180));
                }
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = String.format("Endgame (%.1fs remaining). Navigating to Alliance Parking.", world.matchTimeRemaining());
                break;

            case IDLE:
            default:
                navTarget = world.selfPose();
                intakeCmd = IntakeState.STANDBY;
                shooterCmd = ShooterState.STOPPED;
                rationale = "Idle.";
                break;
        }

        double latencyMs = (System.nanoTime() - startNanos) / 1_000_000.0;
        Logger.recordOutput("JevAI/ActiveObjective", bestObjective.name());
        Logger.recordOutput("JevAI/Confidence", maxUtility);
        Logger.recordOutput("JevAI/Rationale", rationale);
        Logger.recordOutput("JevAI/PolicyLatencyMs", latencyMs);

        return new AIActionIntent(
                bestObjective, navTarget, aimOverride, intakeCmd, shooterCmd, targetRPM, triggerKicker, maxUtility, rationale);
    }

    /**
     * Backward-compatible overload accepting legacy AIArchetype.
     */
    public AIActionIntent evaluatePolicy(WorldState world, AIArchetype archetype) {
        return evaluatePolicy(world, Archetype.fromString(archetype.name()));
    }

    /**
     * Spatial cluster-density piece scent algorithm (Option B).
     * Replaces single closest ball search with a Gaussian density field evaluator that targets
     * dense rows/clusters of 3-6 pieces (depot lines, centerline grid) in continuous sweeps.
     *
     * @param robotPose Current pose of the robot seeking fuel
     * @param isRedAlliance True if robot is on Red Alliance
     * @return Target Pose2d on carpet facing the highest-density fuel cluster
     */
    public Pose2d findClusterWeightedFuelTarget(Pose2d robotPose, boolean isRedAlliance) {
        SimulatedArena arena = SimulatedArena.getInstance();
        Translation2d bestTarget = null;
        double highestScent = -1.0;

        List<Translation2d> candidates = new ArrayList<>();

        if (arena != null) {
            try {
                Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
                if (pieces != null && !pieces.isEmpty()) {
                    for (var piece : pieces) {
                        if (piece == null || !"Fuel".equals(piece.getType())) continue;
                        Translation2d pos = piece.getPoseOnField().getTranslation();

                        // Field boundaries & obstacle avoidance
                        if (pos.getX() < 0.05 || pos.getX() > 16.48 || pos.getY() < 0.05 || pos.getY() > 8.00) continue;
                        if (StaticPathfinder.isPointInObstacle(pos)) continue;

                        // Restrict opposing driver wall zone
                        if (isRedAlliance && pos.getX() < 3.5) continue;
                        if (!isRedAlliance && pos.getX() > 13.0) continue;

                        candidates.add(pos);
                    }
                }
            } catch (Exception ignored) {}
        }

        if (!candidates.isEmpty()) {
            final double clusterRadius = 1.30;
            final double twoSigmaSq = 2.0 * 0.50 * 0.50; // sigma = 0.50m

            for (int i = 0; i < candidates.size(); i++) {
                Translation2d cand = candidates.get(i);
                double density = 1.0;

                for (int j = 0; j < candidates.size(); j++) {
                    if (i == j) continue;
                    double d = cand.getDistance(candidates.get(j));
                    if (d <= clusterRadius) {
                        density += Math.exp(-(d * d) / twoSigmaSq);
                    }
                }

                double dist = robotPose.getTranslation().getDistance(cand);
                Translation2d delta = cand.minus(robotPose.getTranslation());
                double angleDiff = Math.abs(robotPose.getRotation().minus(delta.getAngle()).getRadians());
                double alignBonus = 0.70 + 0.30 * Math.max(0.0, Math.cos(angleDiff));

                double scent = (Math.pow(density, 1.5) / (dist + 0.40)) * alignBonus;

                if (scent > highestScent) {
                    highestScent = scent;
                    bestTarget = cand;
                }
            }
        }

        if (bestTarget != null) {
            double approachX = bestTarget.getX();
            double approachY = bestTarget.getY();
            Rotation2d targetHeading;

            if (bestTarget.getX() > 15.60) {
                approachX = Math.min(15.98, bestTarget.getX() - 0.48);
                targetHeading = Rotation2d.fromDegrees(0);
            } else if (bestTarget.getX() < 0.90) {
                approachX = Math.max(0.55, bestTarget.getX() + 0.48);
                targetHeading = Rotation2d.fromDegrees(180);
            } else if (bestTarget.getY() < 0.90) {
                approachY = Math.max(0.55, bestTarget.getY() + 0.48);
                targetHeading = Rotation2d.fromDegrees(-90);
            } else if (bestTarget.getY() > 7.15) {
                approachY = Math.min(7.50, bestTarget.getY() - 0.48);
                targetHeading = Rotation2d.fromDegrees(90);
            } else {
                targetHeading = bestTarget.minus(robotPose.getTranslation()).getAngle();
                Translation2d offset = new Translation2d(0.35, 0).rotateBy(targetHeading);
                approachX = bestTarget.getX() - offset.getX();
                approachY = bestTarget.getY() - offset.getY();
            }

            Translation2d targetPos = new Translation2d(approachX, approachY);
            return StaticPathfinder.ensurePoseOutsideObstacles(new Pose2d(targetPos, targetHeading), robotPose.getTranslation());
        }

        // Fallback: Midline patrol
        double midX = CENTERLINE_X;
        double midY = (robotPose.getY() > 4.0) ? 5.80 : 2.40;
        return StaticPathfinder.ensurePoseOutsideObstacles(
                new Pose2d(midX, midY, Rotation2d.fromDegrees(isRedAlliance ? 180 : 0)), robotPose.getTranslation());
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

        Translation2d targetHub = isRedAlliance ? 
                new Translation2d(Constants.RED_HUB_LOCATION.getX(), Constants.RED_HUB_LOCATION.getY()) : 
                BLUE_HUB_POS;
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
        if (matchTimeRemaining < 15.0) {
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
                rationale = String.format("Player approaching Depot (%.2fm away); contesting game piece loading.", distToDepot);
                break;

            case RETREAT_DEFENSE:
                boolean opponentIsRed = !isRedAlliance;
                double retreatX = opponentIsRed ? (AllianceFlipUtil.FIELD_LENGTH - RETREAT_X) : RETREAT_X;
                targetPose = new Pose2d(retreatX, 4.0, Rotation2d.fromDegrees(opponentIsRed ? 180 : 0));
                rationale = isHubActive ? "Falling back to alliance defense perimeter." : "Hub inactive; holding defensive position.";
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
        Translation2d targetHub = isRedAlliance ?
                new Translation2d(Constants.RED_HUB_LOCATION.getX(), Constants.RED_HUB_LOCATION.getY()) :
                BLUE_HUB_POS;
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
            waypoint = new Pose2d(targetHub.plus(new Translation2d(isRedAlliance ? 2.5 : -2.5, 0.0)), Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
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

    public Pose2d getSmartGlideTarget(
            Pose2d robotPose,
            boolean hasFuel,
            boolean isHubActive,
            boolean isRedAlliance) {

        Pose2d targetPose;
        String mode;

        double matchTime = Timer.getMatchTime();
        if (matchTime > 0.0 && matchTime <= 20.0) {
            String parkKey = isRedAlliance ? "Red Right Side Climb" : "Blue Right Side Climb";
            targetPose = GlideConstants.GLIDE_POINTS.containsKey(parkKey) ?
                    GlideConstants.GLIDE_POINTS.get(parkKey).pose() :
                    new Pose2d(isRedAlliance ? 15.48 : 1.05, 2.88, Rotation2d.fromDegrees(isRedAlliance ? 0 : 180));
            mode = "ENDGAME_PARK (" + (isRedAlliance ? "Red" : "Blue") + ")";
        } else if (hasFuel && isHubActive) {
            String frontKey = isRedAlliance ? "Red Hub Front" : "Blue Hub Front";
            String backKey = isRedAlliance ? "Red Hub Back" : "Blue Hub Back";

            Pose2d frontPose = GlideConstants.GLIDE_POINTS.containsKey(frontKey) ? 
                    GlideConstants.GLIDE_POINTS.get(frontKey).pose() : 
                    new Pose2d(isRedAlliance ? 11.0 : 5.6, 4.10, Rotation2d.fromDegrees(isRedAlliance ? 0 : 180));
            Pose2d backPose = GlideConstants.GLIDE_POINTS.containsKey(backKey) ? 
                    GlideConstants.GLIDE_POINTS.get(backKey).pose() : 
                    new Pose2d(isRedAlliance ? 13.9 : 2.6, 4.10, Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));

            double distFront = robotPose.getTranslation().getDistance(frontPose.getTranslation());
            double distBack = robotPose.getTranslation().getDistance(backPose.getTranslation());

            targetPose = (distFront <= distBack) ? frontPose : backPose;
            mode = "SCORE_HUB (" + (distFront <= distBack ? "Front" : "Back") + ")";
        } else if (!hasFuel && isHubActive) {
            Pose2d topMid = GlideConstants.GLIDE_POINTS.containsKey("Midfield Top") ?
                    GlideConstants.GLIDE_POINTS.get("Midfield Top").pose() :
                    new Pose2d(CENTERLINE_X, 6.10, Rotation2d.fromDegrees(-90));
            Pose2d botMid = GlideConstants.GLIDE_POINTS.containsKey("Midfield Bottom") ?
                    GlideConstants.GLIDE_POINTS.get("Midfield Bottom").pose() :
                    new Pose2d(CENTERLINE_X, 2.00, Rotation2d.fromDegrees(90));

            double distTop = Math.abs(robotPose.getY() - topMid.getY());
            double distBot = Math.abs(robotPose.getY() - botMid.getY());

            targetPose = (distTop <= distBot) ? topMid : botMid;
            mode = "BALL_HUNT_MIDFIELD (" + (distTop <= distBot ? "Top" : "Bottom") + ")";
        } else {
            String topFeederKey = isRedAlliance ? "Red Feeder Top" : "Blue Feeder Top";
            String botFeederKey = isRedAlliance ? "Red Feeder Bottom" : "Blue Feeder Bottom";

            Pose2d topFeeder = GlideConstants.GLIDE_POINTS.containsKey(topFeederKey) ?
                    GlideConstants.GLIDE_POINTS.get(topFeederKey).pose() :
                    new Pose2d(isRedAlliance ? 15.0 : 1.5, 6.0, Rotation2d.fromDegrees(isRedAlliance ? -145 : -35));
            Pose2d botFeeder = GlideConstants.GLIDE_POINTS.containsKey(botFeederKey) ?
                    GlideConstants.GLIDE_POINTS.get(botFeederKey).pose() :
                    new Pose2d(isRedAlliance ? 15.0 : 1.5, 2.2, Rotation2d.fromDegrees(isRedAlliance ? 145 : 35));

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

        if (opponentPose == null) return robotPose;
        if (opponentVel == null) opponentVel = new Translation2d();
        if (maxRobotSpeed <= 0.1) maxRobotSpeed = Constants.MAX_SPEED;

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

        GlidePoint topPoint = GlideConstants.GLIDE_POINTS.get(topKey);
        GlidePoint botPoint = GlideConstants.GLIDE_POINTS.get(botKey);

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
