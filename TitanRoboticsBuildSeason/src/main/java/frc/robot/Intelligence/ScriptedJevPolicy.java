package frc.robot.Intelligence;

import java.util.Collections;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.ShooterState;

/**
 * Scripted harvest-then-score policy: the executable counterpart of the
 * Python baseline in {@code JevIntelligence/jev-sim} ({@code policy.py}).
 *
 * <p>Decision rule (mirrors {@code decide()} there, verified against 54/54
 * applicable rows of {@code tools/score/decision_cards.tsv}): score the whole
 * hopper when the own hub is live and fuel meets the gate (batch of 8, or 1
 * inside shooting range, batch of 8 in autonomous); otherwise harvest the
 * nearest visible fuel. Dark-hub staging and defense are out of scope by
 * design — install this only where harvest/score covers the job.
 *
 * <p>Stateless: per-agent latches ({@code commitmentIn},
 * {@code fuelTargetMemory}) and {@code weights} are accepted and ignored.
 * Emits no telemetry so it never fights the engine's {@code JevAI/*} keys.
 */
public class ScriptedJevPolicy implements JevPolicy {

    /** Mirrors Jev AUTO_BATCH_MIN_FUEL and the Python BATCH. */
    public static final int BATCH_MIN_FUEL = 8;
    /**
     * Mirrors {@code PolicyWeights.scoreHubShootingRangeMeters} default: inside
     * this radius a single fuel justifies a scoring trip.
     */
    public static final double SCORE_RANGE_M = 4.0;

    /** Engine firing envelope (JevDecisionEngine CYCLE_SCORE_HUB branch). */
    private static final double FIRE_MIN_M = 1.40;
    private static final double FIRE_MAX_M = 3.60;
    private static final double FIRE_HEADING_DEG = 8.0;

    @Override
    public AIActionIntent evaluatePolicy(
            WorldState world, MatchKnowledge knowledge, Archetype archetype,
            String cloudContext, Set<Translation2d> blockedFuel,
            ObjectiveCommitment commitmentIn, FuelTargetMemory fuelTargetMemory,
            PolicyWeights weights, double nowSeconds) {
        if (knowledge == null) {
            knowledge = ObservedKnowledge.selfOnly();
        }
        if (blockedFuel == null) {
            blockedFuel = Collections.emptySet();
        }

        Translation2d selfHub = FieldMap.Hubs.getHubLocation2d(world.isRedAlliance());
        double distToHub = world.selfPose().getTranslation().getDistance(selfHub);
        int minFuel = world.isAutonomous() ? BATCH_MIN_FUEL
                : (distToHub <= SCORE_RANGE_M ? 1 : BATCH_MIN_FUEL);
        boolean score = world.hasShooter()
                && world.heldFuelCount() >= minFuel
                && world.isAllianceHubActive();

        if (score) {
            return scoreIntent(world, selfHub, distToHub);
        }
        return harvestIntent(world, knowledge, blockedFuel);
    }

    private AIActionIntent scoreIntent(WorldState world, Translation2d selfHub, double distToHub) {
        Pose2d navTarget = JevDecisionEngine.getInstance()
                .calculatePolarStandoffPose(world.selfPose(), world.isRedAlliance());
        ShooterState shooterCmd = ShooterState.PREPARING;
        Rotation2d aimOverride = null;
        boolean triggerKicker = false;
        if (distToHub >= FIRE_MIN_M && distToHub <= FIRE_MAX_M
                && FieldMap.AllianceZones.isInAllianceZone(world.selfPose(), world.isRedAlliance())
                && !FieldMap.Trenches.isLowClearance(world.selfPose())) {
            // Lane check omitted: isShootingLaneBlocked is engine-private, and
            // the engine itself assumes clear when no opponent is tracked.
            aimOverride = selfHub.minus(world.selfPose().getTranslation()).getAngle();
            double headingErr = Math.abs(world.selfPose().getRotation().minus(aimOverride).getDegrees());
            if (headingErr < FIRE_HEADING_DEG) {
                shooterCmd = ShooterState.SHOOTING;
                triggerKicker = true;
            }
        }
        return new AIActionIntent(StrategicObjective.CYCLE_SCORE_HUB, navTarget, aimOverride,
                IntakeState.STANDBY, shooterCmd, 3200.0, triggerKicker, 0.98,
                String.format("Scripted: cycling %d fuel to Hub.", world.heldFuelCount()));
    }

    private AIActionIntent harvestIntent(WorldState world, MatchKnowledge knowledge,
            Set<Translation2d> blockedFuel) {
        Translation2d best = null;
        double bestDist = Double.MAX_VALUE;
        for (Translation2d piece : knowledge.fieldFuel()) {
            if (blockedFuel.contains(piece)) {
                continue;
            }
            double d = piece.getDistance(world.selfPose().getTranslation());
            if (d < bestDist) {
                bestDist = d;
                best = piece;
            }
        }
        Pose2d navTarget = (best == null) ? world.selfPose()
                : new Pose2d(best, world.selfPose().getRotation());
        return new AIActionIntent(StrategicObjective.VACUUM_MIDFIELD, navTarget, null,
                IntakeState.INTAKING, ShooterState.STOPPED, 0.0, false, 0.85,
                String.format("Scripted: hunting fuel (%d/%d).", world.heldFuelCount(),
                        world.ballCapacity()));
    }
}
