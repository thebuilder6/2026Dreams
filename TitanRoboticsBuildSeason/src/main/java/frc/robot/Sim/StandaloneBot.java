package frc.robot.Sim;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Data.Constants;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.ObjectiveCommitment;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;
import frc.robot.Navigation.FieldMap;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.intake.IntakeConstants;

/**
 * One kinematic Jev bot for the standalone runner.
 *
 * <p>Each tick the bot builds a {@link WorldState} plus a
 * {@link ClairvoyantKnowledge} from the runner-supplied roster views and
 * {@link FuelStore} (zone counts via {@code FieldMap.AllianceZones}, fuel list
 * passed explicitly — the same seam the engine's selectors already accept),
 * evaluates the shared {@link JevDecisionEngine}, then integrates the returned
 * {@link AIActionIntent}: drive toward {@code navigationTarget()} at capped
 * speed, geometric roller-box pickup on {@code INTAKING}, analytic scoring on
 * {@code triggerFeedKicker()}.
 *
 * <p>Roster semantics mirror
 * {@code WorldStateBuilder.buildMatchKnowledgeForSimBot}: ally lists hold
 * same-alliance bots including self, held/scored sums cover the alliance, and
 * the differential is own-minus-opponent.
 *
 * <p>Known fidelity gaps vs the full sim (deliberate for this cut):
 * <ul>
 *   <li>No shot spread or miss model — every gated volley converts fully.</li>
 *   <li>No fuel-target latch ({@code FuelTargetMemory} reads the wall clock) —
 *   the bot may oscillate between equidistant pieces.</li>
 *   <li>No contact watchdog, deadlock recovery, or trench handling — the runner
 *   applies only a positional peer push-out after each tick.</li>
 *   <li>Speed is {@code Constants.MAX_SPEED * 0.75}, the headless default
 *   ({@code AIRobotSim}: opponent 75%), with instant velocity changes.</li>
 *   <li>The defensive mark is the nearest opponent, not the threat-weighted
 *   assignment ({@code AIRobotSim.selectMarkExcluding}).</li>
 * </ul>
 */
public final class StandaloneBot {
    /** Pre-tick snapshot of one bot, as seen by its peers. */
    public record BotView(Pose2d pose, ChassisSpeeds velocity, int held, int scored) {}

    private Pose2d pose;
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private final Archetype archetype;
    private final boolean isRed;
    private int held;
    private int scored;
    private int autoScored;
    private int teleopScored;
    private int pickedUp;
    private double pathLengthM;
    private final Set<Translation2d> blockedFuel = new HashSet<>();
    private final ObjectiveCommitment commitment = new ObjectiveCommitment();
    private StrategicObjective lastObjective;
    private final double maxSpeedMps;

    public StandaloneBot(Pose2d startPose, Archetype archetype, boolean isRed, int preload) {
        this.pose = startPose;
        this.archetype = archetype;
        this.isRed = isRed;
        this.held = preload;
        // Headless default speed scale (AIRobotSim.java:460-462).
        this.maxSpeedMps = Constants.MAX_SPEED * 0.75;
    }

    public Pose2d getPose() {
        return pose;
    }

    /** Package-private: the runner's peer push-out writes back here. */
    void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public int getHeld() {
        return held;
    }

    public int getScored() {
        return scored;
    }

    public int getAutoScored() {
        return autoScored;
    }

    public int getTeleopScored() {
        return teleopScored;
    }

    public int getPickedUp() {
        return pickedUp;
    }

    public double getPathLengthM() {
        return pathLengthM;
    }

    public Archetype getArchetype() {
        return archetype;
    }

    public boolean isRed() {
        return isRed;
    }

    /** Objective chosen on the last tick, or null before the first tick. */
    public StrategicObjective getObjective() {
        return lastObjective;
    }

    public BotView view() {
        return new BotView(pose, velocity, held, scored);
    }

    /**
     * One fixed-dt tick: decide, move, pick up, shoot.
     *
     * @param allies    pre-tick views of same-alliance bots, including self
     * @param opponents pre-tick views of opposite-alliance bots
     * @param markPose  defensive mark (nearest opponent from the runner)
     * @param elapsedSec sim elapsed (the latch's time base — wall clock is frozen in a fast loop)
     */
    public void step(double dtSec, double matchTimeRemaining, boolean isAuto,
            FuelStore fuel, char shiftSeed, int scoreDifferential, double timeUntilShift,
            List<BotView> allies, List<BotView> opponents, Pose2d markPose, double elapsedSec) {
        HubSchedule.Phase phase = HubSchedule.phaseFor(matchTimeRemaining, isAuto);
        boolean myHub = HubSchedule.isHubActive(isRed, phase, shiftSeed);
        boolean oppHub = HubSchedule.isHubActive(!isRed, phase, shiftSeed);
        HubSchedule.Phase next = HubSchedule.nextPhase(phase);

        List<Translation2d> fieldFuel = fuel.positions();
        int own = 0;
        int mid = 0;
        int theirs = 0;
        for (Translation2d at : fieldFuel) {
            if (FieldMap.AllianceZones.isInMidfield(at)) {
                mid++;
            } else if (FieldMap.AllianceZones.isInAllianceZone(at, isRed)) {
                own++;
            } else {
                theirs++;
            }
        }

        List<Pose2d> allyPoses = new ArrayList<>();
        List<ChassisSpeeds> allyVels = new ArrayList<>();
        int alliesHeld = 0;
        int alliesScored = 0;
        for (BotView a : allies) {
            allyPoses.add(a.pose());
            allyVels.add(a.velocity());
            alliesHeld += a.held();
            alliesScored += a.scored();
        }
        List<Pose2d> oppPoses = new ArrayList<>();
        List<ChassisSpeeds> oppVels = new ArrayList<>();
        int oppHeld = 0;
        int oppScored = 0;
        for (BotView o : opponents) {
            oppPoses.add(o.pose());
            oppVels.add(o.velocity());
            oppHeld += o.held();
            oppScored += o.scored();
        }
        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                scoreDifferential, alliesHeld, oppHeld, alliesScored, oppScored,
                allyPoses, oppPoses, allyVels, oppVels,
                own, mid, theirs, new ArrayList<>(fieldFuel));

        WorldState world = new WorldState(
                pose, velocity, held,
                markPose, new ChassisSpeeds(),
                matchTimeRemaining, myHub, oppHub, timeUntilShift,
                isRed, isAuto,
                HubSchedule.isHubActive(isRed, next, shiftSeed),
                HubSchedule.isHubActive(!isRed, next, shiftSeed));

        AIActionIntent intent = JevDecisionEngine.getInstance()
                .evaluatePolicy(world, knowledge, archetype, null, blockedFuel,
                        commitment, null, null, elapsedSec);
        lastObjective = intent.objective();

        // Kinematic drive toward the intent target.
        Pose2d target = intent.navigationTarget();
        double vx = 0.0;
        double vy = 0.0;
        if (target != null) {
            Translation2d delta = target.getTranslation().minus(pose.getTranslation());
            double dist = delta.getNorm();
            if (dist > 1e-6) {
                double stepLen = Math.min(dist, maxSpeedMps * dtSec);
                Translation2d next2d = pose.getTranslation()
                        .plus(delta.times(stepLen / dist));
                pose = new Pose2d(next2d, new Rotation2d(Math.atan2(delta.getY(), delta.getX())));
                pathLengthM += stepLen;
                vx = delta.getX() / dist * stepLen / dtSec;
                vy = delta.getY() / dist * stepLen / dtSec;
            }
        }
        velocity = new ChassisSpeeds(vx, vy, 0.0);

        // Geometric pickup while the intent intakes.
        if (intent.intakeCommand() == IntakeState.INTAKING
                && held < IntakeConstants.MAX_HELD_BALLS) {
            int room = IntakeConstants.MAX_HELD_BALLS - held;
            int got = fuel.takeInRollerBox(pose.getTranslation(), pose.getRotation().getRadians(), room);
            held += got;
            pickedUp += got;
        }

        // Analytic shot resolution: gated volley converts fully.
        if (intent.triggerFeedKicker() && held > 0 && myHub
                && FieldMap.AllianceZones.isInAllianceZone(pose, isRed)
                && hubTranslation().getDistance(pose.getTranslation())
                        <= FieldMap.Hubs.SHOOTING_MAX_DISTANCE) {
            if (isAuto) {
                autoScored += held;
            } else {
                teleopScored += held;
            }
            scored += held;
            held = 0;
        }
    }

    private Translation2d hubTranslation() {
        return isRed
                ? new Translation2d(FieldMap.Hubs.RED_HUB_X, FieldMap.Hubs.HUB_Y)
                : new Translation2d(FieldMap.Hubs.BLUE_HUB_X, FieldMap.Hubs.HUB_Y);
    }
}
