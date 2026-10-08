package frc.robot.Sim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Data.Constants;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.FuelTargetMemory;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.ObjectiveCommitment;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;
import frc.robot.Navigation.FieldMap;
import frc.robot.Navigation.TargetProgressWatchdog;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.shooter.ShooterConstants;
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
 * <p>The runner now carries the same per-agent latches the full sim does, all
 * driven from <b>sim time</b> (the wall clock is frozen in a fast-forward run):
 * <ul>
 *   <li>{@link ObjectiveCommitment} — passed through the engine's {@code nowSeconds}
 *   overload; already owned by this bot before this change.</li>
 *   <li>{@link FuelTargetMemory} — fuel-piece stickiness, so a heading swing
 *   cannot oscillate the harvest target.</li>
 *   <li>{@link TargetProgressWatchdog} — unreachable-target give-up and the
 *   stationary-harvester arrival watchdog. Blacklisted points are re-stamped
 *   onto a sim-time TTL set ({@code blockedUntil}) rather than the watchdog's
 *   own wall-clock blacklist, and its escape vector overrides the drive for the
 *   duration of the recovery.</li>
 * </ul>
 *
 * <p>Remaining fidelity gaps vs the full sim (deliberate):
 * <ul>
 *   <li>The shot model is analytic, not physics: a seeded volley propagates the
 *   real launch solution plus the {@code AIRobotInstance.launchShot} spread and
 *   scores a ball if its flight passes through the funnel tolerance box
 *   ({@link #FUNNEL_TOL_XY_M} / {@link #FUNNEL_TOL_Z_M}) around the funnel
 *   centre. Missed balls are discarded rather than re-grounded.</li>
 *   <li><b>No fuel recycling.</b> The full sim's RebuiltHub physically returns
 *   scored fuel to the field ({@code GameSim.handleShotsAndScoring}), so its
 *   throughput is unbounded; here the field is a fixed pool, so every match
 *   scores out nearly all of it and the result saturates. This is the dominant
 *   reason the standalone does not yet track the full sim — see the A/B note in
 *   {@code KNOWN_ISSUES.md} §E.</li>
 *   <li>No static-obstacle collision or trench handling — drive is a straight
 *   line to the target, and the runner applies only a positional peer push-out
 *   after each tick.</li>
 *   <li>The defensive mark is the nearest opponent, not the threat-weighted
 *   assignment ({@code AIRobotSim.selectMarkExcluding}).</li>
 * </ul>
 */
public final class StandaloneBot {
    /** Funnel entry height the full sim aims at (`AIRobotInstance.launchShot`). */
    static final double FUNNEL_Z_M = 1.48;
    /** Shooter exit height, m. */
    static final double SHOOTER_Z_M = 0.53;
    /** Funnel half-extents, m — the tolerance `AIRobotInstance` sets on the projectile. */
    static final double FUNNEL_TOL_XY_M = 0.38;
    static final double FUNNEL_TOL_Z_M = 0.20;
    /** Longest flight time sampled when resolving a shot, s. */
    static final double FLIGHT_MAX_SEC = 3.0;
    /** Flight sampling step, s. */
    static final double FLIGHT_SAMPLE_SEC = 0.002;

    /**
     * Endgame window (seconds remaining) in which a climb can register. Mirrors
     * the earliest point {@code RUSH_CLIMB} can win the utility race.
     */
    public static final double CLIMB_WINDOW_SEC = 20.0;

    /**
     * Tower-proximity radius for a successful climb. Mirrors
     * {@code MatchScoreTracker.TOWER_CLIMB_RADIUS_METERS} (single-owned there for
     * the full sim; duplicated so the MapleSim-free runner needs no subsystem
     * singleton).
     */
    public static final double CLIMB_RADIUS_M = 1.20;

    /** Pre-tick snapshot of one bot, as seen by its peers. */
    public record BotView(Pose2d pose, ChassisSpeeds velocity, int held, int scored) {}

    private Pose2d pose;
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private final Archetype archetype;
    private final boolean isRed;
    private final String shotStream;
    private int held;
    private int scored;
    private int autoScored;
    private int teleopScored;
    private int pickedUp;
    private int attemptedShots;
    private int missedShots;
    private int escapes;
    private double pathLengthM;
    private int wastedFuel;
    private int shuttledFuel;
    private boolean climbed;
    private double climbArrivalSec = -1.0;
    private int minorFouls;
    private int majorFouls;
    private double hubActiveTeleopSec;
    private final boolean hasClimber;
    /** Sim-time expiry for watchdog-abandoned fuel points, key -> expiry seconds. */
    private final Map<Translation2d, Double> blockedUntil = new HashMap<>();
    private final ObjectiveCommitment commitment = new ObjectiveCommitment();
    private final FuelTargetMemory fuelTargetMemory = new FuelTargetMemory();
    private final TargetProgressWatchdog progressWatchdog = new TargetProgressWatchdog();
    private StrategicObjective lastObjective;
    private final double maxSpeedMps;

    public StandaloneBot(Pose2d startPose, Archetype archetype, boolean isRed, int preload, int rosterIndex) {
        this(startPose, archetype, isRed, preload, rosterIndex, false);
    }

    /**
     * @param hasClimber whether this slot has a working climber. Drives
     *                   {@code RUSH_CLIMB} eligibility and the climb metrics.
     */
    public StandaloneBot(Pose2d startPose, Archetype archetype, boolean isRed, int preload,
            int rosterIndex, boolean hasClimber) {
        this.pose = startPose;
        this.archetype = archetype;
        this.isRed = isRed;
        this.held = preload;
        this.shotStream = "standalone-shot:" + rosterIndex;
        this.hasClimber = hasClimber;
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

    /** Package-private test seam: set the commanded velocity the referee reads. */
    void setVelocity(ChassisSpeeds velocity) {
        this.velocity = velocity;
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

    public int getAttemptedShots() {
        return attemptedShots;
    }

    public int getMissedShots() {
        return missedShots;
    }

    /** Fuel launched without scoring (dead hub, out of zone/range, or a miss). */
    public int getWastedFuel() {
        return wastedFuel;
    }

    /** Fuel deliberately lobbed home on {@code SHUTTLE_PASS} (not a wasted shot). */
    public int getShuttledFuel() {
        return shuttledFuel;
    }

    public boolean hasClimber() {
        return hasClimber;
    }

    public boolean isClimbed() {
        return climbed;
    }

    /** Seconds into the endgame window when the climb registered, or {@code -1}. */
    public double getClimbArrivalSec() {
        return climbArrivalSec;
    }

    public int getMinorFouls() {
        return minorFouls;
    }

    public int getMajorFouls() {
        return majorFouls;
    }

    /** Teleop seconds this bot's own hub was live (scoring opportunity time). */
    public double getHubActiveTeleopSec() {
        return hubActiveTeleopSec;
    }

    public void recordMinorFoul() {
        minorFouls++;
    }

    public void recordMajorFoul() {
        majorFouls++;
    }

    /** Recovery escapes triggered by the target-progress watchdog. */
    public int getEscapes() {
        return escapes;
    }

    /** Fuel points the watchdog has abandoned, sim-time TTL applied. */
    public Set<Translation2d> getBlockedPoints() {
        return Collections.unmodifiableSet(new HashSet<>(blockedUntil.keySet()));
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
     * @param elapsedSec sim elapsed (the latches' time base — wall clock is frozen
     *                   in a fast loop)
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
                HubSchedule.isHubActive(!isRed, next, shiftSeed),
                true, WorldState.DEFAULT_MAX_CAPACITY, hasClimber);

        // Expire watchdog blacklist entries on sim time (the watchdog's own
        // key TTL reads the wall clock, which is frozen here).
        blockedUntil.keySet().removeIf(p -> blockedUntil.get(p) <= elapsedSec);

        AIActionIntent intent = JevDecisionEngine.getInstance()
                .evaluatePolicy(world, knowledge, archetype, null,
                        new HashSet<>(blockedUntil.keySet()), commitment, fuelTargetMemory, null, elapsedSec);
        lastObjective = intent.objective();
        if (!isAuto && myHub) {
            hubActiveTeleopSec += dtSec;
        }

        // Kinematic command toward the intent target (does not move the pose yet;
        // the watchdog arbitrates between this command and an escape).
        Pose2d target = intent.navigationTarget();
        double vx = 0.0;
        double vy = 0.0;
        double dist = 0.0;
        Translation2d delta = null;
        if (target != null) {
            delta = target.getTranslation().minus(pose.getTranslation());
            dist = delta.getNorm();
            if (dist > 1e-6) {
                vx = delta.getX() / dist * maxSpeedMps;
                vy = delta.getY() / dist * maxSpeedMps;
            }
        }
        ChassisSpeeds commanded = new ChassisSpeeds(vx, vy, 0.0);

        // Sim-time target-progress arbitration. actual == commanded here (the
        // kinematic model has instant velocity), so the distance window owns it.
        // The watchdog sees the real navigation target exactly as the full sim's
        // AIRobotInstance does, so its give-up cadence stays comparable.
        boolean isIntaking = intent.intakeCommand() == IntakeState.INTAKING;
        TargetProgressWatchdog.Result progress = progressWatchdog.update(
                pose, commanded, commanded, target, dtSec);
        if (target != null) {
            TargetProgressWatchdog.Result harvest = progressWatchdog.updateHarvestArrivalWatchdog(
                    pose, target, isIntaking, held, dtSec);
            if (harvest.recovering()) {
                progress = harvest;
            }
        }
        for (Translation2d abandoned : progress.newlyBlacklisted()) {
            blockedUntil.put(abandoned, elapsedSec + TargetProgressWatchdog.BLACKLIST_TTL_SEC);
        }

        boolean escaping = progress.recovering();
        if (escaping) {
            // Escape vector is a field-relative velocity (ESCAPE_SPEED_MPS).
            Translation2d esc = progress.escapeVector();
            pose = clampOnField(pose.getTranslation().plus(esc.times(dtSec)), pose);
            pathLengthM += esc.getNorm() * dtSec;
            velocity = new ChassisSpeeds(esc.getX(), esc.getY(), 0.0);
            escapes++;
        } else {
            if (delta != null && dist > 1e-6) {
                double step = Math.min(dist, maxSpeedMps * dtSec);
                Translation2d next2d = pose.getTranslation().plus(delta.times(step / dist));
                pose = new Pose2d(next2d, new Rotation2d(Math.atan2(delta.getY(), delta.getX())));
                pathLengthM += step;
                velocity = new ChassisSpeeds(delta.getX() / dist * step / dtSec,
                        delta.getY() / dist * step / dtSec, 0.0);
            } else {
                velocity = commanded;
            }
        }

        if (escaping) {
            return; // recovery maneuver: no pickup or shot this tick
        }

        // Geometric pickup while the intent intakes.
        if (isIntaking && held < IntakeConstants.MAX_HELD_BALLS) {
            int room = IntakeConstants.MAX_HELD_BALLS - held;
            int got = fuel.takeInRollerBox(pose.getTranslation(), pose.getRotation().getRadians(), room);
            held += got;
            pickedUp += got;
        }

        // Analytic volley: seeded spread decides each ball against the funnel.
        // Kicks that cannot score (dead hub / out of envelope) or that miss are
        // accounted as wasted fuel; a SHUTTLE_PASS lob is deliberately aimed home
        // (legal) and is tracked separately. `attemptedShots`/`missedShots` keep
        // their original meaning: scoring-capable volleys only.
        if (intent.triggerFeedKicker() && held > 0) {
            boolean inZone = FieldMap.AllianceZones.isInAllianceZone(pose, isRed);
            boolean inRange = hubTranslation().getDistance(pose.getTranslation())
                    <= FieldMap.Hubs.SHOOTING_MAX_DISTANCE;
            boolean shuttle = intent.objective() == StrategicObjective.SHUTTLE_PASS;
            if (myHub && inZone && inRange) {
                int made = resolveVolley(held);
                attemptedShots += held;
                missedShots += held - made;
                if (made > 0) {
                    if (isAuto) {
                        autoScored += made;
                    } else {
                        teleopScored += made;
                    }
                    scored += made;
                }
                wastedFuel += held - made;
            } else if (shuttle) {
                shuttledFuel += held;
            } else {
                wastedFuel += held;
            }
            held = 0;
        }

        // Climb: a climber bot that reaches its tower pole inside the endgame
        // window is credited; arrival is measured into the window (0 = at t=20 s).
        // Analytic stand-in for the full sim's contact-based climb detection.
        if (hasClimber && !climbed && !isAuto
                && matchTimeRemaining <= CLIMB_WINDOW_SEC && matchTimeRemaining > 0.0) {
            Translation2d pole = FieldMap.ClimbingTowers.getTowerPole(isRed);
            if (pose.getTranslation().getDistance(pole) <= CLIMB_RADIUS_M) {
                climbed = true;
                climbArrivalSec = CLIMB_WINDOW_SEC - matchTimeRemaining;
            }
        }
    }

    /** Fires {@code balls} balls, each independently spread; returns how many score. */
    private int resolveVolley(int balls) {
        if (balls <= 0) {
            return 0;
        }
        Random rng = MatchDeterminism.random(shotStream);
        int made = 0;
        for (int i = 0; i < balls; i++) {
            if (shotMakes(pose, velocity, isRed, rng)) {
                made++;
            }
        }
        return made;
    }

    /**
     * One analytic shot: the real launch solution plus the
     * {@code AIRobotInstance.launchShot} spread, propagated and scored against
     * {@link ShotTracker#CAPTURE_RADIUS_METERS} of the funnel centre.
     *
     * <p>Package-private and static so a test can probe the make envelope
     * directly without running a match.
     */
    static boolean shotMakes(Pose2d pose, ChassisSpeeds robotVel, boolean isRed, Random rng) {
        final double g = 9.81;
        final double theta = ShooterConstants.FIRING_ANGLE;
        Translation2d hub2d = isRed
                ? new Translation2d(FieldMap.Hubs.RED_HUB_X, FieldMap.Hubs.HUB_Y)
                : new Translation2d(FieldMap.Hubs.BLUE_HUB_X, FieldMap.Hubs.HUB_Y);
        Translation2d botPos = pose.getTranslation();
        Translation2d vel = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);

        // Lead compensation, exactly as AIRobotInstance does (tof = 0.12 + 0.18 d).
        Translation2d aimTarget = hub2d;
        if (vel.getNorm() > 0.05) {
            double tof = 0.12 + 0.18 * botPos.getDistance(hub2d);
            aimTarget = hub2d.minus(vel.times(tof));
        }
        double d = botPos.getDistance(aimTarget);
        if (d < 1e-3) {
            return true;
        }
        double h = FUNNEL_Z_M - SHOOTER_Z_M;
        double denom = 2.0 * (d * Math.tan(theta) - h);
        double exitVel = denom > 0.1 ? (d / Math.cos(theta)) * Math.sqrt(g / denom) : 6.8;

        // Spread: same distributions as AIRobotInstance.launchShot.
        double v = exitVel * (1.0 + (rng.nextDouble() - 0.5) * 0.04);
        double yaw = aimTarget.minus(botPos).getAngle().getRadians()
                + Math.toRadians((rng.nextDouble() - 0.5) * 1.6);
        double pitch = theta + (rng.nextDouble() - 0.5) * 0.025;

        // Launch vector + robot velocity (RebuiltFuelOnFly adds robotVel).
        Translation2d vHoriz = new Translation2d(
                v * Math.cos(pitch) * Math.cos(yaw),
                v * Math.cos(pitch) * Math.sin(yaw)).plus(vel);
        double vz = v * Math.sin(pitch);

        for (double t = 0.0; t <= FLIGHT_MAX_SEC; t += FLIGHT_SAMPLE_SEC) {
            double z = SHOOTER_Z_M + vz * t - 0.5 * g * t * t;
            if (z < 0.0) {
                break;
            }
            double dx = botPos.getX() + vHoriz.getX() * t - hub2d.getX();
            double dy = botPos.getY() + vHoriz.getY() * t - hub2d.getY();
            double dz = Math.abs(z - FUNNEL_Z_M);
            double horiz = Math.sqrt(dx * dx + dy * dy);
            if (horiz <= FUNNEL_TOL_XY_M && dz <= FUNNEL_TOL_Z_M) {
                return true;
            }
        }
        return false;
    }

    private Translation2d hubTranslation() {
        return isRed
                ? new Translation2d(FieldMap.Hubs.RED_HUB_X, FieldMap.Hubs.HUB_Y)
                : new Translation2d(FieldMap.Hubs.BLUE_HUB_X, FieldMap.Hubs.HUB_Y);
    }

    private static Pose2d clampOnField(Translation2d at, Pose2d keepHeading) {
        double x = Math.min(Math.max(at.getX(), 0.3), FieldMap.FIELD_LENGTH - 0.3);
        double y = Math.min(Math.max(at.getY(), 0.3), FieldMap.FIELD_WIDTH - 0.3);
        return new Pose2d(x, y, keepHeading.getRotation());
    }
}
