package frc.robot.Sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;

/**
 * ShotTracker: reliable scored-shot attribution for simulated fuel launches.
 *
 * <p>MapleSim's {@code RebuiltHub} goal physically captures balls that enter
 * the hub funnel and removes the projectile on the spot. That capture usually
 * happens <i>before</i> the projectile's analytic hit-time elapses, so a
 * {@code withHitTargetCallBack} alone frequently never runs and the score is
 * silently lost. To count every score exactly once, each launch site registers
 * its projectile here; once the projectile leaves the arena's launched set we
 * resolve the outcome from its last known pose:
 * <ul>
 *   <li>vanished within {@link #CAPTURE_RADIUS_METERS} of the funnel while its
 *   hub was active &rarr; scored (per-robot attribution runs),</li>
 *   <li>vanished near the funnel while its hub was inactive &rarr; wasted shot,</li>
 *   <li>anything else &rarr; clean miss, recorded nowhere.</li>
 * </ul>
 * Resolution runs from {@link MatchScoreTracker#simulationUpdate()}, so it
 * ticks with the rest of the sim loop.
 */
public class ShotTracker {

    /** A ball that vanished this close to the funnel counts as captured. */
    public static final double CAPTURE_RADIUS_METERS = 0.75;

    private record PendingShot(
            GamePieceProjectile projectile,
            Translation3d funnelTarget,
            boolean hubIsRed,
            Runnable onScored,
            Runnable onWasted) {
    }

    private static final Map<GamePieceProjectile, PendingShot> pending = new LinkedHashMap<>();
    private static final Map<GamePieceProjectile, Translation3d> lastPose = new LinkedHashMap<>();

    private ShotTracker() {
    }

    /**
     * Registers a freshly launched projectile for outcome resolution.
     *
     * @param projectile The launched projectile (already added to the arena)
     * @param funnelTarget Center of the targeted hub funnel
     * @param hubIsRed True if the targeted hub belongs to Red
     * @param onScored Runs exactly once if the shot scores
     * @param onWasted Runs exactly once if the shot reaches an inactive hub (may be null)
     */
    public static synchronized void track(GamePieceProjectile projectile, Translation3d funnelTarget,
            boolean hubIsRed, Runnable onScored, Runnable onWasted) {
        if (projectile == null || onScored == null) {
            return;
        }
        pending.put(projectile, new PendingShot(projectile, funnelTarget, hubIsRed, onScored, onWasted));
        try {
            lastPose.put(projectile, projectile.getPose3d().getTranslation());
        } catch (Exception ignored) {
        }
    }

    /**
     * Resolves finished shots. Must be called every sim tick.
     */
    public static synchronized void resolve() {
        if (pending.isEmpty() || !RobotBase.isSimulation()) {
            return;
        }
        java.util.Set<GamePieceProjectile> stillFlying;
        try {
            stillFlying = SimulatedArena.getInstance().gamePieceLaunched();
        } catch (Exception ignored) {
            return;
        }

        List<PendingShot> finished = new ArrayList<>();
        for (PendingShot shot : pending.values()) {
            try {
                if (stillFlying.contains(shot.projectile())) {
                    lastPose.put(shot.projectile(), shot.projectile().getPose3d().getTranslation());
                } else {
                    finished.add(shot);
                }
            } catch (Exception ignored) {
                finished.add(shot);
            }
        }

        for (PendingShot shot : finished) {
            pending.remove(shot.projectile());
            Translation3d last = lastPose.remove(shot.projectile());
            double dist = (last == null) ? Double.MAX_VALUE : last.getDistance(shot.funnelTarget());
            boolean nearHub = dist <= CAPTURE_RADIUS_METERS;
            if (!nearHub) {
                continue; // Clean miss: grounded or flew past nowhere near the hub.
            }
            try {
                if (isHubActive(shot.hubIsRed())) {
                    shot.onScored().run();
                } else if (shot.onWasted() != null) {
                    shot.onWasted().run();
                }
            } catch (Exception ignored) {
            }
        }
    }

    /** Clears all in-flight tracking (match reset). */
    public static synchronized void clear() {
        pending.clear();
        lastPose.clear();
    }

    /** Number of in-flight tracked shots (tests/telemetry). */
    public static synchronized int getPendingCount() {
        return pending.size();
    }

    private static boolean isHubActive(boolean hubIsRed) {
        HubSchedule.refreshFromMatchState();
        return HubSchedule.isScoringActive(hubIsRed);
    }
}
