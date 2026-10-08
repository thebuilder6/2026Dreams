package frc.robot.Sim;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * AdvantageScope replay writer for standalone matches.
 *
 * <p>Whether {@code Logger} + {@code WPILOGWriter} work in a plain JVM (no
 * robot, no HAL) is unproven — this helper isolates the experiment so the
 * runner stays usable if it does not. Topics are chosen to render with the
 * existing field layout: per-bot {@code Pose} (Pose2d), {@code Objective}
 * (string), {@code Held} (double), plus match {@code BlueScore}/{@code RedScore}/
 * {@code TimeRemaining} and the {@code Fuel} cloud (Pose3d at rest height).
 *
 * <p>Open the result with AdvantageScope → File → Open Log, add a 3D Field
 * with the 2026 Rebuilt model, and bind robot poses + fuel to the topics.
 */
public final class StandaloneReplay {
    private static boolean started;

    private StandaloneReplay() {}

    /** Starts recording to {@code logPath}. No-op (false) when already started. */
    public static synchronized boolean start(String logPath) {
        if (started) {
            return false;
        }
        Logger.recordMetadata("StandaloneRunner", "true");
        Logger.addDataReceiver(new WPILOGWriter(logPath));
        Logger.start();
        started = true;
        return true;
    }

    /** Records one tick. Safe to call only between {@link #start} and {@link #end}. */
    public static void recordTick(double timeRemainingSec, List<StandaloneBot> bots,
            FuelStore fuel, int blueScored, int redScored) {
        Logger.recordOutput("Standalone/TimeRemaining", timeRemainingSec);
        Logger.recordOutput("Standalone/BlueScore", (double) blueScored);
        Logger.recordOutput("Standalone/RedScore", (double) redScored);
        for (int i = 0; i < bots.size(); i++) {
            StandaloneBot bot = bots.get(i);
            String prefix = "Standalone/" + (bot.isRed() ? "Red" : "Blue") + i + "/";
            Logger.recordOutput(prefix + "Pose", bot.getPose());
            Logger.recordOutput(prefix + "Held", (double) bot.getHeld());
            Logger.recordOutput(prefix + "Scored", (double) bot.getScored());
            if (bot.getObjective() != null) {
                Logger.recordOutput(prefix + "Objective", bot.getObjective().name());
            }
        }
        List<Translation2d> pieces = fuel.positions();
        Pose3d[] cloud = new Pose3d[pieces.size()];
        for (int i = 0; i < pieces.size(); i++) {
            Translation2d at = pieces.get(i);
            cloud[i] = new Pose3d(at.getX(), at.getY(), 0.075, new Rotation3d());
        }
        Logger.recordOutput("Standalone/Fuel", cloud);
    }

    /** Flushes and closes the log. */
    public static synchronized void end() {
        if (!started) {
            return;
        }
        try {
            Logger.end();
        } finally {
            started = false;
        }
    }
}
