package frc.robot.Auto.Actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Interfaces.Actions;

public class TunnelAction implements Actions {
    private final Actions series;

    public TunnelAction(Pose2d entrancePose, Pose2d exitPose) {
        series = new SeriesAction(
                new DriveToPoseAction(entrancePose),
                new DriveToPoseAction(exitPose));
    }

    @Override
    public void start() {
        series.start();
    }

    @Override
    public void update() {
        series.update();
    }

    @Override
    public boolean isFinished() {
        return series.isFinished();
    }

    @Override
    public void done() {
        series.done();
    }
}
