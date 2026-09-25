package frc.robot.Sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Utils.AllianceFlipUtil;

/**
 * Factory utilities for building immutable WorldState snapshots from
 * simulation and real-robot hardware contexts.
 */
public final class WorldStateBuilder {

    private WorldStateBuilder() {}

    /**
     * Builds a WorldState snapshot for the primary player robot (e.g., AutonomousTeleopAgent or MatchCoach).
     *
     * @param heldFuelCount Estimated or sensor-confirmed fuel/game pieces held in hopper
     * @return Immutable WorldState snapshot
     */
    public static WorldState buildForPlayerRobot(int heldFuelCount) {
        Pose2d playerPose = SwerveBase.getInstance().getPose();
        ChassisSpeeds playerVel = SwerveBase.getInstance().getFieldVelocity();

        Pose2d oppPose = new Pose2d();
        ChassisSpeeds oppVel = new ChassisSpeeds();
        try {
            AIRobotSim sim = AIRobotSim.getInstance();
            if (sim != null && sim.getDriveSimulation() != null) {
                oppPose = sim.getDriveSimulation().getActualPoseInSimulationWorld();
            }
        } catch (Exception ignored) {}

        double matchTime = Timer.getMatchTime();
        if (matchTime < 0.0) matchTime = 135.0;

        boolean isPlayerRed = AllianceFlipUtil.isRedAlliance();
        boolean playerHubActive = Dashboard.getInstance().isHubActive();
        boolean oppHubActive = !playerHubActive;
        double timeUntilShift = Dashboard.getInstance().getTimeUntilSwitch();

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
                isPlayerRed
        );
    }

    /**
     * Builds a WorldState snapshot for a simulated AI sparring robot.
     *
     * @param selfPose Current pose of the AI robot in simulation world
     * @param selfVelocity Current field-relative velocity of the AI robot
     * @param heldFuelCount Current fuel pieces in AI robot's intake simulation
     * @param isOpponentRedAlliance True if this AI robot is on the Red Alliance
     * @param isSelfHubActive Whether this AI robot's scoring hub is currently active
     * @return Immutable WorldState snapshot from the perspective of this AI robot
     */
    public static WorldState buildForSimBot(
            Pose2d selfPose,
            ChassisSpeeds selfVelocity,
            int heldFuelCount,
            boolean isOpponentRedAlliance,
            boolean isSelfHubActive) {

        Pose2d playerPose = SwerveBase.getInstance().getPose();
        ChassisSpeeds playerVel = SwerveBase.getInstance().getFieldVelocity();

        double matchTime = Timer.getMatchTime();
        if (matchTime < 0.0) matchTime = 135.0;

        boolean playerHubActive = Dashboard.getInstance().isHubActive();
        double timeUntilShift = Dashboard.getInstance().getTimeUntilSwitch();

        return new WorldState(
                selfPose,
                selfVelocity,
                heldFuelCount,
                playerPose,
                playerVel,
                matchTime,
                isSelfHubActive,
                playerHubActive,
                timeUntilShift,
                isOpponentRedAlliance
        );
    }
}
