package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** TEMP diagnostic for canShootNow hub gate. */
public class TempHubProbeTest {

    @Test
    public void probeHubGate() {
        HAL.initialize(500, 0);
        System.out.println("=== isSimulation=" + RobotBase.isSimulation());
        System.out.println("=== dsAuto=" + DriverStation.isAutonomous());
        System.out.println("=== dsMatchTime=" + DriverStation.getMatchTime());
        System.out.println("=== gameSimTime=" + GameSim.getInstance().getSimTimeRemainingSec());
        HubSchedule.refreshFromMatchState();
        System.out.println("=== redActive=" + HubSchedule.isHubActiveNow(true)
                + " blueActive=" + HubSchedule.isHubActiveNow(false));
        AIRobotSim aiSim = AIRobotSim.getInstance();
        aiSim.reset();
        aiSim.setFuelCount(3);
        frc.robot.Subsystems.SwerveBase.getInstance()
                .resetOdometry(new Pose2d(3.0, 1.0, new Rotation2d()));
        Pose2d shootingPose = new Pose2d(14.34, 4.035, Rotation2d.fromDegrees(180));
        System.out.println("=== validLoc=" + aiSim.isValidShootingLocation(shootingPose, true));
        System.out.println("=== hubActive=" + aiSim.isHubActiveForAlliance(true));
        System.out.println("=== canShoot=" + aiSim.canShootNow(shootingPose, true));
        assertTrue(true);
    }
}
