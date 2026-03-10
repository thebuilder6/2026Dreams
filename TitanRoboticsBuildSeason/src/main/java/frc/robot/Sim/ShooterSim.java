package frc.robot.Sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Data.Constants.ShooterConstants;

public class ShooterSim {
    private final FlywheelSim flywheelSim;

    public ShooterSim() {
        flywheelSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(ShooterConstants.kFlywheelV.get(), ShooterConstants.kFlywheelA.get()),
                DCMotor.getNEO(1),
                ShooterConstants.SIM_GEARING
        );
    }

    public void update(double voltage) {
        flywheelSim.setInput(voltage);
        flywheelSim.update(0.02);
    }

    public double getVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }

    public double getCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
    }
}
