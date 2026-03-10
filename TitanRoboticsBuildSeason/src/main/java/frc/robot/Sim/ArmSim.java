package frc.robot.Sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Data.Constants.IntakeConstants;

public class ArmSim {
    private final SingleJointedArmSim armSim;
    private Mechanism2d mech2d;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d armLigament;

    public ArmSim() {
        armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                IntakeConstants.SIM_ARM_GEARING,
                SingleJointedArmSim.estimateMOI(IntakeConstants.SIM_ARM_LENGTH, IntakeConstants.SIM_ARM_MASS),
                IntakeConstants.SIM_ARM_LENGTH,
                IntakeConstants.ARM_INTAKE_POS - 0.1, // Min Angle
                IntakeConstants.ARM_IDLE_POS + 0.1, // Max Angle
                true, // Simulate Gravity
                IntakeConstants.ARM_IDLE_POS // Starting Angle
        );

        mech2d = new Mechanism2d(2, 2);
        mechRoot = mech2d.getRoot("IntakeRoot", 0.5, 0.5);
        mechRoot.append(new MechanismLigament2d("Tower", 0.0, -90, 6, new Color8Bit(Color.kBlue)));
        armLigament = mechRoot.append(
                new MechanismLigament2d("IntakeArm", IntakeConstants.SIM_ARM_LENGTH, IntakeConstants.ARM_IDLE_POS, 6,
                        new Color8Bit(Color.kYellow)));
        SmartDashboard.putData("Intake Sim", mech2d);
    }

    public void update(double voltage) {
        armSim.setInput(voltage);
        armSim.update(0.02);
        armLigament.setAngle(armSim.getAngleRads() * 180.0 / Math.PI);
    }

    public double getAngleRads() {
        return armSim.getAngleRads();
    }

    public double getVelocityRadsPerSec() {
        return armSim.getVelocityRadPerSec();
    }

    public double getCurrentDrawAmps() {
        return armSim.getCurrentDrawAmps();
    }
}
