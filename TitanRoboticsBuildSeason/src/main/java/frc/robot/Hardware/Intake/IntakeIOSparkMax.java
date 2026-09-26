package frc.robot.Hardware.Intake;

import frc.robot.Subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Data.Constants;
import frc.robot.Data.PortMap;
import frc.robot.Hardware.NeoSparkMaxMotor;

/**
 * Real physical hardware implementation of IntakeIO using REV SparkMax controllers
 * and a throughbore digital absolute encoder.
 */
public class IntakeIOSparkMax implements IntakeIO {

    private final NeoSparkMaxMotor armMotor;
    private final NeoSparkMaxMotor wheelsMotor;
    private final NeoSparkMaxMotor hopperMotor;
    private final DutyCycleEncoder pivotEncoder;

    public IntakeIOSparkMax() {
        armMotor = new NeoSparkMaxMotor(PortMap.INTAKE_ARM_MOTOR_ID);
        armMotor.setInverted(Constants.INTAKE_ARM_INVERTED);
        armMotor.setBrakeMode(true);

        wheelsMotor = new NeoSparkMaxMotor(PortMap.INTAKE_WHEELS_MOTOR_ID);
        wheelsMotor.setInverted(Constants.INTAKE_WHEELS_INVERTED);

        hopperMotor = new NeoSparkMaxMotor(PortMap.HOPPER_MOTOR_CANID);

        pivotEncoder = new DutyCycleEncoder(PortMap.INTAKE_ENCODER_ID);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.encoderConnected = pivotEncoder.isConnected();
        double unmodified = pivotEncoder.get() * 360.0;
        inputs.armPositionDeg = MathUtil.inputModulus(Constants.INTAKE_POSITION_OFFSET - unmodified, 0, 360);
        inputs.armVelocityDegPerSec = armMotor.getSpeed() * 6.0; // Estimate
        inputs.armAppliedVolts = armMotor.getAppliedVoltage();
        inputs.armCurrentAmps = armMotor.getOutputCurrent();
        inputs.armMotorRotations = armMotor.getPosition();

        inputs.rollerAppliedVolts = wheelsMotor.getAppliedVoltage();
        inputs.rollerCurrentAmps = wheelsMotor.getOutputCurrent();
        inputs.rollerVelocityRPM = wheelsMotor.getSpeed();

        inputs.hopperAppliedVolts = hopperMotor.getAppliedVoltage();
        inputs.hopperCurrentAmps = hopperMotor.getOutputCurrent();
        inputs.hopperVelocityRPM = hopperMotor.getSpeed();
    }

    @Override
    public void setArmVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public void setRollerVoltage(double volts) {
        wheelsMotor.setVoltage(volts);
    }

    @Override
    public void setRollerSpeed(double speed) {
        wheelsMotor.set(speed);
    }

    @Override
    public void setHopperVoltage(double volts) {
        hopperMotor.setVoltage(volts);
    }

    @Override
    public void setHopperSpeed(double speed) {
        hopperMotor.set(speed);
    }

    @Override
    public void stop() {
        armMotor.setVoltage(0.0);
        wheelsMotor.set(0.0);
        hopperMotor.set(0.0);
    }
}
