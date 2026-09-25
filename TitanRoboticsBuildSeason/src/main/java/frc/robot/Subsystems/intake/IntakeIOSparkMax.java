package frc.robot.Subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Data.Constants;
import frc.robot.Data.PortMap;
import frc.robot.Devices.NeoSparkMaxMotor;

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
        com.revrobotics.spark.config.SparkMaxConfig armConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        armConfig.inverted(Constants.INTAKE_ARM_INVERTED);
        armConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        armConfig.smartCurrentLimit(40);
        NeoSparkMaxMotor.optimizeCanBusUtilization(armConfig, true, true);
        armMotor.configure(armConfig);

        wheelsMotor = new NeoSparkMaxMotor(PortMap.INTAKE_WHEELS_MOTOR_ID);
        com.revrobotics.spark.config.SparkMaxConfig wheelsConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        wheelsConfig.inverted(Constants.INTAKE_WHEELS_INVERTED);
        wheelsConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        wheelsConfig.smartCurrentLimit((int) IntakeConstants.STALL_CURRENT_LIMIT);
        NeoSparkMaxMotor.optimizeCanBusUtilization(wheelsConfig, false, true);
        wheelsMotor.configure(wheelsConfig);

        hopperMotor = new NeoSparkMaxMotor(PortMap.HOPPER_MOTOR_CANID);
        com.revrobotics.spark.config.SparkMaxConfig hopperConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        hopperConfig.inverted(false);
        hopperConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        hopperConfig.smartCurrentLimit((int) IntakeConstants.STALL_CURRENT_LIMIT);
        NeoSparkMaxMotor.optimizeCanBusUtilization(hopperConfig, false, true);
        hopperMotor.configure(hopperConfig);

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
