package frc.robot.Subsystems.shooter;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Data.PortMap;
import frc.robot.Devices.NeoSparkMaxMotor;

/**
 * Real physical hardware implementation of ShooterIO using REV SparkMax motor controllers.
 */
public class ShooterIOSparkMax implements ShooterIO {

    private final NeoSparkMaxMotor flywheelMotorLeft;
    private final NeoSparkMaxMotor flywheelMotorRight;
    private final NeoSparkMaxMotor kickerMotor;

    public ShooterIOSparkMax() {
        flywheelMotorLeft = new NeoSparkMaxMotor(PortMap.SHOOTER_MOTOR_LEFT_ID);
        flywheelMotorRight = new NeoSparkMaxMotor(PortMap.SHOOTER_MOTOR_RIGHT_ID);
        kickerMotor = new NeoSparkMaxMotor(PortMap.KICKER_MOTOR_ID);

        // Configure Left Flywheel
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.inverted(false);
        leftConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        leftConfig.smartCurrentLimit((int) ShooterConstants.FLYWHEEL_CURRENT_LIMIT);
        flywheelMotorLeft.configure(leftConfig);

        // Configure Right Flywheel
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.inverted(true);
        rightConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        rightConfig.smartCurrentLimit((int) ShooterConstants.FLYWHEEL_CURRENT_LIMIT);
        flywheelMotorRight.configure(rightConfig);

        // Configure Kicker
        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(false);
        kickerConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        kickerConfig.smartCurrentLimit((int) ShooterConstants.KICKER_CURRENT_LIMIT);
        kickerMotor.configure(kickerConfig);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocityRPM = flywheelMotorLeft.getSpeed();
        inputs.rightVelocityRPM = flywheelMotorRight.getSpeed();
        inputs.leftAppliedVolts = flywheelMotorLeft.getAppliedVoltage();
        inputs.rightAppliedVolts = flywheelMotorRight.getAppliedVoltage();
        inputs.kickerAppliedVolts = kickerMotor.getAppliedVoltage();
        inputs.kickerVelocityRPM = kickerMotor.getSpeed();
        inputs.leftCurrentAmps = flywheelMotorLeft.getOutputCurrent();
        inputs.rightCurrentAmps = flywheelMotorRight.getOutputCurrent();
        inputs.kickerCurrentAmps = kickerMotor.getOutputCurrent();
        inputs.leftBusVolts = flywheelMotorLeft.getBusVoltage();
        inputs.rightBusVolts = flywheelMotorRight.getBusVoltage();
    }

    @Override
    public void setFlywheelVoltages(double leftVolts, double rightVolts) {
        flywheelMotorLeft.setVoltage(leftVolts);
        flywheelMotorRight.setVoltage(rightVolts);
    }

    @Override
    public void setKickerVoltage(double kickerVolts) {
        kickerMotor.setVoltage(kickerVolts);
    }

    @Override
    public void stop() {
        flywheelMotorLeft.stop();
        flywheelMotorRight.stop();
        kickerMotor.stop();
    }
}
