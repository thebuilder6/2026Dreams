package frc.robot.Devices;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;

/*
 * Class: NeoSparkMaxMotor
 * Description: Wrapper for REV SparkMax motors supporting both physical hardware
 *              configuration and desktop simulation state tracking.
 * Authors: Mai, InfiniteQuery
 */
public class NeoSparkMaxMotor {

    private SparkMax m_motor;
    private RelativeEncoder encoder;
    private boolean isInverted = false;
    private int CANID;

    // Simulation states
    private double simVelocity = 0;
    private double simPosition = 0;
    private double simSpeed = 0;
    private double lastVoltage = 0.0;

    public NeoSparkMaxMotor(int CANID) {
        this.CANID = CANID;
        try {
            m_motor = new SparkMax(CANID, MotorType.kBrushless);
            encoder = m_motor.getEncoder();
        } catch (Exception e) {
            m_motor = null;
            System.out.println("SparkMax error initializing CANID: " + CANID);
        }
    }

    public void configure(SparkMaxConfig config) {
        if (m_motor != null) {
            m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public void set(double power) {
        simSpeed = power;
        if (m_motor != null) {
            m_motor.set(power);
        }
    }

    public void setVoltage(double voltage) {
        lastVoltage = voltage;
        if (m_motor != null) {
            m_motor.setVoltage(voltage);
        }
    }

    public void setSpeed(double speed) {
        set(speed);
    }

    public double getSpeed() {
        if (RobotBase.isSimulation()) {
            return simVelocity != 0 ? simVelocity : (simSpeed * 5676.0);
        }
        return encoder != null ? encoder.getVelocity() : 0.0;
    }

    public double getVelocity() {
        return getSpeed();
    }

    public double getPosition() {
        if (RobotBase.isSimulation()) {
            return simPosition;
        }
        return encoder != null ? encoder.getPosition() : 0.0;
    }

    public void setInverted(boolean inverted) {
        this.isInverted = inverted;
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        if (m_motor != null) {
            m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public void setBrakeMode(boolean brake) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        if (m_motor != null) {
            m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public double getAppliedOutput() {
        if (m_motor != null) {
            return m_motor.getAppliedOutput();
        }
        return lastVoltage / 12.0;
    }

    public double getBusVoltage() {
        if (m_motor != null) {
            return m_motor.getBusVoltage();
        }
        return 12.0;
    }

    public double getAppliedVoltage() {
        return lastVoltage;
    }

    public double getOutputCurrent() {
        return m_motor != null ? m_motor.getOutputCurrent() : 0.0;
    }

    public void setSimState(double velocityRPM, double positionRotations) {
        this.simVelocity = velocityRPM;
        this.simPosition = positionRotations;
    }

    public void resetEncoder() {
        if (encoder != null) {
            encoder.setPosition(0);
        }
    }

    public void stop() {
        set(0);
        setVoltage(0);
    }

    public int getCANID() {
        return CANID;
    }

    public SparkMax getMotor() {
        return m_motor;
    }
}
