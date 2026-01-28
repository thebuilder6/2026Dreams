package frc.robot.Devices;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

public class NeoSparkMaxMotor {

    private SparkMax m_motor;
    private boolean isInverted;
    private int CANID;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private SparkMaxConfig motorConfig;

    private double simVelocity = 0;
    private double simPosition = 0;

    public NeoSparkMaxMotor(int CANID) {
        this(CANID, false);
    }

    public NeoSparkMaxMotor(int CANID, boolean inverted) {
        this.CANID = CANID;
        this.isInverted = inverted;
        try {
            m_motor = new SparkMax(CANID, MotorType.kBrushless);
            encoder = m_motor.getEncoder();
            closedLoopController = m_motor.getClosedLoopController();
            motorConfig = new SparkMaxConfig();
        } catch (Exception e) {
            m_motor = null;
            System.out.println("SparkMax not found: " + CANID);
        }
    }

    /**
     * Configures the motor with the provided configuration.
     * Uses ResetSafeParameters and PersistParameters by default.
     */
    public void configure(SparkMaxConfig config) {
        if (m_motor != null) {
            this.motorConfig = config;
            m_motor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters,
                    SparkMax.PersistMode.kPersistParameters);
        }
    }

    public double getPosition() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return simPosition;
        }
        return encoder != null ? encoder.getPosition() : 0.0;
    }

    public double getVelocity() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return simVelocity;
        }
        return encoder != null ? encoder.getVelocity() : 0.0;
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

    public void setVoltage(double voltage) {
        if (m_motor != null) {
            m_motor.setVoltage(voltage);
        }
    }

    public void setSpeed(double speed) {
        if (m_motor != null) {
            m_motor.set(isInverted ? -speed : speed);
        }
    }

    public double getSpeed() {
        if (m_motor != null) {
            return isInverted ? -m_motor.get() : m_motor.get();
        }
        return 0.0;
    }

    public void stop() {
        setSpeed(0);
    }

    public int getCANID() {
        return CANID;
    }

    public SparkMax getMotor() {
        return m_motor;
    }

    public double getOutputCurrent() {
        return m_motor != null ? m_motor.getOutputCurrent() : 0.0;
    }
}
