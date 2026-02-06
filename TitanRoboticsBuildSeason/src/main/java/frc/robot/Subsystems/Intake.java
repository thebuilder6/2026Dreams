package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.IntakeConstants;
import frc.robot.Devices.NeoSparkMaxMotor;
import frc.robot.Interfaces.Subsystem;

public class Intake implements Subsystem {

    private static Intake instance = null;

    private final NeoSparkMaxMotor armMotor;
    private final NeoSparkMaxMotor rollerMotor;
    private final NeoSparkMaxMotor hopperMotor;

    public enum IntakeState {
        IDLE,
        INTAKING,
        FEEDING,
        EJECTING
    }

    private IntakeState currentState = IntakeState.IDLE;
    private double targetArmPositionRad = IntakeConstants.ARM_IDLE_POS;

    private final Timer stallTimer = new Timer();
    private final Timer ejectTimer = new Timer();
    private boolean isEjectingJam = false; // Flag to track if we are in auto-eject mode

    private final ArmFeedforward armFeedforward;
    private final ProfiledPIDController armController;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        armMotor = new NeoSparkMaxMotor(IntakeConstants.ARM_MOTOR_ID);
        rollerMotor = new NeoSparkMaxMotor(IntakeConstants.ROLLER_MOTOR_ID);
        hopperMotor = new NeoSparkMaxMotor(IntakeConstants.HOPPER_MOTOR_ID);

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(false);
        // Configure encoder to output radians instead of rotations
        armConfig.encoder.positionConversionFactor(IntakeConstants.ARM_POSITION_CONVERSION);
        armConfig.encoder.velocityConversionFactor(IntakeConstants.ARM_VELOCITY_CONVERSION);
        armMotor.configure(armConfig);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(false);
        rollerMotor.configure(rollerConfig);

        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.inverted(false);
        hopperMotor.configure(hopperConfig);

        armFeedforward = new ArmFeedforward(
                IntakeConstants.kArmS,
                IntakeConstants.kArmG,
                IntakeConstants.kArmV,
                IntakeConstants.kArmA);

        armController = new ProfiledPIDController(
                IntakeConstants.kArmP,
                IntakeConstants.kArmI,
                IntakeConstants.kArmD,
                new TrapezoidProfile.Constraints(
                        IntakeConstants.kMaxArmVelocity,
                        IntakeConstants.kMaxArmAcceleration));

        SubsystemManager.registerSubsystem(this);
    }

    public void setState(IntakeState newState) {
        currentState = newState;
    }

    public IntakeState getState() {
        return currentState;
    }

    public void setArmPosition(double positionRad) {
        targetArmPositionRad = positionRad;
    }

    @Override
    public void update() {
        // Rollers and Hopper logic
        // JAM DETECTION
        if (currentState == IntakeState.INTAKING && !isEjectingJam) {
            double current = rollerMotor.getOutputCurrent();
            if (current > IntakeConstants.STALL_CURRENT_LIMIT) {
                stallTimer.start();
                if (stallTimer.hasElapsed(IntakeConstants.STALL_TIME)) {
                    isEjectingJam = true;
                    stallTimer.stop();
                    stallTimer.reset();
                    ejectTimer.reset();
                    ejectTimer.start();
                }
            } else {
                stallTimer.stop();
                stallTimer.reset();
            }
        }

        if (isEjectingJam) {
            if (ejectTimer.hasElapsed(IntakeConstants.EJECT_TIME)) {
                isEjectingJam = false;
                ejectTimer.stop();
                currentState = IntakeState.INTAKING; // Go back to intaking
            } else {
                // Override state for jam clearing
                rollerMotor.setSpeed(-IntakeConstants.INTAKE_SPEED);
                hopperMotor.setSpeed(-IntakeConstants.HOPPER_SPEED);
            }
        } else {
            // Normal operation
            switch (currentState) {
                case IDLE:
                    rollerMotor.stop();
                    hopperMotor.stop();
                    setArmPosition(IntakeConstants.ARM_IDLE_POS);
                    break;
                case INTAKING:
                    rollerMotor.setSpeed(IntakeConstants.INTAKE_SPEED);
                    hopperMotor.setSpeed(IntakeConstants.HOPPER_SPEED);
                    setArmPosition(IntakeConstants.ARM_INTAKE_POS);
                    break;
                case FEEDING:
                    rollerMotor.stop();
                    hopperMotor.setSpeed(IntakeConstants.HOPPER_SPEED);
                    break;
                case EJECTING:
                    rollerMotor.setSpeed(-IntakeConstants.INTAKE_SPEED);
                    hopperMotor.setSpeed(-IntakeConstants.HOPPER_SPEED);
                    break;
            }
        }

        // Arm Control logic
        double pidOutput = armController.calculate(armMotor.getPosition(), targetArmPositionRad);
        TrapezoidProfile.State setpoint = armController.getSetpoint();
        double ffOutput = armFeedforward.calculate(setpoint.position, setpoint.velocity);
        armMotor.setVoltage(pidOutput + ffOutput);
    }

    /**
     * Stop all motors and set state to IDLE.
     */
    public void stop() {
        currentState = IntakeState.IDLE;
        rollerMotor.stop();
        hopperMotor.stop();
        armMotor.stop();
    }

    public void setArmSpeed(double speed) {
        armMotor.setSpeed(speed);
    }

    @Override
    public void initialize() {
        setState(IntakeState.IDLE);
        armMotor.stop();
        armController.reset(armMotor.getPosition());
    }

    @Override
    public void log() {
        SmartDashboard.putString("Subsystems/Intake/State", currentState.toString());
        SmartDashboard.putNumber("Subsystems/Intake/Arm Position", armMotor.getPosition());
        SmartDashboard.putNumber("Subsystems/Intake/Target Arm Position", targetArmPositionRad);
        SmartDashboard.putNumber("Subsystems/Intake/Arm Velocity", armMotor.getVelocity());
        SmartDashboard.putNumber("Subsystems/Intake/Roller Velocity", rollerMotor.getVelocity());
        SmartDashboard.putNumber("Subsystems/Intake/Hopper Velocity", hopperMotor.getVelocity());
        SmartDashboard.putNumber("Subsystems/Intake/Roller Current", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Subsystems/Intake/Is Jammed", isEjectingJam);
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "Intake";
    }

    public double getArmPosition() {
        return armMotor.getPosition();
    }
}
