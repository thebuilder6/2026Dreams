package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.IntakeConstants;
import frc.robot.Devices.NeoSparkMaxMotor;
import frc.robot.Interfaces.Subsystem;
import edu.wpi.first.math.util.Units;
import frc.robot.Data.PortMap;
import frc.robot.Devices.ModifiedEncoder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Intake implements Subsystem {

    private static Intake instance = null;

    private final NeoSparkMaxMotor armMotor;
    private final NeoSparkMaxMotor rollerMotor;
    private final NeoSparkMaxMotor hopperMotor;
    private final ModifiedEncoder pivotEncoder;

    public enum IntakeState {
        IDLE,
        INTAKING,
        FEEDING,
        EJECTING
    }

    private IntakeState currentState = IntakeState.IDLE;
    private double targetArmPositionDeg = IntakeConstants.ARM_IDLE_POS;
    private double currentArmPositionDeg = 0.0;

    private final Timer stallTimer = new Timer();
    private final Timer ejectTimer = new Timer();
    private boolean isEjectingJam = false; // Flag to track if we are in auto-eject mode

    private final ArmFeedforward armFeedforward;
    private final ProfiledPIDController armController;

    // Simulation
    private frc.robot.Sim.ArmSim armSim;

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

        pivotEncoder = new ModifiedEncoder(PortMap.INTAKE_ENCODER_ID);

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(IntakeConstants.INTAKE_ARM_INVERTED);
        armMotor.configure(armConfig);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(IntakeConstants.INTAKE_WHEELS_INVERTED);
        rollerMotor.configure(rollerConfig);

        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.inverted(false);
        hopperMotor.configure(hopperConfig);

        // No local TunableNumbers needed, using global ones in IntakeConstants

        armFeedforward = new ArmFeedforward(
                IntakeConstants.kArmS.get(),
                IntakeConstants.kArmG.get(),
                IntakeConstants.kArmV.get(),
                IntakeConstants.kArmA.get());

        armController = new ProfiledPIDController(
                IntakeConstants.kArmP.get(),
                IntakeConstants.kArmI.get(),
                IntakeConstants.kArmD.get(),
                new TrapezoidProfile.Constraints(
                        IntakeConstants.kMaxArmVelocity,
                        IntakeConstants.kMaxArmAcceleration));

        // Simulation Setup
        if (RobotBase.isSimulation()) {
            armSim = new frc.robot.Sim.ArmSim();
        }

        SubsystemManager.registerSubsystem(this);
    }

    /**
     * Sets the state of the intake subsystem.
     * Transitions between IDLE, INTAKING, FEEDING, and EJECTING states affect both
     * arm position and motor outputs in the update loop.
     * 
     * @param newState The next state to transition to.
     */
    public void setState(IntakeState newState) {
        currentState = newState;
    }

    public IntakeState getState() {
        return currentState;
    }

    public void setArmPosition(double positionDeg) {
        targetArmPositionDeg = positionDeg;
    }

    @Override
    public void simulationUpdate() {
        if (armSim != null) {
            // Use the actual voltage applied by the control loop in update()
            double voltage = armMotor.getAppliedVoltage();

            armSim.update(voltage);

            // Update the motor's simulated state
            armMotor.setSimState(0, armSim.getAngleRads());
        }
    }

    @Override
    public double getSimulationCurrentDraw() {
        double current = 0.0;
        if (armSim != null) {
            current += armSim.getCurrentDrawAmps();
        }
        // Add rollers
        current += Math.abs(rollerMotor.getSpeed()) * 20.0; // Loaded roller estimate
        current += Math.abs(hopperMotor.getSpeed()) * 10.0; // Hopper estimate
        return current;
    }

    @Override
    public void update() {
        // Rollers and Hopper logic
        // JAM DETECTION: Checks if roller current exceeds threshold for a sustained period.
        // Triggers an automatic reversal (EJECTING) to clear the jam.
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
                    setArmPosition(IntakeConstants.ARM_IDLE_POS);
                    break;
                case EJECTING:
                    rollerMotor.setSpeed(-IntakeConstants.INTAKE_SPEED);
                    hopperMotor.setSpeed(-IntakeConstants.HOPPER_SPEED);
                    break;
            }
        }

        // Arm Control logic
        if (frc.robot.Data.Constants.TUNING_MODE) {
            armController.setP(IntakeConstants.kArmP.get());
            armController.setI(IntakeConstants.kArmI.get());
            armController.setD(IntakeConstants.kArmD.get());
            // Note: armFeedforward and constraints are harder to update live without re-instantiating,
            // but P, I, D are the most common tuning targets.
        }

        double unmodifiedAbsolutePosition = pivotEncoder.getAbsolutePosition();
        currentArmPositionDeg = unmodifiedAbsolutePosition < 180 ? unmodifiedAbsolutePosition + 360
                : unmodifiedAbsolutePosition;
        currentArmPositionDeg -= IntakeConstants.INTAKE_POSITION_OFFSET;

        double pidOutput = armController.calculate(currentArmPositionDeg, targetArmPositionDeg);
        TrapezoidProfile.State setpoint = armController.getSetpoint();
        double ffOutput = armFeedforward.calculate(Math.toRadians(setpoint.position),
                Math.toRadians(setpoint.velocity));
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

    @Override
    public void initialize() {
        setState(IntakeState.IDLE);
        armMotor.stop();

        double unmodifiedAbsolutePosition = pivotEncoder.getAbsolutePosition();
        currentArmPositionDeg = unmodifiedAbsolutePosition < 180 ? unmodifiedAbsolutePosition + 360
                : unmodifiedAbsolutePosition;
        currentArmPositionDeg -= IntakeConstants.INTAKE_POSITION_OFFSET;

        armController.reset(currentArmPositionDeg);
    }

    @Override
    public void log() {
        SmartDashboard.putString("Subsystems/Intake/State", currentState.toString());
        SmartDashboard.putNumber("Subsystems/Intake/Arm Position", currentArmPositionDeg);
        SmartDashboard.putNumber("Subsystems/Intake/Target Arm Position", targetArmPositionDeg);
        SmartDashboard.putNumber("Subsystems/Intake/Arm Velocity", armMotor.getVelocity());
        SmartDashboard.putNumber("Subsystems/Intake/Roller Velocity", rollerMotor.getVelocity());
        SmartDashboard.putNumber("Subsystems/Intake/Hopper Velocity", hopperMotor.getVelocity());
        SmartDashboard.putNumber("Subsystems/Intake/Roller Current", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Subsystems/Intake/Is Jammed", isEjectingJam);

        // 3D Mechanism Visualization
        // Arm pivot is located at front of robot, slightly offset from center
        Translation3d armPivotRobotRelative = new Translation3d(0.25, 0, 0.2); 
        Rotation3d armRotation = new Rotation3d(0, -Units.degreesToRadians(currentArmPositionDeg), 0);
        Pose3d armPose = new Pose3d(armPivotRobotRelative, armRotation);
        
        SmartDashboard.putNumberArray("Subsystems/Intake/ArmPose3d", new double[] {
            armPose.getX(),
            armPose.getY(),
            armPose.getZ(),
            armPose.getRotation().getQuaternion().getW(),
            armPose.getRotation().getQuaternion().getX(),
            armPose.getRotation().getQuaternion().getY(),
            armPose.getRotation().getQuaternion().getZ()
        });
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
        return currentArmPositionDeg;
    }

    public void setArmVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    public double getArmAppliedVoltage() {
        return armMotor.getBusVoltage() * armMotor.getAppliedOutput();
    }

    public double getArmVelocityRads() {
        return Units.degreesToRadians(armMotor.getVelocity()); // Velocity in rad/s if configured, or needs conversion
    }

    public double getArmPositionRads() {
        return Units.degreesToRadians(getArmPosition());
    }

    public boolean isJammed() {
        return isEjectingJam;
    }

    /** Sets roller motor voltage directly (used by Diagnostics). */
    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    /** Gets roller motor velocity in RPM. */
    public double getRollerVelocityRPM() {
        return rollerMotor.getVelocity();
    }

    /** Sets hopper motor voltage directly (used by Diagnostics). */
    public void setHopperVoltage(double volts) {
        hopperMotor.setVoltage(volts);
    }

    /** Gets hopper motor velocity in RPM. */
    public double getHopperVelocityRPM() {
        return hopperMotor.getVelocity();
    }
}
