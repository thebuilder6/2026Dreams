package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.IntakeConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.intake.IntakeIO;
import frc.robot.Subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.Subsystems.intake.IntakeIOSim;
import frc.robot.Subsystems.intake.IntakeIOSparkMax;
import frc.robot.Utils.Alert;
import frc.robot.Utils.Alert.AlertType;

/**
 * Ground intake mechanism featuring an articulated pivot arm with feedforward
 * gravity compensation,
 * closed-loop continuous profiled PID control, automated jam
 * detection/ejection, and physics simulation.
 */
public class Intake implements Subsystem {

    private static Intake instance = null;

    // IO Abstraction (AdvantageKit pattern)
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    /**
     * Discrete operational states for intake pivot and roller mechanisms.
     */
    public enum IntakeState {
        DISABLED("Disabled"),
        STANDBY("Standby"),
        STANDBY_INTAKING("StandbyIntaking"),
        STANDBY_REVERSED("StandbyReversed"),
        INTAKING("Intaking"),
        DOWN("Down"),
        REVERSED("Reversed"),
        MANUAL("Manual"),
        IDLE("Standby"),
        FEEDING("Intaking"),
        EJECTING("Reversed"),
        CHARACTERIZATION("Characterization");

        private final String stateName;

        IntakeState(String stateName) {
            this.stateName = stateName;
        }

        public String getStateName() {
            return stateName;
        }

        public static IntakeState fromString(String name) {
            if (name == null)
                return STANDBY;
            for (IntakeState s : values()) {
                if (s.name().equalsIgnoreCase(name) || s.stateName.equalsIgnoreCase(name)) {
                    return s;
                }
            }
            return STANDBY;
        }
    }

    private IntakeState state = IntakeState.DISABLED;
    private double power = -0.5;
    private double armVoltage = 0.0;

    private final ProfiledPIDController pivotProfiledPIDController;
    private ArmFeedforward feedforward;

    private double currentPosition = 128.0;
    private double upPosition = Constants.INTAKE_UP_POSITION;
    private double downPosition = Constants.INTAKE_DOWN_POSITION;
    private double goal = Constants.INTAKE_UP_POSITION;
    private double manualPosition = Constants.INTAKE_UP_POSITION;

    // Jam detection & Alerts
    private final Timer stallTimer = new Timer();
    private final Timer ejectTimer = new Timer();
    private boolean isEjectingJam = false;
    private final Alert intakeEncoderAlert = new Alert("Intake", "Absolute Encoder Disconnected: Fallback Active",
            AlertType.ERROR);
    private final Alert intakeJamAlert = new Alert("Intake", "Roller Jam Detected: Auto-Clearing", AlertType.WARNING);
    private double lastValidPosition = Constants.INTAKE_UP_POSITION;
    private double lastMotorRotations = 0.0;

    // Simulation
    private frc.robot.Sim.ArmSim armSim;

    /**
     * Gets the singleton instance of Intake, instantiating the appropriate IO
     * layer.
     */
    public static Intake getInstance() {
        if (instance == null) {
            IntakeIO io = RobotBase.isSimulation() ? new IntakeIOSim() : new IntakeIOSparkMax();
            instance = new Intake(io);
        }
        return instance;
    }

    /**
     * Constructs the Intake subsystem with the provided IO layer.
     * 
     * @param io Hardware or simulation IO abstraction layer.
     */
    public Intake(IntakeIO io) {
        this.io = io;

        feedforward = new ArmFeedforward(
                Constants.INTAKE_ARM_KS,
                Constants.INTAKE_ARM_KG,
                Constants.INTAKE_ARM_KV,
                Constants.INTAKE_ARM_KA);

        pivotProfiledPIDController = new ProfiledPIDController(
                Constants.INTAKE_ARM_KP,
                Constants.INTAKE_ARM_KI,
                Constants.INTAKE_ARM_KD,
                new TrapezoidProfile.Constraints(Constants.MAX_ARM_VELOCITY, Constants.MAX_ARM_ACCELERATION));

        // Tell the PID controller that 0 and 360 are continuous
        pivotProfiledPIDController.enableContinuousInput(0, 360);

        if (RobotBase.isSimulation() && io instanceof IntakeIOSim simIO) {
            armSim = simIO.getArmSim();
        } else if (RobotBase.isSimulation()) {
            armSim = new frc.robot.Sim.ArmSim();
        }

        SubsystemManager.registerSubsystem(this);
    }

    /**
     * Calculates combined PID + gravity feedforward output and commands the pivot
     * motor.
     */
    private void updateArmController() {
        // Live Tunable Gains Check
        if (Constants.IntakeConstants.ARM_KP.hasChanged(hashCode())
                || Constants.IntakeConstants.ARM_KI.hasChanged(hashCode())
                || Constants.IntakeConstants.ARM_KD.hasChanged(hashCode())) {
            pivotProfiledPIDController.setPID(
                    Constants.IntakeConstants.ARM_KP.get(),
                    Constants.IntakeConstants.ARM_KI.get(),
                    Constants.IntakeConstants.ARM_KD.get());
        }
        if (Constants.IntakeConstants.ARM_KS.hasChanged(hashCode())
                || Constants.IntakeConstants.ARM_KG.hasChanged(hashCode())
                || Constants.IntakeConstants.ARM_KV.hasChanged(hashCode())
                || Constants.IntakeConstants.ARM_KA.hasChanged(hashCode())) {
            feedforward = new ArmFeedforward(
                    Constants.IntakeConstants.ARM_KS.get(),
                    Constants.IntakeConstants.ARM_KG.get(),
                    Constants.IntakeConstants.ARM_KV.get(),
                    Constants.IntakeConstants.ARM_KA.get());
        }

        // Geofenced Low-Ceiling Intake Arm Protection (Trench Auto-Stow)
        try {
            Pose2d robotPose = SwerveBase.getInstance().getPose();
            boolean inTrench = isPoseInTrenchLowClearanceZone(robotPose);
            if (inTrench && goal > Constants.INTAKE_HORIZONTAL_POSITION) {
                goal = Constants.INTAKE_HORIZONTAL_POSITION;
                SmartDashboard.putBoolean("Intake/TrenchSafetyClamped", true);
            } else {
                SmartDashboard.putBoolean("Intake/TrenchSafetyClamped", false);
            }
        } catch (Exception ignored) {
        }

        pivotProfiledPIDController.setGoal(goal);

        double pidOutput = pivotProfiledPIDController.calculate(currentPosition);
        double targetVelocityRadians = Math.toRadians(pivotProfiledPIDController.getSetpoint().velocity);

        double ffOutput = feedforward.calculate(
                Math.toRadians(currentPosition - Constants.INTAKE_HORIZONTAL_POSITION),
                targetVelocityRadians);

        armVoltage = pidOutput + ffOutput;
        io.setArmVoltage(armVoltage);
    }

    /**
     * Sets the state using a string identifier.
     */
    public void setState(String stateName) {
        setState(IntakeState.fromString(stateName));
    }

    /**
     * Sets the state using the type-safe {@link IntakeState} enum.
     */
    public void setState(IntakeState newState) {
        if (this.state == IntakeState.DISABLED && newState != IntakeState.DISABLED) {
            pivotProfiledPIDController.reset(currentPosition);
        }
        this.state = newState;
    }

    /**
     * Gets the current state string representation.
     */
    public String getStateString() {
        return state.getStateName();
    }

    /**
     * Gets the current {@link IntakeState}.
     */
    public IntakeState getState() {
        return state;
    }

    /**
     * Drives arm position based on normalized manual joystick input [-1.0, 1.0].
     */
    public void manualIntakeControl(double manualInput) {
        this.state = IntakeState.MANUAL;
        double normalizedInput = (manualInput + 1.0) / 2.0;
        double modifiedManualPosition = Constants.INTAKE_DOWN_POSITION
                + (normalizedInput * (Constants.INTAKE_UP_POSITION - Constants.INTAKE_DOWN_POSITION));
        this.manualPosition = MathUtil.clamp(modifiedManualPosition,
                Math.min(Constants.INTAKE_DOWN_POSITION, Constants.INTAKE_UP_POSITION),
                Math.max(Constants.INTAKE_DOWN_POSITION, Constants.INTAKE_UP_POSITION));
    }

    /**
     * Directly sets target arm position in degrees.
     */
    public void setArmPosition(double positionDeg) {
        goal = positionDeg;
        state = IntakeState.MANUAL;
        manualPosition = positionDeg;
    }

    public double getArmPosition() {
        return currentPosition;
    }

    /**
     * Commands open-loop voltage to the arm pivot with hard software stop limits.
     */
    public void setArmVoltage(double volts) {
        state = IntakeState.CHARACTERIZATION;
        // Enforce hard software safety boundaries [240 deg, 355 deg]
        if (currentPosition < 240.0 && volts < 0.0) {
            volts = 0.0; // Cut voltage if driving further down past mechanical ground stop
        } else if (currentPosition > 355.0 && volts > 0.0) {
            volts = 0.0; // Cut voltage if driving further past top mechanical stop
        }
        armVoltage = volts;
        io.setArmVoltage(volts);
    }

    public void setCharacterizationVoltage(double armVolts) {
        setArmVoltage(armVolts);
    }

    public double getArmAppliedVoltage() {
        return armVoltage;
    }

    public double getArmPositionRads() {
        return Math.toRadians(currentPosition);
    }

    public double getArmVelocityRads() {
        return Math.toRadians(inputs.armVelocityDegPerSec);
    }

    public void runRollers(double speed) {
        io.setRollerSpeed(speed);
    }

    public void setRollerVoltage(double volts) {
        io.setRollerVoltage(volts);
    }

    public double getRollerVelocityRPM() {
        return inputs.rollerVelocityRPM;
    }

    public void runHopper(double speed) {
        io.setHopperSpeed(speed);
    }

    public void setHopperVoltage(double volts) {
        io.setHopperVoltage(volts);
    }

    public double getHopperVelocityRPM() {
        return inputs.hopperVelocityRPM;
    }

    public double getArmCurrentAmps() {
        return inputs.armCurrentAmps;
    }

    public double getRollerCurrentAmps() {
        return inputs.rollerCurrentAmps;
    }

    public void stop() {
        setState(IntakeState.DISABLED);
        io.stop();
    }

    public boolean isAtTargetPosition() {
        return Math.abs(currentPosition - goal) < 5.0;
    }

    public boolean isJammed() {
        return isEjectingJam;
    }

    public double getCurrentDraw() {
        return Math.abs(inputs.armCurrentAmps) + Math.abs(inputs.rollerCurrentAmps)
                + Math.abs(inputs.hopperCurrentAmps);
    }

    public IntakeIO getIO() {
        return io;
    }

    public IntakeIOInputs getInputs() {
        return inputs;
    }

    @Override
    public void update() {
        io.updateInputs(inputs);

        boolean encoderHealthy = RobotBase.isSimulation() || inputs.encoderConnected;
        intakeEncoderAlert.set(!encoderHealthy);

        if (encoderHealthy) {
            currentPosition = inputs.armPositionDeg;
            lastValidPosition = currentPosition;
            lastMotorRotations = inputs.armMotorRotations;
        } else {
            // Graceful degradation: Track delta rotations from SparkMax internal relative
            // encoder
            double deltaRotations = inputs.armMotorRotations - lastMotorRotations;
            currentPosition = MathUtil.inputModulus(lastValidPosition + (deltaRotations * 3.6), 0, 360);
        }

        // Automated Jam Detection and Ejection
        if (!RobotBase.isSimulation() && Math.abs(inputs.rollerAppliedVolts) > 1.0
                && inputs.rollerCurrentAmps > IntakeConstants.STALL_CURRENT_LIMIT) {
            stallTimer.start();
            if (stallTimer.hasElapsed(IntakeConstants.STALL_TIME)) {
                isEjectingJam = true;
                ejectTimer.restart();
                stallTimer.reset();
                stallTimer.stop();
            }
        } else {
            stallTimer.reset();
            stallTimer.stop();
        }

        if (isEjectingJam) {
            intakeJamAlert.set(true);
            if (ejectTimer.hasElapsed(IntakeConstants.EJECT_TIME)) {
                isEjectingJam = false;
                ejectTimer.stop();
                ejectTimer.reset();
                intakeJamAlert.set(false);
            }
        } else {
            intakeJamAlert.set(false);
        }

        switch (state) {
            case STANDBY:
            case IDLE:
                goal = upPosition;
                updateArmController();
                io.setRollerSpeed(0);
                io.setHopperSpeed(0);
                break;

            case STANDBY_INTAKING:
                goal = upPosition;
                updateArmController();
                io.setRollerSpeed(power);
                io.setHopperSpeed(Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case STANDBY_REVERSED:
                goal = upPosition;
                updateArmController();
                io.setRollerSpeed(-power);
                io.setHopperSpeed(-Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case INTAKING:
            case FEEDING:
                goal = downPosition;
                updateArmController();
                io.setRollerSpeed(isEjectingJam ? -power : power);
                io.setHopperSpeed(Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case DOWN:
                goal = downPosition;
                updateArmController();
                io.setRollerSpeed(0);
                io.setHopperSpeed(0);
                break;

            case REVERSED:
            case EJECTING:
                goal = downPosition;
                updateArmController();
                io.setRollerSpeed(-power);
                io.setHopperSpeed(-Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case MANUAL:
                goal = manualPosition;
                updateArmController();
                break;

            case CHARACTERIZATION:
                // In characterization mode, armVoltage is maintained and not overridden by
                // ProfiledPID
                break;

            case DISABLED:
            default:
                io.stop();
                break;
        }
    }

    @Override
    public void simulationUpdate() {
    }

    @Override
    public double getSimulationCurrentDraw() {
        double current = inputs.armCurrentAmps;
        current += inputs.rollerCurrentAmps;
        current += inputs.hopperCurrentAmps;
        return current;
    }

    @Override
    public void initialize() {
        if (RobotBase.isSimulation() && io instanceof IntakeIOSim simIO) {
            SwerveBase.getInstance().getMapleSimDrive().ifPresent(simIO::attachMapleSimDrivetrain);
        }
    }

    public swervelib.simulation.ironmaple.simulation.IntakeSimulation getMapleIntakeSim() {
        if (io instanceof IntakeIOSim simIO) {
            if (simIO.getMapleIntakeSim() == null) {
                SwerveBase.getInstance().getMapleSimDrive().ifPresent(simIO::attachMapleSimDrivetrain);
            }
            return simIO.getMapleIntakeSim();
        }
        return null;
    }

    /**
     * Returns true if the intake/hopper contains fuel (game piece).
     */
    public boolean hasFuel() {
        if (getMapleIntakeSim() != null) {
            return getMapleIntakeSim().getGamePiecesAmount() > 0;
        }
        return inputs.hopperCurrentAmps > 3.0;
    }

    public boolean hasGamePiece() {
        return hasFuel();
    }

    /**
     * Checks if a given field pose is inside a low-overhead Trench corridor where
     * arm must be stowed low.
     */
    public static boolean isPoseInTrenchLowClearanceZone(Pose2d pose) {
        if (pose == null)
            return false;
        double x = pose.getX();
        double y = pose.getY();

        // Blue Trench corridors (X in [3.20, 6.10])
        boolean inBlueTrenchX = (x >= 3.20 && x <= 6.10);
        // Red Trench corridors (X in [10.44, 13.34])
        boolean inRedTrenchX = (x >= 10.44 && x <= 13.34);

        boolean inTopTrenchY = (y >= 6.50);
        boolean inBottomTrenchY = (y <= 1.55);

        return (inBlueTrenchX || inRedTrenchX) && (inTopTrenchY || inBottomTrenchY);
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Intake/Pivot Position", currentPosition);
        SmartDashboard.putNumber("Intake/Pivot Goal", goal);
        SmartDashboard.putNumber("Intake/Arm Applied Output", inputs.armAppliedVolts);
        SmartDashboard.putNumber("Intake/Arm Speed", inputs.armVelocityDegPerSec);
        SmartDashboard.putNumber("Intake/Wheels Applied Output", inputs.rollerAppliedVolts);
        SmartDashboard.putString("Intake/State", state.getStateName());
        SmartDashboard.putNumber("Intake/Setpoint Position", pivotProfiledPIDController.getSetpoint().position);
        SmartDashboard.putNumber("Intake/Setpoint Velocity", pivotProfiledPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber("Intake/Arm Voltage", armVoltage);

        // 3D Visualizer
        Translation3d armPivot = new Translation3d(0.2, 0, 0.2);
        Rotation3d armRotation = new Rotation3d(0, -Math.toRadians(currentPosition), 0);
        Pose3d armPose = new Pose3d(armPivot, armRotation);

        org.littletonrobotics.junction.Logger.recordOutput("Subsystems/Intake/ArmPose3d", armPose);
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "IntakeMechanism";
    }
}
