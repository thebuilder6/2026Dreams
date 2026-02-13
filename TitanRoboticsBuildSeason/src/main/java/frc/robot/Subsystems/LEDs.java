package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.LEDConstants;
import frc.robot.Interfaces.Subsystem;
import java.util.Optional;
import frc.robot.Subsystems.SubsystemManager;

public class LEDs implements Subsystem {

    private static LEDs instance;
    private final Spark blinkin;
    private double currentPattern;

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    private LEDs() {
        SubsystemManager.registerSubsystem(this);
        blinkin = new Spark(LEDConstants.BLINKIN_PWM_PORT);
        currentPattern = LEDConstants.RAINBOW;
    }

    public void setPattern(double pattern) {
        currentPattern = pattern;
        blinkin.set(pattern);
    }

    @Override
    public void initialize() {
        setPattern(LEDConstants.RAINBOW);
    }

    @Override
    public void update() {
        if (!DriverStation.isEnabled()) {
            setPattern(LEDConstants.RAINBOW);
        } else {
            // Priority-based LED selection
            Intake intake = Intake.getInstance();
            Shooter shooter = Shooter.getInstance();

            if (intake.isJammed()) {
                setPattern(LEDConstants.STROBE_RED);
            } else if (shooter.isAtTargetVelocity() && shooter.isReadyToFire(
                    shooter.calculateShootingSolution(SwerveBase.getInstance().getPose(),
                            SwerveBase.getInstance().getFieldVelocity()).turretAngle())) {
                setPattern(LEDConstants.SOLID_GREEN);
            } else if (shooter.getTargetVelocityRPM() > 0) {
                setPattern(LEDConstants.STROBE_GOLD);
            } else if (intake.getState() == Intake.IntakeState.INTAKING) {
                setPattern(LEDConstants.LARSON_SCAN_RED);
            } else {
                // Default Alliance Color
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    setPattern(LEDConstants.SOLID_RED);
                } else {
                    setPattern(LEDConstants.SOLID_BLUE);
                }
            }
        }
        SmartDashboard.putNumber("LED/Pattern", currentPattern);
    }

    @Override
    public void simulationUpdate() {
        // Simulation can track the pattern via the Spark motor control simulation if
        // needed
    }

    @Override
    public void log() {
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "LEDs";
    }
}
