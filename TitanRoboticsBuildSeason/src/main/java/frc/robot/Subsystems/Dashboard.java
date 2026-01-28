package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Interfaces.Subsystem;
import java.util.Map;

public class Dashboard implements Subsystem {

    private static Dashboard instance = null;

    private final ShuffleboardTab driverTab;
    private final SendableChooser<String> autoChooser;

    public static Dashboard getInstance() {
        if (instance == null) {
            instance = new Dashboard();
        }
        return instance;
    }

    private Dashboard() {
        driverTab = Shuffleboard.getTab("Driver");
        autoChooser = new SendableChooser<>();

        setupAutoChooser();
        setupLayout();

        SubsystemManager.registerSubsystem(this);
    }

    private void setupAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", "Do Nothing");
        autoChooser.addOption("Score Preload", "Score Preload");
        autoChooser.addOption("Blue Left Shoot Climb", "Blue Left Shoot Climb");
        // Add more autos here as they are created

        // Add to Shuffleboard at specific position and size
        driverTab.add("Auto Routine", autoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
    }

    private void setupLayout() {
        // Match Timer & Phase
        driverTab.addString("Match Phase", () -> {
            if (DriverStation.isAutonomous()) {
                return "AUTO (" + String.format("%.0f", DriverStation.getMatchTime()) + ")";
            } else if (DriverStation.isTeleop()) {
                double time = DriverStation.getMatchTime();
                if (time < 20) {
                    return "ENDGAME (" + String.format("%.0f", time) + ")";
                }
                return "TELEOP (" + String.format("%.0f", time) + ")";
            } else {
                return "DISABLED";
            }
        })
                .withPosition(2, 0)
                .withSize(2, 1);

        // Shooter Status
        driverTab.addBoolean("Shooter Ready", () -> Shooter.getInstance().isAtTargetVelocity())
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FF0000"))
                .withPosition(4, 0)
                .withSize(2, 2);

        // Intake Status
        driverTab.addString("Intake State", () -> Intake.getInstance().getState().toString())
                .withPosition(0, 1)
                .withSize(2, 1);

        // Camera (Placeholder for stream)
        // In proper implementation, CameraServer.startAutomaticCapture() would be
        // called in Robot.java
        // driverTab.add("Front Camera", CameraServer.getVideo())
        // .withWidget(BuiltInWidgets.kCameraStream)
        // .withPosition(2, 1)
        // .withSize(4, 3);

        // Field
        driverTab.add("Field", SwerveBase.getInstance().getField())
                .withWidget(BuiltInWidgets.kField)
                .withPosition(6, 0)
                .withSize(6, 4);
    }

    @Override
    public void update() {
        // Shuffleboard updates automatically for suppliers
    }

    @Override
    public void initialize() {
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
        return "Dashboard";
    }

    public SendableChooser<String> getAutoChooser() {
        return autoChooser;
    }
}
