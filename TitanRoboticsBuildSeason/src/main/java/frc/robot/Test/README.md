# Titan Robotics Test Mode System

## Overview

The Test Mode system provides a comprehensive framework for testing, tuning, and diagnosing all robot subsystems. It's designed to be used during development, testing sessions, and competition for quick diagnostics and tuning.

## Architecture

### Core Components

- **TestMode**: Main orchestrator that manages all test categories
- **ShooterTuning**: Comprehensive shooter testing and PID tuning
- **IntakeTesting**: Intake arm, roller, and hopper testing
- **DriveCharacterization**: Swerve drive testing and SysID integration
- **SystemDiagnostics**: Hardware verification and motor testing
- **VisionTesting**: Camera and vision system testing
- **SysIDIntegration**: Enhanced SysID wrapper for system identification

### Test Categories

1. **Shooter Tuning**
   - Manual velocity control
   - Auto-aim testing at different distances
   - PID gain tuning via NetworkTables
   - Physics-based shooting validation

2. **Intake Testing**
   - Arm position control and calibration
   - Roller and hopper speed testing
   - Jam detection simulation
   - Position accuracy validation

3. **Drive Characterization**
   - SysID quasistatic and dynamic tests
   - Individual module testing
   - Kinematics validation
   - Odometry accuracy testing

4. **System Diagnostics**
   - Motor direction verification
   - Encoder functionality checks
   - CAN communication testing
   - Automated test sequences

5. **Vision Testing**
   - Limelight functionality testing
   - PhotonVision integration testing
   - Target tracking accuracy
   - Vision-based odometry validation

## Usage

### Enabling Test Mode

Test mode can be enabled via:
1. Dashboard toggle: `TestMode/Enabled`
2. Programmatic: `TestMode.getInstance().setEnabled(true)`

### Controller Layout

#### Driver Controller (Primary)
- **D-pad**: Switch between test categories
  - Up: Shooter Tuning
  - Right: Intake Testing  
  - Down: Drive Characterization
  - Left: System Diagnostics
  - Both Bumpers: Vision Testing

- **Face Buttons (A, B, X, Y)**: Switch between test modes within category
- **Triggers**: Activate tests
- **Joysticks**: Manual control during tests

#### Operator Controller (Secondary)
- **Face Buttons**: Specific test activation
- **D-pad**: Quick presets and parameter changes
- **Triggers**: Variable speed/voltage control
- **Joysticks**: Fine manual control

### Dashboard Integration

All test parameters and results are available on SmartDashboard under the `Test/` prefix:

#### Test Mode Controls
- `TestMode/Enabled`: Master toggle
- `TestMode/Category`: Current active category
- `TestMode/Running`: Test status indicator

#### Category-Specific Parameters
- `Test/Shooter/*`: Shooter tuning parameters
- `Test/Intake/*`: Intake test parameters
- `Test/Drive/*`: Drive test parameters
- `Test/Diagnostics/*`: Diagnostic test parameters
- `Test/Vision/*`: Vision test parameters

## LED Feedback

The LED system provides visual feedback for test status:
- **Solid Red**: Shooter testing (strobe when active)
- **Solid Blue**: Intake/Drive testing (strobe when active)
- **Breath Blue**: System diagnostics (heartbeat red if errors)
- **Strobe Gold**: Vision testing (solid yellow if no target)

## Tuning Workflow

### Shooter Tuning
1. Enable Test Mode → Shooter Tuning
2. Use Manual Velocity mode to verify basic functionality
3. Switch to Auto-Aim Test to validate shooting solutions
4. Use PID Tuning mode to optimize gains
5. Monitor performance metrics on dashboard

### Intake Testing
1. Enable Test Mode → Intake Testing
2. Test arm movement with Arm Control mode
3. Verify roller speeds with Roller Testing
4. Test hopper functionality
5. Calibrate positions with Position Calibration

### Drive Characterization
1. Enable Test Mode → Drive Characterization
2. Run SysID tests for system identification
3. Test individual modules for issues
4. Validate kinematics and odometry
5. Export data for analysis

## Safety Features

### Automatic Safety
- Motor current limiting
- Emergency stop functionality
- Test timeout protection
- Safe voltage limits

### Manual Safety
- Driver E-stop via controller
- Automatic subsystem shutdown on test exit
- Vision system safe modes
- Motor brake control

## Integration with Existing Code

### Robot Class Integration
```java
// In Robot constructor
testMode = TestMode.getInstance();

// In robotPeriodic()
if (testMode != null) {
    testMode.update();
}
```

### Subsystem Integration
- All subsystems continue normal operation when test mode is disabled
- Test mode automatically stops normal operations when enabled
- Clean cleanup on test mode exit

## Data Logging

All test sessions are automatically logged:
- Performance metrics
- Test parameters
- Error conditions
- Timestamps

Data can be exported for analysis and comparison.

## Troubleshooting

### Common Issues
1. **Test mode not responding**: Check `TUNING_MODE` flag in Constants
2. **Dashboard values not updating**: Verify NetworkTables connection
3. **Motors not responding**: Check CAN connection and motor controllers
4. **LED feedback not working**: Verify LED controller connection

### Debug Information
- Console output shows test mode transitions
- Dashboard shows detailed test status
- Error messages logged to console and dashboard

## Future Enhancements

Planned improvements:
- Automated test sequences
- Data export functionality
- Remote test control
- Advanced analytics
- Integration with AdvantageKit
- Historical data comparison

## Best Practices

1. **Always verify safety** before running tests
2. **Use appropriate voltage limits** for bench testing
3. **Monitor current draw** during motor tests
4. **Document test results** for future reference
5. **Keep test mode disabled** during competition unless needed
6. **Regular calibration** of encoders and sensors
7. **Backup configuration** before making changes

## Support

For questions or issues:
1. Check console output for error messages
2. Verify dashboard connections
3. Review test parameters
4. Consult subsystem documentation
5. Contact programming leads
