# 🤖 2026 Robot Operator's Guide

Welcome to Johnathan's 2026 Dream Driver's Manual. This guide covers all controls, automated features, and dashboard indicators for our latest robot.

---

## 🎮 Driver Controls (Xbox Controller)

Common driving is **Field-Oriented** (unless disabled on Dashboard). The robot automatically adjusts for Red/Blue alliance orientation.

### Primary Driving
| Control | Action | Details |
| :--- | :--- | :--- |
| **Left Joystick** | **Move / Strafe** | Controls robot translation on the field. |
| **Right Joystick (X)** | **Manual Rotation** | Rotate the robot manually. |
| **A Button** | **Zero Gyro** | Resets "Forward" to point away from your driver station. |

### **Automated Driving & Navigation**
| Control | Mode | Behavior |
| :--- | :--- | :--- |
| **Left Trigger** | **Glide Mode** | Hold to automatically drive to the **nearest Glide Point**. The robot will calculate the path to the closest strategic location listed below. |
| **Left Bumper** | **Ball Hunt** | Hold to use vision/sensors to automatically track and pick up nearest balls. |
| **Right Joystick (Flick)**| **Snap to Turn** | Quickly flick the stick to the edge to snap the robot to that field angle. |

---

## 📍 Glide Points & Auto-Tunneling

The **Glide Mode** (Left Trigger) is one of the most powerful navigation tools. When held, the robot identifies the nearest predefined waypoint and takes control to move you there precisely.

### **Available Glide Locations**
Depending on your alliance, the robot tracks:
- **Feeders (Top/Bottom)**: Perfect for quickly returning to the loading zone.
- **Hub (Front/Back)**: Positions the robot for optimal scoring or safe zones near the hub.
- **Trenches (Top/Bottom)**: **Auto-Tunneling**. If you glide to a Trench, the robot will not only drive to the entrance but will automatically navigate through the tunnel to the exit point.
- **Midfield (Top/Bottom)**: Strategic waypoints for crossing the field safely.

> [!TIP]
> The Dashboard shows the **"Nearest Glide"** point name in real-time. Use this to confirm where the robot will go before holding the trigger!

---

## ☄️ Scoring & Combat

Our robot features a fully automated shooting solution to maximize accuracy during fast-paced play.

### **Auto-Aim & Shoot**
- **Trigger**: Hold **Right Trigger** (Threshold: 50%).
- **Logic**: 
    1. Robot calculates a physics-based shooting solution including momentum compensation.
    2. Robot automatically rotates to the target.
    3. Flywheels spin up to the required RPM.
    4. **Auto-Fire**: The feeder launches the ball ONLY when the robot is accurately aimed (< 2.5° error) and the shooter is at speed.

---

## 📦 Intake Management

The intake system is designed to be highly automated with built-in jam protection.

### **Controls**
- **X Button (Toggle)**: Switches between **Intake** (Arm down, rollers on) and **Feed** (Arm up, hopper only).
- **Y Button**: **Stop/Idle**. Resets intake state and stops all rollers.
- **B Button**: **Eject**. Runs rollers and hopper in reverse to clear obstructions.

### **Smart Intake Features**
- **Auto-Jam Clear**: If the intake rollers detect a high current stall (jam), the robot will automatically reverse for a brief period before resuming.

---

## 📊 Dashboard & Monitoring (Elastic/SmartDashboard)

Monitor these critical indicators during the match:

### **Status Lights**
- ✅ **Shooter Ready**: Green when the flywheels are at the correct speed for the target distance.
- 🎯 **Lined Up**: Indicates the swerve base has achieved the correct heading for the auto-aim solution.
- 🏢 **Hub Active**: **CRITICAL**. Tells you if scoring is allowed right now.
- 📍 **Nearest Glide**: Displays the name of the point the robot will target if Glide Mode is activated.

### **Feature Toggles**
You can enable/disable these features via the Dashboard to customize driver feel:
- `Slow Mode`: Caps speed at 35% for precision maneuvering.
- `Field Oriented`: Toggles between field-relative and robot-relative driving.
- `Auto Aim`: Enables/Disables trigger-based shooting assistance.
- `Ball Hunt`: Toggle vision-tracked ball collection.
- `Glide Points`: **Master Toggle** for the Left Trigger navigation system.
- `Snap to Turn`: Toggle flick-based rotation.

---

## 🕒 2026 Match Rules Reference
**Hub Status Shifts**:
The Alliance Hub activates and deactivates throughout the match. 
- **Auto/Transition/End Game**: Hub is always **Active**.
- **Teleop SHIFTS**: The Hub flips status every ~25 seconds based on Game Data ('R' or 'B'). Check the Dashboard indicator or GameData string to confirm status.
