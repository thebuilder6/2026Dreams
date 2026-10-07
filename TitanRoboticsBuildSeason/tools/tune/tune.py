#!/usr/bin/env python3
"""
Team 8334 Unified Subsystem Tuning & Pit Calibration Tool.

An interactive and CLI automation utility designed to make robot calibration,
SysId characterization, and pit diagnostics effortless for students and mentors.

Usage:
    python tools/tune/tune.py                          # Interactive Menu
    python tools/tune/tune.py shooter 3.25             # Instant ballistic solution & zone check
    python tools/tune/tune.py shooter --compare        # Compare table with physics
    python tools/tune/tune.py intake 347 250           # Motion profile kinematics
    python tools/tune/tune.py swerve 10.0 16.2         # Effective wheel radius roll test
    python tools/tune/tune.py controls                 # Controller cheat sheet
    python tools/tune/tune.py checklist                # Pre-match calibration checklist
"""

import argparse
import math
import sys
import os

# Import subsystem calibrators
try:
    from calibrate_shooter import (
        calculate_exit_velocity,
        calculate_time_of_flight,
        velocity_to_rpm,
        compare_table,
        generate_java_table,
        estimate_sysid as shooter_sysid,
        HOOD_ANGLE_DEG,
        APERTURE_HEIGHT,
        SHOOTER_MOUNT_HEIGHT
    )
except ImportError:
    # If run from repo root
    sys.path.append(os.path.join(os.path.dirname(__file__)))
    from calibrate_shooter import (
        calculate_exit_velocity,
        calculate_time_of_flight,
        velocity_to_rpm,
        compare_table,
        generate_java_table,
        estimate_sysid as shooter_sysid,
        HOOD_ANGLE_DEG,
        APERTURE_HEIGHT,
        SHOOTER_MOUNT_HEIGHT
    )

try:
    from calibrate_intake import (
        estimate_feedforward_gains as intake_feedforward,
        solve_trapezoid_profile,
        print_sysid_estimates as intake_sysid,
        print_profile_analysis as intake_profile,
        MAX_VELOCITY_DEG_S,
        MAX_ACCEL_DEG_S2
    )
except ImportError:
    sys.path.append(os.path.join(os.path.dirname(__file__)))
    from calibrate_intake import (
        estimate_feedforward_gains as intake_feedforward,
        solve_trapezoid_profile,
        print_sysid_estimates as intake_sysid,
        print_profile_analysis as intake_profile,
        MAX_VELOCITY_DEG_S,
        MAX_ACCEL_DEG_S2
    )

try:
    from calibrate_vision import (
        solve_camera_pitch,
        verify_apriltag_distance,
        generate_camera_config_java
    )
except ImportError:
    sys.path.append(os.path.join(os.path.dirname(__file__)))
    from calibrate_vision import (
        solve_camera_pitch,
        verify_apriltag_distance,
        generate_camera_config_java
    )

BLUE_ALLIANCE_MAX_X = 4.6256
RED_ALLIANCE_MIN_X = 11.9154

CONTROLLER_CHEAT_SHEET = """
================================================================================
                    TEAM 8334 TEST MODE CONTROLLER CHEAT SHEET
================================================================================

[DRIVER CONTROLLER (Port 0) - Category & Primary Navigation]
--------------------------------------------------------------------------------
Hold [LB] + D-Pad to Select Category:
  * LB + D-Pad UP    (POV   0): SYSID CHARACTERIZATION (5-Mechanism selection)
  * LB + D-Pad DOWN  (POV 180): SHOOTER TUNING (Velocity, auto-aim, live PID)
  * LB + D-Pad LEFT  (POV 270): INTAKE TESTING (Arm position, rollers, hopper)
  * LB + D-Pad RIGHT (POV  90): SYSTEM DIAGNOSTICS (15s automated pit checks)

Within Active Category:
  * [A, B, X, Y]      : Select specific test routine or SysId test motion
  * [Right Trigger]   : Execute test / Step command (> 0.5 axis)
  * [Left/Right Stick]: Manual analog voltage / speed override

--------------------------------------------------------------------------------
[OPERATOR CONTROLLER (Port 1) - Mechanism Fine Controls & Presets]
--------------------------------------------------------------------------------
Shooter Mode:
  * D-Pad UP (0)     : 4500 RPM Preset        * Right Bumper: Manual Kicker Feed
  * D-Pad RIGHT (90) : 3500 RPM Preset        * Left Trigger: Variable Spinup
  * D-Pad DOWN (180) : 2500 RPM Preset
  * D-Pad LEFT (270) : 1000 RPM Preset

Intake Mode:
  * D-Pad UP (0)     : Standby Position (347 deg)
  * D-Pad DOWN (180) : Ground Deploy (250 deg)
  * Left Stick Y     : Manual Arm Jog (2.0V limit for calibration)
  * Right Bumper     : Save Current Arm Angle to Target Tunable
================================================================================
"""

PIT_CHECKLIST = """
================================================================================
                   PRE-MATCH & PIT CALIBRATION CHECKLIST
================================================================================
[ ] STATION 1: MECHANICAL CLEARANCE & IDLE MODES (Power OFF)
    [ ] Flywheels spin completely freely by hand (zero rub on hood polycarbonate).
    [ ] Flywheels and kicker confirmed in COAST mode (kCoast), NOT Brake.
    [ ] Intake arm moves smoothly between 250 deg (ground) and 347 deg (standby).
    [ ] Chain and polycord tension verified; no slack catching on frame.

[ ] STATION 2: 15-SECOND AUTOMATED PRE-FLIGHT (Tethered / Test Mode)
    [ ] Enable Test Mode -> LB + D-Pad Right (Diagnostics).
    [ ] Pull Trigger to run automated sequence:
        - Swerve motor pulse (+1.5V forward sign verification)
        - Steer azimuth delta verification (>= 1.0 deg)
        - Intake arm profile travel (< 25A current limit)
        - Shooter ramp to 1500 RPM (error < 50 RPM)
        - Vision link and AprilTag detection check
    [ ] Verify all green scorecard on Elastic tab 'Pre-Flight Diagnostics'.

[ ] STATION 3: SENSOR & ZERO ALIGNMENTS
    [ ] Arm absolute encoder reads within +/- 1 deg of physical position.
    [ ] Swerve modules point true forward (0 deg) with zero steer drift.
    [ ] Limelight and PhotonVision cameras clean; lenses free of dust/grease.

[ ] STATION 4: FIELD CARPET SHOOTER BENCHMARK (3 Quick Shots)
    [ ] 1.8m shot: Verify 2700 L / 2750 R RPM lands center funnel.
    [ ] 3.0m shot: Verify 3500 L / 3550 R RPM lands center funnel.
    [ ] Check recovery time during ball intake (< 0.15s to recover 150 RPM).

[ ] STATION 5: DRIVE TEAM HANDOFF
    [ ] Battery resting voltage >= 12.6V (Internal resistance <= 0.015 ohms).
    [ ] Driver controller haptics confirmed (Target lock & ball capture pulses).
    [ ] Set FMS match auto mission on Elastic dashboard.
================================================================================
"""

def solve_shooter_distance(distance_m, efficiency=0.42, spin_diff=50.0):
    v0 = calculate_exit_velocity(distance_m)
    if v0 is None:
        print(f"\n[ERROR] Distance {distance_m:.2f} m is geometrically unreachable at {HOOD_ANGLE_DEG} deg.")
        return

    rpm = velocity_to_rpm(v0, efficiency)
    tof = calculate_time_of_flight(distance_m, v0)
    left_rpm = rpm - (spin_diff / 2.0)
    right_rpm = rpm + (spin_diff / 2.0)
    
    # Check Alliance Zone legality
    in_blue_zone = distance_m <= BLUE_ALLIANCE_MAX_X
    
    print("=" * 70)
    print(f"SHOOTER FIRING SOLUTION FOR DISTANCE: {distance_m:.2f} m")
    print("=" * 70)
    print(f"Target Distance from Hub Center:     {distance_m:.2f} m")
    print(f"Required Exit Launch Velocity (v0):  {v0:.2f} m/s")
    print(f"Estimated Time of Flight (TOF):      {tof:.3f} s")
    print(f"Base Flywheel Setpoint:              {rpm:.1f} RPM")
    print(f"  * Left Flywheel (Top / Counter):   {left_rpm:.0f} RPM")
    print(f"  * Right Flywheel (Bottom / Drive): {right_rpm:.0f} RPM (+{spin_diff:.0f} backspin)")
    print("-" * 70)
    if in_blue_zone:
        print(f"Alliance Zone Rule Check:            [LEGAL] (X <= {BLUE_ALLIANCE_MAX_X:.2f} m)")
    else:
        print(f"Alliance Zone Rule Check:            [WARNING] Exceeds Alliance Zone ({distance_m:.2f}m > {BLUE_ALLIANCE_MAX_X:.2f}m)")
        print("                                     Shot will be BLOCKED by FieldMap.AllianceZones.")
    print("=" * 70)

def calculate_wheel_radius(measured_carpet_distance_m, motor_revolutions, gear_ratio=6.75):
    """
    Computes effective wheel radius based on actual distance travelled on carpet.
    Useful for accounting for tread wear and carpet pile compression.
    """
    wheel_rotations = motor_revolutions / gear_ratio
    wheel_circumference = measured_carpet_distance_m / wheel_rotations
    radius_meters = wheel_circumference / (2.0 * math.pi)
    radius_inches = radius_meters / 0.0254
    nominal_radius_m = 0.0508 # 2 inches
    error_percent = ((radius_meters - nominal_radius_m) / nominal_radius_m) * 100.0

    print("=" * 70)
    print("SWERVE DRIVE EFFECTIVE WHEEL RADIUS CALIBRATION")
    print("=" * 70)
    print(f"Measured Carpet Distance:            {measured_carpet_distance_m:.3f} m")
    print(f"Drive Motor Rotations:               {motor_revolutions:.2f} revs (Gear Ratio: {gear_ratio:.2f}:1)")
    print(f"Effective Wheel Circumference:       {wheel_circumference:.4f} m")
    print(f"Calibrated Effective Wheel Radius:   {radius_meters:.5f} m ({radius_inches:.3f} in)")
    print(f"Nominal 2-inch Radius Deviation:     {error_percent:+.2f}%")
    print("-" * 70)
    print(f"Recommended Constants.java setting:  WHEEL_RADIUS_METERS = {radius_meters:.5f};")
    print("=" * 70)

def interactive_menu():
    while True:
        print("\n" + "=" * 60)
        print("      FRC TEAM 8334 — ROBOT TUNING & CALIBRATION SUITE")
        print("=" * 60)
        print("  [1] Shooter: Distance Ballistic Solver (Query RPM & TOF)")
        print("  [2] Shooter: Compare Production Table vs Physics")
        print("  [3] Shooter: SysId Feedforward Theoretical Estimates")
        print("  [4] Intake: Arm Motion Profile & Transit Time Solver")
        print("  [5] Intake: Gravitational Feedforward (kG, kV) Estimates")
        print("  [6] Swerve: Wheel Radius Carpet Roll Calibration")
        print("  [7] Vision: Camera Pitch, AprilTag & Mount Calibration")
        print("  [8] Controller Cheat Sheet & TestMode Mapping")
        print("  [9] Pre-Match Pit Calibration Checklist")
        print("  [0] Exit")
        print("=" * 60)
        
        choice = input("Select an option [0-9]: ").strip()
        
        if choice == "1":
            val = input("\nEnter distance to Hub center in meters (e.g., 2.5): ").strip()
            try:
                dist = float(val)
                solve_shooter_distance(dist)
            except ValueError:
                print("[ERROR] Invalid number.")
        elif choice == "2":
            print()
            compare_table()
        elif choice == "3":
            print()
            shooter_sysid()
        elif choice == "4":
            s_val = input("\nEnter start angle in degrees [default 347.0]: ").strip()
            e_val = input("Enter end angle in degrees [default 250.0]: ").strip()
            s = float(s_val) if s_val else 347.0
            e = float(e_val) if e_val else 250.0
            print()
            intake_profile(s, e)
        elif choice == "5":
            print()
            intake_sysid()
        elif choice == "6":
            dist_str = input("\nEnter measured distance on carpet in meters (e.g., 10.0): ").strip()
            rev_str = input("Enter drive motor rotations from AdvantageScope (e.g., 208.5): ").strip()
            try:
                d = float(dist_str)
                r = float(rev_str)
                calculate_wheel_radius(d, r)
            except ValueError:
                print("[ERROR] Invalid input.")
        elif choice == "7":
            print("\n" + "-" * 50)
            print("  VISION CALIBRATION & ALIGNMENT SUBMENU")
            print("-" * 50)
            print("  [1] Solve Camera Downward Pitch from Tape Distance")
            print("  [2] Verify AprilTag Distance vs Measured Ground Truth")
            print("  [3] Generate CameraConfig Java Code")
            v_choice = input("Select vision tool [1-3]: ").strip()
            if v_choice == "1":
                h_str = input("Camera height off carpet in meters [default 0.45]: ").strip()
                d_str = input("Tape ground distance to target center in meters (e.g. 2.0): ").strip()
                p_str = input("Camera measured target pitch in degrees [default 0.0]: ").strip()
                try:
                    h = float(h_str) if h_str else 0.45
                    d = float(d_str)
                    p = float(p_str) if p_str else 0.0
                    solve_camera_pitch(h, d, p)
                except ValueError:
                    print("[ERROR] Invalid numeric input.")
            elif v_choice == "2":
                t_str = input("True physical distance to AprilTag in meters (e.g. 2.50): ").strip()
                v_str = input("Vision measured distance in meters (e.g. 2.54): ").strip()
                try:
                    t = float(t_str)
                    v = float(v_str)
                    verify_apriltag_distance(t, v)
                except ValueError:
                    print("[ERROR] Invalid numeric input.")
            elif v_choice == "3":
                name = input("Camera name [default limelight-front]: ").strip() or "limelight-front"
                ctype = input("Camera type (LIMELIGHT or PHOTONVISION) [default LIMELIGHT]: ").strip() or "LIMELIGHT"
                f_str = input("Forward offset in meters [default 0.25]: ").strip()
                h_str = input("Mount height in meters [default 0.45]: ").strip()
                p_str = input("Pitch angle in degrees [default 15.0]: ").strip()
                try:
                    f = float(f_str) if f_str else 0.25
                    h = float(h_str) if h_str else 0.45
                    p = float(p_str) if p_str else 15.0
                    generate_camera_config_java(name=name, cam_type=ctype, forward_m=f, height_m=h, pitch_deg=p)
                except ValueError:
                    print("[ERROR] Invalid numeric input.")
        elif choice == "8":
            print(CONTROLLER_CHEAT_SHEET)
        elif choice == "9":
            print(PIT_CHECKLIST)
        elif choice == "0":
            print("\nExiting tuning suite. Good luck in your match!")
            break
        else:
            print("\n[ERROR] Unknown option. Please choose [0-9].")

def main():
    parser = argparse.ArgumentParser(description="Team 8334 Subsystem Tuning & Calibration Suite")
    subparsers = parser.add_subparsers(dest="command", help="Subsystem or tool to execute")
    
    # Shooter subcommand
    p_shooter = subparsers.add_parser("shooter", help="Shooter ballistics and lookup tuning")
    p_shooter.add_argument("distance", nargs="?", type=float, help="Distance to Hub center in meters")
    p_shooter.add_argument("--compare", action="store_true", help="Compare current table with physics")
    p_shooter.add_argument("--generate", action="store_true", help="Generate Java lookup table code")
    p_shooter.add_argument("--sysid", action="store_true", help="Show theoretical feedforward constants")
    
    # Intake subcommand
    p_intake = subparsers.add_parser("intake", help="Intake arm kinematics and motion profiling")
    p_intake.add_argument("start_angle", nargs="?", type=float, default=347.0, help="Start angle (degrees)")
    p_intake.add_argument("end_angle", nargs="?", type=float, default=250.0, help="End angle (degrees)")
    p_intake.add_argument("--sysid", action="store_true", help="Show gravitational feedforward constants")
    
    # Swerve subcommand
    p_swerve = subparsers.add_parser("swerve", help="Swerve drive calibration utilities")
    p_swerve.add_argument("distance", type=float, help="Measured carpet distance in meters")
    p_swerve.add_argument("motor_rotations", type=float, help="Measured drive motor rotations")
    p_swerve.add_argument("--gear-ratio", type=float, default=6.75, help="Drive gearing (default: 6.75:1)")

    # Vision subcommand
    p_vision = subparsers.add_parser("vision", help="Camera calibration, pitch alignment, and mount generation")
    p_vision.add_argument("--solve-pitch", nargs=3, type=float, metavar=("CAM_HEIGHT", "GROUND_DIST", "MEASURED_PITCH"),
                          help="Solve downward camera tilt from height, measured distance, and target pitch")
    p_vision.add_argument("--tag-verify", nargs=2, type=float, metavar=("TRUE_DIST", "VISION_DIST"),
                          help="Compare measured AprilTag distance vs true distance")
    p_vision.add_argument("--generate", action="store_true", help="Generate CameraConfig Java snippet")
    p_vision.add_argument("--name", type=str, default="limelight-front", help="Camera name")
    p_vision.add_argument("--type", type=str, default="LIMELIGHT", choices=["LIMELIGHT", "PHOTONVISION"], help="Camera type")
    p_vision.add_argument("--forward", type=float, default=0.25, help="Forward offset in meters")
    p_vision.add_argument("--height", type=float, default=0.45, help="Mount height in meters")
    p_vision.add_argument("--pitch", type=float, default=15.0, help="Pitch in degrees")
    
    # Controls & Checklist
    subparsers.add_parser("controls", help="Display controller bindings for Test Mode")
    subparsers.add_parser("checklist", help="Display pre-match pit calibration checklist")
    
    args = parser.parse_args()
    
    if args.command == "shooter":
        if args.compare:
            compare_table()
        elif args.generate:
            generate_java_table()
        elif args.sysid:
            shooter_sysid()
        elif args.distance is not None:
            solve_shooter_distance(args.distance)
        else:
            compare_table()
    elif args.command == "intake":
        if args.sysid:
            intake_sysid()
        else:
            intake_profile(args.start_angle, args.end_angle)
    elif args.command == "swerve":
        calculate_wheel_radius(args.distance, args.motor_rotations, args.gear_ratio)
    elif args.command == "vision":
        if args.solve_pitch:
            solve_camera_pitch(args.solve_pitch[0], args.solve_pitch[1], args.solve_pitch[2])
        elif args.tag_verify:
            verify_apriltag_distance(args.tag_verify[0], args.tag_verify[1])
        elif args.generate:
            generate_camera_config_java(
                name=args.name,
                cam_type=args.type,
                forward_m=args.forward,
                height_m=args.height,
                pitch_deg=args.pitch
            )
        else:
            solve_camera_pitch(0.45, 2.0, 0.0)
    elif args.command == "controls":
        print(CONTROLLER_CHEAT_SHEET)
    elif args.command == "checklist":
        print(PIT_CHECKLIST)
    else:
        interactive_menu()

if __name__ == "__main__":
    main()
