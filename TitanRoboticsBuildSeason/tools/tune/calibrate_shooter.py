#!/usr/bin/env python3
"""
Shooter Ballistics & Feedforward Calibration Tool for Team 8334.

Automates the calculations for:
1. Theoretical 2D ballistic trajectory calculations (70° fixed hood, Hub aperture height).
2. Comparing and validating the distance-to-RPM lookup table in Shooter.java.
3. Estimating SysId feedforward constants (kS, kV, kA) and initial feedback kP gains.
4. Exporting updated Java code for InterpolatingDoubleTreeMap.

Usage:
    python tools/tune/calibrate_shooter.py --compare
    python tools/tune/calibrate_shooter.py --calc-rpm 3.25
    python tools/tune/calibrate_shooter.py --generate --efficiency 0.42
    python tools/tune/calibrate_shooter.py --sysid-estimate
"""

import argparse
import math
import sys

# Robot physical parameters
WHEEL_DIAMETER_METERS = 0.1016 # 4 inches
HOOD_ANGLE_DEG = 70.0
HOOD_ANGLE_RAD = math.radians(HOOD_ANGLE_DEG)
SHOOTER_MOUNT_HEIGHT = 0.53 # meters from carpet
GOAL_HEIGHT = 1.8288 # Hub rim top height (meters)
APERTURE_HEIGHT = 1.48 # Hub inner scoring funnel center (meters)
GRAVITY = 9.80665

# Motor specifications (REV NEO Brushless, 1:1 gearing to flywheels)
NEO_FREE_SPEED_RPM = 5676.0
NOMINAL_VOLTAGE = 12.0

# Current production lookup table in Shooter.java
PRODUCTION_TABLE = [
    (1.20, 2400.0, 2450.0),
    (1.92, 2700.0, 2750.0),
    (2.47, 2900.0, 2950.0),
    (3.05, 3500.0, 3550.0),
    (3.48, 3550.0, 3600.0),
    (4.18, 3750.0, 3800.0),
    (5.00, 4100.0, 4150.0),
    (6.00, 4500.0, 4550.0),
]

def calculate_exit_velocity(distance_m, target_height=APERTURE_HEIGHT):
    """
    Solves for projectile launch velocity v0 required to reach (distance_m, target_height).
    y(x) = h0 + x * tan(theta) - (g * x^2) / (2 * v0^2 * cos^2(theta)) = target_height
    """
    delta_h = target_height - SHOOTER_MOUNT_HEIGHT
    tan_theta = math.tan(HOOD_ANGLE_RAD)
    cos_theta = math.cos(HOOD_ANGLE_RAD)
    
    numerator = GRAVITY * (distance_m ** 2)
    denominator = 2.0 * (cos_theta ** 2) * (distance_m * tan_theta - delta_h)
    
    if denominator <= 0:
        return None
    
    v0_squared = numerator / denominator
    return math.sqrt(v0_squared)

def calculate_time_of_flight(distance_m, exit_velocity_mps):
    """Calculates projectile flight time in seconds."""
    vx = exit_velocity_mps * math.cos(HOOD_ANGLE_RAD)
    return distance_m / vx

def velocity_to_rpm(exit_velocity_mps, efficiency=0.42):
    """
    Converts projectile exit velocity to flywheel surface speed and motor RPM.
    v_surface = exit_velocity / efficiency
    RPM = (v_surface / (pi * D)) * 60
    """
    wheel_circumference = math.pi * WHEEL_DIAMETER_METERS
    wheel_surface_speed = exit_velocity_mps / efficiency
    return (wheel_surface_speed / wheel_circumference) * 60.0

def rpm_to_velocity(rpm, efficiency=0.42):
    """Converts flywheel RPM to projectile exit velocity."""
    wheel_circumference = math.pi * WHEEL_DIAMETER_METERS
    wheel_surface_speed = (rpm / 60.0) * wheel_circumference
    return wheel_surface_speed * efficiency

def compare_table(efficiency=0.42):
    print("=" * 80)
    print("TEAM 8334 SHOOTER TABLE BALLISTIC COMPARISON")
    print(f"Fixed Hood Angle: {HOOD_ANGLE_DEG} deg | Mount Height: {SHOOTER_MOUNT_HEIGHT:.2f} m | Target Aperture: {APERTURE_HEIGHT:.2f} m")
    print(f"Assumed Launch Transfer Efficiency: {efficiency:.2%}")
    print("=" * 80)
    print(f"{'Dist (m)':<10} {'Prod Left':<12} {'Prod Right':<12} {'Prod Avg':<10} {'Model RPM':<12} {'Diff (RPM)':<12} {'TOF (s)':<8}")
    print("-" * 80)
    
    for dist, left_rpm, right_rpm in PRODUCTION_TABLE:
        prod_avg = (left_rpm + right_rpm) / 2.0
        v0 = calculate_exit_velocity(dist)
        if v0 is not None:
            model_rpm = velocity_to_rpm(v0, efficiency)
            tof = calculate_time_of_flight(dist, v0)
            diff = prod_avg - model_rpm
            print(f"{dist:<10.2f} {left_rpm:<12.0f} {right_rpm:<12.0f} {prod_avg:<10.0f} {model_rpm:<12.0f} {diff:>+12.1f} {tof:<8.2f}")
        else:
            print(f"{dist:<10.2f} {left_rpm:<12.0f} {right_rpm:<12.0f} {prod_avg:<10.0f} {'UNREACHABLE':<12} {'N/A':<12} {'N/A':<8}")
    print("=" * 80)

def generate_java_table(min_dist=1.2, max_dist=5.5, step=0.4, efficiency=0.42, spin_diff=50.0):
    print(f"// Generated Distance-to-RPM Tables (Efficiency: {efficiency:.2f}, Spin Differential: {spin_diff:.0f} RPM)")
    print("// Hood: 70 deg fixed | Mount Height: 0.53m | Target Aperture: 1.48m\n")
    
    dists = []
    curr = min_dist
    while curr <= max_dist + 1e-5:
        dists.append(round(curr, 2))
        curr += step
    
    left_entries = []
    right_entries = []
    
    for d in dists:
        v0 = calculate_exit_velocity(d)
        if v0 is not None:
            base_rpm = round(velocity_to_rpm(v0, efficiency) / 25.0) * 25.0 # round to nearest 25 RPM
            left_rpm = base_rpm - (spin_diff / 2.0)
            right_rpm = base_rpm + (spin_diff / 2.0)
            left_entries.append(f"leftRpmTable.put({d:.2f}, {left_rpm:.1f});")
            right_entries.append(f"rightRpmTable.put({d:.2f}, {right_rpm:.1f});")
    
    print("// Left Flywheel (Top / Counter-spin)")
    for entry in left_entries:
        print(f"        {entry}")
    print("\n// Right Flywheel (Bottom / Drive-spin)")
    for entry in right_entries:
        print(f"        {entry}")

def estimate_sysid():
    print("=" * 80)
    print("SHOOTER MOTOR FEEDFORWARD & PID THEORETICAL ESTIMATES (REV NEO 1:1)")
    print("=" * 80)
    # kV = nominal_voltage / free_speed_rpm
    kv = NOMINAL_VOLTAGE / NEO_FREE_SPEED_RPM
    # kS is typically 0.10 to 0.25 V to overcome initial brush/bearing friction
    ks_min, ks_max = 0.10, 0.25
    # kP: For a flywheel, a good starting proportional gain supplies ~1.0V for every 1000 RPM of error
    kp_nominal = 1.0 / 1000.0 # 0.0010 V/RPM
    
    print(f"Theoretical kV (Velocity Gain):      {kv:.6f} V / RPM  (approx {kv*1000:.3f} V per 1k RPM)")
    print(f"Measured Code kV (ShooterConstants): 0.002200 V / RPM")
    print(f"Expected kS (Static Friction):       {ks_min:.2f} - {ks_max:.2f} V")
    print(f"Recommended Initial kP:              {kp_nominal:.6f} V / RPM error (0.0005 - 0.0012)")
    print(f"Recommended Initial kI:              0.000000 (rely on feedforward kV)")
    print(f"Recommended Initial kD:              0.000000 (damping usually unnecessary for high-inertia flywheels)")
    print("=" * 80)

def main():
    parser = argparse.ArgumentParser(description="Shooter Ballistics & Feedforward Calibration Tool")
    parser.add_argument("--compare", action="store_true", help="Compare current table with ballistic model")
    parser.add_argument("--generate", action="store_true", help="Generate Java code for interpolation tables")
    parser.add_argument("--sysid-estimate", action="store_true", help="Display theoretical SysId and PID starting values")
    parser.add_argument("--calc-rpm", type=float, metavar="DIST", help="Calculate exact RPM and TOF for given distance")
    parser.add_argument("--efficiency", type=float, default=0.42, help="Energy transfer efficiency from flywheel to ball (default: 0.42)")
    parser.add_argument("--spin-diff", type=float, default=50.0, help="RPM differential between left and right flywheels (default: 50.0)")
    
    args = parser.parse_args()
    
    if args.calc_rpm is not None:
        v0 = calculate_exit_velocity(args.calc_rpm)
        if v0 is None:
            print(f"Distance {args.calc_rpm:.2f}m is geometrically unreachable at {HOOD_ANGLE_DEG} deg.")
        else:
            rpm = velocity_to_rpm(v0, args.efficiency)
            tof = calculate_time_of_flight(args.calc_rpm, v0)
            print(f"Distance:        {args.calc_rpm:.2f} m")
            print(f"Exit Velocity:   {v0:.2f} m/s")
            print(f"Time of Flight:  {tof:.3f} s")
            print(f"Flywheel RPM:    {rpm:.1f} RPM (Left: {rpm - args.spin_diff/2:.0f}, Right: {rpm + args.spin_diff/2:.0f})")
    elif args.generate:
        generate_java_table(efficiency=args.efficiency, spin_diff=args.spin_diff)
    elif args.sysid_estimate:
        estimate_sysid()
    else:
        compare_table(args.efficiency)

if __name__ == "__main__":
    main()
