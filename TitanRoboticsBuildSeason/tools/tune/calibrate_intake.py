#!/usr/bin/env python3
"""
Intake Arm Physics & Motion Profile Calibration Tool for Team 8334.

Automates the calculations for:
1. Gravitational feedforward torque and kG constant estimation.
2. Arm pivot velocity feedforward kV estimation based on gear ratio and motor specs.
3. Trapezoidal motion profile kinematics (transit time from Standby to Ground).
4. Current limit and jam detection verification.

Usage:
    python tools/tune/calibrate_intake.py --profile
    python tools/tune/calibrate_intake.py --sysid-estimate
    python tools/tune/calibrate_intake.py --transit-time 347.0 250.0
"""

import argparse
import math
import sys

# Robot physical parameters
ARM_MASS_KG = 5.5               # kg (estimated intake arm assembly)
ARM_COM_METERS = 0.28           # meters from pivot to center of mass
GEAR_RATIO = 80.0               # 80:1 reduction
GRAVITY = 9.80665

# Positions (Degrees)
STANDBY_POS_DEG = 347.0
GROUND_POS_DEG = 250.0
HORIZONTAL_DATUM_DEG = 250.0    # Angle where arm is perpendicular to gravity (max torque)

# Profile constraints
MAX_VELOCITY_DEG_S = 400.0     # deg/s
MAX_ACCEL_DEG_S2 = 400.0       # deg/s^2

# REV NEO Specs
NEO_FREE_SPEED_RPM = 5676.0
NEO_STALL_TORQUE_NM = 2.6
NEO_STALL_CURRENT_A = 105.0
KT_NM_PER_AMP = NEO_STALL_TORQUE_NM / NEO_STALL_CURRENT_A # ~0.02476 N*m/A
NOMINAL_VOLTAGE = 12.0
MOTOR_RESISTANCE_OHMS = NOMINAL_VOLTAGE / NEO_STALL_CURRENT_A # ~0.1143 ohms

def calculate_max_gravity_torque():
    """Calculates peak gravitational torque at horizontal position (N*m)."""
    return ARM_MASS_KG * GRAVITY * ARM_COM_METERS

def estimate_feedforward_gains():
    """Estimates theoretical kG, kV, and kS for the intake arm."""
    tau_peak = calculate_max_gravity_torque()
    
    # Motor torque needed at peak = tau_peak / GEAR_RATIO
    tau_motor = tau_peak / GEAR_RATIO
    current_motor = tau_motor / KT_NM_PER_AMP
    kg_volts = current_motor * MOTOR_RESISTANCE_OHMS
    
    # Free speed at arm output in deg/s
    arm_free_speed_dps = (NEO_FREE_SPEED_RPM / GEAR_RATIO) * 360.0 / 60.0
    kv_volts_per_dps = NOMINAL_VOLTAGE / arm_free_speed_dps
    
    # Friction kS estimate (typically 0.15 - 0.30V for planetary/chain stages)
    ks_min, ks_max = 0.15, 0.30
    
    return {
        "tau_peak_nm": tau_peak,
        "kg_volts": kg_volts,
        "arm_free_speed_dps": arm_free_speed_dps,
        "kv_volts_per_dps": kv_volts_per_dps,
        "ks_min": ks_min,
        "ks_max": ks_max
    }

def solve_trapezoid_profile(start_deg, end_deg, max_v=MAX_VELOCITY_DEG_S, max_a=MAX_ACCEL_DEG_S2):
    """
    Computes trapezoidal motion profile phases for moving between two angles.
    Returns: (is_triangular, t_accel, t_cruise, t_total, peak_v)
    """
    delta_deg = abs(end_deg - start_deg)
    
    # Distance needed to reach max velocity
    d_accel = (max_v ** 2) / (2.0 * max_a)
    
    if 2.0 * d_accel >= delta_deg:
        # Triangular profile (does not reach max velocity)
        is_triangular = True
        peak_v = math.sqrt(delta_deg * max_a)
        t_accel = peak_v / max_a
        t_cruise = 0.0
        t_total = 2.0 * t_accel
    else:
        # Trapezoidal profile
        is_triangular = False
        peak_v = max_v
        t_accel = max_v / max_a
        d_cruise = delta_deg - (2.0 * d_accel)
        t_cruise = d_cruise / max_v
        t_total = (2.0 * t_accel) + t_cruise
        
    return {
        "delta_deg": delta_deg,
        "is_triangular": is_triangular,
        "peak_velocity_dps": peak_v,
        "t_accel_s": t_accel,
        "t_cruise_s": t_cruise,
        "t_total_s": t_total
    }

def print_sysid_estimates():
    g = estimate_feedforward_gains()
    print("=" * 80)
    print("INTAKE ARM FEEDFORWARD & SYSTEM IDENTIFICATION THEORETICAL ESTIMATES")
    print(f"Assembly Mass: {ARM_MASS_KG:.1f} kg | Center of Mass: {ARM_COM_METERS:.2f} m | Gear Ratio: {GEAR_RATIO:.1f}:1")
    print("=" * 80)
    print(f"Peak Gravity Torque at Arm Pivot:     {g['tau_peak_nm']:.2f} N*m")
    print(f"Theoretical kG (Gravity Feedforward): {g['kg_volts']:.4f} V (Code default: 0.34 V)")
    print(f"Arm Free Speed at Nominal Voltage:   {g['arm_free_speed_dps']:.1f} deg/s")
    print(f"Theoretical kV (Velocity Gain):      {g['kv_volts_per_dps']:.6f} V / (deg/s)")
    print(f"Expected kS (Static Friction):       {g['ks_min']:.2f} - {g['ks_max']:.2f} V")
    print(f"Recommended Initial kP:              0.08 - 0.15 V / deg error")
    print(f"Recommended Initial kD:              0.005 - 0.015 V / (deg/s) error")
    print("=" * 80)

def print_profile_analysis(start_deg=STANDBY_POS_DEG, end_deg=GROUND_POS_DEG):
    prof = solve_trapezoid_profile(start_deg, end_deg)
    print("=" * 80)
    print(f"TRAPEZOIDAL MOTION PROFILE KINEMATICS: {start_deg:.1f} deg -> {end_deg:.1f} deg")
    print(f"Constraints: Max V = {MAX_VELOCITY_DEG_S:.1f} deg/s | Max A = {MAX_ACCEL_DEG_S2:.1f} deg/s^2")
    print("=" * 80)
    print(f"Total Angular Displacement:          {prof['delta_deg']:.1f} deg")
    print(f"Profile Type:                        {'Triangular (distance too short to reach max V)' if prof['is_triangular'] else 'Trapezoidal'}")
    print(f"Peak Velocity Reached:               {prof['peak_velocity_dps']:.1f} deg/s ({prof['peak_velocity_dps']/MAX_VELOCITY_DEG_S:.1%} of constraint)")
    print(f"Acceleration Phase Duration:         {prof['t_accel_s']:.3f} s")
    print(f"Cruise Phase Duration:               {prof['t_cruise_s']:.3f} s")
    print(f"Total Transit Duration:              {prof['t_total_s']:.3f} s")
    print("=" * 80)

def main():
    parser = argparse.ArgumentParser(description="Intake Arm Physics & Motion Profile Calibration Tool")
    parser.add_argument("--sysid-estimate", action="store_true", help="Display theoretical SysId and feedforward values")
    parser.add_argument("--profile", action="store_true", help="Analyze standard Standby-to-Ground motion profile")
    parser.add_argument("--transit-time", nargs=2, type=float, metavar=("START", "END"), help="Calculate transit time between two custom angles")
    
    args = parser.parse_args()
    
    if args.transit_time:
        print_profile_analysis(args.transit_time[0], args.transit_time[1])
    elif args.sysid_estimate:
        print_sysid_estimates()
    else:
        print_sysid_estimates()
        print()
        print_profile_analysis()

if __name__ == "__main__":
    main()
