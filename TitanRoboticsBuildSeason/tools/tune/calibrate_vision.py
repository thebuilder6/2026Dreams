#!/usr/bin/env python3
"""
Team 8334 Vision Subsystem Calibration & Alignment Tool.

Automates:
1. Solving exact camera pitch tilt from ground target distance measurements.
2. Comparing measured AprilTag distance vs ground truth and estimating stdDevMultiplier.
3. Generating copy-pasteable Java CameraConfig code for VisionConfig.java.

Usage:
    python tools/tune/calibrate_vision.py --solve-pitch 0.45 2.0 0.0
    python tools/tune/calibrate_vision.py --tag-verify 2.0 2.04
    python tools/tune/calibrate_vision.py --generate
"""

import argparse
import math
import sys

# Robot default vision parameters
DEFAULT_CAMERA_HEIGHT_M = 0.45
DEFAULT_FUEL_RADIUS_M = 0.08
DEFAULT_CAMERA_FORWARD_M = 0.25

def solve_camera_pitch(camera_height_m, ground_dist_m, target_pitch_deg, target_height_m=DEFAULT_FUEL_RADIUS_M):
    """
    Solves for the downward camera mount tilt theta_camera:
    d = (h_cam - h_target) / tan(theta_cam + theta_target)
    theta_cam + theta_target = atan((h_cam - h_target) / d)
    theta_cam = atan((h_cam - h_target) / d) - theta_target
    """
    delta_h = camera_height_m - target_height_m
    if ground_dist_m <= 0.05 or delta_h <= 0:
        print("[ERROR] Invalid geometry: ground distance and camera height must be positive.")
        return None

    # Total angle between horizon and target line of sight
    total_angle_rad = math.atan2(delta_h, ground_dist_m)
    total_angle_deg = math.degrees(total_angle_rad)

    # In robot frame, downward tilt is negative pitch
    # total_angle = -camera_pitch + target_pitch
    camera_pitch_deg = -(total_angle_deg - target_pitch_deg)

    # Sensitivity: delta_distance / delta_angle
    # d(d)/d(theta) = -(h_cam - h_target) / sin^2(total_angle)
    sensitivity_m_per_deg = (delta_h / (math.sin(total_angle_rad) ** 2)) * math.radians(1.0)

    print("\n" + "=" * 65)
    print("       TEAM 8334 — CAMERA PITCH & GROUND TILT SOLVER")
    print("=" * 65)
    print(f"  Camera Height off Carpet:     {camera_height_m:.3f} m ({camera_height_m * 39.37:.1f} in)")
    print(f"  Target Height (Fuel Center):   {target_height_m:.3f} m ({target_height_m * 39.37:.1f} in)")
    print(f"  Physical Tape Distance:       {ground_dist_m:.3f} m ({ground_dist_m * 3.28084:.2f} ft)")
    print(f"  Camera Measured Target Pitch: {target_pitch_deg:+.2f}°")
    print("-" * 65)
    print(f"  CALCULATED CAMERA PITCH:      {camera_pitch_deg:+.2f}°")
    print(f"  Angle Sensitivity at {ground_dist_m:.1f}m:    ±{sensitivity_m_per_deg * 100:.1f} cm error per 1.0° tilt error")
    print("=" * 65)

    return camera_pitch_deg

def verify_apriltag_distance(true_distance_m, vision_distance_m):
    """
    Evaluates AprilTag distance error and recommends a standard deviation multiplier.
    """
    error_m = vision_distance_m - true_distance_m
    percent_error = (error_m / true_distance_m) * 100.0

    # Base noise variance model: sigma = 0.08 + (d^2 / 25.0)
    expected_std_dev = 0.08 + (true_distance_m ** 2) / 25.0
    recommended_multiplier = max(0.5, min(3.0, (abs(error_m) / max(0.04, expected_std_dev))))

    print("\n" + "=" * 65)
    print("       TEAM 8334 — APRILTAG DISTANCE & TRUST VERIFICATION")
    print("=" * 65)
    print(f"  Physical Tape Ground Truth:   {true_distance_m:.3f} m ({true_distance_m * 3.28084:.2f} ft)")
    print(f"  Vision Measured Distance:     {vision_distance_m:.3f} m ({vision_distance_m * 3.28084:.2f} ft)")
    print(f"  Measurement Residual Error:   {error_m:+.3f} m ({error_m * 39.37:+.1f} in)")
    print(f"  Percentage Error:             {percent_error:+.2f}%")
    print("-" * 65)
    status = "EXCELLENT (< 3 cm)" if abs(error_m) < 0.03 else ("ACCEPTABLE (< 8 cm)" if abs(error_m) < 0.08 else "HIGH ERROR — INSPECT LENS / MOUNT")
    print(f"  Calibration Health:           {status}")
    print(f"  Recommended stdDevMultiplier: {recommended_multiplier:.2f}")
    print("=" * 65)

    return recommended_multiplier

def generate_camera_config_java(name="limelight-front", cam_type="LIMELIGHT", role="HYBRID",
                                forward_m=DEFAULT_CAMERA_FORWARD_M, side_m=0.0, height_m=DEFAULT_CAMERA_HEIGHT_M,
                                pitch_deg=15.0, yaw_deg=0.0, multiplier=1.0, megatag2=True):
    """
    Generates Java CameraConfig code snippet ready to paste into VisionConfig.java.
    """
    yaw_rad_expr = f"Math.toRadians({yaw_deg:.1f})" if abs(yaw_deg) > 0.01 else "0.0"
    pitch_rad_expr = f"Math.toRadians({pitch_deg:.1f})"

    factory = "limelight" if cam_type.upper() == "LIMELIGHT" else "photonVision"

    java_code = f"""
// ── Generated CameraConfig for {name} ──────────────────────────────────────
CameraConfig.addCamera(
    CameraConfig.{factory}("{name}")
        .withTransform(new Transform3d(
            new Translation3d({forward_m:.3f}, {side_m:.3f}, {height_m:.3f}),
            new Rotation3d(0.0, {pitch_rad_expr}, {yaw_rad_expr})))
        .withRole(CameraRole.{role})
        .withStdDevMultiplier({multiplier:.2f})"""

    if cam_type.upper() == "LIMELIGHT":
        java_code += f"\n        .withMegaTag2({str(megatag2).lower()})"

    java_code += "\n);"

    print("\n" + "=" * 65)
    print("       GENERATED JAVA CONFIGURATION SNIPPET")
    print("=" * 65)
    print(java_code)
    print("=" * 65)
    return java_code

def main():
    parser = argparse.ArgumentParser(description="Team 8334 Vision Subsystem Calibration & Alignment Tool")
    parser.add_argument("--solve-pitch", nargs=3, type=float, metavar=("CAM_HEIGHT", "GROUND_DIST", "MEASURED_PITCH"),
                        help="Solve downward camera tilt from height, measured distance, and target pitch")
    parser.add_argument("--tag-verify", nargs=2, type=float, metavar=("TRUE_DIST", "VISION_DIST"),
                        help="Compare measured AprilTag distance vs true distance")
    parser.add_argument("--generate", action="store_true", help="Generate CameraConfig Java snippet")
    parser.add_argument("--name", type=str, default="limelight-front", help="Camera name")
    parser.add_argument("--type", type=str, default="LIMELIGHT", choices=["LIMELIGHT", "PHOTONVISION"], help="Camera type")
    parser.add_argument("--role", type=str, default="HYBRID", choices=["APRILTAG", "OBJECT_DETECTION", "HYBRID"], help="Camera role")
    parser.add_argument("--forward", type=float, default=0.25, help="Forward offset in meters")
    parser.add_argument("--height", type=float, default=0.45, help="Mount height in meters")
    parser.add_argument("--pitch", type=float, default=15.0, help="Pitch in degrees")

    args = parser.parse_args()

    if args.solve_pitch:
        solve_camera_pitch(args.solve_pitch[0], args.solve_pitch[1], args.solve_pitch[2])
    elif args.tag_verify:
        verify_apriltag_distance(args.tag_verify[0], args.tag_verify[1])
    elif args.generate:
        generate_camera_config_java(
            name=args.name,
            cam_type=args.type,
            role=args.role,
            forward_m=args.forward,
            height_m=args.height,
            pitch_deg=args.pitch
        )
    else:
        # Quick interactive demo
        print("Team 8334 Vision Calibrator. Use --solve-pitch, --tag-verify, or --generate.")
        solve_camera_pitch(0.45, 2.0, 0.0)

if __name__ == "__main__":
    main()
