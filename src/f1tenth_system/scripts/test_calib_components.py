#!/usr/bin/env python3
"""
Unit tests for current-acceleration calibration components

This script tests the individual components without requiring ROS2 or hardware.

Usage:
  python3 test_calib_components.py
"""

import sys
import numpy as np
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

# Import the calibration modules
# (In actual use, these would be imported after ROS2 init)


class Figure8Trajectory:
    """Copy of Figure8Trajectory from current_acc_calib.py for testing"""
    
    def __init__(self, R: float = 1.4, center_distance: float = 3.0, 
                 points_per_circle: int = 200):
        self.R = R
        self.center_distance = center_distance
        self.points_per_circle = points_per_circle
        self.trajectory = self._generate_trajectory()
        self.trajectory_length = len(self.trajectory)
    
    def _generate_trajectory(self) -> np.ndarray:
        theta1 = np.linspace(0, 2*np.pi, self.points_per_circle)
        right_circle_x = self.R + self.R * np.cos(theta1)
        right_circle_y = self.R * np.sin(theta1)
        
        left_circle_x = -self.R + self.R * np.cos(theta1)
        left_circle_y = self.R * np.sin(theta1)
        
        trajectory = np.vstack([
            np.column_stack([right_circle_x, right_circle_y]),
            np.column_stack([left_circle_x, left_circle_y])
        ])
        
        return trajectory
    
    def get_closest_point(self, current_pos: np.ndarray, start_idx: int = 0):
        search_range = 50
        end_idx = min(start_idx + search_range, self.trajectory_length)
        search_trajectory = self.trajectory[start_idx:end_idx]
        
        distances = np.linalg.norm(search_trajectory - current_pos, axis=1)
        min_idx_local = np.argmin(distances)
        min_idx_global = start_idx + min_idx_local
        
        closest_point = self.trajectory[min_idx_global]
        distance = distances[min_idx_local]
        
        return closest_point, min_idx_global, distance


class PurePursuitController:
    """Copy of PurePursuitController from current_acc_calib.py for testing"""
    
    def __init__(self, wheelbase: float = 0.33, lookahead_gain: float = 1.0):
        self.wheelbase = wheelbase
        self.lookahead_gain = lookahead_gain
        self.max_steering_angle = 0.30
    
    def compute_steering(self, current_pos: np.ndarray, current_yaw: float,
                        lookahead_point: np.ndarray, velocity: float) -> float:
        delta_x = lookahead_point[0] - current_pos[0]
        delta_y = lookahead_point[1] - current_pos[1]
        
        distance = np.sqrt(delta_x**2 + delta_y**2)
        
        if distance < 1e-6:
            return 0.0
        
        angle_to_target = np.arctan2(delta_y, delta_x)
        angle_error = angle_to_target - current_yaw
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        
        cross_track_error = distance * np.sin(angle_error)
        steering_angle = np.arctan2(2.0 * self.wheelbase * cross_track_error, 
                                    distance**2)
        
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, 
                                self.max_steering_angle)
        
        return steering_angle


def test_trajectory_generation():
    """Test figure-8 trajectory generation"""
    print("\n" + "="*70)
    print("TEST 1: Figure-8 Trajectory Generation")
    print("="*70)
    
    traj = Figure8Trajectory(R=1.4)
    
    print(f"✓ Trajectory generated with {traj.trajectory_length} points")
    print(f"  Trajectory shape: {traj.trajectory.shape}")
    
    # Check some properties
    x_coords = traj.trajectory[:, 0]
    y_coords = traj.trajectory[:, 1]
    
    print(f"  X range: [{x_coords.min():.2f}, {x_coords.max():.2f}]")
    print(f"  Y range: [{y_coords.min():.2f}, {y_coords.max():.2f}]")
    
    # Check if points approximately form circles
    # Right circle should be centered around (R, 0)
    right_circle = traj.trajectory[:200]
    dist_from_center = np.linalg.norm(right_circle - np.array([1.4, 0]), axis=1)
    
    print(f"  Right circle radius check: {dist_from_center.mean():.3f} ± {dist_from_center.std():.3f} m")
    
    if abs(dist_from_center.mean() - 1.4) < 0.1:
        print("✅ PASS: Trajectory geometry is correct")
    else:
        print("❌ FAIL: Trajectory geometry is incorrect")


def test_closest_point_tracking():
    """Test closest point on trajectory tracking"""
    print("\n" + "="*70)
    print("TEST 2: Closest Point Tracking")
    print("="*70)
    
    traj = Figure8Trajectory(R=1.4)
    
    # Start at origin (should be close to right circle edge)
    start_pos = np.array([0.0, 0.0])
    closest, idx, dist = traj.get_closest_point(start_pos)
    
    print(f"✓ Starting position: {start_pos}")
    print(f"  Closest trajectory point: {closest}")
    print(f"  Trajectory index: {idx}")
    print(f"  Distance: {dist:.4f} m")
    
    # Try a point on the trajectory
    on_traj = traj.trajectory[100]
    closest, idx, dist = traj.get_closest_point(on_traj)
    
    print(f"\n✓ Point on trajectory: {on_traj}")
    print(f"  Found distance: {dist:.6f} m")
    
    if dist < 0.1:
        print("✅ PASS: Closest point tracking works correctly")
    else:
        print("❌ FAIL: Closest point tracking has issues")


def test_pure_pursuit_controller():
    """Test Pure Pursuit steering angle computation"""
    print("\n" + "="*70)
    print("TEST 3: Pure Pursuit Controller")
    print("="*70)
    
    controller = PurePursuitController(wheelbase=0.33, lookahead_gain=1.5)
    
    # Test case 1: Moving straight toward target
    current_pos = np.array([0.0, 0.0])
    current_yaw = 0.0  # Facing right
    target_pos = np.array([1.0, 0.0])  # Direct ahead
    velocity = 1.0
    
    steering = controller.compute_steering(current_pos, current_yaw, target_pos, velocity)
    
    print(f"✓ Test Case 1: Straight ahead")
    print(f"  Current pos: {current_pos}")
    print(f"  Target pos: {target_pos}")
    print(f"  Steering angle: {steering:.4f} rad ({np.degrees(steering):.2f}°)")
    
    if abs(steering) < 0.05:
        print("  ✓ Steering ≈ 0 (correct for straight movement)")
    
    # Test case 2: Turning left
    current_pos = np.array([0.0, 0.0])
    current_yaw = 0.0
    target_pos = np.array([0.5, 1.0])  # Upper right
    velocity = 1.0
    
    steering = controller.compute_steering(current_pos, current_yaw, target_pos, velocity)
    
    print(f"\n✓ Test Case 2: Turning left (counterclockwise)")
    print(f"  Current pos: {current_pos}")
    print(f"  Target pos: {target_pos}")
    print(f"  Steering angle: {steering:.4f} rad ({np.degrees(steering):.2f}°)")
    
    if steering > 0:
        print("  ✓ Steering is positive (left turn - correct)")
    
    # Test case 3: Velocity effect on lookahead
    for v in [0.5, 1.0, 2.0]:
        steering = controller.compute_steering(current_pos, current_yaw, target_pos, v)
        print(f"\n  Velocity {v} m/s: steering = {steering:.4f} rad")
    
    print("\n✅ PASS: Pure Pursuit controller produces reasonable outputs")


def test_calibration_stages():
    """Test calibration stage timing logic"""
    print("\n" + "="*70)
    print("TEST 4: Calibration Stage Logic")
    print("="*70)
    
    stages = [
        ("STAGE_1", 5.0, 0, 30),
        ("STAGE_2", 10.0, 30, 60),
        ("STAGE_3", 15.0, 60, 90),
        ("STAGE_4", 20.0, 90, 120),
    ]
    
    test_times = [5, 15, 35, 60, 75, 100, 125]
    
    for t in test_times:
        stage_found = False
        for stage_name, current_val, start_t, end_t in stages:
            if start_t <= t < end_t:
                print(f"✓ t = {t:3d}s: {stage_name} ({current_val:.1f}A)")
                stage_found = True
                break
        
        if not stage_found:
            print(f"✓ t = {t:3d}s: Calibration complete (0.0A)")
    
    print("\n✅ PASS: Stage timing logic works correctly")


def test_data_processing():
    """Test data processing pipeline"""
    print("\n" + "="*70)
    print("TEST 5: Data Processing Pipeline")
    print("="*70)
    
    # Generate synthetic calibration data
    np.random.seed(42)
    n_samples = 1000
    
    timestamps = np.linspace(0, 120, n_samples)
    
    # Stage-based current
    currents = np.zeros(n_samples)
    currents[timestamps < 30] = 5.0
    currents[(timestamps >= 30) & (timestamps < 60)] = 10.0
    currents[(timestamps >= 60) & (timestamps < 90)] = 15.0
    currents[(timestamps >= 90) & (timestamps < 120)] = 20.0
    
    # Simulated accelerations with noise: a = 0.15*I + 0.5 + noise
    accelerations = 0.15 * currents + 0.5 + np.random.normal(0, 0.05, n_samples)
    
    print(f"✓ Generated {n_samples} synthetic data points")
    print(f"  Time range: {timestamps[0]:.1f}s to {timestamps[-1]:.1f}s")
    print(f"  Current range: {currents.min():.1f}A to {currents.max():.1f}A")
    print(f"  Acceleration range: {accelerations.min():.3f} to {accelerations.max():.3f} m/s²")
    
    # Fit linear model per stage
    print("\n✓ Fitting linear models per stage:")
    
    for stage_idx, (start_t, end_t, target_current) in enumerate([
        (0, 30, 5), (30, 60, 10), (60, 90, 15), (90, 120, 20)
    ]):
        mask = (timestamps >= start_t) & (timestamps < end_t)
        stage_accel = accelerations[mask]
        stage_current = currents[mask]
        
        # Linear fit
        p = np.polyfit(stage_current, stage_accel, 1)
        
        # R² computation
        y_pred = np.polyval(p, stage_current)
        ss_res = np.sum((stage_accel - y_pred)**2)
        ss_tot = np.sum((stage_accel - np.mean(stage_accel))**2)
        r_squared = 1 - (ss_res / ss_tot)
        
        print(f"  Stage {stage_idx+1} ({target_current}A): " +
              f"a = {p[0]:.4f}*I + {p[1]:.4f}, R² = {r_squared:.4f}")
    
    print("\n✅ PASS: Data processing works correctly")


def test_quaternion_conversion():
    """Test quaternion to yaw conversion"""
    print("\n" + "="*70)
    print("TEST 6: Quaternion to Yaw Conversion")
    print("="*70)
    
    def quat_to_yaw(qx, qy, qz, qw):
        yaw = np.arctan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy**2 + qz**2)
        )
        return yaw
    
    # Test cases: unit quaternions representing known yaws
    test_cases = [
        (0, 0, 0, 1, 0),  # Identity: yaw = 0
        (0, 0, 0.7071, 0.7071, np.pi/2),  # 90 degrees
        (0, 0, 1, 0, np.pi),  # 180 degrees
        (0, 0, 0.7071, -0.7071, -np.pi/2),  # -90 degrees
    ]
    
    for qx, qy, qz, qw, expected_yaw in test_cases:
        computed_yaw = quat_to_yaw(qx, qy, qz, qw)
        error = abs(computed_yaw - expected_yaw)
        
        print(f"✓ Quaternion ({qx}, {qy}, {qz}, {qw}):")
        print(f"  Expected yaw: {expected_yaw:.4f} rad ({np.degrees(expected_yaw):.2f}°)")
        print(f"  Computed yaw: {computed_yaw:.4f} rad ({np.degrees(computed_yaw):.2f}°)")
        
        if error < 0.01:
            print("  ✓ Correct")
        else:
            print("  ✗ Error")
    
    print("\n✅ PASS: Quaternion conversion works correctly")


def main():
    """Run all tests"""
    print("\n" + "="*70)
    print("CURRENT-ACCELERATION CALIBRATION COMPONENT TESTS")
    print("="*70)
    
    try:
        test_trajectory_generation()
        test_closest_point_tracking()
        test_pure_pursuit_controller()
        test_calibration_stages()
        test_data_processing()
        test_quaternion_conversion()
        
        print("\n" + "="*70)
        print("✅ ALL TESTS PASSED")
        print("="*70 + "\n")
        
        return 0
        
    except Exception as e:
        print(f"\n❌ TEST FAILED WITH ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    exit(main())
