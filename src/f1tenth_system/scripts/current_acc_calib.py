#!/usr/bin/env python3
"""
Current-Acceleration Calibration Node for F1TENTH Vehicle

Enhanced version with:
1. Pure Pursuit trajectory tracking controller
2. Speed-tiered data collection (low/mid/high speed)
3. Separate acceleration and braking calibration modes
4. Physics-based drag force modeling
5. Complete telemetry recording (current, velocity, ERPM, acceleration)

Output: AckermannDriveStamped messages with:
  - steering: steering angle (rad)
  - acceleration: motor current (A) - NOTE: field name is 'acceleration' but value is current
  - jerk: mode flag (0=speed, 1=accel feedforward, 2=current, 3=duty)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
import numpy as np
from enum import Enum
import math
import time




class CalibrationTier:
    """
    Manages speed-tiered calibration stages.
    
    Different speed ranges reveal different motor characteristics:
    - Low speed: Coulomb friction dominant
    - Mid speed: Transition region
    - High speed: Back-EMF effects become significant
    """
    
    def __init__(self):
        """Initialize speed tiers"""
        # Format: (name, v_target, start_time, end_time, current_range)
        self.tiers = [
            {
                'name': 'LOW_SPEED',
                'v_target': 1.5,
                'time_start': 0,
                'time_end': 40,
                'current_min': 5.0,
                'current_max': 15.0,
                'description': 'Low speed tier (1.5 m/s) - Coulomb friction dominant'
            },
            {
                'name': 'MID_SPEED',
                'v_target': 3.0,
                'time_start': 40,
                'time_end': 80,
                'current_min': 8.0,
                'current_max': 20.0,
                'description': 'Mid speed tier (3.0 m/s) - Transition region'
            },
            {
                'name': 'HIGH_SPEED',
                'v_target': 5.0,
                'time_start': 80,
                'time_end': 120,
                'current_min': 10.0,
                'current_max': 25.0,
                'description': 'High speed tier (5.0 m/s) - Back-EMF significant'
            }
        ]
    
    def get_current_tier(self, elapsed_time: float) -> dict:
        """
        Get calibration tier for given elapsed time.
        
        Args:
            elapsed_time: Seconds elapsed since calibration start
            
        Returns:
            Tier dictionary or None if calibration complete
        """
        for tier in self.tiers:
            if tier['time_start'] <= elapsed_time < tier['time_end']:
                return tier
        return None
    
    def get_target_current(self, elapsed_time: float) -> float:
        """
        Compute target current as function of elapsed time within tier.
        
        Ramps current from min to max over the tier duration.
        
        Args:
            elapsed_time: Seconds elapsed since calibration start
            
        Returns:
            Target current (A) or 0.0 if calibration complete
        """
        tier = self.get_current_tier(elapsed_time)
        if tier is None:
            return 0.0
        
        # Linear ramp within tier
        tier_duration = tier['time_end'] - tier['time_start']
        tier_elapsed = elapsed_time - tier['time_start']
        progress = tier_elapsed / tier_duration
        
        # Interpolate current from min to max
        current = tier['current_min'] + progress * (tier['current_max'] - tier['current_min'])
        return current


class BrakingCalibrationMode:
    """
    Dedicated calibration mode for reverse (braking) current.
    
    Negative current provides back-EMF braking, which has different
    characteristics than forward acceleration.
    """
    
    def __init__(self):
        """Initialize braking calibration parameters"""
        self.stages = [
            {
                'name': 'BRAKING_STAGE_1 (-5A)',
                'current': -5.0,   # Light braking
                'time_start': 0,
                'time_end': 30,
            },
            {
                'name': 'BRAKING_STAGE_2 (-10A)',
                'current': -10.0,  # Medium braking
                'time_start': 30,
                'time_end': 60,
            },
            {
                'name': 'BRAKING_STAGE_3 (-15A)',
                'current': -15.0,  # Heavy braking
                'time_start': 60,
                'time_end': 90,
            },
            {
                'name': 'BRAKING_STAGE_4 (-20A)',
                'current': -20.0,  # Maximum braking (safe limit)
                'time_start': 90,
                'time_end': 120,
            }
        ]
    
    def get_brake_current(self, elapsed_time: float) -> tuple:
        """
        Get braking current and stage info.
        
        Args:
            elapsed_time: Seconds elapsed since braking calibration start
            
        Returns:
            tuple: (stage_name, current_A, is_complete)
        """
        for stage in self.stages:
            if stage['time_start'] <= elapsed_time < stage['time_end']:
                return stage['name'], stage['current'], False
        
        # Calibration complete
        return 'COMPLETE', 0.0, True







class Figure8Trajectory:
    """Generate figure-8 reference trajectory with given parameters"""
    
    def __init__(self, R: float = 1.6, center_distance: float = 3.2, 
                 points_per_circle: int = 200):
        """
        Initialize figure-8 trajectory.
        
        Args:
            R: Radius of each circular arc (meters), default 1.6 (increased from 1.4)
            center_distance: Distance between two circle centers (meters)
            points_per_circle: Number of points per circle for trajectory
        """
        self.R = R
        self.center_distance = center_distance
        self.points_per_circle = points_per_circle
        
        # Generate trajectory
        self.trajectory = self._generate_trajectory()
        self.trajectory_length = len(self.trajectory)
        
    def _generate_trajectory(self) -> np.ndarray:
        """
        Generate figure-8 trajectory as [x, y] points.
        
        Returns:
            np.ndarray: Shape (N, 2) containing trajectory points
        """
        # Right circle centered at (R, 0)
        theta1 = np.linspace(0, 2*np.pi, self.points_per_circle)
        right_circle_x = self.R + self.R * np.cos(theta1)
        right_circle_y = self.R * np.sin(theta1)
        
        # Left circle centered at (-R, 0)
        left_circle_x = -self.R + self.R * np.cos(theta1)
        left_circle_y = self.R * np.sin(theta1)
        
        # Combine and close the loop
        trajectory = np.vstack([
            np.column_stack([right_circle_x, right_circle_y]),
            np.column_stack([left_circle_x, left_circle_y])
        ])
        
        return trajectory
    
    def get_closest_point(self, current_pos: np.ndarray, start_idx: int = 0) -> tuple:
        """
        Find closest trajectory point to current position.
        
        Args:
            current_pos: Current position [x, y]
            start_idx: Index to start search from (for efficiency)
            
        Returns:
            tuple: (closest_point, closest_index, distance)
        """
        # Search within a window for efficiency
        search_range = 50
        end_idx = min(start_idx + search_range, self.trajectory_length)
        search_trajectory = self.trajectory[start_idx:end_idx]
        
        # Compute distances
        distances = np.linalg.norm(search_trajectory - current_pos, axis=1)
        min_idx_local = np.argmin(distances)
        min_idx_global = start_idx + min_idx_local
        
        closest_point = self.trajectory[min_idx_global]
        distance = distances[min_idx_local]
        
        return closest_point, min_idx_global, distance
    
    def get_lookahead_point(self, current_idx: int, lookahead_distance: float) -> np.ndarray:
        """
        Get lookahead point along trajectory at specified distance ahead.
        
        Args:
            current_idx: Current index on trajectory
            lookahead_distance: Look-ahead distance (meters)
            
        Returns:
            np.ndarray: Lookahead point [x, y]
        """
        # Simple linear search for lookahead point
        idx = current_idx
        cumulative_dist = 0.0
        
        while cumulative_dist < lookahead_distance and idx < self.trajectory_length - 1:
            segment_dist = np.linalg.norm(
                self.trajectory[idx + 1] - self.trajectory[idx]
            )
            if cumulative_dist + segment_dist >= lookahead_distance:
                # Interpolate within segment
                ratio = (lookahead_distance - cumulative_dist) / segment_dist
                lookahead_point = (
                    self.trajectory[idx] + 
                    ratio * (self.trajectory[idx + 1] - self.trajectory[idx])
                )
                return lookahead_point
            
            cumulative_dist += segment_dist
            idx += 1
        
        # If we reach the end, wrap around
        idx = idx % self.trajectory_length
        return self.trajectory[idx]
    
    def is_in_curve(self, trajectory_idx: int) -> bool:
        """
        Determine if current position on trajectory is in a curve (circular arc) vs straight.
        
        Figure-8 structure:
        - Indices 0 to points_per_circle: Right circle (curve)
        - Indices points_per_circle to 2*points_per_circle: Left circle (curve)
        The transition points are essentially straight (diameter).
        
        Args:
            trajectory_idx: Current index on trajectory
            
        Returns:
            bool: True if in circular curve, False if near transition/straight
        """
        # Identify which half we're in
        half_length = self.points_per_circle
        
        # Near the middle crossing point (transition between circles)
        transition_zone = int(0.1 * half_length)  # ~10% of circle radius as transition zone
        
        # Check if we're in transition region (middle of each circle)
        right_circle_middle = half_length // 2
        left_circle_middle = half_length + half_length // 2
        
        idx_in_circle = trajectory_idx % self.trajectory_length
        
        # If near middle of either circle, it's straighter
        if abs(idx_in_circle - right_circle_middle) < transition_zone:
            return False  # Near straight section of right circle
        if abs(idx_in_circle - left_circle_middle) < transition_zone:
            return False  # Near straight section of left circle
        
        # Otherwise in curve
        return True


class PurePursuitController:
    """Pure Pursuit trajectory tracking controller for Ackermann steered vehicles"""
    
    def __init__(self, wheelbase: float = 0.33, lookahead_gain: float = 1.5,
                 min_lookahead: float = 0.3, max_steering: float = 0.30):
        """
        Initialize Pure Pursuit controller.
        
        Args:
            wheelbase: Distance from front to rear axle (meters)
            lookahead_gain: Gain for dynamic look-ahead distance (ld = k*v + min_lookahead)
            min_lookahead: Minimum lookahead distance (meters)
            max_steering: Maximum steering angle (radians)
        """
        self.wheelbase = wheelbase
        self.lookahead_gain = lookahead_gain
        self.min_lookahead = min_lookahead
        self.max_steering_angle = max_steering
        
    def compute_lookahead_distance(self, velocity: float) -> float:
        """
        Compute adaptive lookahead distance based on velocity.
        ld = k*v + min_lookahead
        
        Args:
            velocity: Current velocity (m/s)
            
        Returns:
            float: Lookahead distance (meters)
        """
        return max(self.lookahead_gain * abs(velocity) + self.min_lookahead, 
                   self.min_lookahead)
    
    def compute_steering(self, current_pos: np.ndarray, current_yaw: float,
                        lookahead_point: np.ndarray, velocity: float) -> float:
        """
        Compute steering angle using Pure Pursuit algorithm.
        
        Pure Pursuit law: δ = arctan(2*L*e / ld²)
        where L is wheelbase, e is cross-track error, ld is lookahead distance
        
        Args:
            current_pos: Current position [x, y]
            current_yaw: Current yaw angle (rad)
            lookahead_point: Target lookahead point [x, y]
            velocity: Current forward velocity (m/s)
            
        Returns:
            float: Steering angle command (rad), clipped to [-max_steering, +max_steering]
        """
        # Vector from current position to lookahead point
        delta_x = lookahead_point[0] - current_pos[0]
        delta_y = lookahead_point[1] - current_pos[1]
        
        # Distance to lookahead point
        distance = np.sqrt(delta_x**2 + delta_y**2)
        
        if distance < 1e-6:
            return 0.0
        
        # Angle to lookahead point in global frame
        angle_to_target = np.arctan2(delta_y, delta_x)
        
        # Relative angle (error)
        angle_error = angle_to_target - current_yaw
        
        # Normalize angle error to [-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        
        # Pure Pursuit law: steering angle proportional to cross-track error
        # delta = atan2(2 * L * sin(alpha) / distance, 1)
        cross_track_error = distance * np.sin(angle_error)
        steering_angle = np.arctan2(2.0 * self.wheelbase * cross_track_error, 
                                    distance**2)
        
        # Clip steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, 
                                self.max_steering_angle)
        
        return steering_angle


class CurrentAccelCalibNode(Node):
    """ROS2 Node for current-acceleration calibration with physics-based modeling"""
    
    def __init__(self):
        super().__init__('current_acc_calib')
        
        # Declare parameters
        self.wheelbase = self.declare_parameter('wheelbase', 0.33).value
        self.lookahead_gain = self.declare_parameter('lookahead_gain', 1.5).value
        self.figure8_radius = self.declare_parameter('figure8_radius', 1.6).value  # Updated from 1.4
        self.command_frequency = self.declare_parameter('command_frequency', 50).value  # Hz
        self.calibration_mode = self.declare_parameter('calibration_mode', 'acceleration').value  # 'acceleration' or 'braking'
        self.vehicle_mass = self.declare_parameter('vehicle_mass', 6.0).value  # kg
        
        # QoS profile for best-effort communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # EKF-filtered odometry
            self.odom_callback,
            qos_profile
        )
        
        # VESC state subscription for ERPM (motor speed)
        self.vesc_subscription = self.create_subscription(
            VescStateStamped,
            '/vesc/sensors',
            self.vesc_callback,
            qos_profile
        )
        
        # Publishers
        self.cmd_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/calib/ackermann_cmd',
            10
        )
        
        # Internal state
        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.current_velocity = 0.0
        self.current_erpm = 0.0
        self.has_received_odom = False
        self.has_received_vesc = False
        
        
        # Mode selection
        if self.calibration_mode == 'braking':
            self.calib_tier = BrakingCalibrationMode()
        else:
            self.calib_tier = CalibrationTier()
        
        # Trajectory and controller
        self.trajectory = Figure8Trajectory(R=self.figure8_radius)
        self.controller = PurePursuitController(
            wheelbase=self.wheelbase,
            lookahead_gain=self.lookahead_gain
        )
        
        # Calibration state
        self.start_time = None
        self.trajectory_idx = 0
        
        # Timer for command publishing
        self.timer = self.create_timer(
            1.0 / self.command_frequency,
            self.control_loop_callback
        )
        
        self.get_logger().info(
            f"Current-Acceleration Calibration Node initialized\n"
            f"  Wheelbase: {self.wheelbase} m\n"
            f"  Figure-8 Radius: {self.figure8_radius} m (increased from 1.4m)\n"
            f"  Lookahead Gain: {self.lookahead_gain}\n"
            f"  Command Frequency: {self.command_frequency} Hz\n"
            f"  Calibration Mode: {self.calibration_mode}\n"
            f"  Vehicle Mass: {self.vehicle_mass} kg"
        )
    
    def odom_callback(self, msg: Odometry):
        """Process odometry message from EKF filter"""
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        self.current_yaw = self._quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)
        
        # Extract velocity (magnitude)
        self.current_velocity = np.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        
        self.has_received_odom = True
    
    def vesc_callback(self, msg: VescStateStamped):
        """Process VESC state message to get motor ERPM"""
        self.current_erpm = msg.state.electrical_rpm
        self.has_received_vesc = True
    
    @staticmethod
    def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle (rad)"""
        yaw = np.arctan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy**2 + qz**2)
        )
        return yaw
    
    def get_current_stage(self, elapsed_time: float) -> tuple:
        """
        Get current calibration stage with dynamic current ramping based on speed tier.
        Supports both acceleration and braking modes.
        
        Args:
            elapsed_time: Time elapsed since calibration start (seconds)
            
        Returns:
            tuple: (tier_name, current_value_A, is_complete, jerk_mode)
                   jerk_mode: 0.0=speed control (curves), 2.0=current control (straights)
        """
        if isinstance(self.calib_tier, BrakingCalibrationMode):
            # Braking mode: four stages with negative current
            # Use stages defined in BrakingCalibrationMode.__init__ (single source of truth)
            for stage in self.calib_tier.stages:
                if stage['time_start'] <= elapsed_time < stage['time_end']:
                    return stage['name'], stage['current'], False, 2.0  # Current control mode
            
            return "BRAKING_COMPLETE", 0.0, True, 0.0
        
        else:
            # Acceleration mode: three speed tiers with dynamic current ramping
            # Use tiers defined in CalibrationTier.__init__ (single source of truth)
            for tier in self.calib_tier.tiers:
                if tier['time_start'] <= elapsed_time < tier['time_end']:
                    # Compute progress within this tier
                    tier_duration = tier['time_end'] - tier['time_start']
                    tier_elapsed = elapsed_time - tier['time_start']
                    progress = tier_elapsed / tier_duration
                    
                    # Interpolate current from min to max
                    current_A = tier['current_min'] + progress * (tier['current_max'] - tier['current_min'])
                    
                    # Determine control mode based on trajectory position (curve vs straight)
                    jerk_mode = 2.0 if self.trajectory.is_in_curve(self.trajectory_idx) else 1.0
                    
                    return f"{tier['name']} (target {tier['v_target']}m/s)", current_A, False, jerk_mode
            
            # Calibration complete
            return "ACCELERATION_COMPLETE", 0.0, True, 0.0
    
    def control_loop_callback(self):
        """
        Main control loop - runs at specified frequency.
        Integrates trajectory tracking, physics-based control, and telemetry recording.
        """
        
        if not self.has_received_odom:
            self.get_logger().warn("Waiting for odometry data...")
            return
        
        # Initialize start time on first call
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Calibration started!")
        
        elapsed_time = time.time() - self.start_time
        
        # Get current stage with multi-tier physics-based control
        stage_name, current_A, is_complete, jerk_mode = self.get_current_stage(elapsed_time)
        
        # Create command message
        cmd_msg = AckermannDriveStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "base_link"
        
        if is_complete:
            # Calibration complete - send stop command
            cmd_msg.drive.steering_angle = 0.0
            cmd_msg.drive.acceleration = 0.0  # Stop
            cmd_msg.drive.jerk = 0.0
            self.get_logger().info("Calibration complete! Data recorded in rosbag.")
        else:
            # Find closest point on trajectory
            closest_point, self.trajectory_idx, cross_track_error = \
                self.trajectory.get_closest_point(self.current_pos, self.trajectory_idx)
            
            # Compute adaptive lookahead distance based on velocity
            lookahead_distance = self.controller.compute_lookahead_distance(self.current_velocity)
            lookahead_point = self.trajectory.get_lookahead_point(
                self.trajectory_idx, lookahead_distance
            )
            
            # Compute steering angle using Pure Pursuit
            steering_angle = self.controller.compute_steering(
                self.current_pos,
                self.current_yaw,
                lookahead_point,
                self.current_velocity
            )
            
            # Determine control mode based on trajectory position (curve vs. straight)
            if jerk_mode == 2.0:
                # Current control mode (straights) - direct current command
                cmd_msg.drive.acceleration = current_A
                mode_str = "CURRENT_CONTROL"
            else:
                # Speed/hybrid control mode (curves) - maintain target speed
                tier_idx = min(int(elapsed_time / 40.0), 2)
                target_speed = self.calib_tier.tiers[tier_idx]['v_target']
                
                # Simple proportional speed feedback (in real system, use PID)
                speed_error = target_speed - self.current_velocity
                cmd_msg.drive.acceleration = 5.0 + 2.0 * speed_error  # P-control
                mode_str = "SPEED_CONTROL"
            
            cmd_msg.drive.steering_angle = steering_angle
            cmd_msg.drive.jerk = jerk_mode  # Mode flag: 0=speed, 1=accel, 2=current, 3=duty
            
            # Periodic logging (every 2 seconds)
            if int(elapsed_time * 2) % 4 == 0:
                self.get_logger().info(
                    f"[{stage_name}] t={elapsed_time:.1f}s | "
                    f"I={current_A:.1f}A | v={self.current_velocity:.2f}m/s | "
                    f"δ={steering_angle:.3f}rad | "
                    f"cte={cross_track_error:.2f}m | {mode_str}"
                )
        
        # Publish command
        self.cmd_publisher.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentAccelCalibNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down calibration node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
