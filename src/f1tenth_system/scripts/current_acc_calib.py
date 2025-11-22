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


def _angle_wrap(angle: float) -> float:
    """Wrap any angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))



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
                'current_min': 0.0,
                'current_max': 150.0,
                'description': 'Low speed tier (1.5 m/s) - Coulomb friction dominant'
            },
            {
                'name': 'MID_SPEED',
                'v_target': 3.0,
                'time_start': 40,
                'time_end': 80,
                'current_min': 0.0,
                'current_max': 150.0,
                'description': 'Mid speed tier (3.0 m/s) - Transition region'
            },
            {
                'name': 'HIGH_SPEED',
                'v_target': 5.0,
                'time_start': 80,
                'time_end': 120,
                'current_min': 0.0,
                'current_max': 150.0,
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
    """Generate racetrack (stadium) trajectory: straight lines + semicircular turns"""
    
    def __init__(self, R: float = 2.0, straight_length: float = 6.0, 
                 points_per_straight: int = 150, points_per_semicircle: int = 100):
        """
        Initialize racetrack trajectory (like a school stadium).
        
        Args:
            R: Radius of semicircular turns (meters), default 2.0 (larger for smoother turns)
            straight_length: Length of each straight section (meters), default 6.0
            points_per_straight: Number of points per straight section
            points_per_semicircle: Number of points per semicircular turn
        """
        self.R = R
        self.straight_length = straight_length
        self.points_per_straight = points_per_straight
        self.points_per_semicircle = points_per_semicircle
        
        # Generate trajectory
        self.trajectory = self._generate_trajectory()
        self.trajectory_length = len(self.trajectory)
        
    def _generate_trajectory(self) -> np.ndarray:
        """
        Generate racetrack trajectory: straight-line sections + semicircular turns.
        
        Layout (like a stadium):
        - Bottom straight: accelerate from left to right (current calibration)
        - Right semicircle: decelerate and turn around (speed control)
        - Top straight: accelerate from right to left
        - Left semicircle: decelerate and turn around (speed control)
        
        Returns:
            np.ndarray: Shape (N, 2) containing trajectory points
        """
        # Bottom straight line: from (-straight_length/2, 0) to (straight_length/2, 0)
        bottom_x = np.linspace(-self.straight_length/2, self.straight_length/2, 
                              self.points_per_straight, endpoint=False)
        bottom_y = np.zeros(self.points_per_straight)
        
        # Right semicircle: clockwise turn from bottom to top
        # Center at (straight_length/2, R), from 270° to 90°
        theta_right = np.linspace(-np.pi/2, np.pi/2, self.points_per_semicircle, endpoint=False)
        right_x = self.straight_length/2 + self.R * np.cos(theta_right)
        right_y = self.R + self.R * np.sin(theta_right)
        
        # Top straight line: from (straight_length/2, 2*R) to (-straight_length/2, 2*R)
        top_x = np.linspace(self.straight_length/2, -self.straight_length/2, 
                           self.points_per_straight, endpoint=False)
        top_y = np.full(self.points_per_straight, 2 * self.R)
        
        # Left semicircle: clockwise turn from top to bottom
        # Center at (-straight_length/2, R), from 90° to 270°
        theta_left = np.linspace(np.pi/2, 3*np.pi/2, self.points_per_semicircle, endpoint=False)
        left_x = -self.straight_length/2 + self.R * np.cos(theta_left)
        left_y = self.R + self.R * np.sin(theta_left)
        
        # Combine all sections into closed loop
        trajectory = np.column_stack([
            np.concatenate([bottom_x, right_x, top_x, left_x]),
            np.concatenate([bottom_y, right_y, top_y, left_y])
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
    
    def get_lookahead_point(self, current_idx: int, lookahead_distance: float) -> tuple:
        """
        Get lookahead point along trajectory at specified distance ahead.
        
        Args:
            current_idx: Current index on trajectory
            lookahead_distance: Look-ahead distance (meters)
            
        Returns:
            tuple: (lookahead_point, lookahead_idx)
                lookahead_point: np.ndarray [x, y]
                lookahead_idx: int, index of lookahead point
        """
        # Simple linear search for lookahead point
        idx = current_idx % self.trajectory_length
        cumulative_dist = 0.0
        
        while cumulative_dist < lookahead_distance:
            next_idx = (idx + 1) % self.trajectory_length
            segment_dist = np.linalg.norm(
                self.trajectory[next_idx] - self.trajectory[idx]
            )
            if cumulative_dist + segment_dist >= lookahead_distance:
                # Interpolate within segment
                ratio = (lookahead_distance - cumulative_dist) / max(segment_dist, 1e-6)
                lookahead_point = (
                    self.trajectory[idx] + 
                    ratio * (self.trajectory[next_idx] - self.trajectory[idx])
                )
                return lookahead_point, next_idx
            
            cumulative_dist += segment_dist
            idx = next_idx
        
        # If we reach the end, return current point
        return self.trajectory[idx], idx
    
    def get_heading(self, idx: int) -> float:
        """
        Get trajectory heading (tangent direction) at specified index.
        
        Args:
            idx: Index on trajectory
            
        Returns:
            float: Heading angle (radians) in global frame
        """
        prev_pt = self.trajectory[(idx - 1) % self.trajectory_length]
        next_pt = self.trajectory[(idx + 1) % self.trajectory_length]
        return math.atan2(next_pt[1] - prev_pt[1], next_pt[0] - prev_pt[0])
    
    def get_curvature(self, idx: int) -> float:
        """
        Estimate path curvature at specified index using Menger curvature.
        
        Menger curvature: κ = 4*Area / (a*b*c)
        where a, b, c are side lengths of triangle formed by 3 consecutive points.
        
        Args:
            idx: Index on trajectory
            
        Returns:
            float: Curvature (1/m), positive for left turn, negative for right turn
        """
        p_prev = self.trajectory[(idx - 1) % self.trajectory_length]
        p_curr = self.trajectory[idx % self.trajectory_length]
        p_next = self.trajectory[(idx + 1) % self.trajectory_length]
        
        # Triangle side lengths
        a = np.linalg.norm(p_curr - p_prev)
        b = np.linalg.norm(p_next - p_curr)
        c = np.linalg.norm(p_next - p_prev)
        
        denom = max(a * b * c, 1e-6)
        if denom < 1e-6:
            return 0.0
        
        # Signed area using cross product
        area = (
            p_prev[0] * (p_curr[1] - p_next[1]) +
            p_curr[0] * (p_next[1] - p_prev[1]) +
            p_next[0] * (p_prev[1] - p_curr[1])
        ) / 2.0
        
        return 4.0 * area / denom
    
    def is_in_curve(self, trajectory_idx: int) -> bool:
        """
        Determine if current position is in a curve (semicircle turn) vs straight section.
        
        Racetrack structure:
        - Segment 0: Bottom straight (indices 0 to points_per_straight) - STRAIGHT
        - Segment 1: Right semicircle (points_per_straight to points_per_straight + points_per_semicircle) - CURVE
        - Segment 2: Top straight (continuing from segment 1) - STRAIGHT
        - Segment 3: Left semicircle (remaining indices) - CURVE
        
        Args:
            trajectory_idx: Current index on trajectory
            
        Returns:
            bool: True if in semicircular turn (use speed control), False if in straight (use current control)
        """
        idx = trajectory_idx % self.trajectory_length
        
        # Segment boundaries
        bottom_end = self.points_per_straight
        right_end = bottom_end + self.points_per_semicircle
        top_end = right_end + self.points_per_straight
        # left semicircle goes from top_end to trajectory_length
        
        # Check which segment we're in
        if idx < bottom_end:
            return False  # Bottom straight - use current control for acceleration
        elif idx < right_end:
            return True   # Right semicircle - use speed control for deceleration/turn
        elif idx < top_end:
            return False  # Top straight - use current control for acceleration
        else:
            return True   # Left semicircle - use speed control for deceleration/turn


class PurePursuitController:
    """
    Enhanced Pure Pursuit trajectory tracking controller with:
    1. Lateral error compensation (standard Pure Pursuit)
    2. Heading error compensation (reduces oscillation)
    3. Curvature feed-forward (improves cornering)
    """
    
    def __init__(self, wheelbase: float = 0.33, lookahead_gain: float = 1.6,
                 min_lookahead: float = 0.3, max_lookahead: float = 3.5,
                 lateral_error_gain: float = 1.0,
                 heading_error_gain: float = 0.4,
                 curvature_ff_gain: float = 0.1,
                 max_steering: float = 0.35):
        """
        Initialize Enhanced Pure Pursuit controller.
        
        Args:
            wheelbase: Distance from front to rear axle (meters)
            lookahead_gain: Gain for dynamic look-ahead distance (ld = k*v + min_lookahead)
            min_lookahead: Minimum lookahead distance (meters)
            max_lookahead: Maximum lookahead distance (meters)
            lateral_error_gain: Gain for lateral error term (1.0 = standard PP)
            heading_error_gain: Gain for heading error compensation (0.0 = disabled)
            curvature_ff_gain: Gain for curvature feed-forward (0.0 = disabled)
            max_steering: Maximum steering angle (radians)
        """
        self.wheelbase = wheelbase
        self.lookahead_gain = lookahead_gain
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lateral_error_gain = lateral_error_gain
        self.heading_error_gain = heading_error_gain
        self.curvature_ff_gain = curvature_ff_gain
        self.max_steering_angle = max_steering
        
    def compute_lookahead(self, velocity: float) -> float:
        """
        Compute adaptive lookahead distance based on velocity.
        ld = k*v + min_lookahead, clipped to [min_lookahead, max_lookahead]
        
        Args:
            velocity: Current velocity (m/s)
            
        Returns:
            float: Lookahead distance (meters)
        """
        ld = self.lookahead_gain * abs(velocity) + self.min_lookahead
        return float(np.clip(ld, self.min_lookahead, self.max_lookahead))
    
    def compute_steering(self, current_pos: np.ndarray, current_yaw: float,
                        lookahead_point: np.ndarray, lookahead_heading: float,
                        path_curvature: float, velocity: float) -> tuple:
        """
        Compute steering angle using Enhanced Pure Pursuit algorithm.
        
        Three-term control law:
        1. Lateral term: δ_lat = arctan(L * 2*y_ld / ld²) * lateral_gain
        2. Heading term: δ_head = heading_gain * (ψ_path - ψ_vehicle)
        3. Curvature FF: δ_ff = curvature_gain * κ_path
        
        Args:
            current_pos: Current position [x, y]
            current_yaw: Current yaw angle (rad)
            lookahead_point: Target lookahead point [x, y]
            lookahead_heading: Path heading at lookahead point (rad)
            path_curvature: Path curvature at lookahead point (1/m)
            velocity: Current forward velocity (m/s)
            
        Returns:
            tuple: (steering_angle, debug_info)
                steering_angle: Steering command (rad), clipped to [-max, +max]
                debug_info: dict with diagnostic values
        """
        # Compute lookahead distance
        ld = max(self.compute_lookahead(velocity), 1e-3)
        
        # Transform lookahead point to vehicle frame
        dx = lookahead_point[0] - current_pos[0]
        dy = lookahead_point[1] - current_pos[1]
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        y_ld = -sin_yaw * dx + cos_yaw * dy  # Lateral offset in vehicle frame
        
        # Term 1: Lateral error compensation (standard Pure Pursuit)
        curvature_term = 2.0 * y_ld / max(ld**2, 1e-6)
        steering = math.atan(self.wheelbase * curvature_term)
        steering *= self.lateral_error_gain
        
        # Term 2: Heading error compensation
        heading_error = _angle_wrap(lookahead_heading - current_yaw)
        steering += self.heading_error_gain * heading_error
        
        # Term 3: Curvature feed-forward
        steering += self.curvature_ff_gain * path_curvature
        
        # Clip steering angle
        steering = float(np.clip(steering, -self.max_steering_angle, 
                                self.max_steering_angle))
        
        # Debug info
        debug_info = {
            'lookahead_distance': ld,
            'lateral_offset': y_ld,
            'heading_error': heading_error,
            'curvature_term': curvature_term,
            'path_curvature': path_curvature
        }
        
        return steering, debug_info


class CurrentAccelCalibNode(Node):
    """ROS2 Node for current-acceleration calibration with physics-based modeling"""
    
    def __init__(self):
        super().__init__('current_acc_calib')
        
        # Declare parameters
        self.wheelbase = self.declare_parameter('wheelbase', 0.33).value
        self.lookahead_gain = self.declare_parameter('lookahead_gain', 1.6).value
        self.min_lookahead = self.declare_parameter('min_lookahead', 0.3).value
        self.max_lookahead = self.declare_parameter('max_lookahead', 3.5).value
        self.lateral_error_gain = self.declare_parameter('lateral_error_gain', 1.0).value
        self.heading_error_gain = self.declare_parameter('heading_error_gain', 0.4).value
        self.curvature_ff_gain = self.declare_parameter('curvature_ff_gain', 0.1).value
        
        # Racetrack trajectory parameters
        self.track_radius = self.declare_parameter('track_radius', 2.0).value  # Semicircle radius (meters)
        self.track_straight_length = self.declare_parameter('track_straight_length', 6.0).value  # Straight length (meters)
        self.track_points_per_straight = self.declare_parameter('track_points_per_straight', 150).value
        self.track_points_per_semicircle = self.declare_parameter('track_points_per_semicircle', 100).value
        
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
        self.trajectory = Figure8Trajectory(
            R=self.track_radius,
            straight_length=self.track_straight_length,
            points_per_straight=self.track_points_per_straight,
            points_per_semicircle=self.track_points_per_semicircle
        )
        self.controller = PurePursuitController(
            wheelbase=self.wheelbase,
            lookahead_gain=self.lookahead_gain,
            min_lookahead=self.min_lookahead,
            max_lookahead=self.max_lookahead,
            lateral_error_gain=self.lateral_error_gain,
            heading_error_gain=self.heading_error_gain,
            curvature_ff_gain=self.curvature_ff_gain,
            max_steering=0.35
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
            f"  Racetrack Radius: {self.track_radius} m\n"
            f"  Racetrack Straight Length: {self.track_straight_length} m\n"
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
        self.current_velocity = math.sqrt(
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
        yaw = math.atan2(
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
                    jerk_mode = 2.0 if self.trajectory.is_in_curve(self.trajectory_idx) else 0.0
                    
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
            cmd_msg.drive.speed = 0.0
            cmd_msg.drive.jerk = 0.0
            self.get_logger().info("Calibration complete! Data recorded in rosbag.")
        else:
            # Find closest point on trajectory
            closest_point, self.trajectory_idx, cross_track_error = \
                self.trajectory.get_closest_point(self.current_pos, self.trajectory_idx)
            
            # Compute adaptive lookahead distance based on velocity
            lookahead_distance = self.controller.compute_lookahead(self.current_velocity)
            lookahead_point, lookahead_idx = self.trajectory.get_lookahead_point(
                self.trajectory_idx, lookahead_distance
            )
            
            # Get trajectory heading and curvature at lookahead point
            lookahead_heading = self.trajectory.get_heading(lookahead_idx)
            path_curvature = self.trajectory.get_curvature(lookahead_idx)
            
            # Compute steering angle using Enhanced Pure Pursuit
            steering_angle, debug_info = self.controller.compute_steering(
                self.current_pos,
                self.current_yaw,
                lookahead_point,
                lookahead_heading,
                path_curvature,
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
                cmd_msg.drive.target_speed = target_speed
                mode_str = "SPEED_CONTROL"
            
            cmd_msg.drive.steering_angle = steering_angle
            cmd_msg.drive.jerk = jerk_mode  # Mode flag: 0=speed, 1=accel, 2=current, 3=duty
            
            # Periodic logging (every 2 seconds)
            if int(elapsed_time * 2) % 4 == 0:
                self.get_logger().info(
                    f"[{stage_name}] t={elapsed_time:.1f}s | "
                    f"I={current_A:.1f}A | v={self.current_velocity:.2f}m/s | "
                    f"δ={steering_angle:.3f}rad | ld={debug_info['lookahead_distance']:.2f}m | "
                    f"cte={cross_track_error:.2f}m | h_err={debug_info['heading_error']:.3f}rad | {mode_str}"
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
