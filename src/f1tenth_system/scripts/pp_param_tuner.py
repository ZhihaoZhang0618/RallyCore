#!/usr/bin/env python3
"""Dedicated Pure Pursuit parameter tuning node.

Goal: adjust PP parameters so the vehicle hugs the trajectory at any speed without
oscillation or over-shooting corners. The node isolates PP logic from current
calibration and adds tooling for live tuning.

Features
========
- Internal figure-8 generator or optional external ``nav_msgs/Path`` input
- Adaptive lookahead with on-the-fly parameter updates via ``ros2 param set``
- Steering damping (rate limit + exponential smoothing) to avoid "扭来扭去"
- Manual speed control for incremental testing
- Rolling tracking metrics (cross-track RMS, heading RMS, mean steering)

Quick start::

    ros2 run f1tenth_system pp_param_tuner --ros-args \
        -p lookahead_gain:=1.8 -p steering_rate_limit:=3.0 \
        -p target_speed:=2.0

Adjust parameters live::

    ros2 param set /pp_param_tuner lookahead_gain 1.6
    ros2 param set /pp_param_tuner lateral_error_gain 1.2
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange


def _angle_wrap(angle: float) -> float:
    """Wrap any angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TrajectoryHelper:
    """Utility for sampling closest/heading/curvature along a closed path."""

    def __init__(self, points: np.ndarray):
        self.update_points(points)

    def update_points(self, points: np.ndarray) -> None:
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("Trajectory points must be of shape (N, 2)")
        if len(points) < 2:
            raise ValueError("Trajectory requires at least two points")
        self.points = points
        self.length = len(points)

    def closest_point(
        self, current_pos: np.ndarray, start_idx: int, window: int = 80
    ) -> Tuple[np.ndarray, int, float]:
        indices = (np.arange(start_idx, start_idx + window) % self.length).astype(int)
        segment = self.points[indices]
        distances = np.linalg.norm(segment - current_pos, axis=1)
        local = int(np.argmin(distances))
        global_idx = int(indices[local])
        return self.points[global_idx], global_idx, float(distances[local])

    def lookahead_point(
        self, current_idx: int, lookahead_distance: float
    ) -> Tuple[np.ndarray, int]:
        idx = current_idx % self.length
        distance = 0.0
        while distance < lookahead_distance:
            next_idx = (idx + 1) % self.length
            segment = np.linalg.norm(self.points[next_idx] - self.points[idx])
            if distance + segment >= lookahead_distance:
                ratio = (lookahead_distance - distance) / max(segment, 1e-6)
                point = self.points[idx] + ratio * (self.points[next_idx] - self.points[idx])
                return point, next_idx
            distance += segment
            idx = next_idx
        return self.points[idx], idx

    def heading(self, idx: int) -> float:
        prev_pt = self.points[(idx - 1) % self.length]
        next_pt = self.points[(idx + 1) % self.length]
        return math.atan2(next_pt[1] - prev_pt[1], next_pt[0] - prev_pt[0])

    def curvature(self, idx: int) -> float:
        p_prev = self.points[(idx - 1) % self.length]
        p_curr = self.points[idx % self.length]
        p_next = self.points[(idx + 1) % self.length]
        a = np.linalg.norm(p_curr - p_prev)
        b = np.linalg.norm(p_next - p_curr)
        c = np.linalg.norm(p_next - p_prev)
        denom = max(a * b * c, 1e-6)
        if denom < 1e-6:
            return 0.0
        area = abs(
            p_prev[0] * (p_curr[1] - p_next[1])
            + p_curr[0] * (p_next[1] - p_prev[1])
            + p_next[0] * (p_prev[1] - p_curr[1])
        ) / 2.0
        return 4.0 * area / denom


class Figure8Trajectory(TrajectoryHelper):
    """Classic figure-8 path used for PP testing."""

    def __init__(self, radius: float = 1.6, points_per_circle: int = 200):
        self.radius = radius
        self.points_per_circle = points_per_circle
        super().__init__(self._generate())

    def _generate(self) -> np.ndarray:
        theta = np.linspace(0.0, 2.0 * math.pi, self.points_per_circle, endpoint=False)
        right = np.column_stack(
            [self.radius + self.radius * np.cos(theta), self.radius * np.sin(theta)]
        )
        left = np.column_stack(
            [-self.radius + self.radius * np.cos(theta), self.radius * np.sin(theta)]
        )
        return np.vstack([right, left])

    def regenerate(self, radius: float) -> None:
        self.radius = radius
        super().update_points(self._generate())


class AdaptivePurePursuit:
    """Pure Pursuit controller with tunable gains and feed-forward hooks."""

    def __init__(
        self,
        wheelbase: float,
        lookahead_gain: float,
        min_lookahead: float,
        max_lookahead: float,
        lateral_error_gain: float,
        heading_error_gain: float,
        curvature_ff_gain: float,
        max_steering: float,
    ):
        self.wheelbase = wheelbase
        self.lookahead_gain = lookahead_gain
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lateral_error_gain = lateral_error_gain
        self.heading_error_gain = heading_error_gain
        self.curvature_ff_gain = curvature_ff_gain
        self.max_steering = max_steering

    def update(self, **kwargs: float) -> None:
        for name, value in kwargs.items():
            if hasattr(self, name):
                setattr(self, name, float(value))

    def compute_lookahead(self, velocity: float) -> float:
        ld = self.lookahead_gain * abs(velocity) + self.min_lookahead
        return float(np.clip(ld, self.min_lookahead, self.max_lookahead))

    def compute(
        self,
        current_pos: np.ndarray,
        current_yaw: float,
        lookahead_point: np.ndarray,
        lookahead_heading: float,
        path_curvature: float,
        velocity: float,
        cross_track_error: float,
    ) -> Tuple[float, Dict[str, float]]:
        ld = max(self.compute_lookahead(velocity), 1e-3)
        dx = lookahead_point[0] - current_pos[0]
        dy = lookahead_point[1] - current_pos[1]
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        x_ld = cos_yaw * dx + sin_yaw * dy
        y_ld = -sin_yaw * dx + cos_yaw * dy
        curvature_term = 2.0 * y_ld / max(ld**2, 1e-6)
        steering = math.atan(self.wheelbase * curvature_term)
        steering *= self.lateral_error_gain
        heading_error = _angle_wrap(lookahead_heading - current_yaw)
        steering += self.heading_error_gain * heading_error
        steering += self.curvature_ff_gain * path_curvature
        steering = float(np.clip(steering, -self.max_steering, self.max_steering))
        debug = {
            "lookahead": ld,
            "heading_error": heading_error,
            "curvature_term": curvature_term,
            "cross_track_error": cross_track_error,
        }
        return steering, debug


class SteeringFilter:
    """Rate limiter + exponential smoothing to remove oscillations."""

    def __init__(self, rate_limit: float, alpha: float):
        self.rate_limit = max(rate_limit, 0.0)
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self._last_value = 0.0
        self._last_time = None  # seconds

    def update_gains(self, rate_limit: float, alpha: float) -> None:
        self.rate_limit = max(rate_limit, 0.0)
        self.alpha = float(np.clip(alpha, 0.0, 1.0))

    def reset(self) -> None:
        self._last_value = 0.0
        self._last_time = None

    def filter(self, target: float, now: float) -> float:
        if self._last_time is not None and self.rate_limit > 0.0:
            dt = max(now - self._last_time, 1e-3)
            max_delta = self.rate_limit * dt
            target = float(np.clip(target, self._last_value - max_delta, self._last_value + max_delta))
        filtered = self.alpha * target + (1.0 - self.alpha) * self._last_value
        self._last_value = filtered
        self._last_time = now
        return filtered


class TrackingMetrics:
    """Rolling statistics to visualize tuning progress in logs."""

    def __init__(self, window: int = 200):
        self.cte: Deque[float] = deque(maxlen=window)
        self.heading: Deque[float] = deque(maxlen=window)
        self.steering: Deque[float] = deque(maxlen=window)

    def update(self, cross_track: float, heading_error: float, steering: float) -> None:
        self.cte.append(abs(cross_track))
        self.heading.append(abs(heading_error))
        self.steering.append(abs(steering))

    def summary(self) -> str:
        def _rms(values: Deque[float]) -> float:
            if not values:
                return 0.0
            arr = np.array(values)
            return float(np.sqrt(np.mean(arr**2)))

        mean_steer = float(np.mean(self.steering)) if self.steering else 0.0
        return (
            f"cte_rms={_rms(self.cte):.03f}m | "
            f"heading_rms={_rms(self.heading):.03f}rad | "
            f"mean_steer={mean_steer:.03f}rad"
        )


class PPTuningNode(Node):
    """ROS2 node that concentrates purely on Pure Pursuit tuning."""

    def __init__(self) -> None:
        super().__init__('pp_param_tuner')
        self._declare_parameters()
        self._build_helpers()
        self._setup_interfaces()
        self._start_time = self.get_clock().now()
        self._trajectory_idx = 0
        self._last_log_time = 0.0
        self._odom_ready = False
        self.get_logger().info('PP tuner ready 🚗 start driving and tweak parameters!')

    # Parameter plumbing -------------------------------------------------------
    def _declare_parameters(self) -> None:
        # Control parameters with descriptors for rqt_reconfigure
        lookahead_desc = ParameterDescriptor(
            description='Lookahead gain multiplier for velocity',
            floating_point_range=[FloatingPointRange(from_value=0.5, to_value=5.0, step=0.1)]
        )
        self.wheelbase = self.declare_parameter('wheelbase', 0.33).value
        self.max_steering = self.declare_parameter('max_steering', 0.35).value
        self.lookahead_gain = self.declare_parameter('lookahead_gain', 1.6, lookahead_desc).value
        self.min_lookahead = self.declare_parameter('min_lookahead', 0.30).value
        self.max_lookahead = self.declare_parameter('max_lookahead', 3.5).value
        self.lateral_error_gain = self.declare_parameter('lateral_error_gain', 1.0).value
        self.heading_error_gain = self.declare_parameter('heading_error_gain', 0.4).value
        self.curvature_ff_gain = self.declare_parameter('curvature_ff_gain', 0.1).value
        self.steering_rate_limit = self.declare_parameter('steering_rate_limit', 2.5).value
        self.steering_smoothing_alpha = self.declare_parameter('steering_smoothing_alpha', 0.4).value
        self.command_frequency = self.declare_parameter('command_frequency', 50.0).value
        self.figure8_radius = self.declare_parameter('figure8_radius', 1.6).value
        self.use_external_path = self.declare_parameter('use_external_path', False).value
        
        # Target speed with descriptor for easy tuning
        speed_desc = ParameterDescriptor(
            description='Target speed (m/s) for trajectory tracking',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)]
        )
        self.target_speed = self.declare_parameter('target_speed', 1.0, speed_desc).value
        self.log_interval = self.declare_parameter('log_interval', 1.0).value
        self.metrics_window = int(self.declare_parameter('metrics_window', 200).value)

        self.add_on_set_parameters_callback(self._on_parameter_change)

    def _build_helpers(self) -> None:
        self.figure8 = Figure8Trajectory(radius=self.figure8_radius)
        self.external_traj: Optional[TrajectoryHelper] = None
        self.controller = AdaptivePurePursuit(
            wheelbase=self.wheelbase,
            lookahead_gain=self.lookahead_gain,
            min_lookahead=self.min_lookahead,
            max_lookahead=self.max_lookahead,
            lateral_error_gain=self.lateral_error_gain,
            heading_error_gain=self.heading_error_gain,
            curvature_ff_gain=self.curvature_ff_gain,
            max_steering=self.max_steering,
        )
        self.steering_filter = SteeringFilter(
            rate_limit=self.steering_rate_limit,
            alpha=self.steering_smoothing_alpha,
        )
        self.metrics = TrackingMetrics(window=self.metrics_window)
        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.current_velocity = 0.0

    def _setup_interfaces(self) -> None:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, '/odom', self._odom_callback, qos)
        self.create_subscription(Path, '/pp/reference_path', self._path_callback, qos)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.trajectory_pub = self.create_publisher(Path, '/pp/current_trajectory', 10)
        self.create_timer(1.0 / max(self.command_frequency, 1.0), self._control_loop)
        self.create_timer(5.0, self._publish_trajectory)  # Publish trajectory every 5 seconds

    def _on_parameter_change(self, params: List[Parameter]) -> SetParametersResult:
        controller_updates: Dict[str, float] = {}
        for param in params:
            name = param.name
            value = param.value
            if name in {
                'lookahead_gain', 'min_lookahead', 'max_lookahead', 'lateral_error_gain',
                'heading_error_gain', 'curvature_ff_gain', 'max_steering'
            }:
                controller_updates[name] = float(value)
            elif name == 'steering_rate_limit':
                self.steering_rate_limit = float(value)
                self.steering_filter.update_gains(self.steering_rate_limit, self.steering_filter.alpha)
            elif name == 'steering_smoothing_alpha':
                self.steering_smoothing_alpha = float(value)
                self.steering_filter.update_gains(self.steering_filter.rate_limit, self.steering_smoothing_alpha)
            elif name == 'target_speed':
                self.target_speed = float(value)
            elif name == 'log_interval':
                self.log_interval = max(float(value), 0.1)
            elif name == 'figure8_radius':
                self.figure8_radius = float(value)
                self.figure8.regenerate(self.figure8_radius)
            elif name == 'metrics_window':
                self.metrics_window = int(max(value, 10))
                self.metrics = TrackingMetrics(window=self.metrics_window)
            elif name == 'use_external_path':
                self.use_external_path = bool(value)

        if controller_updates:
            self.controller.update(**controller_updates)

        return SetParametersResult(successful=True)

    # Subscriptions ------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])
        quat = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2),
        )
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        self._odom_ready = True

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        points = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
        if len(points) < 2:
            self.get_logger().warning('Received external path with <2 points, ignoring')
            return
        self.external_traj = TrajectoryHelper(points)
        self.get_logger().info(f'Loaded external path with {len(points)} points')

    # Core control loop -------------------------------------------------------
    def _active_trajectory(self) -> TrajectoryHelper:
        if self.use_external_path and self.external_traj is not None:
            return self.external_traj
        return self.figure8

    def _control_loop(self) -> None:
        if not self._odom_ready:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        trajectory = self._active_trajectory()
        closest, self._trajectory_idx, _ = trajectory.closest_point(
            self.current_pos, self._trajectory_idx
        )
        heading_at_closest = trajectory.heading(self._trajectory_idx)
        dx = self.current_pos[0] - closest[0]
        dy = self.current_pos[1] - closest[1]
        cross_track = -math.sin(heading_at_closest) * dx + math.cos(heading_at_closest) * dy
        lookahead_distance = self.controller.compute_lookahead(self.current_velocity)
        lookahead_point, lookahead_idx = trajectory.lookahead_point(
            self._trajectory_idx, lookahead_distance
        )
        lookahead_heading = trajectory.heading(lookahead_idx)
        path_curvature = trajectory.curvature(lookahead_idx)
        steering_cmd, debug = self.controller.compute(
            self.current_pos,
            self.current_yaw,
            lookahead_point,
            lookahead_heading,
            path_curvature,
            self.current_velocity,
            cross_track,
        )
        filtered_steering = self.steering_filter.filter(steering_cmd, now)
        self.metrics.update(cross_track, debug['heading_error'], filtered_steering)
        
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = float(self.target_speed)
        cmd.drive.steering_angle = filtered_steering
        self.publisher.publish(cmd)

        if now - self._last_log_time > self.log_interval:
            self._last_log_time = now
            self.get_logger().info(
                f"v={self.current_velocity:.2f}m/s -> target={self.target_speed:.2f} | "
                f"δ={filtered_steering:.3f}rad | ld={debug['lookahead']:.2f}m | "
                f"cte={cross_track:.3f}m | {self.metrics.summary()}"
            )

    def _publish_trajectory(self) -> None:
        """Publish the current active trajectory as a Path message for visualization."""
        trajectory = self._active_trajectory()
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for point in trajectory.points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.trajectory_pub.publish(path_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PPTuningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PP tuner...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

