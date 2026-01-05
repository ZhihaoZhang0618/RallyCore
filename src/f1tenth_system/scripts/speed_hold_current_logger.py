#!/usr/bin/env python3
"""Speed-hold current logger (optional RC steering, no localization).

Purpose
=======
Command a sequence of target speeds (speed mode) and measure the average motor
phase current for each speed after the vehicle settles.

Typical use
===========
ros2 run f1tenth_system speed_hold_current_logger.py --ros-args \
  -p speeds:="[1,2,3,4,5,6,7,8]" -p hold_time_sec:=10.0 \
  -p vesc_topic:=/sensors/core -p odom_topic:=/odom -p output_path:=/tmp/speed_current.txt

Outputs
=======
- A human-readable txt file mapping speed->avg_current
- Optional CSV time series for debugging

Notes
=====
- Publishes /calib/ackermann_cmd with jerk=0.0 (speed mode).
- Steering can be either fixed (steering_rad) or taken from RC:
    - In deadzone: treat as straight and publish steering=0.0
    - Out of deadzone: publish steering=RC mapped value
- Current is extracted from VescStateStamped with robust fallbacks.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

try:
    from crsf_receiver_msg.msg import CRSFChannels16  # type: ignore
except Exception:  # pragma: no cover
    CRSFChannels16 = None  # type: ignore

try:
    from vesc_msgs.msg import VescStateStamped  # type: ignore
except Exception:  # pragma: no cover
    VescStateStamped = None  # type: ignore


@dataclass
class StageResult:
    target_speed: float
    mean_current: float
    std_current: float
    samples: int


def _try_get_attr(obj, path: str):
    cur = obj
    for part in path.split('.'):
        if not hasattr(cur, part):
            return None
        cur = getattr(cur, part)
    return cur


def _extract_current_a(msg) -> Tuple[Optional[float], str]:
    """Return (current_A, source_field)."""
    # Prefer phase/motor current-like fields.
    candidates = [
        'state.avg_iq',
        'state.iq',
        'state.current_motor',
        'state.avg_motor_current',
        'state.current',
        'state.current_input',
        'state.avg_input_current',
    ]
    for field in candidates:
        value = _try_get_attr(msg, field)
        if value is None:
            continue
        try:
            val = float(value)
        except Exception:
            continue
        if math.isfinite(val):
            return val, field
    return None, 'N/A'





class SpeedHoldCurrentLogger(Node):
    def __init__(self) -> None:
        super().__init__('speed_hold_current_logger')

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Topics
        self.cmd_topic = self.declare_parameter('cmd_topic', '/calib/ackermann_cmd').value
        self.vesc_topic = self.declare_parameter('vesc_topic', '/sensors/core').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value

        # RC steering (optional)
        self.use_rc_steering = bool(self.declare_parameter('use_rc_steering', True).value)
        self.rc_topic = self.declare_parameter('rc_topic', '/rc/channels').value
        self.rc_timeout_sec = float(self.declare_parameter('rc_timeout_sec', 0.25).value)
        self.post_turn_settle_sec = float(self.declare_parameter('post_turn_settle_sec', 0.8).value)

        # Steering mapping (mirror joystick_control_v2 behavior)
        self.steering_channel = int(self.declare_parameter('steering_channel', 4).value)
        self.steering_limit = float(self.declare_parameter('steering_limit', 0.40).value)
        self.steering_reverse = bool(self.declare_parameter('steering_reverse', True).value)
        self.channel_mid = int(self.declare_parameter('steering_channel_mid', 968).value)
        self.channel_deadzone = int(self.declare_parameter('channel_deadzone', 100).value)
        self.channel_max_range = int(self.declare_parameter('channel_max_range', 2000).value)
        self.channel_min_range = int(self.declare_parameter('channel_min_range', 0).value)

        # Commanding
        self.command_frequency = float(self.declare_parameter('command_frequency', 50.0).value)
        self.steering_rad = float(self.declare_parameter('steering_rad', 0.0).value)

        # Speed program
        self.speeds: List[float] = [float(v) for v in list(self.declare_parameter('speeds', [1, 2, 3, 4, 5, 6, 7, 8]).value)]
        self.hold_time_sec = float(self.declare_parameter('hold_time_sec', 10.0).value)
        self.speed_tolerance = float(self.declare_parameter('speed_tolerance', 0.2).value)
        self.require_speed_stable = bool(self.declare_parameter('require_speed_stable', True).value)
        self.stable_required_sec = float(self.declare_parameter('stable_required_sec', 1.0).value)

        # Outputs
        self.output_path = Path(str(self.declare_parameter('output_path', 'speed_hold_current_results.txt').value)).expanduser()
        self.csv_path = str(self.declare_parameter('csv_path', '').value).strip()

        self._csv_file = None
        self._csv_writer = None
        if self.csv_path:
            csv_path = Path(self.csv_path).expanduser()
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = csv_path.open('w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(['t_ros_sec', 'target_speed', 'measured_speed', 'current_A', 'current_field'])

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)

        self.rc_sub = None
        if self.use_rc_steering:
            if CRSFChannels16 is None:
                raise RuntimeError('crsf_receiver_msg is not available in this environment (needed for RC steering)')
            self.rc_sub = self.create_subscription(CRSFChannels16, self.rc_topic, self._rc_cb, qos_best_effort)

        if VescStateStamped is None:
            raise RuntimeError('vesc_msgs is not available in this environment')
        self.vesc_sub = self.create_subscription(VescStateStamped, self.vesc_topic, self._vesc_cb, qos_best_effort)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos_best_effort)

        self._rc_channels: Optional[List[int]] = None
        self._last_rc_time = self.get_clock().now()

        self._latest_current: Optional[float] = None
        self._latest_speed: Optional[float] = None
        self._current_field = 'N/A'
        self._latest_vesc_time = self.get_clock().now()

        self._stage_idx = 0
        self._stage_t0 = self.get_clock().now()
        self._collecting = False
        self._stable_time_acc = 0.0
        self._target_speed_reached = False
        self._sample_time_acc = 0.0
        self._post_turn_time_acc = 0.0
        self._was_turning = False

        self._currents: List[float] = []
        self._results: List[StageResult] = []
        self._done = False

        period = 1.0 / max(self.command_frequency, 1e-3)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'speed_hold_current_logger started; cmd={self.cmd_topic}, vesc={self.vesc_topic}, odom={self.odom_topic}, speeds={self.speeds}, use_rc_steering={self.use_rc_steering}'
        )

        if self.use_rc_steering:
            self.get_logger().info(f'RC steering: rc_topic={self.rc_topic}, steering_channel={self.steering_channel}, mid={self.channel_mid}, deadzone={self.channel_deadzone}, limit={self.steering_limit}, reverse={self.steering_reverse}')

    def _rc_cb(self, msg) -> None:
        # Keep same indexing style as joystick_control_v2 (1..16)
        self._rc_channels = [
            0,
            msg.ch1,
            msg.ch2,
            msg.ch3,
            msg.ch4,
            msg.ch5,
            msg.ch6,
            msg.ch7,
            msg.ch8,
            msg.ch9,
            msg.ch10,
            msg.ch11,
            msg.ch12,
            msg.ch13,
            msg.ch14,
            msg.ch15,
            msg.ch16,
        ]
        self._last_rc_time = self.get_clock().now()

    def _steering_from_rc(self) -> Tuple[float, bool]:
        """Return (steering_angle_rad, in_deadzone)."""
        if self._rc_channels is None:
            return 0.0, True

        idx = self.steering_channel
        if idx < 1 or idx >= len(self._rc_channels):
            return 0.0, True

        raw = int(self._rc_channels[idx])

        # Deadzone判定：参考 joystick_control_v2：abs(raw-mid) <= deadzone*0.2
        if abs(raw - self.channel_mid) <= int(self.channel_deadzone * 0.2):
            return 0.0, True

        # Normalize to [-1, 1]
        if raw > self.channel_mid:
            denom = max(self.channel_max_range - self.channel_mid, 1)
            normalized = (raw - self.channel_mid) / denom
        else:
            denom = max(self.channel_mid - self.channel_min_range, 1)
            normalized = (raw - self.channel_mid) / denom

        steering = float(normalized * self.steering_limit)
        steering = max(-self.steering_limit, min(self.steering_limit, steering))
        if self.steering_reverse:
            steering = -steering
        return steering, False

    def _vesc_cb(self, msg) -> None:
        current, cfield = _extract_current_a(msg)
        self._current_field = cfield
        if current is not None:
            self._latest_current = current
        self._latest_vesc_time = self.get_clock().now()

    def _odom_cb(self, msg: Odometry) -> None:
        """Extract speed from Odometry message."""
        try:
            speed = float(msg.twist.twist.linear.x)
            if math.isfinite(speed):
                self._latest_speed = speed
        except Exception:
            pass

    def _publish_cmd(self, speed_mps: float, steering_rad: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed_mps)
        msg.drive.steering_angle = float(steering_rad)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        for _ in range(5):
            self._publish_cmd(0.0, 0.0)

    def _finish(self, reason: str) -> None:
        if self._done:
            return
        self._done = True
        self._publish_stop()

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open('w') as f:
            f.write('# speed_hold_current_logger results\n')
            f.write(f'# vesc_topic={self.vesc_topic} odom_topic={self.odom_topic}\n')
            f.write(f'# current_field={self._current_field}\n')
            f.write(f'# hold_time_sec={self.hold_time_sec}\n')
            f.write('# format: speed_mps\tmean_current_A\tstd_current_A\tsamples\n')
            for r in self._results:
                f.write(f'{r.target_speed:.3f}\t{r.mean_current:.4f}\t{r.std_current:.4f}\t{r.samples}\n')

        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()

        self.get_logger().info(f'[DONE] {reason} -> wrote {self.output_path}')

    def _on_timer(self) -> None:
        if self._done:
            return

        now = self.get_clock().now()

        dt = 1.0 / max(self.command_frequency, 1e-3)

        steering_cmd = float(self.steering_rad)
        in_deadzone = True
        if self.use_rc_steering:
            if (now - self._last_rc_time).nanoseconds / 1e9 > self.rc_timeout_sec:
                self._finish(f'RC timeout (> {self.rc_timeout_sec}s)')
                return
            steering_rc, in_deadzone = self._steering_from_rc()
            steering_cmd = 0.0 if in_deadzone else float(steering_rc)
        # TODO add z angle speed callback to steering to guarantee going in a straight line
        if self._stage_idx >= len(self.speeds):
            self._finish('All stages complete')
            return

        target_speed = float(self.speeds[self._stage_idx])
        self._publish_cmd(target_speed, steering_cmd)

        t_stage = (now - self._stage_t0).nanoseconds / 1e9

        measured_speed = self._latest_speed

        # Wait for target speed to be reached (within tolerance)
        if not self._target_speed_reached:
            if measured_speed is not None and abs(measured_speed - target_speed) <= self.speed_tolerance:
                self._target_speed_reached = True
                self._stage_t0 = now  # Reset timer when target speed is first reached
                self._stable_time_acc = 0.0
                self._sample_time_acc = 0.0
                self._post_turn_time_acc = 0.0
                self._was_turning = False
                self.get_logger().debug(
                    f'[STAGE {self._stage_idx+1}] v={target_speed:.1f} m/s reached, starting collection'
                )
            return

        # Target speed is reached; now check for stability before collecting
        if self.require_speed_stable and measured_speed is not None:
            if abs(measured_speed - target_speed) <= self.speed_tolerance:
                self._stable_time_acc += 1.0 / max(self.command_frequency, 1e-3)
            else:
                self._stable_time_acc = 0.0

            if self._stable_time_acc < self.stable_required_sec:
                return

        self._collecting = True

        if self._latest_current is None:
            return

        # When RC steering is enabled, do NOT collect samples during turning
        # (out of deadzone), because cornering increases required current.
        if self.use_rc_steering and (not in_deadzone):
            self._was_turning = True
            self._post_turn_time_acc = 0.0
            return

        # After a turn ends (RC returns to deadzone), wait a short settle window
        # to let current return to normal before sampling.
        if self.use_rc_steering and self._was_turning and self.post_turn_settle_sec > 0.0:
            self._post_turn_time_acc += dt
            if self._post_turn_time_acc < self.post_turn_settle_sec:
                return
            self._was_turning = False

        # Collect sample
        self._currents.append(float(self._latest_current))
        self._sample_time_acc += dt

        if self._csv_writer is not None:
            t_ros = now.nanoseconds / 1e9
            self._csv_writer.writerow(
                [
                    f'{t_ros:.6f}',
                    f'{target_speed:.3f}',
                    '' if measured_speed is None else f'{measured_speed:.3f}',
                    f'{float(self._latest_current):.6f}',
                    self._current_field,
                ]
            )

        # Stage done? Use straight-only sample time when RC steering is enabled.
        if self._sample_time_acc >= self.hold_time_sec:
            values = self._currents
            if not values:
                mean = float('nan')
                std = float('nan')
                n = 0
            else:
                n = len(values)
                mean = sum(values) / n
                var = sum((v - mean) ** 2 for v in values) / max(n - 1, 1)
                std = math.sqrt(var)

            self._results.append(StageResult(target_speed=target_speed, mean_current=mean, std_current=std, samples=n))
            self.get_logger().info(
                f'[STAGE {self._stage_idx+1}/{len(self.speeds)}] v={target_speed:.1f} m/s -> I={mean:.2f}A (std={std:.2f}, n={n})'
            )

            # Next stage
            self._stage_idx += 1
            self._stage_t0 = now
            self._collecting = False
            self._stable_time_acc = 0.0
            self._target_speed_reached = False
            self._sample_time_acc = 0.0
            self._post_turn_time_acc = 0.0
            self._was_turning = False
            self._currents = []


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedHoldCurrentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt -> stopping')
        node._finish('KeyboardInterrupt')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
