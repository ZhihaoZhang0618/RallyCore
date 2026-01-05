#!/usr/bin/env python3
"""Manual-steer, speed-only staged runner.

目标
- 纵向：程序只控制 speed（分三阶段不同速度）
- 横向：方向由遥控器介入；当方向在死区内认为“直线”，发布 steering=0
  一旦遥控器方向超出死区认为“弯道”，发布 steering=遥控器转向值
- 输出：/calib/ackermann_cmd（AckermannDriveStamped）

注意
- 本节点不依赖定位/IMU，不使用 fastlio。
- drive.jerk 固定置为 0.0（speed mode），与 vesc_ackermann 的模式约定一致。

参数（可 ros2 param set 或 launch 注入）
- rc_topic: /rc/channels
- cmd_topic: /calib/ackermann_cmd
- command_frequency: 50
- steering_channel: 4
- steering_channel_mid: 984
- channel_deadzone: 100
- steering_limit: 0.40
- steering_reverse: true
- direction_reverse: false
- stage_speeds: [1.5, 3.0, 5.0]
- stage_durations: [60.0, 60.0, 60.0]
- rc_timeout_sec: 0.25

运行示例
- ros2 run f1tenth_system manual_steer_speed_stages.py

"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from crsf_receiver_msg.msg import CRSFChannels16


class ManualSteerSpeedStages(Node):
    def __init__(self) -> None:
        super().__init__("manual_steer_speed_stages")

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Topics
        self.rc_topic = self.declare_parameter("rc_topic", "/rc/channels").value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/calib/ackermann_cmd").value

        # Frequency / timeouts
        self.command_frequency = float(self.declare_parameter("command_frequency", 50.0).value)
        self.rc_timeout_sec = float(self.declare_parameter("rc_timeout_sec", 0.25).value)

        # Steering mapping (mirror joystick_control_v2 behavior)
        self.steering_channel = int(self.declare_parameter("steering_channel", 4).value)
        self.steering_limit = float(self.declare_parameter("steering_limit", 0.40).value)
        self.steering_reverse = bool(self.declare_parameter("steering_reverse", True).value)
        self.channel_mid = int(self.declare_parameter("steering_channel_mid", 968).value)
        self.channel_deadzone = int(self.declare_parameter("channel_deadzone", 100).value)

        # RC raw range assumptions (same defaults as joystick_control_v2)
        self.channel_max_range = int(self.declare_parameter("channel_max_range", 2000).value)
        self.channel_min_range = int(self.declare_parameter("channel_min_range", 0).value)

        # Direction reverse for speed sign
        self.direction_reverse = bool(self.declare_parameter("direction_reverse", False).value)

        # Stages
        self.stage_speeds: List[float] = list(self.declare_parameter("stage_speeds", [1.5, 3.0, 5.0]).value)
        self.stage_durations: List[float] = list(self.declare_parameter("stage_durations", [10.0, 10.0, 10.0]).value)
        if len(self.stage_speeds) != 3 or len(self.stage_durations) != 3:
            raise ValueError("stage_speeds and stage_durations must both have length 3")

        # IO
        self.rc_sub = self.create_subscription(CRSFChannels16, self.rc_topic, self._rc_callback, qos_best_effort)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)

        # State
        self._rc_channels: Optional[List[int]] = None
        self._last_rc_time = self.get_clock().now()
        self._t0 = self.get_clock().now()
        self._finished = False

        period = 1.0 / max(self.command_frequency, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"manual_steer_speed_stages started; rc={self.rc_topic}, cmd={self.cmd_topic}")
        self.get_logger().info(f"Stages: speeds={self.stage_speeds}, durations={self.stage_durations}")

    def _rc_callback(self, msg: CRSFChannels16) -> None:
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

        raw = self._rc_channels[idx]

        # Deadzone判定：完全参考 joystick_control_v2：deadzone*0.2
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

    def _get_stage_speed(self, elapsed_sec: float) -> Tuple[float, bool]:
        """Return (speed_mps, finished)."""
        t = elapsed_sec
        t0 = 0.0
        for i in range(3):
            t1 = t0 + float(self.stage_durations[i])
            if t < t1:
                return float(self.stage_speeds[i]), False
            t0 = t1
        return 0.0, True

    def _publish_cmd(self, speed_mps: float, steering_rad: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed_mps)
        msg.drive.steering_angle = float(steering_rad)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0  # speed mode
        self.cmd_pub.publish(msg)

    def _stop_and_finish(self, reason: str) -> None:
        if self._finished:
            return
        self._finished = True
        # Publish stop a few times for safety
        for _ in range(5):
            self._publish_cmd(0.0, 0.0)
        self.get_logger().info(f"[DONE] {reason} -> speed=0 published")

    def _on_timer(self) -> None:
        if self._finished:
            return

        now = self.get_clock().now()

        # RC timeout safety: stop if RC missing (we rely on RC for curve steering)
        if (now - self._last_rc_time).nanoseconds / 1e9 > self.rc_timeout_sec:
            self._stop_and_finish(f"RC timeout (> {self.rc_timeout_sec}s)")
            return

        elapsed = (now - self._t0).nanoseconds / 1e9
        stage_speed, done = self._get_stage_speed(elapsed)
        if done:
            self._stop_and_finish("All stages complete")
            return

        if self.direction_reverse:
            stage_speed = -stage_speed

        steering_rc, in_deadzone = self._steering_from_rc()

        # Straight: steering demand ~0 -> command steering=0
        # Curve: RC intervenes -> command steering=RC
        steering_cmd = 0.0 if in_deadzone else steering_rc

        self._publish_cmd(stage_speed, steering_cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualSteerSpeedStages()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_and_finish("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
