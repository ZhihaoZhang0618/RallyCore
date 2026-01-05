#!/usr/bin/env python3
"""Manual-steer assisted current sweep calibration (no localization).

需求实现（严格版）
- 三阶段（不同目标速度）：stage_speeds=[v1,v2,v3]
- 每阶段在“直线”时进行电流从 0→60A 的扫掠（按 ramp_rate A/s 连续上升）
- 方向由遥控器介入：
  - 遥控器方向在死区内 => 认为直线 => 发布 current mode（jerk=2.0），steering=0
  - 遥控器方向超死区 => 认为弯道 => 发布 speed mode（jerk=0.0），speed=stage_speed，steering=遥控器
- 输出话题：/calib/ackermann_cmd

消息约定（必须与中控一致）
- jerk==2.0: current mode
  - drive.acceleration = current(A)
  - drive.steering_angle = steering(rad)
  - drive.speed 不使用（置 0）
- jerk==0.0: speed mode
  - drive.speed = target speed(m/s)
  - drive.steering_angle = steering(rad)
  - drive.acceleration 置 0

直线/弯道判定
- 参考 joystick_control_v2 的 deadzone 逻辑：abs(raw-mid) <= deadzone*0.2 视为“方向为0”。

电流跨直线衔接
- 从直线进入弯道时，记录该直线结束电流 I_end
- 再次进入直线时，将电流重置为 0.8 * I_end（可通过 reset_ratio 调整）

安全
- RC 超时则停止（发布 speed=0 / current=0），并打印 [DONE]

"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from crsf_receiver_msg.msg import CRSFChannels16


@dataclass
class StageConfig:
    target_speed: float
    current_min: float = 0.0
    current_max: float = 60.0


class ManualSteerCurrentSweepStages(Node):
    def __init__(self) -> None:
        super().__init__("manual_steer_current_sweep_stages")

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Topics
        self.rc_topic = self.declare_parameter("rc_topic", "/rc/channels").value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/calib/ackermann_cmd").value

        # Timing
        self.command_frequency = float(self.declare_parameter("command_frequency", 50.0).value)
        self.rc_timeout_sec = float(self.declare_parameter("rc_timeout_sec", 0.25).value)

        # Steering mapping (mirror joystick_control_v2 behavior)
        self.steering_channel = int(self.declare_parameter("steering_channel", 4).value)
        self.steering_limit = float(self.declare_parameter("steering_limit", 0.40).value)
        self.steering_reverse = bool(self.declare_parameter("steering_reverse", True).value)
        self.channel_mid = int(self.declare_parameter("steering_channel_mid", 975).value)
        self.channel_deadzone = int(self.declare_parameter("channel_deadzone", 100).value)
        self.channel_max_range = int(self.declare_parameter("channel_max_range", 2000).value)
        self.channel_min_range = int(self.declare_parameter("channel_min_range", 0).value)

        # Longitudinal sweep
        self.current_ramp_rate = float(self.declare_parameter("current_ramp_rate", 3.0).value)  # A/s
        self.reset_ratio = float(self.declare_parameter("reset_ratio", 0.9).value)
        self.current_min = float(self.declare_parameter("current_min", 0.0).value)
        self.current_max = float(self.declare_parameter("current_max", 100.0).value)

        # Stages
        stage_speeds: List[float] = list(self.declare_parameter("stage_speeds", [1.5, 3.0, 5.0]).value)
        if len(stage_speeds) != 3:
            raise ValueError("stage_speeds must have length 3")
        self.stages: List[StageConfig] = [
            StageConfig(target_speed=float(v), current_min=self.current_min, current_max=self.current_max)
            for v in stage_speeds
        ]

        # IO
        self.rc_sub = self.create_subscription(CRSFChannels16, self.rc_topic, self._rc_callback, qos_best_effort)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)

        # State
        self._rc_channels: Optional[List[int]] = None
        self._last_rc_time = self.get_clock().now()
        self._last_cmd_time = self.get_clock().now()

        self._finished = False
        self._stage_idx = 0

        self._in_straight_prev: Optional[bool] = None
        self._current_cmd = float(self.current_min)
        self._last_straight_end_current = float(self.current_min)

        period = 1.0 / max(self.command_frequency, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"manual_steer_current_sweep_stages started; rc={self.rc_topic}, cmd={self.cmd_topic}"
        )
        self.get_logger().info(
            f"Stages target speeds: {[s.target_speed for s in self.stages]} m/s; sweep {self.current_min}..{self.current_max}A @ {self.current_ramp_rate}A/s"
        )

    def _rc_callback(self, msg: CRSFChannels16) -> None:
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

    def _publish_speed_mode(self, speed_mps: float, steering_rad: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed_mps)
        msg.drive.steering_angle = float(steering_rad)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.cmd_pub.publish(msg)

    def _publish_current_mode(self, current_a: float, steering_rad: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = float(steering_rad)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = float(current_a)  # current in A
        msg.drive.jerk = 2.0
        self.cmd_pub.publish(msg)

    def _stop_and_finish(self, reason: str) -> None:
        if self._finished:
            return
        self._finished = True
        for _ in range(5):
            self._publish_speed_mode(0.0, 0.0)
            self._publish_current_mode(0.0, 0.0)
        self.get_logger().info(f"[DONE] {reason} -> published speed=0 and current=0")

    def _advance_stage_if_done(self) -> None:
        if self._stage_idx >= len(self.stages):
            return
        stage = self.stages[self._stage_idx]
        if self._current_cmd >= stage.current_max - 1e-6:
            self.get_logger().info(
                f"Stage {self._stage_idx+1} complete (I reached {self._current_cmd:.2f}A)."
            )
            self._stage_idx += 1
            self._current_cmd = float(self.current_min)
            self._last_straight_end_current = float(self.current_min)
            self._in_straight_prev = None

    def _on_timer(self) -> None:
        if self._finished:
            return

        now = self.get_clock().now()

        # RC timeout safety
        if (now - self._last_rc_time).nanoseconds / 1e9 > self.rc_timeout_sec:
            self._stop_and_finish(f"RC timeout (> {self.rc_timeout_sec}s)")
            return

        if self._stage_idx >= len(self.stages):
            self._stop_and_finish("All stages complete")
            return

        stage = self.stages[self._stage_idx]

        steering_rc, in_deadzone = self._steering_from_rc()
        in_straight = in_deadzone

        # Detect transitions for reset rule
        if self._in_straight_prev is None:
            self._in_straight_prev = in_straight
        else:
            # straight -> curve
            if self._in_straight_prev and (not in_straight):
                self._last_straight_end_current = float(self._current_cmd)
            # curve -> straight
            if (not self._in_straight_prev) and in_straight:
                self._current_cmd = float(self.reset_ratio * self._last_straight_end_current)
                if self._current_cmd < self.current_min:
                    self._current_cmd = float(self.current_min)
                if self._current_cmd > stage.current_max:
                    self._current_cmd = float(stage.current_max)
            self._in_straight_prev = in_straight

        dt = (now - self._last_cmd_time).nanoseconds / 1e9
        self._last_cmd_time = now
        dt = max(0.0, min(dt, 0.2))  # cap for safety

        if in_straight:
            # Straight: current sweep, steering forced to 0
            self._current_cmd += self.current_ramp_rate * dt
            if self._current_cmd > stage.current_max:
                self._current_cmd = float(stage.current_max)

            self._publish_current_mode(self._current_cmd, 0.0)

            self._advance_stage_if_done()
            if self._stage_idx >= len(self.stages):
                self._stop_and_finish("All stages complete")

        else:
            # Curve: fixed speed at stage target, steering from RC
            self._publish_speed_mode(stage.target_speed, steering_rc)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualSteerCurrentSweepStages()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_and_finish("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
