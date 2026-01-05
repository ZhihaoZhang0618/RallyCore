#!/usr/bin/env python3
"""Speed-interval acceleration sweep (current mode, optional RC steering, no localization).

Goal
====
For each speed interval [v, v+dv] (default dv=1 m/s), test different phase
currents and measure time-to-reach the upper speed, then estimate acceleration.

Workflow
========
1) Hold speed v in speed mode (jerk=0.0) until stable
2) Apply current command (jerk=2.0) and time until measured_speed >= v+dv
3) Return to speed v (reset) and repeat for next current
4) Repeat for v=1..7 (for 1-2,2-3,...,7-8)

Base current
============
You can optionally start current sweep from a baseline current measured from
speed_hold_current_logger results (speed->I). Provide base_current_file.

Outputs
=======
A txt file with per-trial metrics, plus optional CSV.

Safety
======
- By default steering is fixed via steering_rad (usually 0.0)
- If RC steering is enabled (use_rc_steering:=true), trials are gated to straight
    segments only (RC in deadzone). Turning aborts/pause the trial to avoid
    cornering-induced current/accel bias.
- On Ctrl+C publishes stop commands
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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


def _try_get_attr(obj, path: str):
    cur = obj
    for part in path.split('.'):
        if not hasattr(cur, part):
            return None
        cur = getattr(cur, part)
    return cur


def _extract_current_a(msg) -> Tuple[Optional[float], str]:
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


def _extract_speed_mps(msg) -> Tuple[Optional[float], str]:
    # Deprecated for this node: speed feedback should come from Odometry.
    # Kept to avoid breaking external imports.
    return None, 'N/A'


def _load_base_currents(path: Path) -> Dict[float, float]:
    mapping: Dict[float, float] = {}
    if not path.exists():
        return mapping
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        try:
            v = float(parts[0])
            i = float(parts[1])
        except Exception:
            continue
        mapping[v] = i
    return mapping


@dataclass
class TrialResult:
    v0: float
    v1: float
    current_a: float
    t_sec: float
    accel_mps2: float
    reached: bool


class SpeedIntervalAccelSweep(Node):
    def __init__(self) -> None:
        super().__init__('speed_interval_accel_sweep')

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_topic = self.declare_parameter('cmd_topic', '/calib/ackermann_cmd').value
        self.vesc_topic = self.declare_parameter('vesc_topic', '/sensors/core').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.command_frequency = float(self.declare_parameter('command_frequency', 50.0).value)
        self.steering_rad = float(self.declare_parameter('steering_rad', 0.0).value)

        # RC steering (optional)
        self.use_rc_steering = bool(self.declare_parameter('use_rc_steering', False).value)
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

        self.v_start = float(self.declare_parameter('v_start', 1.0).value)
        self.v_end = float(self.declare_parameter('v_end', 8.0).value)
        self.dv = float(self.declare_parameter('dv', 1.0).value)

        self.current_step = float(self.declare_parameter('current_step', 3.0).value)
        self.current_max = float(self.declare_parameter('current_max', 80.0).value)
        self.current_min_override = float(self.declare_parameter('current_min_override', float('nan')).value)

        self.reset_hold_time_sec = float(self.declare_parameter('reset_hold_time_sec', 4.0).value)
        self.reset_settle_time_sec = float(self.declare_parameter('reset_settle_time_sec', 2.0).value)
        self.speed_tolerance = float(self.declare_parameter('speed_tolerance', 0.2).value)
        self.stable_required_sec = float(self.declare_parameter('stable_required_sec', 1.0).value)

        self.reach_tolerance = float(self.declare_parameter('reach_tolerance', 0.05).value)
        self.trial_timeout_sec = float(self.declare_parameter('trial_timeout_sec', 8.0).value)

        base_file = str(self.declare_parameter('base_current_file', 'speed_hold_current_results.txt').value)
        self.base_current_file = Path(base_file).expanduser()
        self.base_currents = _load_base_currents(self.base_current_file)

        self.output_path = Path(str(self.declare_parameter('output_path', 'speed_interval_accel_results.txt').value)).expanduser()
        self.csv_path = str(self.declare_parameter('csv_path', '').value).strip()

        self._csv_file = None
        self._csv_writer = None
        if self.csv_path:
            csv_path = Path(self.csv_path).expanduser()
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = csv_path.open('w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(['v0', 'v1', 'current_A', 't_sec', 'accel_mps2', 'reached', 'current_field', 'speed_field'])

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

        self._latest_speed: Optional[float] = None
        self._latest_current: Optional[float] = None
        self._speed_field = 'odom.twist.twist.linear.x'
        self._current_field = 'N/A'

        # State machine
        self._v0 = self.v_start
        self._v1 = self._v0 + self.dv
        self._current_list: List[float] = []
        self._current_idx = 0

        self._phase = 'RESET'  # RESET -> TRIAL -> (next current or next interval)
        self._phase_t0 = self.get_clock().now()
        self._stable_time_acc = 0.0
        self._trial_t0_ros: Optional[float] = None

        # Straight gating (when RC steering enabled)
        self._post_turn_time_acc = 0.0
        self._was_turning = False

        self._results: List[TrialResult] = []
        self._done = False

        period = 1.0 / max(self.command_frequency, 1e-3)
        self._timer = self.create_timer(period, self._on_timer)

        self._prepare_currents_for_interval()

        self.get_logger().info(
            f'speed_interval_accel_sweep started; v=[{self.v_start},{self.v_end}] dv={self.dv}, I_step={self.current_step}, I_max={self.current_max}, base_file={self.base_current_file}, use_rc_steering={self.use_rc_steering}'
        )

        if self.use_rc_steering:
            self.get_logger().info(
                f'RC steering: rc_topic={self.rc_topic}, steering_channel={self.steering_channel}, mid={self.channel_mid}, deadzone={self.channel_deadzone}, limit={self.steering_limit}, reverse={self.steering_reverse}, post_turn_settle_sec={self.post_turn_settle_sec}'
            )

    def _finish(self, reason: str) -> None:
        if self._done:
            return
        self._publish_stop()
        self._write_results(reason)
        self._done = True
        self._timer.cancel()

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

    def _odom_cb(self, msg: Odometry) -> None:
        try:
            speed = float(msg.twist.twist.linear.x)
        except Exception:
            return
        if math.isfinite(speed):
            self._latest_speed = speed

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
        msg.drive.acceleration = float(current_a)
        msg.drive.jerk = 2.0
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        for _ in range(5):
            self._publish_speed_mode(0.0, 0.0)
            self._publish_current_mode(0.0, 0.0)

    def _prepare_currents_for_interval(self) -> None:
        # Baseline from first stage results
        base = self.base_currents.get(round(self._v0, 3))
        if base is None:
            base = self.base_currents.get(self._v0)

        if not math.isfinite(self.current_min_override):
            start = float(base) if base is not None else 0.0
        else:
            start = float(self.current_min_override)

        start = max(0.0, start)
        currents: List[float] = []
        cur = start
        # Always include start
        while cur <= self.current_max + 1e-6:
            currents.append(float(cur))
            cur += self.current_step

        # De-dup and clamp
        currents = sorted(set(min(self.current_max, max(0.0, c)) for c in currents))
        self._current_list = currents
        self._current_idx = 0

        self.get_logger().info(
            f'Interval {self._v0:.1f}->{self._v1:.1f} m/s: base_I={base if base is not None else float("nan"):.2f}A, sweep={currents[0]:.1f}..{currents[-1]:.1f}A step={self.current_step:.1f}A'
        )

    def _advance_interval(self) -> None:
        self._v0 += self.dv
        self._v1 = self._v0 + self.dv
        if self._v1 > self.v_end + 1e-6:
            # Mark completion by letting _on_timer hit the end condition and call _finish().
            return
        self._prepare_currents_for_interval()

    def _write_results(self, reason: str) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open('w') as f:
            f.write('# speed_interval_accel_sweep results\n')
            f.write(f'# vesc_topic={self.vesc_topic} odom_topic={self.odom_topic}\n')
            f.write(f'# current_field={self._current_field} speed_field={self._speed_field}\n')
            f.write(f'# dv={self.dv} current_step={self.current_step} current_max={self.current_max}\n')
            f.write('# format: v0\tv1\tcurrent_A\tt_sec\taccel_mps2\treached\n')
            for r in self._results:
                f.write(
                    f'{r.v0:.3f}\t{r.v1:.3f}\t{r.current_a:.3f}\t{r.t_sec:.4f}\t{r.accel_mps2:.4f}\t{int(r.reached)}\n'
                )

        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()

        self.get_logger().info(f'[DONE] {reason} -> wrote {self.output_path}')

    def _on_timer(self) -> None:
        if self._done:
            return

        now = self.get_clock().now()
        dt = 1.0 / max(self.command_frequency, 1e-3)
        t_phase = (now - self._phase_t0).nanoseconds / 1e9

        steering_cmd = float(self.steering_rad)
        in_deadzone = True
        if self.use_rc_steering:
            if (now - self._last_rc_time).nanoseconds / 1e9 > self.rc_timeout_sec:
                self._finish(f'RC timeout (> {self.rc_timeout_sec}s)')
                return
            steering_rc, in_deadzone = self._steering_from_rc()
            steering_cmd = 0.0 if in_deadzone else float(steering_rc)

            # Turning: abort/pause trials and return to RESET (do not advance current)
            if not in_deadzone:
                self._was_turning = True
                self._post_turn_time_acc = 0.0
                if self._phase != 'RESET':
                    self.get_logger().info('Turning detected -> abort trial and reset to v0')
                self._phase = 'RESET'
                self._phase_t0 = now
                self._stable_time_acc = 0.0
                self._publish_speed_mode(self._v0, steering_cmd)
                return

            # After turn ends, wait a settle window before resuming trials
            if in_deadzone and self._was_turning and self.post_turn_settle_sec > 0.0:
                self._post_turn_time_acc += dt
                self._publish_speed_mode(self._v0, 0.0)
                if self._post_turn_time_acc < self.post_turn_settle_sec:
                    return
                self._was_turning = False
                # Restart RESET timing after the settle window
                self._phase = 'RESET'
                self._phase_t0 = now
                self._stable_time_acc = 0.0
                return

        # Need speed feedback
        if self._latest_speed is None:
            self._publish_speed_mode(self._v0, steering_cmd)
            return

        # All done?
        if self._v1 > self.v_end + 1e-6:
            self._finish('All intervals complete')
            return

        if self._phase == 'RESET':
            # Hold v0 to reset starting speed
            self._publish_speed_mode(self._v0, steering_cmd)

            # Wait settle time then require stable for stable_required_sec
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return

            if abs(self._latest_speed - self._v0) <= self.speed_tolerance:
                self._stable_time_acc += 1.0 / max(self.command_frequency, 1e-3)
            else:
                self._stable_time_acc = 0.0

            if self._stable_time_acc < self.stable_required_sec:
                return

            if t_phase < self.reset_hold_time_sec:
                return

            # Start trial
            self._phase = 'TRIAL'
            self._phase_t0 = now
            self._trial_t0_ros = now.nanoseconds / 1e9
            return

        if self._phase == 'TRIAL':
            if self._current_idx >= len(self._current_list):
                # Move to next interval
                self._advance_interval()
                self._phase = 'RESET'
                self._phase_t0 = now
                self._stable_time_acc = 0.0
                return

            current_a = float(self._current_list[self._current_idx])
            self._publish_current_mode(current_a, steering_cmd)

            # Reached upper speed?
            reached = self._latest_speed >= (self._v1 - self.reach_tolerance)
            t_trial = t_phase
            if reached:
                dv = self._v1 - self._v0
                accel = dv / max(t_trial, 1e-3)
                self._results.append(
                    TrialResult(v0=self._v0, v1=self._v1, current_a=current_a, t_sec=t_trial, accel_mps2=accel, reached=True)
                )
                if self._csv_writer is not None:
                    self._csv_writer.writerow([f'{self._v0:.3f}', f'{self._v1:.3f}', f'{current_a:.3f}', f'{t_trial:.4f}', f'{accel:.4f}', '1', self._current_field, self._speed_field])

                self.get_logger().info(
                    f'[OK] {self._v0:.1f}->{self._v1:.1f} m/s @ {current_a:.1f}A: t={t_trial:.3f}s, a={accel:.3f} m/s^2'
                )

                self._current_idx += 1
                self._phase = 'RESET'
                self._phase_t0 = now
                self._stable_time_acc = 0.0
                return

            # Timeout
            if t_trial >= self.trial_timeout_sec:
                dv = self._v1 - self._v0
                accel = dv / max(t_trial, 1e-3)
                self._results.append(
                    TrialResult(v0=self._v0, v1=self._v1, current_a=current_a, t_sec=t_trial, accel_mps2=accel, reached=False)
                )
                if self._csv_writer is not None:
                    self._csv_writer.writerow([f'{self._v0:.3f}', f'{self._v1:.3f}', f'{current_a:.3f}', f'{t_trial:.4f}', f'{accel:.4f}', '0', self._current_field, self._speed_field])

                self.get_logger().warning(
                    f'[TIMEOUT] {self._v0:.1f}->{self._v1:.1f} m/s @ {current_a:.1f}A: t={t_trial:.3f}s (not reached)'
                )

                self._current_idx += 1
                self._phase = 'RESET'
                self._phase_t0 = now
                self._stable_time_acc = 0.0
                return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedIntervalAccelSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt -> stopping')
        node._publish_stop()
        node._write_results('KeyboardInterrupt')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
