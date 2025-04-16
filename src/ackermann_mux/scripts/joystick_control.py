#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from crsf_receiver_msg.msg import CRSFChannels16
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class JoystickControl(Node):
    def __init__(self):
        super().__init__("joystick_control")
        self.get_logger().info("joystick_control started")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 或者使用 ReliabilityPolicy.BEST_EFFORT
            history=HistoryPolicy.KEEP_LAST,  # KEEP_LAST 表示只保留最后N条消息
            depth=10,  # depth 参数定义要保留的消息数量
        )
        # 订阅CRSFChannels16消息
        self.subscription_joystick = self.create_subscription(
            CRSFChannels16, "/rc/channels", self.joystick_callback, qos_profile
        )

        self.teleop_publisher = self.create_publisher(
            AckermannDriveStamped, "teleop", 10
        )

        self.speed_limit = self.declare_parameter("speed_limit", 3.0).value
        self.speed_reverse = self.declare_parameter("speed_reverse", False).value
        self.speed_channel = self.declare_parameter("speed_channel", 3).value # channel x is x, don't worry
        self.steering_channel = self.declare_parameter("steering_channel", 4).value
        self.steering_limit = self.declare_parameter("steering_limit", 0.40).value
        self.steering_reverse = self.declare_parameter("steering_reverse", True).value
        self.channel_mid = self.declare_parameter(
            "steering_channel_mid", 984
        ).value  # depand by servo channel, servo didn't have deadzone
        self.channel_deadzone = self.declare_parameter(
            "steering_channel_max", 100
        ).value
        self.channel_max_range = 2000
        self.channel_min_range = 0

        self.channel = None
        # 状态变量
        self.rc_connected = False
        self.locked = False
        self.control_mode = "none"
        self.last_joystick_time = self.get_clock().now()
        self.get_logger().info("speed_limit: %f" % self.speed_limit)
        self.get_logger().info("speed_reverse: %s" % self.speed_reverse)
        self.get_logger().info("speed_channel: %d" % self.speed_channel)
        self.get_logger().info("steering_channel: %d" % self.steering_channel)
        self.get_logger().info("steering_limit: %f" % self.steering_limit)
        self.get_logger().info("steering_reverse: %s" % self.steering_reverse)
        self.get_logger().info("steering_channel_mid: %d" % self.channel_mid)
        self.get_logger().info("steering_channel_max: %d" % self.channel_deadzone)
        # 200hz
        self.timer = self.create_timer(0.005, self.timer_callback)

    def joystick_callback(self, msg):
        # 假设通道5和6为bool值，具体实现可能需要调整
        # 200 992 1810
        self.channel = [
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

        self.locked = self.channel[5] < 1200
        self.control_mode = "teleop" if self.channel[7] < 1200 else "nav"

        self.last_joystick_time = self.get_clock().now()
        # self.get_logger().info("locked: %s" % self.locked)
        # self.get_logger().info("control_mode: %s" % self.control_mode)

    def publish_none(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.teleop_publisher.publish(msg)

    def timer_callback(self):
        if self.locked:
            self.publish_none()
            return

        if (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9 < 0.2:
            self.rc_connected = True
        else:
            self.rc_connected = False
            self.get_logger().warn("No joystick input")
            self.publish_none()
            if (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().warn("No joystick input for 5 seconds, shutting down...")
                rclpy.shutdown()
            return

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.control_mode == "teleop" and self.rc_connected:
            # 速度和转角控制
            raw_speed = self.channel[self.speed_channel]
            # speed deadzone
            if abs(raw_speed - self.channel_mid) > self.channel_deadzone:
                # Normalize speed value to -1 to 1
                normalized_speed = (raw_speed - self.channel_mid) / (
                    self.channel_max_range - self.channel_mid
                )
                msg.drive.speed = normalized_speed * self.speed_limit
                if self.speed_reverse:
                    msg.drive.speed = -msg.drive.speed
            else:
                msg.drive.speed = 0.0

            raw_steering = self.channel[self.steering_channel]
            # Normalize steering value to -1 to 1
            if raw_steering > self.channel_mid:
                normalized_steering = (raw_steering - self.channel_mid) / (
                    self.channel_max_range - self.channel_mid
                )
            else:
                normalized_steering = (raw_steering - self.channel_mid) / (
                    self.channel_mid - self.channel_min_range
                )
            steering_range = self.steering_limit
            steering_offset = normalized_steering * steering_range
            msg.drive.steering_angle = steering_offset
            if self.steering_reverse:
                msg.drive.steering_angle = -msg.drive.steering_angle
            self.teleop_publisher.publish(msg)
        elif self.control_mode == "nav":
            # 导航控制
            pass


def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    # Clean up
    joystick_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
