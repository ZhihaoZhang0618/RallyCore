#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from crsf_receiver_msg.msg import CRSFChannels16
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32


class JoystickControl(Node):
    def __init__(self):
        super().__init__("joystick_control")
        self.get_logger().info("joystick_control started")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # 订阅CRSFChannels16消息
        self.subscription_joystick = self.create_subscription(
            CRSFChannels16, "/rc/channels", self.joystick_callback, qos_profile
        )
        
        self.ackermann_subscriber = self.create_subscription(
            AckermannDriveStamped,
            "/drive",
            self.ackermann_callback,
            qos_profile
        )

        self.ackermann_calib_subscriber = self.create_subscription(
            AckermannDriveStamped,
            "/calib/ackermann_cmd",
            self.calib_ackermann_callback,
            qos_profile
        )
        
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)

        self.channel8_min_value = self.declare_parameter("channel8_min_value", 191).value
        self.channel8_max_value = self.declare_parameter("channel8_max_value", 1792).value
        
        self.direction_reverse = self.declare_parameter("direction_reverse", False).value
        self.speed_channel = self.declare_parameter("speed_channel", 3).value # channel x is x, don't worry
        self.steering_channel = self.declare_parameter("steering_channel", 4).value

        self.speed_channel8_min_speed = self.declare_parameter("speed_channel8_min_speed", 2.0).value
        self.speed_channel8_max_speed = self.declare_parameter("speed_channel8_max_speed", 12.0).value
        self.steering_limit = self.declare_parameter("steering_limit", 0.40).value
        self.steering_reverse = self.declare_parameter("steering_reverse", True).value
        self.channel_mid = self.declare_parameter(
            "steering_channel_mid", 984
        ).value  # depand by servo channel, servo didn't have deadzone
        self.channel_deadzone = self.declare_parameter(
            "channel_deadzone", 100
        ).value
        self.channel_max_range = 2000
        self.channel_min_range = 0

        self.channel = None
        self.nav_ackermann_msg = None
        self.calib_ackermann_msg = None

        

        self.current_channel8_min_current = self.declare_parameter("current_channel8_min_current", 3.0).value
        self.current_channel8_max_current = self.declare_parameter("current_channel8_max_current", 20.0).value
        
        # 状态变量
        self.rc_connected = False
        self.locked = False
        self.control_mode = "none"  # 可能的值: "teleop", "nav", "none"
        self.esc_control_mode = "none"  # 可能的值: "speed", "current", "none"
        self.calib_mode = False
        self.last_joystick_time = self.get_clock().now()
        self.last_nav_time = self.get_clock().now()
        
        # 追踪状态变化，只在改变时print
        self.last_control_mode = "none"
        self.last_esc_control_mode = "none"
        self.last_calib_mode = False
        self.last_locked = False

        self.get_logger().info("direction_reverse: %s" % self.direction_reverse)
        self.get_logger().info("speed_channel: %d" % self.speed_channel)
        self.get_logger().info("steering_channel: %d" % self.steering_channel)
        self.get_logger().info("channel8_min_value: %d, max_value: %d" % (self.channel8_min_value, self.channel8_max_value))
        self.get_logger().info("speed_channel8_min_speed: %f, max_speed: %f" % (self.speed_channel8_min_speed, self.speed_channel8_max_speed))
        self.get_logger().info("steering_limit: %f" % self.steering_limit)
        self.get_logger().info("steering_reverse: %s" % self.steering_reverse)
        self.get_logger().info("steering_channel_mid: %d" % self.channel_mid)
        self.get_logger().info("channel_deadzone: %d" % self.channel_deadzone)
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
        # 修改：使用字符串表示控制模式
        self.control_mode = "teleop" if self.channel[7] < 1200 else "nav"
        
        if self.channel[6] < 600:
            self.esc_control_mode = "speed"
        elif self.channel[6] < 1200:
            self.esc_control_mode = "current"
        else:
            self.esc_control_mode = "duty"
            
        if self.channel[10] > 1000:
            self.calib_mode = True
        else:
            self.calib_mode = False
            
        self.last_joystick_time = self.get_clock().now()
        
        # 只在状态改变时print
        if self.control_mode != self.last_control_mode:
            self.get_logger().info("Control mode changed: %s -> %s" % (self.last_control_mode, self.control_mode))
            self.last_control_mode = self.control_mode
            
        if self.esc_control_mode != self.last_esc_control_mode:
            self.get_logger().info("ESC control mode changed: %s -> %s" % (self.last_esc_control_mode, self.esc_control_mode))
            self.last_esc_control_mode = self.esc_control_mode
            
        if self.calib_mode != self.last_calib_mode:
            self.get_logger().info("Calibration mode changed: %s -> %s" % (self.last_calib_mode, self.calib_mode))
            self.last_calib_mode = self.calib_mode
        
        if self.locked != self.last_locked:
            self.get_logger().info("Locked changed: %s -> %s" % (self.last_locked, self.locked))
            self.last_locked = self.locked
            
        if self.calib_mode and self.esc_control_mode != "current":
            self.get_logger().warn("Calibration model now & Please use current mode & Make sure everything is ready")


    def ackermann_callback(self, msg):
        self.nav_ackermann_msg = msg
        self.last_nav_time = self.get_clock().now()  # ✅ 添加这一行
        
    def calib_ackermann_callback(self, msg):
        self.calib_ackermann_msg = msg
    def handle_channel_input(self, channel_index,
                            channel8_min_value, channel8_max_value, 
                            channel8_min_range, channel8_max_range):
        raw_value = self.channel[channel_index]
        
        # 处理channel8缩放
        channel8_value = self.channel[8]
        channel8_value = max(channel8_min_value, min(channel8_value, channel8_max_value))
        ratio = (channel8_value - channel8_min_value) / (
            channel8_max_value - channel8_min_value
        )
        current_max = channel8_min_range + ratio * (channel8_max_range - channel8_min_range)
        current_min = -current_max

        # 处理死区
        if abs(raw_value - self.channel_mid) > self.channel_deadzone:
            # Normalize to -1 to 1
            if raw_value > self.channel_mid:
                normalized = (raw_value - self.channel_mid) / (
                    self.channel_max_range - self.channel_mid
                )
            else:
                normalized = (raw_value - self.channel_mid) / (
                    self.channel_mid - self.channel_min_range
                )
            
            # ✅ 直接乘以 current_max（normalized 已经是 -1 到 1）
            value = normalized * current_max
            
            # Clamp to min and max（其实已经不需要了，但保险起见可以保留）
            value = max(current_min, min(value, current_max))
        else:
            value = 0.0
            
        return value

    def handle_teleop_speed(self):
        speed_value = self.handle_channel_input(
            self.speed_channel,
            self.channel8_min_value,
            self.channel8_max_value,
            self.speed_channel8_min_speed,
            self.speed_channel8_max_speed
        )
        
        if self.direction_reverse:
            speed_value = -speed_value
            
        return speed_value
        
    def handle_teleop_current(self):
        current_value = self.handle_channel_input(
            self.speed_channel,
            self.channel8_min_value,
            self.channel8_max_value,
            self.current_channel8_min_current,
            self.current_channel8_max_current
        )
        
        if self.direction_reverse:
            current_value = -current_value
            
        return current_value


    def handle_teleop_duty(self):
        duty_value = self.handle_channel_input(
            self.speed_channel,
            self.channel8_min_value,
            self.channel8_max_value,
            0.0, # 0.0 to 1.0 min 
            0.8 # 0.0 to 1.0 max
        )
        if self.direction_reverse:
            duty_value = -duty_value
            
        return duty_value 
    def handle_teleop_steer(self):
    
        raw_steering = self.channel[self.steering_channel]
                
        if abs(raw_steering - self.channel_mid) <= self.channel_deadzone*0.2:
            return 0.0
        
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
        steering_value = normalized_steering * steering_range
        if self.steering_reverse:
            steering_value = -steering_value
        return steering_value

        
    def publish_ackermann_none(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.ackermann_publisher.publish(msg)

    def publish_ackermann(self, steering_angle, speed):
        # 常规steering_angle + speed
        # 用于遥控器 和 nav2导航模式
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0 # 标志位，用来区分是否是常规模式
        self.ackermann_publisher.publish(msg)

    def publish_ackermann_acceleration(self, steering_angle, speed,acceleration):
        # 加速度前馈模式 steering_angle + speed + acceleration
        # 用于mpc输出acc + speed
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = acceleration
        msg.drive.jerk = 1.0 # 标志位，用来区分是否是加速度前馈模式
        self.ackermann_publisher.publish(msg)

    def publish_ackermann_current(self, steering_angle,current):
        # 电流控制模式 steering_angle + current
        # 用于teleop模式 和 calibration模式
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = current
        msg.drive.jerk = 2.0 # 标志位，用来区分是否current直接驱动模式
        self.ackermann_publisher.publish(msg)

    def publish_ackermann_duty(self, steering_angle,duty):
        # 占空比控制模式 steering_angle + duty
        # 用于teleop模式 和 calibration模式
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = duty
        msg.drive.jerk = 3.0 # 标志位，用来区分是否duty直接驱动模式
        self.ackermann_publisher.publish(msg)

        
    def timer_callback(self):
        
        if self.channel is None:
            self.get_logger().warn("Waiting for RC input...")
            return

        if self.locked:
            self.publish_ackermann_none()
            return

        # 检查rc输入是否超时
        rc_timeout = (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9 > 0.2
        # 检查drive输入是否超时
        nav_timeout = (self.get_clock().now() - self.last_nav_time).nanoseconds / 1e9 > 0.2
        
        
        # 判断rc响应
        if not rc_timeout:
            self.rc_connected = True
        else:
            self.rc_connected = False
            self.get_logger().warn("No joystick input")
            self.publish_ackermann_none()
            if (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().warn("No joystick input for 5 seconds, shutting down...")
                rclpy.shutdown()
            return
            
        if self.calib_mode and self.esc_control_mode != "current" and not rc_timeout:
            self.publish_ackermann_none()
            return
        elif self.calib_mode and self.esc_control_mode == "current" and not rc_timeout:
            if self.calib_ackermann_msg != None:
                self.publish_ackermann_current(self.calib_ackermann_msg.drive.steering_angle, self.calib_ackermann_msg.drive.speed)
            elif self.calib_ackermann_msg == None:
                self.get_logger().warn("No calibration message")
            return
        
        if self.esc_control_mode == "speed":
            if not self.rc_connected:
                self.publish_ackermann_none()
                return

            if self.control_mode == "nav":
                if self.nav_ackermann_msg is None:
                    self.get_logger().warn("No nav message received yet")
                    self.publish_ackermann_none()
                    return
                elif nav_timeout:
                    self.get_logger().warn("No nav message received for 0.2 seconds")
                else:    
                    self.publish_ackermann(
                        self.nav_ackermann_msg.drive.steering_angle,
                        self.nav_ackermann_msg.drive.speed
                    )
            elif self.control_mode == "teleop":
                speed_value = self.handle_teleop_speed()
                steering_value = self.handle_teleop_steer()
                self.publish_ackermann(steering_value, speed_value)
        elif self.esc_control_mode == "current":
            if not self.rc_connected:
                self.publish_ackermann_none()
                return

            if self.control_mode == "nav":
                if self.nav_ackermann_msg is None:
                    self.get_logger().warn("No nav message received yet")
                    self.publish_ackermann_none()
                    return
                elif nav_timeout:
                    self.get_logger().warn("No nav message received for 0.2 seconds")
                else:    
                    self.publish_ackermann_acceleration(
                        self.nav_ackermann_msg.drive.steering_angle,
                        self.nav_ackermann_msg.drive.speed,
                        self.nav_ackermann_msg.drive.acceleration
                    )
                    
            elif self.control_mode == "teleop":
                current_value = self.handle_teleop_current()
                steering_value = self.handle_teleop_steer()
                self.publish_ackermann_current(steering_value, current_value)

        elif self.esc_control_mode == "duty":
            if not self.rc_connected:
                self.publish_ackermann_none()
                return
            
            if self.control_mode == "teleop":
                duty_value = self.handle_teleop_duty()
                steering_value = self.handle_teleop_steer()
                self.publish_ackermann_duty(steering_value, duty_value)
            elif self.control_mode == "nav":
                if self.nav_ackermann_msg is None:
                    self.get_logger().warn("No nav message received yet")
                    self.publish_ackermann_none()
                    return
                elif nav_timeout:
                    self.get_logger().warn("No nav message received for 0.2 seconds")
                else:    
                    self.get_logger().warn("Duty mode not supported for nav")
def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    # Clean up
    joystick_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
