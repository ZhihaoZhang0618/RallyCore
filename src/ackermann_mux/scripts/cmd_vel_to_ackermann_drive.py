#!/usr/bin/env python3

# Author: christoph.roesmann@tu-dortmund.de
# Modified by: zhangzhihao0618@gmail.com
#!/usr/bin/env python3

# Author: christoph.roesmann@tu-dortmund.de (Converted to ROS 2 by [Your Name])

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


class CmdVelToAckermannDrive(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_drive')
        
        self.twist_cmd_topic = self.declare_parameter('twist_cmd_topic', '/cmd_vel').value
        self.ackermann_cmd_topic = self.declare_parameter('ackermann_cmd_topic', '/drive').value
        self.wheelbase = self.declare_parameter('wheelbase', 0.33).value
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 或者使用 ReliabilityPolicy.BEST_EFFORT
            history=HistoryPolicy.KEEP_LAST,  # KEEP_LAST 表示只保留最后N条消息
            depth=10,  # depth 参数定义要保留的消息数量
        )

        self.subscriber = self.create_subscription(
            Twist,
            self.twist_cmd_topic,
            self.cmd_callback,
            qos_profile
        )
        
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            self.ackermann_cmd_topic,
            10
        )
        
        self.get_logger().info(f"Node 'cmd_vel_to_ackermann_drive' started.\nListening to {self.twist_cmd_topic}, publishing to {self.ackermann_cmd_topic}. wheelbase: {self.wheelbase}")
    
    def cmd_callback(self, data):
        v = data.linear.x
        steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, self.wheelbase)
        
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(v)
        
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermannDrive()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
