# MIT License

# Copyright (c) 2024 Zhihao Zhang

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_system'),
        'params',
        'vesc.yaml'
    )


    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')

    mux_config = os.path.join(
        get_package_share_directory('f1tenth_system'),
        'params',
        'mux.yaml'
    )
    # ekf_config = os.path.join(
    #     get_package_share_directory('f1tenth_system'),
    #     'params',
    #     'ekf.yaml'
    # )
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    # ekf_la = DeclareLaunchArgument(
    #     'ekf_config',
    #     default_value=ekf_config,
    #     description='Descriptions for ackermann mux configs')
    ld = LaunchDescription([vesc_la,mux_la])


    crsf_receiver_node = Node(
        package='crsf_receiver',
        executable='crsf_receiver_node',
        name='crsf_receiver_node',
        parameters=[
            {'device': '/dev/ttyTHS1'},
            {'baud_rate': 420000},
            {'link_stats': True}
        ],
        output='screen'
    )

    imu_driver=Node(
    package="wheeltec_n100_imu",
    executable="imu_node",
    parameters=[{'if_debug_': False,
        'serial_port':'/dev/ttyIMU',
        'serial_baud':921600}],
    output="screen"
    )

    ################### user configure parameters for ros2 start ###################
    xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'laser'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    # cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    # cur_config_path = cur_path + '../config'
    # user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
    user_config_path = os.path.join(get_package_share_directory("f1tenth_system"), 'params', 'MID360_config.json')
    ################### user configure parameters for ros2 end #####################

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    lidar_driver = Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=livox_ros2_params
            )
    
    joystick_control_node = Node(
        package='ackermann_mux',
        executable='joystick_control.py',
        name='joystick_control_node',
        output='screen'
    )
    
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("f1tenth_system"), 'params', 'ekf.yaml')],
    )
    
    
    lio_config_path = os.path.join(get_package_share_directory("f1tenth_system"), 'params', 'mid360_lio.yaml')


    lio=Node(
    package="arps_lidar_inertial_odometry",
    executable="lio_node",
    parameters=[{'config_file': lio_config_path}]
    )
        
    static_tf_node_bl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.13', '0.0', '0.03', '0.0', '0.261799', '0.0', 'base_link', 'laser']
    )
    static_tf_node_mo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )
    static_tf_node_bi = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.00', '0.0', '0.05', '0.0', '0.0', '0.7071068', '0.7071068', 'base_link', 'imu_link']
    )
    base_footprint_to_base_link = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       name="base_link_to_base_footprint",
                       arguments = ["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"])
    ld.add_action(base_footprint_to_base_link)
    # finalize
 
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(crsf_receiver_node)
    ld.add_action(joystick_control_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(imu_driver)
    ld.add_action(lidar_driver)
    ld.add_action(robot_localization_node)
    ld.add_action(lio)



    ld.add_action(static_tf_node_bl)
    # ld.add_action(static_tf_node_mo)
    ld.add_action(static_tf_node_bi)

    return ld
