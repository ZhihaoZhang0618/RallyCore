from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # VESC config
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_system'),
        'params',
        'vesc.yaml'
    )

    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs'
    )

    # Localizer config
    localizer_config = os.path.join(
        get_package_share_directory('f1tenth_system'),
        'params',
        'pointlio_localizer.yaml'
    )

    localizer_la = DeclareLaunchArgument(
        'localizer_config',
        default_value=localizer_config,
        description='Configuration for Localizer node'
    )

    # Point-LIO config (使用 f1tenth_system 的配置，与 FAST-LIO2 保持相同的外参)
    pointlio_config = os.path.join(
        get_package_share_directory('f1tenth_system'),
        'params',
        'pointlio.yaml'
    )

    pointlio_la = DeclareLaunchArgument(
        'pointlio_config',
        default_value=pointlio_config,
        description='Configuration YAML for Point-LIO'
    )

    ld = LaunchDescription([vesc_la, pointlio_la, localizer_la])

    # CRSF Receiver (ELRS遥控器接收器)
    crsf_receiver_node = Node(
        package='crsf_receiver',
        executable='crsf_receiver_node',
        name='crsf_receiver_node',
        parameters=[
            {'device': '/dev/ttyELRS'},
            {'baud_rate': 420000},
            {'link_stats': True}
        ],
        output='screen'
    )

    # IMU Driver
    imu_driver = Node(
        package="wheeltec_n100_imu",
        executable="imu_node",
        parameters=[{
            'if_debug_': False,
            'serial_port': '/dev/ttyIMU',
            'serial_baud': 921600
        }],
        output="screen"
    )

    ################### Livox LiDAR Configuration ###################
    # Point-LIO 使用 Livox CustomMsg 格式，必须设置 xfer_format = 1
    xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'laser'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    user_config_path = os.path.join(
        get_package_share_directory("f1tenth_system"),
        'params',
        'MID360_config.json'
    )

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

    # 使用 joystick_control_v2.py（集成了mux功能）
    joystick_control_v2_node = Node(
        package='ackermann_mux',
        executable='joystick_control_v2.py',
        name='joystick_control_v2',
        output='screen',
        parameters=[
            # 遥控器通道配置
            {'speed_channel': 3},
            {'steering_channel': 4},
            {'channel8_min_value': 191},
            {'channel8_max_value': 1792},

            # 速度模式参数
            {'speed_channel8_min_speed': 2.0},
            {'speed_channel8_max_speed': 12.0},

            # 电流模式参数
            {'current_channel8_min_current': 3.0},
            {'current_channel8_max_current': 120.0},

            # 转向参数
            {'steering_limit': 0.40},
            {'steering_reverse': True},
            {'steering_channel_mid': 984},
            {'channel_deadzone': 100},

            # 方向反转
            {'direction_reverse': False},
        ],
        remappings=[
            # joystick_control_v2输出 -> vesc输入
            ('/ackermann_cmd', '/ackermann_cmd'),
        ]
    )

    # VESC驱动节点
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

    # Point-LIO (替换 FAST-LIO2；不使用 EKF)
    pointlio_params = [
        LaunchConfiguration('pointlio_config'),
        {
            'use_imu_as_input': False,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,
            'space_down_sample': True,
            'filter_size_surf': 0.5,
            'filter_size_map': 0.5,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
        }
    ]

    lio_point_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=pointlio_params
    )

    # Localizer (重定位节点，保留)
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[{'config_path': LaunchConfiguration('localizer_config')}]
    )

    # 静态TF变换
    static_tf_node_bl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.13', '0.0', '0.03', '0.0', '0.261799', '0.0', 'base_link', 'laser']
    )

    static_tf_node_bi = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.00', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    base_footprint_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_footprint",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
    )

    # 添加所有节点到LaunchDescription
    ld.add_action(base_footprint_to_base_link)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(crsf_receiver_node)
    ld.add_action(joystick_control_v2_node)
    ld.add_action(imu_driver)
    ld.add_action(lidar_driver)
    # 移除 EKF；改为 Point-LIO
    ld.add_action(static_tf_node_bl)
    ld.add_action(static_tf_node_bi)
    ld.add_action(lio_point_node)
    # ld.add_action(localizer_node)

    return ld
