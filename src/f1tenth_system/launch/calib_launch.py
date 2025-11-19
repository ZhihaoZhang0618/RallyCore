#!/usr/bin/env python3
"""
Launch file for current-acceleration calibration with physics-based modeling.

Supports both acceleration and braking calibration modes with three-tier speed collection.

Usage:
  ros2 launch f1tenth_system calib_launch.py
  
Optional arguments:
  calibration_mode:=acceleration  # Default: acceleration calibration
  calibration_mode:=braking       # Braking calibration (negative current)
  figure8_radius:=1.6             # Figure-8 radius (m), increased from 1.4
  vehicle_mass:=6.0               # Vehicle mass (kg)
  command_frequency:=50           # Control loop frequency (Hz)
  
Example:
  ros2 launch f1tenth_system calib_launch.py calibration_mode:=braking
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for physics-based calibration node."""
    
    # Declare launch arguments
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='acceleration',
        description='Calibration mode: "acceleration" (default) or "braking"'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.33',
        description='Wheelbase of the vehicle (meters)'
    )
    
    lookahead_gain_arg = DeclareLaunchArgument(
        'lookahead_gain',
        default_value='1.5',
        description='Pure Pursuit lookahead gain (ld = k*v + min_lookahead)'
    )
    
    figure8_radius_arg = DeclareLaunchArgument(
        'figure8_radius',
        default_value='1.6',
        description='Figure-8 trajectory radius (meters), increased from 1.4 to avoid wheel slip'
    )
    
    command_frequency_arg = DeclareLaunchArgument(
        'command_frequency',
        default_value='50',
        description='Control loop frequency (Hz)'
    )
    
    vehicle_mass_arg = DeclareLaunchArgument(
        'vehicle_mass',
        default_value='6.0',
        description='Vehicle mass (kg) for physics model'
    )
    
    # Current-acceleration calibration node with physics-based modeling
    calib_node = Node(
        package='f1tenth_system',
        executable='current_acc_calib.py',
        name='current_acc_calib',
        output='screen',
        parameters=[
            {
                'calibration_mode': LaunchConfiguration('calibration_mode'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'lookahead_gain': LaunchConfiguration('lookahead_gain'),
                'figure8_radius': LaunchConfiguration('figure8_radius'),
                'command_frequency': LaunchConfiguration('command_frequency'),
                'vehicle_mass': LaunchConfiguration('vehicle_mass'),
            }
        ],
        remappings=[
            ('/odom', '/odom'),  # EKF-filtered odometry
            ('/vesc/sensors', '/vesc/sensors'),  # VESC state (ERPM)
            ('/calib/ackermann_cmd', '/calib/ackermann_cmd'),  # Output command
        ]
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(calibration_mode_arg)
    ld.add_action(wheelbase_arg)
    ld.add_action(lookahead_gain_arg)
    ld.add_action(figure8_radius_arg)
    ld.add_action(command_frequency_arg)
    ld.add_action(vehicle_mass_arg)
    
    # Add info messages
    ld.add_action(LogInfo(msg=['Starting Physics-Based Current-Acceleration Calibration']))
    ld.add_action(LogInfo(msg=['Calibration Mode: ', LaunchConfiguration('calibration_mode')]))
    ld.add_action(LogInfo(msg=['Figure-8 Radius: ', LaunchConfiguration('figure8_radius'), ' m']))
    ld.add_action(LogInfo(msg=['Vehicle Mass: ', LaunchConfiguration('vehicle_mass'), ' kg']))
    
    # Add node
    ld.add_action(calib_node)
    
    return ld
