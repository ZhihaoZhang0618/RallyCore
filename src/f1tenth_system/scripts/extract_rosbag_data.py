#!/usr/bin/env python3
"""
Extract calibration data from rosbag file to CSV format

This script reads a rosbag recording from the calibration run and extracts
relevant messages into a CSV file for post-processing.

Usage:
  python3 extract_rosbag_data.py --bag calibration.db3 --output calib_data.csv
"""

import argparse
from pathlib import Path
import sys
from datetime import datetime

try:
    from rosbag2_py import SequentialReader
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Error: rosbag2_py not installed.")
    print("Install with: sudo apt install ros-humble-rosbag2")
    sys.exit(1)


def extract_calib_data(bag_path, output_csv):
    """
    Extract calibration data from rosbag to CSV.
    
    Args:
        bag_path: Path to rosbag directory
        output_csv: Output CSV file path
    """
    print(f"Reading rosbag from: {bag_path}")
    print(f"Output CSV: {output_csv}")
    
    # Initialize reader
    reader = SequentialReader()
    reader.open(str(bag_path))
    
    # Storage info
    storage_filter = reader.storage_filter
    storage_filter.topics = ['/odom', '/calib/ackermann_cmd']
    reader.set_filter(storage_filter)
    
    # Data collection
    timestamps = []
    odom_data = {'x': [], 'y': [], 'vx': [], 'vy': [], 'yaw': []}
    cmd_data = {'current': [], 'steering': []}
    
    # Track data synchronization
    last_odom_time = None
    last_cmd_time = None
    odom_idx = 0
    cmd_idx = 0
    
    msg_count = 0
    
    while reader.has_next():
        topic, msg, t_nsec = reader.read_next()
        msg_count += 1
        
        if msg_count % 1000 == 0:
            print(f"  Processing... ({msg_count} messages)")
        
        t_sec = t_nsec / 1e9  # Convert to seconds
        
        if topic == '/odom':
            # Odometry message processing
            odom_data['x'].append(msg.pose.pose.position.x)
            odom_data['y'].append(msg.pose.pose.position.y)
            odom_data['vx'].append(msg.twist.twist.linear.x)
            odom_data['vy'].append(msg.twist.twist.linear.y)
            
            # Extract yaw from quaternion
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            yaw = _quat_to_yaw(qx, qy, qz, qw)
            odom_data['yaw'].append(yaw)
            
            last_odom_time = t_sec
            odom_idx += 1
        
        elif topic == '/calib/ackermann_cmd':
            # Command message processing
            cmd_data['current'].append(msg.drive.acceleration)  # Motor current
            cmd_data['steering'].append(msg.drive.steering_angle)
            
            last_cmd_time = t_sec
            cmd_idx += 1
    
    print(f"Total messages: {msg_count}")
    print(f"Odometry messages: {odom_idx}")
    print(f"Command messages: {cmd_idx}")
    
    # Synchronize data and write CSV
    print(f"\nSynchronizing data...")
    
    if odom_idx == 0:
        print("Error: No odometry data found in rosbag!")
        return False
    
    # Compute elapsed time from first message
    if cmd_idx > 0:
        min_len = min(odom_idx, cmd_idx)
        
        # Calculate accelerations from velocity
        accelerations = []
        for i in range(min_len):
            if i == 0:
                ax = 0.0
            else:
                # Simple finite difference
                dt = 0.02  # Assuming 50 Hz
                vx_prev = odom_data['vx'][i-1]
                vx_curr = odom_data['vx'][i]
                ax = (vx_curr - vx_prev) / dt
            
            accelerations.append(ax)
        
        # Calculate cross-track error (simplified: distance from origin)
        cross_track_errors = []
        for i in range(min_len):
            cte = (odom_data['x'][i]**2 + odom_data['y'][i]**2)**0.5
            cross_track_errors.append(cte)
        
        # Write CSV
        try:
            with open(output_csv, 'w') as f:
                # Header
                f.write("timestamp,current,velocity,acceleration,steering_angle,cross_track_error\n")
                
                # Data rows
                for i in range(min_len):
                    velocity = (odom_data['vx'][i]**2 + odom_data['vy'][i]**2)**0.5
                    timestamp = i * 0.02  # 50 Hz
                    
                    f.write(f"{timestamp:.3f},{cmd_data['current'][i]:.1f},"
                           f"{velocity:.4f},{accelerations[i]:.4f},"
                           f"{cmd_data['steering'][i]:.4f},"
                           f"{cross_track_errors[i]:.4f}\n")
            
            print(f"Data saved to: {output_csv}")
            print(f"Total rows: {min_len}")
            return True
            
        except Exception as e:
            print(f"Error writing CSV: {e}")
            return False
    
    else:
        print("Error: No command data found in rosbag!")
        return False


def _quat_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle"""
    import math
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy**2 + qz**2)
    )
    return yaw


def main():
    parser = argparse.ArgumentParser(
        description='Extract F1TENTH calibration data from rosbag'
    )
    parser.add_argument('--bag', type=str, required=True,
                       help='Path to rosbag directory')
    parser.add_argument('--output', type=str, default='calib_data.csv',
                       help='Output CSV file path')
    
    args = parser.parse_args()
    
    success = extract_calib_data(args.bag, args.output)
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
