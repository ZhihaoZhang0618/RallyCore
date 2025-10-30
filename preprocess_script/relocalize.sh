#!/bin/bash

# ./relocalize.sh ~/maps/20240416_211810/map.pcd

# Function to get latest map folder
get_latest_map_folder() {
    base_path="$HOME/maps"
    latest_folder=$(ls -td $base_path/*/ | head -1)
    echo "$latest_folder"
}

# Set default map path if not provided
if [ $# -eq 0 ]; then
    LATEST_FOLDER=$(get_latest_map_folder)
    if [ -z "$LATEST_FOLDER" ]; then
        echo "Error: No map folder found in ~/maps"
        echo "Usage: $0 [pcd_map_path]"
        echo "Example: $0 ~/maps/20240416_211810/map.pcd"
        exit 1
    fi
    PCD_PATH="${LATEST_FOLDER}map.pcd"
    echo "Using latest map: $PCD_PATH"
else
    PCD_PATH="$1"
fi

echo "Relocalizing with map: $PCD_PATH"
echo "Using default pose: x=0.0, y=0.0, z=0.0, yaw=0.0, pitch=0.0, roll=0.0"

# Call the ROS2 service to relocalize
ros2 service call /localizer/relocalize interface/srv/Relocalize "{
    pcd_path: '$PCD_PATH',
    x: 0.0,
    y: 0.0,
    z: 0.0,
    yaw: 0.0,
    pitch: 0.0,
    roll: 0.0
}"

# Check if the service call was successful
if [ $? -eq 0 ]; then
    echo "Relocalization successful"
else
    echo "Error: Failed to relocalize"
    exit 1
fi 