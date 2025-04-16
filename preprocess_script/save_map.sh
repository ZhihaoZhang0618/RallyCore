#!/bin/bash

# Set default save directory if no argument is provided
DEFAULT_SAVE_DIR="$HOME/maps/$(date +%Y%m%d_%H%M%S)"
SAVE_DIR="${1:-$DEFAULT_SAVE_DIR}"

# Create directory if it doesn't exist
mkdir -p "$SAVE_DIR"

echo "Saving maps to: $SAVE_DIR"

# Call the ROS2 service to save maps
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '$SAVE_DIR', save_patches: true}"

# Check if the service call was successful
if [ $? -eq 0 ]; then
    echo "Maps successfully saved to: $SAVE_DIR"
else
    echo "Error: Failed to save maps"
    exit 1
fi