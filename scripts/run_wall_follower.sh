#!/bin/bash
# Script to run wall following node with sim_time and debug logging

set -e

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

# Source workspace setup
if [ -f "/home/lost/workspace/2025-ros-ws/install/setup.bash" ]; then
    source /home/lost/workspace/2025-ros-ws/install/setup.bash
else
    echo "ERROR: Workspace not built. Please run 'colcon build' first."
    exit 1
fi

echo "Starting wall following node..."
echo "Configuration:"
echo "  - use_sim_time: true"
echo "  - log_level: DEBUG (for wall_follower)"
echo ""

# Run wall following node with sim_time and debug logs
ros2 run turtlebot3_wall_following_node turtlebot3_wall_following_node \
    --ros-args \
    -p use_sim_time:=true \
    --log-level wall_follower:=DEBUG
