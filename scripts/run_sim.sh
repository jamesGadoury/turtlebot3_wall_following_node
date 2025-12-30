#!/bin/bash
# Script to launch Gazebo simulation with Turtlebot3 world

set -e

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

# Source workspace setup if it exists
if [ -f "/home/lost/workspace/2025-ros-ws/install/setup.bash" ]; then
    source /home/lost/workspace/2025-ros-ws/install/setup.bash
fi

# Export required Turtlebot3 model
export TURTLEBOT3_MODEL=burger

echo "Launching Turtlebot3 Gazebo simulation..."
echo "Model: $TURTLEBOT3_MODEL"
echo ""

# Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=-2.0 y_pose:=-1.0
