#!/bin/bash
# Script to launch RViz with custom wall following visualization configuration

set -e

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

# Path to RViz config file
RVIZ_CONFIG="$PACKAGE_DIR/rviz/view_workflow.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "ERROR: RViz config file not found at: $RVIZ_CONFIG"
    exit 1
fi

echo "Starting RViz with custom configuration..."
echo "Config file: $RVIZ_CONFIG"
echo ""

# Launch RViz with custom config
rviz2 -d "$RVIZ_CONFIG"
