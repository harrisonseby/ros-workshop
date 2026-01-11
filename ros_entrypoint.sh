#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/humble/setup.bash

# Source workspace setup if it exists
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

# Execute the command
exec "$@"
