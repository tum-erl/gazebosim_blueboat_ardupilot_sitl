#!/usr/bin/env bash
set -e

# Source ROS base entrypoint if present
[ -f /ros_entrypoint.sh ] && source /ros_entrypoint.sh

# Source your workspaces/scripts
[ -f /home/blueboat_sitl/colcon_ws/install/setup.bash ] && source /home/blueboat_sitl/colcon_ws/install/setup.bash
[ -f /home/blueboat_sitl/gz_ws/install/setup.bash ] && source /home/blueboat_sitl/gz_ws/install/setup.bash
[ -f /home/blueboat_sitl/gz_ws/gazebo_exports.sh ] && source /home/blueboat_sitl/gz_ws/gazebo_exports.sh

exec "$@"