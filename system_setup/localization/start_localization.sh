#!/bin/bash
# Starts FusionCore (IMU + wheel odometry + stereo visual odometry) with fusioncore_config.yaml.
set -e
source /opt/ros/jazzy/setup.bash
[ -f /fusioncore_ws/install/setup.bash ] && source /fusioncore_ws/install/setup.bash

exec ros2 launch fusioncore_ros fusioncore.launch.py \
    fusioncore_config:=/config/fusioncore_config.yaml
