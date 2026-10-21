#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
cd /root/share/ga-taxi
colcon build --symlink-install
exec bash --rcfile /root/share/ga-taxi/scripts/container_rc.sh
