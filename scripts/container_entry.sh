#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
cd /root/share/GA_Capstone_2026
colcon build --symlink-install
exec bash --rcfile /root/share/GA_Capstone_2026/scripts/container_rc.sh
