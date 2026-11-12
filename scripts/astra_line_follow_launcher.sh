#!/bin/bash
# Launch Astra line following on the Nano: opens the container, then lidar + follow_line windows.
# Usage: astra_line_follow_launcher.sh [DELAY_SECONDS]   (default 3)

IMAGE=192.168.2.51:5000/ros-humble-a1:1.1.1
RC=/root/share/GA_Capstone_2026/scripts/container_rc.sh

container() { docker ps -q --filter "ancestor=$IMAGE" | head -n1; }

# Start the container (pull/build happens in this window) if it isn't already running
if [ -z "$(container)" ]; then
    gnome-terminal --title=container -- bash "${RUN_HUMBLE:-$HOME/run_humble.sh}"
    until [ -n "$(container)" ]; do sleep 1; done
fi
C=$(container)

# Wait for the colcon build to finish (container_entry.sh then execs "bash --rcfile ...")
until docker exec "$C" pgrep -f "bash --rcfile" >/dev/null 2>&1; do sleep 1; done

run() {  # run <title> <ros command>
    gnome-terminal --title="$1" -- docker exec -it "$C" bash -c "source $RC; $2; exec bash --rcfile $RC"
}

run lidar       "ros2 launch yahboomcar_bringup laser_bringup_launch.py"
sleep "${1:-3}"
run line_follow "ros2 run yahboomcar_astra follow_line"
