#!/bin/bash
# Launch everything needed for line following. Run on the Nano (e.g. from the NoMachine desktop).
# Window 1 runs run_humble.sh (starts the container, pulls, builds, leaves a sourced shell).
# Windows 2-4 are shells in that same container running: camera, lidar, depth_follow_line (last).
# Every window stays open after its command exits (or Ctrl-C).
#
# Usage: line_follow_launcher.sh [DELAY_SECONDS]   (default 3)
# Override the run_humble.sh location with: RUN_HUMBLE=/path/to/run_humble.sh

RUN_HUMBLE="${RUN_HUMBLE:-$HOME/run_humble.sh}"
IMAGE=192.168.2.51:5000/ros-humble-a1:1.1.1
DELAY="${1:-3}"
WS=/root/share/GA_Capstone_2026

running_container() { docker ps -q --filter "ancestor=$IMAGE" | head -n1; }

open_term() {  # open_term <title> <command...>
    local title="$1"; shift
    if command -v gnome-terminal >/dev/null; then gnome-terminal --title="$title" -- "$@"
    elif command -v xfce4-terminal >/dev/null; then xfce4-terminal --title="$title" -x "$@"
    elif command -v konsole >/dev/null; then konsole -p "tabtitle=$title" -e "$@"
    elif command -v x-terminal-emulator >/dev/null; then x-terminal-emulator -T "$title" -e "$@"
    elif command -v xterm >/dev/null; then xterm -T "$title" -e "$@"
    else echo "No supported terminal emulator found" >&2; exit 1
    fi
}

CONTAINER=$(running_container)
if [ -z "$CONTAINER" ]; then
    [ -x "$RUN_HUMBLE" ] || [ -f "$RUN_HUMBLE" ] || { echo "run_humble.sh not found at $RUN_HUMBLE (set RUN_HUMBLE)"; exit 1; }
    open_term "container" bash "$RUN_HUMBLE"
    echo "Waiting for container..."
    until CONTAINER=$(running_container); [ -n "$CONTAINER" ]; do sleep 1; done
fi

# container_entry.sh finishes its colcon build and then execs "bash --rcfile ..."; wait for that.
echo "Waiting for colcon build to finish..."
until docker exec "$CONTAINER" pgrep -f "bash --rcfile" >/dev/null 2>&1; do sleep 1; done

run_in_container() {  # run_in_container <title> <ros command>
    open_term "$1" docker exec -it "$CONTAINER" bash -c \
        "source $WS/scripts/container_rc.sh; $2; exec bash --rcfile $WS/scripts/container_rc.sh"
}

run_in_container "camera"      "ros2 launch ascamera hp60c.launch.py"
run_in_container "lidar"       "ros2 launch yahboomcar_bringup laser_bringup_launch.py"
sleep "$DELAY"
run_in_container "line_follow" "ros2 run yahboomcar_depth depth_follow_line"
