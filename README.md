# GA_Capstone_2026
General Atomics CPE Capstone Group, working with a YAHBOOM ROSMASTER A1 built around a Jetson Nano. This repository will be used to house ROS2 code

`/scripts`:
- Scripts called from within `run_humble.sh` that are used to source all ROS files - the Yahboom ones and the ones from this Repo - when entering the Docker container

`/src`:
- Where we will put all of our ROS code. If you put a package in this directory which has the same name as a package in the Yahboom workspace, THIS one will be used instead of the Yahboom one. That's because the scripts intentionally source this Repo's files after Yahboom. This will be nice if we wanted to create an augmented version of LineFollowing, for example
