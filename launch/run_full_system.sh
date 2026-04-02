#!/bin/bash
source /home/mie_g28/venv/bin/activate
source /opt/ros/<ROS_DISTRO>/setup.bash   # replace <ROS_DISTRO> with e.g. humble, jazzy
source ~/ros2_ws/install/setup.bash
ros2 launch ScanToCAD full_system.launch.py "$@"
