#!/bin/bash
export QT_QPA_PLATFORM=xcb
source /opt/ros/jazzy/setup.bash
source /home/b/RemotePiRos2/install/setup.bash
ros2 run remote_pi_pkg main
