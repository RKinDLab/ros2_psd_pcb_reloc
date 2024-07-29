#!/bin/bash

# Excellent tutorial https://www.youtube.com/watch?v=3GbrKQ7G2P0&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&index=3

# ------------------------ ROS2 Network Settings for base station-----------------

# Add Python installation path to the PATH variable
export PATH=$PATH:/usr/bin/python3

# Source ROS Humble primitives and packages from main workspace
source /opt/ros/humble/setup.bash

# Source colcon related variables and files
# source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Find path to home directory
HOME_DIRECTORY=$HOME

# Type in the name of the workspace you want to use
# Make sure the "/" symbol is placed before the name of the workspace
#ROS2_WS="/ros2_ws"
ROS2_WS="/ros2_psd_pcb_reloc"

# Define the name of the workspace
# Export allows other programs to access $ROS2_WS_PATH$ variable
export ROS2_WS_PATH="$HOME_DIRECTORY$ROS2_WS" 

# Source the workspace.
source "$ROS2_WS_PATH/install/setup.bash"

# Local ROS Master
#export ROS_IP=167.96.128.47
export ROS_MASTER_URI=http://localhost:11311


echo ""
echo -e "ROS2_WS_PATH: $ROS2_WS_PATH"
echo -e "ROS NETWORK CONFIGURATION"
echo -e "ROS_MASTER_URI\t= $ROS_MASTER_URI" 
# echo -e "ROS_IP\t= $ROS_IP" 
echo ""

# ------------------------ ROS Network Settings for base station-----------------
