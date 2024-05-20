#!/bin/bash
source ./install/setup.bash
ros2 run orb_slam3_ros2 vslam_node --ros-args -p agent_name:=robot0
