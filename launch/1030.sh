#!/bin/bash

source /opt/ros/indigo/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

export ROS_PACKAGE_PATH=/home/amsl/ros_catkin_ws:/home/amsl/AMSL_ros_pkg:$ROS_PACKAGE_PATH
export ROS_WORKSPACE=/home/amsl/ros_catkin_ws

sleep 2s

roscore &
rosbag play 201510281204_odom_ang_scan_30rpm -l -s 50 -u 90 &
roslaunch senior_mapping 1030.launch
