#!/bin/bash

set -e pipefail

xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e " roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch"
