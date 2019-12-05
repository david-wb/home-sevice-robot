#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

function finish {
    echo "Killing xterminals"
    killall xterm
}
trap finish EXIT

export TURTLEBOT_GAZEBO_WORLD_FILE="${DIR}/../home-service.world"
export TURTLEBOT_GAZEBO_MAP_FILE="${DIR}/../home-service.yaml"

cd ${DIR}/.. && catkin_make && source devel/setup.bash

xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm  -e " roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &

echo "Press any key to exit."
read -n 1 