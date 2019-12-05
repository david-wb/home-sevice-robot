#!/bin/bash

set -e pipefail

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

rosparam set /home_service/pickup_x 5.0
rosparam set /home_service/pickup_y 3.0
rosparam set /home_service/dropoff_x -5.0
rosparam set /home_service/dropoff_y -4.0

xterm  -e " roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " rosrun add_markers add_markers" &
sleep 5
rostopic pub /robot_status std_msgs/Int8 1 --once
sleep 5
rostopic pub /robot_status std_msgs/Int8 2 --once
rostopic pub /robot_status std_msgs/Int8 3 --once

echo "Press any key to continue."
read -n 1 