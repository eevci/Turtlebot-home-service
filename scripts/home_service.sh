#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/mnt/hgfs/robotics_engineer/home_service_ws/src/worlds/Enver.world " &
sleep 5 
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/mnt/hgfs/robotics_engineer/home_service_ws/src/map/WAIMap.yaml" & 
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
xterm  -e  " rosrun add_markers add_markers" &
xterm  -e  " rosrun pick_objects pick_objects" 
