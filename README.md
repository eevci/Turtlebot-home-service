# Turtlebot Home Service

## 1) Introduction
It is a Udacity project that I have finished. You can test mapping, localization and path planning algorithms with the given tasks and scripts.

## 2) Packages
* slam_gmapping
* turtlebot_rviz_launchers
* turtlebot_gazebo
* turtlebot_teleop
* pick_objects
* add_markers 

packages are used in this project.

## 3) Tests
### SLAM testing
* slam_gmapping
* turtlebot_rviz_launchers
* turtlebot_gazebo
* turtlebot_teleop
 
 are used. slam_gmapping is used for laser-based slam algorithm. 
 turtlebot_teleop is used for controlling turtlebot.

We can create a map for the given world.

### Navigation testing
* turtlebot_rviz_launchers
* turtlebot_gazebo are used. 

amcl_demo.launch and turtlebot_world.launch are both the launch files from turtlebot_gazebo. amcl_demo is used for localization. 
AMCL algorithm parameters are tuned for better localization. 

### Home Service testing

* turtlebot_rviz_launchers
* turtlebot_gazebo
* add_markers
* pick_objects 

are used.

In the add_markers package src there are two files. Both have the same node name "add_markers" but filenames are different in order to split the tasks that is assigned. 

check_markers does;

	Publish the marker at the pickup zone
	Pause 5 seconds
	Hide the marker
	Pause 5 seconds
	Publish the marker at the drop off zone  

add_markers does;

	Initially show the marker at the pickup zone
	Hide the marker once your robot reaches the pickup zone
	Wait 5 seconds to simulate a pickup
	Show the marker at the drop off zone once your robot reaches it

## 4) How to Start
Take these ROS packages into your catkin workspace and call;

`catkin_make`

You can find the scripts in order to start tests mentioned above.