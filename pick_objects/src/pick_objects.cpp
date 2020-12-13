#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


actionlib::SimpleClientGoalState gotoPosition(geometry_msgs::Pose goalPose, MoveBaseClient& ac)
{
	move_base_msgs::MoveBaseGoal goal;
	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose = goalPose;
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	// Wait an infinite time for the results
	ac.waitForResult();
	return ac.getState();
}

actionlib::SimpleClientGoalState gotoPosition(float x, float y, MoveBaseClient& ac)
{
	geometry_msgs::Pose goalPose1;

	goalPose1.position.x = x;
	goalPose1.position.y = y;
	goalPose1.orientation.w = 1.0;
	return gotoPosition(goalPose1, ac);
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  float goal1x = -7.43f;
  float goal1y = 11.251f;


  actionlib::SimpleClientGoalState result = gotoPosition(goal1x, goal1y, ac);
  // Check if the robot reached its goal
  if(result == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("pickup zone reached!");
  else
    ROS_INFO("Path to the object couldn`t be found");

	sleep(5);
	float goal2x = -2.0f; 
	float goal2y = 1.0f;


  result = gotoPosition(goal2x, goal2y, ac);
  if(result == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal reached!");
  else
    ROS_INFO("Path to the object couldn`t be found");

  return 0;
}
