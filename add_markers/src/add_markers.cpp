/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <utility>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
ros::Publisher marker_pub;
visualization_msgs::Marker marker;
float x,y;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
{
  x = amcl_msg->pose.pose.position.x;
  y = amcl_msg->pose.pose.position.y; 
}

void showMarker(float x, float y)
{
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;

	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

    marker.action = visualization_msgs::Marker::ADD;
	marker_pub.publish(marker);

}

void deleteMarker()
{
	marker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker);
}

double distance(double x1, double x2, double y1, double y2)
{
	return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
 
  ros::Subscriber sub1 = n.subscribe("/amcl_pose", 100, poseCallback);
  marker.header.frame_id = "/map";
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE; 
    bool isPickupZoneReached = false;
    bool isDropoutZoneReached = false;
    while(ros::ok())
	{
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		
		if(!isPickupZoneReached && !isDropoutZoneReached)
		{
			showMarker(-7.43f, 11.251f);
			if(distance(x, -7.43, y, 11.251)<1)
			{
				sleep(5);
				deleteMarker();
				isPickupZoneReached = true;
			}		
		} 
		else if(isPickupZoneReached && !isDropoutZoneReached)
		{
			if(distance(x, -2.0, y, 1.0)<1)
			{
				isDropoutZoneReached = true;
                
			}
		}
		else if(isPickupZoneReached && isDropoutZoneReached)
		{
			showMarker(-2.0f, 1.0f);
			return 0;
		}
	    ros::spinOnce();
    	r.sleep();
	}

}

