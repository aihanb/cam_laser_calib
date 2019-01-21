#include <ros/ros.h>
#include <iostream>
#include "std_msgs/string.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

void chatterCallback1(const sensor_msgs::ImageConstPtr& img)
{
	ROS_INFO("img");
	
}

void chatterCallback2(const sensor_msgs::PointCloud2ConstPtr& pc2)
{
	ROS_INFO("pc2");
	printf("%f\n", pc2->header.stamp.toSec());
	
}

int main(int argc, char** argv)
{
	ros::init(argc. argv, "listenner");
	ros::Subscriber sub = n.subscribe("/camera/image_color", 1000, chatterCallback1);
    ros::Subscriber sub1 = n.subscribe("/pandar_points", 1000, chatterCallback2);

	ros::spin();
	return 0;
}
