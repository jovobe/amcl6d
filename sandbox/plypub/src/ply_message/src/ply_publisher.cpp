#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/String.h"

#include <iostream>

int main(int argc, char** args)
{
	std::string node_name = "ply_map";
	int maximum_msg_size = 1000;

	ros::init(argc, args, node_name);

	ros::NodeHandle node_handle;
	
//	ros::Publisher publisher = node_handle.advertise<sensor_msgs::PointCloud>(node_name, maximum_msg_size);
	ros::Publisher publisher = node_handle.advertise<std_msgs::String>(node_name, maximum_msg_size);
	
	ros::Rate loop_rate(10);
	
	int count = 0;
	while(ros::ok()) 
	{
		std_msgs::String text_msg;
//		sensor_msgs::PointCloud pcl;
		text_msg.data = "hello";
		ROS_INFO("%s", text_msg.data.c_str());

		publisher.publish(text_msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;

}
