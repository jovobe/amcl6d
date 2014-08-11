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
	
    ros::Publisher publisher = node_handle.advertise<sensor_msgs::PointCloud>(node_name, maximum_msg_size);
//	ros::Publisher publisher = node_handle.advertise<std_msgs::String>(node_name, maximum_msg_size);
	
    ros::Rate loop_rate(3000);
	
    // prepare point cloud
    sensor_msgs::PointCloud point_cloud;
//    point_cloud.header = ????
    point_cloud.points.resize(2);
    for(int i = 0; i < point_cloud.points.size(); i++)
    {
        point_cloud.points[i].x = i;
        point_cloud.points[i].y = 2*i;
        point_cloud.points[i].z = 0;
    }


    // publish point cloud
	while(ros::ok()) 
	{
//		std_msgs::String text_msg;
//        text_msg.data = "hello";
//        ROS_INFO("%s", text_msg.data.c_str());





        publisher.publish(point_cloud);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;

}
/*


  sensor_msgs::PointCloud cloud1;
  cloud1.header = scan1->header;
  cloud1.points.resize(scan1->ranges.size());


cloud1.points[i].x = scan1->ranges[i] * cos(angle);
    cloud1.points[i].y = scan1->ranges[i] * sin(angle);























  */
