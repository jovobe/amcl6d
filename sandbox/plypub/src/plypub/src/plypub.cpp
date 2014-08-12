#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <iostream>

int main(int argc, char** args)
{
  // meta data
  std::string node_name = "plypub";
  int maximum_msg_size = 1000;
  
  // initialize
  ros::init(argc, args, node_name);

  // init node handle and publisher
  ros::NodeHandle node_handle;
  ros::Publisher publisher = node_handle.advertise<sensor_msgs::PointCloud>(node_name, maximum_msg_size);
	
  // init tranform and its broadcaster
  tf::TransformBroadcaster tf_broadcaster;
  tf::Transform transform;

  // set loop rate
  ros::Rate loop_rate(3000);
	

  // prepare point cloud
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header.frame_id = "map";
  point_cloud.points.resize(200);
  for(int i = 0; i < point_cloud.points.size(); i++)
  {
    point_cloud.points[i].x = (i-100.0)/200.0;
    point_cloud.points[i].y = sin((i-100.0)/200.0);
    point_cloud.points[i].z = cos((i-100.0)/200.0);
  }

  // set transformation
  transform.setOrigin(tf::Vector3(0,0,0));
  transform.setRotation(tf::Quaternion(0,0,0));

  // publish and broadcast
  while(ros::ok()) 
  {
    // broadcast tranform
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

    // publish point cloud
    publisher.publish(point_cloud);

    // spin and sleep
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
