/**
 * This is a node to publish mesh data in the lvr format on
 * a given topic.
 *
 * Additionally it publishes the vertices of the mash as a
 * point cloud for easy visualization in rqt or rviz on the
 * topic /mesh_point_cloud .
 *
 * This node also broadcasts a default transformation from
 * world to mesh. If you need another transformation please
 * redirect it or add the frame parameter and provide a 
 * custom transformation.
 */
#include "ros/ros.h"

#include <iostream>
#include <math.h>
#include <stdlib.h>

#include "mesh_publisher.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv)
{
  // initialize
  ros::init(argc, argv, "mesh_publisher");
  ros::NodeHandle nh;
  
  // check params
  if(!nh.hasParam("mesh_path"))
  {
    ROS_INFO("Please set the 'mesh_path' parameter. Exiting.");
    ROS_INFO("Note: mesh_path should be relative to /home/user !");
    Logger::instance()->log("No mesh_path parameter found.");
    return 1;
  }

  if(!nh.hasParam("mesh"))
  {
    ROS_INFO("Please set the 'mesh' parameter. Exiting.");
    Logger::instance()->log("No mesh parameter found.");
    return 1;
  }

  if(!nh.hasParam("topic"))
  {
    ROS_INFO("'topic' was not set. Falling back to default. Subscribe to /mesh!");
  }

  if(nh.hasParam("frame"))
  {
    ROS_INFO("Custom frame given.");
  }

  // load path variables
  std::string home_dir(getenv("HOME"));
  std::string mesh_path;
  std::string mesh;
  nh.getParam("mesh_path", mesh_path);
  nh.getParam("mesh", mesh);
  std::string model_path = home_dir + mesh_path + mesh;
  Logger::instance()->logX("ss", "Mesh:", model_path.c_str());

  // load topic
  std::string topic;
  nh.param<std::string>("topic", topic, "mesh");
  Logger::instance()->logX("ss", "Topic:", topic.c_str());

  // load frame
  std::string frame_name;
  nh.param<std::string>("frame", frame_name, "mesh");
  Logger::instance()->logX("ss", "Frame:", frame_name.c_str());

  // initialize mesh publisher (loads model automatically)
  mesh_publisher* mp = new mesh_publisher(model_path, frame_name);
  
  // initialize publishers
  ros::Publisher publisher = nh.advertise<amcl6d_tools::Mesh>(topic, 1000, true);
  ros::Publisher pcl_pub   = nh.advertise<sensor_msgs::PointCloud>("mesh_point_cloud", 1000, true);

  // provide transformations
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
     
  // publish and broadcast  
  ros::Rate loop_rate(1000/30); // 30 fps
  while(ros::ok())
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frame_name));
    publisher.publish(mp->get_message());
    pcl_pub.publish(mp->get_pointcloud());

    ros::spinOnce();
    loop_rate.sleep();
  }

  // clean up
  delete mp;

  return 0;
}
