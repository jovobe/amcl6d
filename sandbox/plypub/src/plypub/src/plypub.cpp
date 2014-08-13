#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <math.h>
#include "polymap/PolyMap.h"

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
  ros::Publisher publisher = node_handle.advertise<sensor_msgs::PointCloud>(node_name, maximum_msg_size, true);
  // set loop rate
  ros::Rate loop_rate(3000);
	

  // load map
  PolyMap* map = new PolyMap("/home/student/s/shoeffner/thesis/amcl6d/sandbox/maps/pringlescan.ply");

  // prepare mesh
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header.frame_id = "map";
  point_cloud.points.resize(map->face_count()*3);


  for(int i = 0; i < point_cloud.points.size()/3; ++i)
  {
    Face face = map->get_face(i);
    for(int j = 0; j < 3; ++j)
    {
      point_cloud.points[i*3+j].x = face.get_vertex(j)->get_value("x");
      point_cloud.points[i*3+j].y = face.get_vertex(j)->get_value("y");
      point_cloud.points[i*3+j].z = face.get_vertex(j)->get_value("z");
    }
  }

  // publish and broadcast
  while(ros::ok()) 
  {
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
