#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <math.h>
#include "polymap/PolyMap.h"
#include "CGALRaytracer.h"
#include "CameraParameters.h"
#include <iostream>

void load_map(sensor_msgs::PointCloud* point_cloud, PolyMap* map)
{
    point_cloud->header.frame_id = "map";
    point_cloud->points.resize(map->vertex_count());

    for(int i = 0; i < point_cloud->points.size(); ++i)
    {
        Vertex vertex = map->get_vertex(i);
        point_cloud->points[i].x = vertex.get_value("x");
        point_cloud->points[i].y = vertex.get_value("y");
        point_cloud->points[i].z = vertex.get_value("z");
    }

}

void prepare_rt_pcl(sensor_msgs::PointCloud* point_cloud, double** points, int size)
{
    point_cloud->points.resize(size);

    for(int i = 0; i < point_cloud->points.size(); ++i)
    {
        point_cloud->points[i].x = points[i][0];
        point_cloud->points[i].y = points[i][1];
        point_cloud->points[i].z = points[i][2];
    }
}

void set_identity(double* d_arr, int size)
{
    for(int i = 0; i < sqrt(size); ++i)
    {
        for(int j = 0; j < sqrt(size); ++j)
        {
           d_arr[i*(int)sqrt(size)+j] = 0;
           if(i == j)
           {
              d_arr[i*(int)sqrt(size)+j] = 1;
           }
        }
    }
}

int main(int argc, char** args)
{
  // meta data
  std::string node_name = "plypub";
  std::string map_topic = "map";
  std::string ray_topic = "raytrace";
  int maximum_msg_size = 1000;
  std::string root_dir = "/home/student/s/shoeffner/ros-hydro/wet/src/amcl6d/";
  
  // initialize program
  ros::init(argc, args, node_name);

  // init node handles and publishers
  ros::NodeHandle map_node_handle;
  ros::Publisher map_publisher = map_node_handle.advertise<sensor_msgs::PointCloud>(map_topic, maximum_msg_size, true);
  ros::NodeHandle ray_node_handle;
  ros::Publisher ray_publisher = ray_node_handle.advertise<sensor_msgs::PointCloud>(ray_topic, maximum_msg_size);

  // set loop rate
  ros::Rate loop_rate(1000/30); // 30 fps

  // init map
  PolyMap* map = new PolyMap(root_dir + "maps/flur2_ascii.ply");

  // prepare map mesh
  sensor_msgs::PointCloud map_cloud;
  load_map(&map_cloud, map);

  // prepare raytracer
  ConfigFile* cfg_file = new ConfigFile(root_dir + "config/cameraparameters.cfg", true);
  CameraParameters* cam_params = new CameraParameters(cfg_file);
  CGALRaytracer* raytracer = new CGALRaytracer(map, cam_params);
  sensor_msgs::PointCloud raytrace;
  raytrace.header.frame_id = "map";
  double matrix[16];
  set_identity(matrix, 16);
  double** points;
  int npoints = 0;

  // publish and broadcast
  while(ros::ok()) 
  {
    // publish map
    map_publisher.publish(map_cloud);

    // do a raytrace and publish the point cloud
    raytracer->simulatePointCloud(matrix, points, npoints);
    prepare_rt_pcl(&raytrace, points, npoints);
    ray_publisher.publish(raytrace);

    // spin and sleep
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete raytracer;
  delete cam_params;
  delete cfg_file;
  delete map;

  return 0;
}
/*


  sensor_msgs::PointCloud cloud1;
  cloud1.header = scan1->header;
  cloud1.points.resize(scan1->ranges.size());


cloud1.points[i].x = scan1->ranges[i] * cos(angle);
    cloud1.points[i].y = scan1->ranges[i] * sin(angle);























  */
