/**
 * MeshPublisher.h
 * @description A simple MeshPublisher
 * @author Sebastian Höffner
 * @version 1.0
 * @created 2014-08-14
 */
#ifndef MESH_PUBLISHER_H
#define MESH_PUBLISHER_H

#include <string>
#include "amcl6d_tools/Logger.h"
#include "io/ModelFactory.hpp" 
               // from meshing.pg2013, note the workaround in CMakeLists.txt !!!
               // since meshing.pg2013 is not catkinized
#include "amcl6d_tools/Mesh.h" // Mesh message, should later maybe
                               // be changed to lvr/Mesh.h or something
                               // note that it is also in the code
#include "boost/shared_array.hpp"
#include "sensor_msgs/PointCloud.h"

class mesh_publisher {
public:

	/**
	 * default ctor
   * @param mesh mesh path
   * @param frame frame name
	 */
	mesh_publisher(std::string mesh, std::string frame = "mesh");

  /**
   * default dtor
   */
  ~mesh_publisher();

  /**
   * Returns the mesh message
   */
  amcl6d_tools::Mesh get_message();

  /**
   * Returns the mesh point cloud message
   */
  sensor_msgs::PointCloud get_pointcloud();

private:

  lvr::ModelPtr m_model;
  amcl6d_tools::Mesh m_mesh_message;
  sensor_msgs::PointCloud m_mesh_pcl;
  std::string m_frame;

  void convert_model_to_messages();

};

#endif // MESH_PUBLISHER_H
