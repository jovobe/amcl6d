/**
 * MeshPublisher.h
 * 
 * This is a very simple mesh publisher which uses the roslvr tools to 
 * read and publish meshes.
 *
 * Created: 2014-08-14
 * Author: Sebastian Höffner
 * Last Modified: 2014-08-18
 * Author: Sebastian Höffner
 */
#ifndef MESH_PUBLISHER_H
#define MESH_PUBLISHER_H

#include <string>
#include "amcl6d_tools/Logger.h"
#include "io/ModelFactory.hpp" 
               // from meshing.pg2013, note the workaround in CMakeLists.txt
               // since meshing.pg2013 is not catkinized
#include "amcl6d_tools/Mesh.h"
#include "sensor_msgs/PointCloud.h"
#include "boost/shared_array.hpp"

class mesh_publisher {
public:

    /**
     * Default Constructor
     * @param mesh mesh path
     * @param frame frame name
     */
    mesh_publisher(std::string mesh, std::string frame = "mesh");

    /**
     * default destructor
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
