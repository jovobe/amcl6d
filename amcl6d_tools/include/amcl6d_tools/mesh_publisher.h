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
#include <fstream>

#include "amcl6d_tools/Logger.h"

#include "io/ModelFactory.hpp" 
               // from meshing.pg2013, note the workaround in CMakeLists.txt
               // since meshing.pg2013 is not catkinized
#include "amcl6d_tools/Mesh.h"
#include "sensor_msgs/PointCloud.h"

#include "amcl6d_tools/mesh_publisherConfig.h"

#include "boost/shared_array.hpp"
#include "boost/thread/shared_mutex.hpp"

class mesh_publisher
{
public:

    /**
     * Default constructor
     */
    mesh_publisher();

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

    /**
     * reconfiguration callback to reload the mesh.
     */
    void reconfigure_callback(
            amcl6d_tools::mesh_publisherConfig &config, 
            uint32_t level);
    /**
     * Sets the mesh. Semantic alias for replace_mesh.
     */
    void set_mesh(std::string mesh);

    /**
     * Replaces the current mesh with the new mesh.
     */
    void replace_mesh(std::string mesh);

    /**
     * Replaces the current frame.
     */
    void replace_frame(std::string m_frame);

    /**
     * Returns the current frame name.
     */
    std::string get_frame();

private:

    lvr::ModelPtr m_model;
    amcl6d_tools::Mesh m_mesh_message;
    sensor_msgs::PointCloud m_mesh_pcl;
    std::string m_frame;

    boost::shared_mutex m_mutex;

    void convert_model_to_messages();
    
    bool mesh_exists(std::string mesh);
};

#endif // MESH_PUBLISHER_H
