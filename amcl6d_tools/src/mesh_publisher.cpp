/**
 * mesh_publisher.cpp
 *
 * Created: 2014-08-14
 * Author: Sebastian Höffner
 * Last Modified: 2014-08-18
 * Author: Sebastian Höffner
 */
#include "amcl6d_tools/mesh_publisher.h"

mesh_publisher::mesh_publisher()
{
}

mesh_publisher::~mesh_publisher()
{
    Logger::instance()->log("~mesh_publisher()");
}

amcl6d_tools::Mesh mesh_publisher::get_message()
{
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return m_mesh_message;
}

sensor_msgs::PointCloud mesh_publisher::get_pointcloud()
{
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return m_mesh_pcl;
}

void mesh_publisher::reconfigure_callback(
        amcl6d_tools::mesh_publisherConfig &config, 
        uint32_t level)
{
    replace_frame(config.frame.compare("")? config.frame : "map_mesh");
    Logger::instance()->logX("ss","Current frame:",m_frame.c_str());
    Logger::instance()->logX("si","level:",(int)level);
    std::string home_dir(getenv("HOME"));
    std::string mesh_path = config.mesh_path;
    std::string mesh_name = config.mesh_name;
    std::string mesh = home_dir + mesh_path + mesh_name;
    if(mesh_name.compare("") || mesh.compare(""))
    {
        Logger::instance()->logX("ss", "Trying to load mesh:", mesh.c_str());
        if(mesh_exists(mesh))
        {
            replace_mesh(mesh);
        }
    }
}

void mesh_publisher::set_mesh(std::string mesh)
{
    replace_mesh(mesh);
}

void mesh_publisher::replace_mesh(std::string mesh)
{
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    Logger::instance()->log("Setting mesh");
    m_model = lvr::ModelFactory::readModel(mesh);
    convert_model_to_messages();
}

void mesh_publisher::replace_frame(std::string frame)
{
    m_frame = frame;
}

std::string mesh_publisher::get_frame()
{
    return m_frame;
}

void mesh_publisher::convert_model_to_messages()
{
    Logger::instance()->log("Generating mesh message.");
    
    // set message header ##############################
    m_mesh_message.header.frame_id = m_frame;
    m_mesh_pcl.header.frame_id = m_frame;

    // set message mesh ################################
    
    // header
    m_mesh_message.mesh.header.frame_id = m_frame;

    // vertices
    size_t num_vertices = 0;
    boost::shared_array<float> vertex_array(
            m_model-> m_mesh-> getVertexArray(num_vertices));
    m_mesh_message.mesh.vertices.resize(num_vertices);
    m_mesh_pcl.points.resize(num_vertices);
    Logger::instance()->logX("sis","Mesh has", num_vertices, "vertices");
    
    for(int i = 0; i < num_vertices; ++i)
    {
        m_mesh_message.mesh.vertices[i].x = vertex_array[i*3+0];
        m_mesh_message.mesh.vertices[i].y = vertex_array[i*3+1];
        m_mesh_message.mesh.vertices[i].z = vertex_array[i*3+2];
        m_mesh_pcl.points[i].y = vertex_array[i*3+1];
        m_mesh_pcl.points[i].x = vertex_array[i*3+0];
        m_mesh_pcl.points[i].z = vertex_array[i*3+2];
    }

    // faces
    size_t num_faces = 0;
    boost::shared_array<unsigned int> face_array(
            m_model-> m_mesh-> getFaceArray(num_faces));
    m_mesh_message.mesh.faces.resize(num_faces);
    Logger::instance()->logX("sis","Mesh has", num_faces, "faces");
    
    for(int i = 0; i < num_faces; ++i)
    {
        m_mesh_message.mesh.faces[i].i = face_array[i*3+0];
        m_mesh_message.mesh.faces[i].j = face_array[i*3+1];
        m_mesh_message.mesh.faces[i].k = face_array[i*3+2];
    }

    // pose
    m_mesh_message.mesh.pose.position.x = 0;
    m_mesh_message.mesh.pose.position.y = 0;
    m_mesh_message.mesh.pose.position.z = 0;
    m_mesh_message.mesh.pose.orientation.x = 0;
    m_mesh_message.mesh.pose.orientation.y = 0;
    m_mesh_message.mesh.pose.orientation.z = 0;
    m_mesh_message.mesh.pose.orientation.w = 1;
}

bool mesh_publisher::mesh_exists(std::string mesh)
{
    std::ifstream ifs;
    ifs.open(mesh.c_str(), std::ifstream::in);
    if(ifs.good())
    {
        ifs.close(); 
        return true;
    } 
    else 
    {
        ifs.close();
        return false;
    }
}
