#include "mesh_publisher.h"
#include <iostream>
mesh_publisher::mesh_publisher(std::string mesh, std::string frame)
{
  m_frame = frame;
  m_model = lvr::ModelFactory::readModel(mesh);

  convert_model_to_messages();
}

mesh_publisher::~mesh_publisher()
{
  Logger::instance()->log("~mesh_publisher()");
}

amcl6d_tools::Mesh mesh_publisher::get_message()
{
  return m_mesh_message;
}

sensor_msgs::PointCloud mesh_publisher::get_pointcloud()
{
  return m_mesh_pcl;
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
  unsigned int num_vertices = 0;
  boost::shared_array<float> vertex_array(m_model->m_mesh->getVertexArray(num_vertices));
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
  unsigned int num_faces = 0;
  boost::shared_array<unsigned int> face_array(m_model->m_mesh->getFaceArray(num_faces));
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
