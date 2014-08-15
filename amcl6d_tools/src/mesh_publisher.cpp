#include "mesh_publisher.h"

mesh_publisher::mesh_publisher(std::string mesh)
{
  lvr::ModelPtr model = lvr::ModelFactory::readModel(mesh);
  Logger::instance()->log("read model"); 
}

mesh_publisher::~mesh_publisher()
{
  Logger::instance()->log("mesh_publisher destroyed.");
}

amcl6d_tools::Mesh get_message()
{
  return amcl6d_tools::Mesh();
}

