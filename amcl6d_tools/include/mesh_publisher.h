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
#include "Logger.h"
#include "io/ModelFactory.hpp" 
               // from meshing.pg2013, note the workaround in CMakeLists.txt !!!
#include "amcl6d_tools/Mesh.h" // Mesh message, should later maybe be changed to lvr/Mesh.h or something

class mesh_publisher {
public:

	/**
	 * default ctor, pass mesh as param to rosrun:
   * rosrun amcl6d mesh_publisher _mesh:=PATH/TO/MESH
	 */
	mesh_publisher(std::string mesh);

  /**
   * default dtor
   */
  ~mesh_publisher();

  /**
   * Returns the mesh message
   */
  amcl6d_tools::Mesh get_message();

private:

  lvr::ModelPtr m_model;


};

#endif // MESH_PUBLISHER_H
