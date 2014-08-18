cmake_minimum_required(VERSION 2.8.3)
project(amcl6d_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  BoundingBox.msg
  Face.msg
  LabeledFaces.msg
  MaterialGroup.msg
  Mesh.msg
  PointNormals.msg
  Textures.msg
  TriangleMeshGeometry.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lvr_logger
  CATKIN_DEPENDS message_runtime
)

# this is because meshing.pg2013 is not catkinized
include_directories(
  $ENV{HOME}/ros-hydro/dry/meshing.pg2013/include/liblvr
)
# end workaround

# find boost
find_package(Boost COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
)

add_library(lvr_logger SHARED
  src/Logger.cpp
)

add_executable(mesh_publisher
  src/mesh_publisher_node.cpp
  src/mesh_publisher.cpp
)

target_link_libraries(mesh_publisher
  lvr_logger 
  ${catkin_LIBRARIES} 
  $ENV{HOME}/ros-hydro/dry/meshing.pg2013/lib/liblvr.so # workaround
  ${Boost_LIBRARIES}
)

