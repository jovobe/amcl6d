cmake_minimum_required(VERSION 2.8.3)
project(amcl6d_tools)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
#  lvr
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
  LIBRARIES lvr_logger #lvr
  CATKIN_DEPENDS message_runtime
)

# this is because meshing.pg2013 is not catkinized
include_directories(
  /home/student/s/shoeffner/ros-hydro/dry/meshing.pg2013/include/liblvr
)

#find_library(
#  lvr
#  PATHS /home/student/s/shoeffner/ros-hydro/dry/meshing.pg2013/lib/liblvr.so
#)
# end workaround

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
  lvr_logger /home/student/s/shoeffner/ros-hydro/dry/meshing.pg2013/lib/liblvr.so
)

