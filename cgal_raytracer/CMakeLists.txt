cmake_minimum_required(VERSION 2.8.3)
project(cgal_raytracer)

find_package(catkin REQUIRED COMPONENTS
  amcl6d_tools
  message_generation
)

find_package(CGAL)

add_service_files(
  FILES
  RaytraceAtPose.srv
)

generate_messages(
  #DEPENDENCIES
  #std_msgs #?
)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES cgal_raytracer
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
)

# CGAL needs -frounding-math
add_definitions("-frounding-math")

#add_library(logger SHARddED
#  src/Logger.cpp)

#add_library(polymap SHARED
#  src/polymap/Vertex.cpp
#  src/polymap/Face.cpp
#  src/polymap/PolyMap.cpp)

#add_library(cgal_raytracer SHARED
#  src/ConfigFile.cpp 
#  src/CameraParameters.cpp  
#  src/CGALRaytracer.cpp)

#target_link_libraries(polymap logger)

add_executable(cgal_raytracer 
  src/CGALRaytracer.cpp
)

target_link_libraries(cgal_raytracer CGAL)

