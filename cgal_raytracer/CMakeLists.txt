cmake_minimum_required(VERSION 2.8.3)
project(cgal_raytracer)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cgal_raytracer logger polymap
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
)

# CGAL needs -frounding-math
add_definitions("-frounding-math")

add_library(logger SHARED
  src/Logger.cpp)

add_library(polymap SHARED
  src/polymap/Vertex.cpp
  src/polymap/Face.cpp
  src/polymap/PolyMap.cpp)

add_library(cgal_raytracer SHARED
  src/ConfigFile.cpp 
  src/CameraParameters.cpp  
  src/CGALRaytracer.cpp)

target_link_libraries(polymap logger)
target_link_libraries(cgal_raytracer polymap CGAL)
