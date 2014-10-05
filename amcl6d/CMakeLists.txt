cmake_minimum_required(VERSION 2.8.3)
project(amcl6d)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    eigen_conversions
    geometry_msgs
    roscpp
    sensor_msgs
    tf
    visualization_msgs
    amcl6d_tools
    Eigen
    eigen_conversions
    cgal_raytracer
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS dynamic_reconfigure eigen_conversions geometry_msgs roscpp sensor_msgs tf visualization_msgs amcl6d_tools
)

# common libraries
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(Eigen REQUIRED)
find_package(CGAL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)

# manually added libraries
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
#find_package(Flann REQUIRED)

if(OPENMP_FOUND)
    message("OpenMP optimizations available")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    #    ${Flann_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# add c++11 support + set -frounding-math for CGAL
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -frounding-math")

# for gdb
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_executable(amcl6d
    src/amcl6d_node.cpp
    src/pose_factory.cpp
    src/pose_sample.cpp
    src/pose_visualizer.cpp
    src/amcl6d.cpp
)

target_link_libraries(amcl6d 
    ${catkin_LIBRARIES} 
    lvr_logger 
    ${Boost_LIBRARIES}
    CGAL
    #    ${Flann_LIBRARIES}
    ${PCL_LIBRARIES}
)

