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

find_package(Boost COMPONENTS system thread REQUIRED)
find_package(Eigen)
find_package(CGAL)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++0x -frounding-math")

add_executable(amcl6d
    src/amcl6d_node.cpp
    src/pose_factory.cpp
    src/pose_visualizer.cpp
    src/amcl6d.cpp
)

target_link_libraries(amcl6d 
    ${catkin_LIBRARIES} 
    lvr_logger 
    ${Boost_LIBRARIES}
    CGAL
)

