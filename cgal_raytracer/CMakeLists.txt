cmake_minimum_required(VERSION 2.8.3)
project(cgal_raytracer)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    amcl6d_tools
    message_generation
    geometry_msgs
    sensor_msgs
    Eigen
    eigen_conversions
)

generate_dynamic_reconfigure_options(
    cfg/CamParam.cfg
)

add_service_files(
    FILES
    RaytraceAtPose.srv
)

generate_messages(
    DEPENDENCIES 
    geometry_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

find_package(CGAL)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include ${catkin_INCLUDE_DIRS}
)

# CGAL needs -frounding-math
add_definitions("-frounding-math")

add_executable(raytracer_service
    src/raytracer_service.cpp
    src/CGALRaytracer.cpp
    src/CameraParameters.cpp
)

add_dependencies(raytracer_service 
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(raytracer_service 
    ${catkin_LIBRARIES} 
    CGAL 
    lvr_logger 
    ${Boost_LIBRARIES}
)

