cmake_minimum_required(VERSION 2.8.3)
project(amcl6d_tools)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    eigen_conversions
)

generate_dynamic_reconfigure_options(
    cfg/mesh_publisher.cfg
    cfg/raytracer_client.cfg
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

add_service_files(
    FILES
    RequestMap.srv
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
    CATKIN_DEPENDS 
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
        roscpp
)

# this is because meshing.pg2013 is not catkinized
include_directories(
    $ENV{HOME}/ros-hydro/dry/meshing.pg2013/include/liblvr
)
# end workaround

# find boost
find_package(Boost COMPONENTS system thread REQUIRED)
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

add_dependencies(mesh_publisher 
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(mesh_publisher
    lvr_logger 
    ${catkin_LIBRARIES} 
    $ENV{HOME}/ros-hydro/dry/meshing.pg2013/lib/liblvr.so # workaround
    ${Boost_LIBRARIES}
)

add_executable(map_service
    src/mesh_publisher_service.cpp
    src/mesh_publisher.cpp
)

add_dependencies(map_service
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
)

target_link_libraries(map_service
    lvr_logger
    ${catkin_LIBRARIES}
    $ENV{HOME}/ros-hydro/dry/meshing.pg2013/lib/liblvr.so # workaround
    ${Boost_LIBRARIES}
)

add_executable(raytracer_client
    src/raytracer_client.cpp
)

add_dependencies(raytracer_client
    cgal_raytracer_gencpp
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(raytracer_client
    lvr_logger
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
