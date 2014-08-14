cmake_minimum_required(VERSION 2.8.3)
project(plypub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plypub
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
)

# my libs
include_directories(/home/student/s/shoeffner/thesis/amcl6d/sandbox/raytracer/src)
add_library( polymap SHARED IMPORTED )
set_target_properties(polymap PROPERTIES IMPORTED_LOCATION "/home/student/s/shoeffner/thesis/amcl6d/sandbox/raytracer/lib/polymap.a")
include_directories(/home/student/s/shoeffner/thesis/amcl6d/sandbox/raytracer/src/polymap)
add_library( raytracer SHARED IMPORTED )
set_target_properties(raytracer PROPERTIES IMPORTED_LOCATION "/home/student/s/shoeffner/thesis/amcl6d/sandbox/raytracer/lib/raytracer.a")

# CGAL needs -frounding-math
add_definitions("-frounding-math")

add_executable(plypub src/plypub.cpp)
target_link_libraries(plypub ${catkin_LIBRARIES} polymap raytracer CGAL)


