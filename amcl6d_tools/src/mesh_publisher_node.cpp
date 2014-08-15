#include "mesh_publisher.h"

int main(int argc, char** argv)
{
  mesh_publisher* mp = new mesh_publisher("/home/student/s/shoeffner/ros-hydro/wet/src/amcl6d/maps/flur2_ascii.ply");
  delete mp;
  return 0;
}
