#include "ros/ros.h"

#include <string>
#include <math.h>
#include <stdlib.h>

#include "amcl6d_tools/Mesh.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"
#include "amcl6d_tools/Logger.h"
#include "cgal_raytracer/CGALRaytracer.h"
#include "cgal_raytracer/CameraParameters.h"
#include "dynamic_reconfigure/server.h"
#include "cgal_raytracer/CamParamConfig.h"

#include "cgal_raytracer/RaytraceAtPose.h"

class raytracer_service {

  public:
    raytracer_service();

    ~raytracer_service();

    bool raytrace(cgal_raytracer::RaytraceAtPose::Request  &request,
                  cgal_raytracer::RaytraceAtPose::Response &response);

    void message_callback(const amcl6d_tools::Mesh::ConstPtr &message);

    void reconfigure_callback(cgal_raytracer::CamParamConfig &config, 
                              uint32_t level);
    bool has_mesh();

    std::string m_frame;
  private:

    amcl6d_tools::Mesh m_mesh;

    bool m_mesh_received;

    CameraParameters m_cam_params;

    CGALRaytracer* m_raytracer;
};
