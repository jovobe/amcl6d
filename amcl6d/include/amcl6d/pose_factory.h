#ifndef POSE_FACTORY_H
#define POSE_FACTORY_H

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "amcl6d_tools/Mesh.h"

#include "amcl6d_tools/Logger.h"

class pose_factory {

public:
    pose_factory();
    ~pose_factory();

    void set_bounds(amcl6d_tools::Mesh mesh);
    
    geometry_msgs::Pose generate_random_pose();

private:
    double m_min_x;
    double m_max_x;
    double m_min_y;
    double m_max_y; 
    double m_min_z;
    double m_max_z;

};
#endif
