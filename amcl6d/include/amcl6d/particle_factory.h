#ifndef PARTICLE_FACTORY_H
#define PARTICLE_FACTORY_H

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "amcl6d_tools/Mesh.h"

#include <limits>

class particle_factory {

public:
    particle_factory();
    ~particle_factory();

    visualization_msgs::Marker generate_particle(int id);

    visualization_msgs::Marker remove_particle(int id);
    
    void set_bounds(amcl6d_tools::Mesh mesh);
private:

    geometry_msgs::Pose generate_random_pose();

    double m_min_x;
    double m_max_x;
    double m_min_y;
    double m_max_y; 
    double m_min_z;
    double m_max_z;

};
#endif
