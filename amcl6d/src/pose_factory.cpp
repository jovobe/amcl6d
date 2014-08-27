#include "amcl6d/pose_factory.h"

pose_factory::pose_factory()
{
    m_min_x = m_min_y = m_min_z = -1;
    m_max_x = m_max_y = m_max_z = 1;
}
    
pose_factory::~pose_factory()
{
}

void pose_factory::set_bounds(amcl6d_tools::Mesh mesh)
{
    m_min_x = m_min_y = m_min_z = std::numeric_limits<double>::max();
    m_max_x = m_max_y = m_max_z = std::numeric_limits<double>::min();
    for(int i = 0; i < mesh.mesh.vertices.size(); ++i)
    {
        m_min_x = mesh.mesh.vertices[i].x < m_min_x? mesh.mesh.vertices[i].x : m_min_x;
        m_max_x = mesh.mesh.vertices[i].x > m_max_x? mesh.mesh.vertices[i].x : m_max_x;
        m_min_y = mesh.mesh.vertices[i].y < m_min_y? mesh.mesh.vertices[i].y : m_min_y;
        m_max_y = mesh.mesh.vertices[i].y > m_max_y? mesh.mesh.vertices[i].y : m_max_y;
        m_min_z = mesh.mesh.vertices[i].z < m_min_z? mesh.mesh.vertices[i].z : m_min_z;
        m_max_z = mesh.mesh.vertices[i].z > m_max_z? mesh.mesh.vertices[i].z : m_max_z;
    }
}

geometry_msgs::Pose pose_factory::generate_random_pose() 
{
    geometry_msgs::Pose pose;
    pose.position.x = m_min_x + (double) rand() / RAND_MAX * (m_max_x - m_min_x);
    pose.position.y = m_min_y + (double) rand() / RAND_MAX * (m_max_y - m_min_y);
    pose.position.z = m_min_z + (double) rand() / RAND_MAX * (m_max_z - m_min_z);

    // quaternion sampling after K. Shoemake
    double u_1 = (double) rand() / RAND_MAX;
    double u_2 = (double) rand() / RAND_MAX;
    double u_3 = (double) rand() / RAND_MAX;
    double s_1 = sqrt(1 - u_1);
    double s_2 = sqrt(u_1);
    double p_1 = 2 * M_PI * u_2;
    double p_2 = 2 * M_PI * u_3;
    pose.orientation.x = s_1 * sin(p_1);
    pose.orientation.y = s_1 * cos(p_1);
    pose.orientation.z = s_2 * sin(p_2);
    pose.orientation.w = s_2 * cos(p_2);

    return pose;
}
