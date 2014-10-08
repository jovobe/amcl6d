#include "amcl6d/pose_factory.h"

pose_factory::pose_factory()
{
    Logger::instance()->log("[Pose factory] Initialized.");
    m_min_x = m_min_y = m_min_z = -1;
    m_max_x = m_max_y = m_max_z = 1;


    double a = 5 * M_PI / 180;
    Eigen::Matrix6d covar;
    covar << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, a, 0, 0,
             0, 0, 0, 0, a, 0,
             0, 0, 0, 0, 0, a;
    m_cholesky_decomp = covar.llt().matrixL();
}
    
pose_factory::~pose_factory()
{
    Logger::instance()->log("[Pose factory] Shut down.");
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
    Logger::instance()->logX("ssdsds", "[Pose factory] Set boundaries:", "[",m_min_x,",",m_max_x,"]");
    Logger::instance()->logX("ssdsds", "                              ", "[",m_min_y,",",m_max_y,"]");
    Logger::instance()->logX("ssdsds", "                              ", "[",m_min_z,",",m_max_z,"]");
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

geometry_msgs::Pose pose_factory::generate_pose_near(geometry_msgs::Pose pose)
{
    Eigen::Vector6d values;
    values << m_distribution(m_generator), 
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator);
    Eigen::Vector6d noise = m_cholesky_decomp.transpose() * values;

    geometry_msgs::Pose sample_pose = pose;
    Eigen::Quaterniond orientation(sample_pose.orientation.w,
                                   sample_pose.orientation.x,
                                   sample_pose.orientation.y,
                                   sample_pose.orientation.z);
    
    Eigen::Quaterniond rotZ(Eigen::AngleAxisd(noise(3), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond rotY(Eigen::AngleAxisd(noise(4), Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond rotX(Eigen::AngleAxisd(noise(5), Eigen::Vector3d::UnitX()));
    orientation = rotX * rotY * rotZ * orientation;
        
    sample_pose.position.x += noise(0);
    sample_pose.position.y += noise(1);
    sample_pose.position.z += noise(2);
    sample_pose.orientation.x = orientation.x();
    sample_pose.orientation.y = orientation.y();
    sample_pose.orientation.z = orientation.z();
    sample_pose.orientation.w = orientation.w();

    return sample_pose;
}

double pose_factory::get_maximum_distance() 
{
    double d_x = fabs(m_min_x - m_max_x);
    double d_y = fabs(m_min_y - m_max_y);
    double d_z = fabs(m_min_z - m_min_z);
    return std::max(d_x, std::max(d_y, d_z));
}

    
