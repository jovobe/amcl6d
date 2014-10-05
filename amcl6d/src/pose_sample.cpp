#include <amcl6d/pose_sample.h>

void pose_sample::set_pose(geometry_msgs::Pose pose) 
{
   // boost::unique_lock<boost::shared_mutex> lock(m_mutex);  
    this->m_pose = pose;
}

geometry_msgs::Pose pose_sample::get_pose()
{
  //  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return this->m_pose;
}

void pose_sample::set_raytrace(sensor_msgs::PointCloud pcl)
{
 //   boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    this->m_raytrace = pcl;
}

sensor_msgs::PointCloud pose_sample::get_raytrace()
{
//    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return this->m_raytrace;
}

void pose_sample::set_probability(double probability)
{
  //  boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    this->m_probability = probability;
}

double pose_sample::get_probability()
{
 //   boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return this->m_probability;
}

void pose_sample::set_likelihood(double likelihood)
{
    this->m_likelihood = likelihood;
}

double pose_sample::get_likelihood()
{
    return this->m_likelihood;
}

void pose_sample::update_pose(double add_x, double add_y, double add_z, Eigen::Quaterniond set_orientation)
{
//    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    m_pose.position.x += add_x;
    m_pose.position.y += add_y;
    m_pose.position.z += add_z;
    m_pose.orientation.x = set_orientation.x();
    m_pose.orientation.y = set_orientation.y();
    m_pose.orientation.z = set_orientation.z();
    m_pose.orientation.w = set_orientation.w();
}

void pose_sample::normalize_raytrace()
{
    Eigen::Affine3d eigenPose(Eigen::Affine3d::Identity());
    tf::poseMsgToEigen(this->m_pose, eigenPose);
    Eigen::Affine3d invertedPose = eigenPose.inverse();
/*
    std::cout << "Pose:" << std::endl;
    std::cout << eigenPose(0, 0) << " " << eigenPose(0, 1) << " " << eigenPose(0, 2) << " " << eigenPose(0, 3) << std::endl
              << eigenPose(1, 0) << " " << eigenPose(1, 1) << " " << eigenPose(1, 2) << " " << eigenPose(1, 3) << std::endl
              << eigenPose(2, 0) << " " << eigenPose(2, 1) << " " << eigenPose(2, 2) << " " << eigenPose(2, 3) << std::endl
              <<               0 << " " <<               0 << " " <<               0 << " " <<               1 << std::endl;

    std::cout << "Inverted:" << std::endl;
    std::cout << invertedPose(0, 0) << " " << invertedPose(0, 1) << " " << invertedPose(0, 2) << " " << invertedPose(0, 3) << std::endl
              << invertedPose(1, 0) << " " << invertedPose(1, 1) << " " << invertedPose(1, 2) << " " << invertedPose(1, 3) << std::endl
              << invertedPose(2, 0) << " " << invertedPose(2, 1) << " " << invertedPose(2, 2) << " " << invertedPose(2, 3) << std::endl
              <<                  0 << " " <<                  0 << " " <<                  0 << " " <<                  1 << std::endl;
*/
    for(int i = 0; i < m_raytrace.points.size(); ++i)
    {
        double x = m_raytrace.points[i].x;
        double y = m_raytrace.points[i].y;
        double z = m_raytrace.points[i].z;
        m_raytrace.points[i].x = invertedPose(0, 0) * x 
                               + invertedPose(0, 1) * y 
                               + invertedPose(0, 2) * z
                               + invertedPose(0, 3);
        m_raytrace.points[i].y = invertedPose(1, 0) * x
                               + invertedPose(1, 1) * y
                               + invertedPose(1, 2) * z
                               + invertedPose(1, 3);
        m_raytrace.points[i].z = invertedPose(2, 0) * x
                               + invertedPose(2, 1) * y
                               + invertedPose(2, 2) * z
                               + invertedPose(2, 3);
    }
}
/* // this should be added in case we need boost again - also = has to be implemented then
pose_sample::pose_sample()
{

}

pose_sample::pose_sample(const pose_sample& copy)
{
    set_pose(copy.m_pose);
    set_probability(copy.m_probability);
    set_raytrace(copy.m_raytrace);
}*/

