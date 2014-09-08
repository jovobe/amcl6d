#include "amcl6d/amcl6d.h"

amcl6d::amcl6d(ros::NodeHandle nodehandle)
{
    m_sample_number = 100;
    m_pose_publisher = nodehandle.advertise<geometry_msgs::PoseArray>("pose_samples", 1000);
    m_poses.header.frame_id = "world";
    m_factory = new pose_factory();
}

amcl6d::~amcl6d() {
    // publish cleared poses
    clear();
    publish();

    if(m_factory != NULL)
    {
        delete m_factory;
        m_factory = NULL;
    }
}

void amcl6d::clear()
{
    m_poses.poses.clear();
}

void amcl6d::publish()
{
    m_pose_publisher.publish(m_poses);
}

void amcl6d::update_poses()
{
    // TODO THIS IS CURRENTLY NOISE FREE!!
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        m_pose_samples[i].pose.pose.position.x += m_diff_position.x();
        m_pose_samples[i].pose.pose.position.y += m_diff_position.y();
        m_pose_samples[i].pose.pose.position.z += m_diff_position.z();
        Eigen::Quaterniond orientation(m_pose_samples[i].pose.pose.orientation.w,
                                       m_pose_samples[i].pose.pose.orientation.x,
                                       m_pose_samples[i].pose.pose.orientation.y,
                                       m_pose_samples[i].pose.pose.orientation.z);
        Eigen::Quaterniond n_orientation = m_diff_orientation * orientation;
        m_pose_samples[i].pose.pose.orientation.x = n_orientation.x();
        m_pose_samples[i].pose.pose.orientation.y = n_orientation.y();
        m_pose_samples[i].pose.pose.orientation.z = n_orientation.z();
        m_pose_samples[i].pose.pose.orientation.w = n_orientation.w();
        
        m_poses.poses[i] = m_pose_samples[i].pose.pose;

        // TODO raytracing, particle regeneration
    }

}

void amcl6d::generate_poses()
{
    if(m_factory == NULL)
    {
        ROS_INFO("amcl6d::generate_poses(): no factory.");
        return;
    }

    clear();

    m_pose_samples.resize(m_sample_number);
    m_poses.poses.resize(m_sample_number);
    for(int i = 0; i < m_sample_number; ++i)
    {
        m_pose_samples[i].pose.pose = m_factory->generate_random_pose();
        init_covariance(m_pose_samples[i].pose.covariance);
        m_pose_samples[i].probability = 1.0 / m_sample_number;
        m_poses.poses[i] = m_pose_samples[i].pose.pose;
    }
}

// TODO Change to meaningful covariances or get them from robot
void amcl6d::init_covariance(boost::array<double, 36> covar)
{
    covar[0*6+0] = 
    covar[1*6+1] = covar[2*6+2] = covar[3*6+3] = covar[4*6+4] = covar[5*6+5] = 1;
}

void amcl6d::mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message)
{
    if(m_factory == NULL || message->mesh.vertices.size() == m_mesh.mesh.vertices.size())
    {
        return;
    }
    m_mesh = amcl6d_tools::Mesh(*message.get());
    m_factory->set_bounds(m_mesh);
    // TODO pose reset: avoid update when first pose arrives later!
    
    generate_poses();
}
#include <iostream>
void amcl6d::move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    m_last_pose = geometry_msgs::Pose(m_current_pose);
    m_current_pose = geometry_msgs::Pose((*pose_msg.get()).pose);

    // calculate difference between last and current pose
    Eigen::Vector3d    last_position(m_last_pose.position.x, 
                                     m_last_pose.position.y, 
                                     m_last_pose.position.z);
    Eigen::Quaterniond last_orientation(m_last_pose.orientation.w, 
                                        m_last_pose.orientation.x, 
                                        m_last_pose.orientation.y, 
                                        m_last_pose.orientation.z);
    Eigen::Vector3d    current_position(m_current_pose.position.x, 
                                        m_current_pose.position.y, 
                                        m_current_pose.position.z);
    Eigen::Quaterniond current_orientation(m_current_pose.orientation.w, 
                                           m_current_pose.orientation.x, 
                                           m_current_pose.orientation.y, 
                                           m_current_pose.orientation.z);
    m_diff_position    = current_position - last_position;
    m_diff_orientation = last_orientation * current_orientation.inverse();

    // if there is a difference, provide an update
    double diff = m_diff_orientation.vec().squaredNorm() + m_diff_position.squaredNorm();

    if(diff > 0)
    {
        ROS_DEBUG("Moved.");
        update_poses();
    }
}

