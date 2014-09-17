#include "amcl6d/amcl6d.h"

amcl6d::amcl6d(ros::NodeHandle nodehandle)
{
    m_sample_number = 100;
    m_pose_publisher = nodehandle.advertise<geometry_msgs::PoseArray>("pose_samples", 1000);
    m_poses.header.frame_id = "world";
    m_factory = new pose_factory();
    m_distribution = std::normal_distribution<double>(0.0, 1.0);

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
    // TODO rotations are still noise free
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        Eigen::Vector6d noise = sample();
        m_pose_samples[i].pose.position.x += m_diff_position.x() + noise(0);
        m_pose_samples[i].pose.position.y += m_diff_position.y() + noise(1);
        m_pose_samples[i].pose.position.z += m_diff_position.z() + noise(2);
        Eigen::Quaterniond orientation(m_pose_samples[i].pose.orientation.w,
                                       m_pose_samples[i].pose.orientation.x,
                                       m_pose_samples[i].pose.orientation.y,
                                       m_pose_samples[i].pose.orientation.z);
        Eigen::Quaterniond n_orientation = m_diff_orientation * orientation;
        
        n_orientation = n_orientation;
        m_pose_samples[i].pose.orientation.x = n_orientation.x();
        m_pose_samples[i].pose.orientation.y = n_orientation.y();
        m_pose_samples[i].pose.orientation.z = n_orientation.z();
        m_pose_samples[i].pose.orientation.w = n_orientation.w();
        
        m_poses.poses[i] = m_pose_samples[i].pose;

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
        m_pose_samples[i].pose= m_factory->generate_random_pose();
        m_pose_samples[i].probability = 1.0 / m_sample_number;
        m_poses.poses[i] = m_pose_samples[i].pose;
    }
    init_mu();
    init_covariance();
}

// TODO remove
#include <iostream>

Eigen::Vector6d amcl6d::sample()
{
    // the sampling is taken from mvnrnd in matlab:
    /*
       c = cholesky_factorization(covariances)
       result = rand * c + mu;
     */

    Eigen::Vector6d values;
    values << m_distribution(m_generator), 
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator);
    // TODO remove std::cout stuff
    std::cout << "rand:" << std::endl << values << std::endl; 
    std::cout << "transpose" << std::endl << values.transpose()<<std::endl;
    std::cout << "mu:" << std::endl << m_mu << std::endl; 
    std::cout << "transpose" << std::endl << m_mu.transpose()<<std::endl;
    std::cout << "decomp" << std::endl << m_cholesky_decomp << std::endl;
    std::cout << "values * decomp" << std::endl << values.transpose() * m_cholesky_decomp << std::endl;
    Eigen::Vector6d result = (values.transpose() * m_cholesky_decomp) + m_mu.transpose();
    std::cout << "result:" << std::endl;
    std::cout << result << std::endl;
    return result;
}

void amcl6d::update_cholesky_decomposition()
{
    //m_cholesky_decomp = m_covar.llt().matrixL();
    m_cholesky_decomp = m_covar.ldlt().matrixL();
}

// TODO Change to meaningful covariances or get them from robot
// TODO also change initialization way to take other arguments?
void amcl6d::init_covariance()
{
    m_covar << 1, 0, 0, 0, 0, 0,
               0, 1, 0, 0, 0, 0,
               0, 0, 1, 0, 0, 0,
               0, 0, 0, 1, 0, 0,
               0, 0, 0, 0, 1, 0,
               0, 0, 0, 0, 0, 1;
    update_cholesky_decomposition();
    ROS_INFO("Updated covariances and cholesky decomposition.");
}

// TODO Change to meaningful mus or get them from robot
void amcl6d::init_mu()
{
    m_mu << 0, 0, 0, 0, 0, 0;
    ROS_INFO("Updated mu's.");
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

