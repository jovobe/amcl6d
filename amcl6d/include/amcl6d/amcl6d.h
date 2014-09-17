#ifndef AMCL6D_H
#define AMCL6D_H

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "amcl6d_tools/Mesh.h"
#include "amcl6d/pose_factory.h"

#include <eigen_conversions/eigen_msg.h>
#include <vector>

#include <boost/array.hpp>

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <random>

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

class amcl6d {
public:

    struct pose_sample {
        public: 
            geometry_msgs::Pose pose;
            double probability;
    };

    amcl6d(ros::NodeHandle nodehandle);
    ~amcl6d();

    void generate_poses();
    void mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message);
    void move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    void publish(); 

    void clear();

    
private:
    void update_poses();
    void init_covariance();
    void init_mu();
    void update_cholesky_decomposition();
    Eigen::Vector6d sample();

    Eigen::Vector6d m_mu;
    Eigen::Matrix6d m_covar;
    Eigen::Matrix6d m_cholesky_decomp;
    
    int m_sample_number;

    pose_factory* m_factory;

    geometry_msgs::PoseArray m_poses;
    std::vector<pose_sample> m_pose_samples;

    ros::Publisher m_pose_publisher;

    amcl6d_tools::Mesh m_mesh;

    geometry_msgs::Pose m_last_pose;
    geometry_msgs::Pose m_current_pose;
    Eigen::Vector3d    m_diff_position;
    Eigen::Quaterniond m_diff_orientation;

    std::default_random_engine m_generator;
    std::normal_distribution<double> m_distribution;
};

#endif
