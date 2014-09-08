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

class amcl6d {

public:
    struct pose_sample {
        public: 
            geometry_msgs::PoseWithCovariance pose;
            double probability;
    };

    amcl6d(ros::NodeHandle nodehandle);
    ~amcl6d();

    void generate_poses();
    void mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message);
    void move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    void publish(); 

    void clear();

//    void amcl(geometry_msgs::PoseWithCovarianceStamped odom_pose, 
    

private:
    void update_poses();
    void init_covariance(boost::array<double, 36> covar);
    
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

};

#endif
