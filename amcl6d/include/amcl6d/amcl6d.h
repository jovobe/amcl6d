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
#include "cgal_raytracer/raytracer_service.h"

#include <eigen_conversions/eigen_msg.h>
#include <vector>

#include <boost/array.hpp>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <random>
#include <atomic>
#include <mutex>
#include <thread>
#include <time.h>

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

class amcl6d {
public:

    struct pose_sample {
        public:
            void set_pose(geometry_msgs::Pose pose);
            geometry_msgs::Pose get_pose();
            void update_pose(double add_x, double add_y, double add_z, Eigen::Quaterniond set_orientation);

            void set_probability(double probability);
            double get_probability();

            pose_sample();
            pose_sample(const pose_sample& copy);
        private: 
            geometry_msgs::Pose pose;
            double probability;

            // some lock to lock/unlock this sample
            boost::shared_mutex m_mutex;
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

    void init_number_of_threads();

    sensor_msgs::PointCloud issue_raytrace(geometry_msgs::Pose pose);
    double evaluate_raytrace(sensor_msgs::PointCloud pcl);

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
    sensor_msgs::PointCloud m_current_raytrace;

    std::default_random_engine m_generator;
    std::normal_distribution<double> m_distribution;

    std::atomic_int m_current_threads;

    ros::NodeHandle m_node_handle;

    ros::ServiceClient m_service_client;
    unsigned int m_number_of_threads;

};

#endif
