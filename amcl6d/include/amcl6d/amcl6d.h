#ifndef AMCL6D_H
#define AMCL6D_H

#include "ros/ros.h"

#include "amcl6d_tools/Logger.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "amcl6d_tools/Mesh.h"
#include "amcl6d/pose_factory.h"
#include "cgal_raytracer/raytracer_service.h"

#include <boost/array.hpp>
#include <boost/thread.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <vector>
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

            void set_raytrace(sensor_msgs::PointCloud pcl);
            sensor_msgs::PointCloud get_raytrace();
            
            void normalize_raytrace();

            pose_sample();
            pose_sample(const pose_sample& copy);
        private: 
            geometry_msgs::Pose m_pose;
            double m_probability;
            sensor_msgs::PointCloud m_raytrace;

            // some lock to lock/unlock this sample
            boost::shared_mutex m_mutex;
    };

    amcl6d(ros::NodeHandle nodehandle);
    ~amcl6d();

    void generate_poses();
    void mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message);
    void move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    void set_mesh(amcl6d_tools::Mesh mesh);

    void publish(); 

    void clear();

    bool has_mesh();
    
private:
    void update_poses();
    void init_covariance();
    void init_mu();
    void update_cholesky_decomposition();
    Eigen::Vector6d sample();

    sensor_msgs::PointCloud issue_raytrace(geometry_msgs::Pose pose);
    double evaluate_sample(pose_sample sample);

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
    Eigen::Vector3d     m_diff_position;
    Eigen::Quaterniond  m_diff_orientation;

    pose_sample m_current_pose;

    std::default_random_engine m_generator;
    std::normal_distribution<double> m_distribution;

    ros::NodeHandle m_node_handle;

    ros::ServiceClient m_service_client;

    bool m_has_mesh; 

    void prepare_kd_tree();
    pcl::KdTreeFLANN<pcl::PointXYZ> m_kd_tree;
};

#endif
