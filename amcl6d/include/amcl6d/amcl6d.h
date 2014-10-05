#ifndef AMCL6D_H
#define AMCL6D_H

#include "ros/ros.h"
#include "ros/callback_queue.h"

#include "amcl6d_tools/Logger.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "amcl6d_tools/Mesh.h"
#include "amcl6d/pose_factory.h"
#include "amcl6d/pose_sample.h"
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
#include <algorithm>

namespace Eigen 
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

class amcl6d 
{
public:
    amcl6d(ros::NodeHandle nodehandle, ros::CallbackQueue* queue);
    ~amcl6d();

    void generate_poses();
    void mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message);
    void move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    void set_mesh(amcl6d_tools::Mesh mesh);

    void publish(); 
    void spinOnce();

    void clear();

    bool has_mesh();

    geometry_msgs::PoseStamped get_best();
    
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
    ros::Publisher m_best_pose_publisher;

    amcl6d_tools::Mesh m_mesh;

    geometry_msgs::Pose m_last_pose;
    Eigen::Vector3d     m_diff_position;
    Eigen::Quaterniond  m_diff_orientation;

    pose_sample m_current_pose;

    std::default_random_engine m_generator;
    std::normal_distribution<double> m_distribution;

    ros::NodeHandle m_node_handle;
    ros::CallbackQueue* m_queue;
    bool m_busy;

    ros::ServiceClient m_service_client;

    bool m_has_mesh; 
    bool m_moved;
    bool m_has_guess;

    bool prepare_kd_tree();
    pcl::KdTreeFLANN<pcl::PointXYZ> m_kd_tree;

    double m_discard_percentage;
    double m_close_respawn_percentage;
    double m_top_percentage;
    double m_random_respawn_percentage; 
   
    geometry_msgs::PoseStamped m_current_best_pose;
};

#endif
