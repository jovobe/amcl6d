#ifndef POSE_SAMPLE_H
#define POSE_SAMPLE_H

#include <boost/thread.hpp>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Pose.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>


class pose_sample 
{
public:
    void set_pose(geometry_msgs::Pose pose);
    geometry_msgs::Pose get_pose();
    void update_pose(double add_x, double add_y, double add_z, Eigen::Quaterniond set_orientation);

    void set_probability(double probability);
    double get_probability();
    void set_likelihood(double likelihood);
    double get_likelihood();
    void set_raytrace(sensor_msgs::PointCloud pcl);
    sensor_msgs::PointCloud get_raytrace();
        
    void normalize_raytrace();
  //  pose_sample();
  //  pose_sample(const pose_sample& copy);

private: 
    geometry_msgs::Pose m_pose;
    sensor_msgs::PointCloud m_raytrace;

    double m_probability;
    double m_likelihood;
    // some lock to lock/unlock this sample
//    boost::shared_mutex m_mutex;
};


#endif
