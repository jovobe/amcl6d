#ifndef AMCL6D_H
#define AMCL6D_H

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "amcl6d_tools/Mesh.h"
#include "amcl6d/pose_factory.h"

class amcl6d {

public:
    amcl6d(ros::NodeHandle nodehandle);
    ~amcl6d();

    void generate_poses();
    void mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message);
    void move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    void publish(); 

    void clear();
    

private:
    int m_sample_number;

    pose_factory* m_factory;

    geometry_msgs::PoseArray m_poses;

    ros::Publisher m_pose_publisher;

    amcl6d_tools::Mesh m_mesh;

    void update_poses();

    geometry_msgs::Pose m_last_pose;
    geometry_msgs::Pose m_current_pose;

};

#endif
