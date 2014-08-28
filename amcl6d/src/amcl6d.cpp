#include "amcl6d/amcl6d.h"

amcl6d::amcl6d(ros::NodeHandle nodehandle)
{
    m_sample_number = 10000;
    m_pose_publisher = nodehandle.advertise<geometry_msgs::PoseArray>("pose_samples", 1000);
    m_poses.header.frame_id = "world";
    m_factory = new pose_factory();
}

amcl6d::~amcl6d() {
    clear();
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
    clear();
    generate_poses();
}

void amcl6d::generate_poses()
{
    if(m_factory == NULL)
    {
        ROS_INFO("amcl6d::generate_poses(): no factory.");
        return;
    }
    for(int i = 0; i < m_sample_number; ++i)
    {
        m_poses.poses.push_back(m_factory->generate_random_pose());
    }
}

void amcl6d::mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message)
{
    if(m_factory == NULL || message->mesh.vertices.size() == m_mesh.mesh.vertices.size())
    {
        return;
    }
    m_mesh = amcl6d_tools::Mesh(*message.get());
    m_factory->set_bounds(m_mesh);
    // TODO pose reset
    

    clear();
    generate_poses();
}

void amcl6d::move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    m_last_pose = geometry_msgs::Pose(m_current_pose);
    m_current_pose = geometry_msgs::Pose((*pose_msg.get()).pose);

    double diff = 0;
    diff += abs(m_last_pose.position.x - m_current_pose.position.x);
    diff += abs(m_last_pose.position.y - m_current_pose.position.y);
    diff += abs(m_last_pose.position.z - m_current_pose.position.z);
    diff += abs(m_last_pose.orientation.x - m_current_pose.orientation.x);
    diff += abs(m_last_pose.orientation.y - m_current_pose.orientation.y);
    diff += abs(m_last_pose.orientation.z - m_current_pose.orientation.z);
    diff += abs(m_last_pose.orientation.w - m_current_pose.orientation.w);

    if(diff > 0)
    {
        ROS_INFO("Moved.");
        update_poses();
    }
}

