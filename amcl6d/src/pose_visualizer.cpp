#include "amcl6d/pose_visualizer.h"

pose_visualizer::pose_visualizer(ros::NodeHandle* node_handle)
{
    m_node = node_handle;
    m_publisher = m_node->advertise<visualization_msgs::MarkerArray>("pose_samples", 1000);
    Logger::instance()->log("[Pose visualizer] Initialized.");
}

pose_visualizer::~pose_visualizer()
{
    Logger::instance()->log("[Pose visualizer] Shut down.");
}
        
void pose_visualizer::add_pose(int id, geometry_msgs::Pose pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "poses";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    
    marker.pose.position.x = pose.position.x;
    marker.pose.position.y = pose.position.y;
    marker.pose.position.z = pose.position.z;
    marker.pose.orientation.x = pose.orientation.x;
    marker.pose.orientation.y = pose.orientation.y;
    marker.pose.orientation.z = pose.orientation.z;
    marker.pose.orientation.w = pose.orientation.w;

    m_markers.markers.push_back(marker);
}

void pose_visualizer::add_pose(int id, geometry_msgs::PoseStamped pose)
{
    add_pose(id, pose.pose);
}

void pose_visualizer::remove_pose(int id)
{
    for(int i = 0; i < m_markers.markers.size(); ++i)
    {
        if(m_markers.markers[i].id == id)
        {
            m_markers.markers[i].action = visualization_msgs::Marker::DELETE;
            return;
        }
    }
}

void pose_visualizer::modify_pose(int id, geometry_msgs::Pose pose)
{
    int i = 0;
    for(; i < m_markers.markers.size(); ++i)
    {
        if(m_markers.markers[i].id == id)
        {
            break;
        }
    }
    if(i >= m_markers.markers.size()) 
    {
        return;
    }

    m_markers.markers[i].pose.position.x = pose.position.x;
    m_markers.markers[i].pose.position.y = pose.position.y;
    m_markers.markers[i].pose.position.z = pose.position.z;
    m_markers.markers[i].pose.orientation.x = pose.orientation.x;
    m_markers.markers[i].pose.orientation.y = pose.orientation.y;
    m_markers.markers[i].pose.orientation.z = pose.orientation.z;
    m_markers.markers[i].pose.orientation.w = pose.orientation.w;
}

void pose_visualizer::modify_pose(int id, geometry_msgs::PoseStamped pose)
{
    modify_pose(id, pose.pose);
}

void pose_visualizer::set_global_color(double r, double g, double b, double a)
{
    for(int i = 0; i < m_markers.markers.size(); ++i)
    {
        m_markers.markers[i].color.r = r;
        m_markers.markers[i].color.g = g;
        m_markers.markers[i].color.b = b;
        m_markers.markers[i].color.a = a;
    }
    Logger::instance()->logX("sdddd", "[Pose visualizer] Color (rgba) changed:", r, g, b, a);
}

void pose_visualizer::set_color(int id, double r, double g, double b)
{
    m_markers.markers[id].color.r = r;
    m_markers.markers[id].color.g = g;
    m_markers.markers[id].color.b = b;
}

void pose_visualizer::set_color(int id, double r, double g, double b, double a)
{
    m_markers.markers[id].color.a = a;
    set_color(id, r, g, b);
}

void pose_visualizer::remove_all()
{
    Logger::instance()->log("[Pose visualizer] Removing all markers.");
    for(int i = 0; i < m_markers.markers.size(); ++i)
    {
        m_markers.markers[i].action = visualization_msgs::Marker::DELETE;
    }
}

void pose_visualizer::publish()
{
    m_publisher.publish(m_markers);
}

void pose_visualizer::set_topic(std::string topic)
{
    m_topic = topic;
    m_publisher = m_node->advertise<visualization_msgs::MarkerArray>(m_topic, 1000);
    Logger::instance()->logX("ss","[Pose visualized] Set topic:", topic.c_str());
}

std::string pose_visualizer::get_topic() 
{
    return m_topic;
}

