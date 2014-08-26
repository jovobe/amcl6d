#include "amcl6d/particle_visualizer.h"

particle_visualizer::particle_visualizer(ros::NodeHandle* node_handle)
{
    m_node = node_handle;
    m_publisher = m_node->advertise<visualization_msgs::MarkerArray>("pose_samples", 1000);
}

particle_visualizer::~particle_visualizer()
{
    delete m_node;
}
        
void particle_visualizer::add_pose(int id, geometry_msgs::Pose pose)
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
}

void particle_visualizer::add_pose(int id, geometry_msgs::PoseStamped pose)
{
    add_pose(id, pose.pose);
}

void particle_visualizer::remove_pose(int id)
{
    for(int i = 0; i < m_markers.markers.size(); ++i)
    {
        if(m_markers.markers[i].id == id)
        {
            m_markers.markers[i].action = visualization_msgs::Marker::DELETE;
            break;
        }
    }
}

void particle_visualizer::modify_pose(int id, geometry_msgs::Pose pose)
{
}

void particle_visualizer::modify_pose(int id, geometry_msgs::PoseStamped pose)
{
}

void particle_visualizer::set_color(int id, double r, double g, double b)
{
    m_markers.markers[id].color.r = r;
    m_markers.markers[id].color.g = g;
    m_markers.markers[id].color.b = b;
}
void particle_visualizer::set_color(int id, double r, double g, double b, double a)
{
    m_markers.markers[id].color.a = a;
    set_color(id, r, g, b);
}

void particle_visualizer::remove_all()
{
    for(int i = 0; i < m_markers.markers.size(); ++i)
    {
        m_markers.markers[i].action = visualization_msgs::Marker::DELETE;
    }
}

void particle_visualizer::publish()
{
    m_publisher.publish(m_markers);
}

void particle_visualizer::set_topic(std::string topic)
{
    m_topic = topic;
    m_publisher = m_node->advertise<visualization_msgs::MarkerArray>(m_topic, 1000);
}

std::string particle_visualizer::get_topic() 
{
    return m_topic;
}
                
