#ifndef PARTICLE_VISUALIZER_H
#define PARTICLE_VISUALIZER_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <string>


class particle_visualizer
{
    public:
        particle_visualizer(ros::NodeHandle* node_handle);

        ~particle_visualizer();
        
        void add_pose(int id, geometry_msgs::Pose pose);

        void add_pose(int id, geometry_msgs::PoseStamped pose);

        void remove_pose(int id);

        void modify_pose(int id, geometry_msgs::Pose pose);

        void modify_pose(int id, geometry_msgs::PoseStamped pose);
        
        void set_color(int id, double r, double g, double b);

        void set_color(int id, double r, double g, double b, double a);

        void remove_all();

        void publish();

        void set_topic(std::string topic);

        std::string get_topic();

    private:
        visualization_msgs::MarkerArray m_markers;
        std::string m_topic;
        ros::Publisher m_publisher;
        ros::NodeHandle* m_node;
};



#endif
