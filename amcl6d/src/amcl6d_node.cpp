#include "ros/ros.h"

#include "visualization_msgs/MarkerArray.h"
#include "amcl6d/particle_factory.h"
#include "amcl6d/particle_visualizer.h"
#include "amcl6d_tools/Mesh.h"

namespace amcl6d {
int num = 1000;
    visualization_msgs::MarkerArray marker_array;
particle_factory* pf;
void generate() {
    marker_array.markers.resize(num);
    for(int i = 0; i < num; ++i)
    {
        marker_array.markers[i] = visualization_msgs::Marker(pf->generate_particle(i));
    }
}

void remove() {
    marker_array.markers.resize(num);
    for(int i = 0; i < num; ++i)
    {
        marker_array.markers[i] = pf->remove_particle(i);
    }
}

amcl6d_tools::Mesh current_mesh;
void mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message)
{
    if(message->mesh.vertices.size() == current_mesh.mesh.vertices.size())
        return;

    current_mesh = amcl6d_tools::Mesh(*message.get());
    pf->set_bounds(current_mesh);
}
}
int main(int argc, char** args) 
{
    ros::init(argc, args, "amcl6d");
    ros::NodeHandle nh;


    // init publisher and message
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("pose_samples", 5000);
    ros::Subscriber mesh_sub = nh.subscribe("map_mesh", 1000, amcl6d::mesh_callback);

    amcl6d::pf = new particle_factory();

    ros::Rate loop(50);
    while(ros::ok())
    {
            amcl6d::generate();
        
        // publish message and sleep
        marker_pub.publish(amcl6d::marker_array);
        ros::spinOnce();
        loop.sleep();
    }

    delete amcl6d::pf;


    return 0;
}
