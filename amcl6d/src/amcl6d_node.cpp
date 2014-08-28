#include "ros/ros.h"
#include "signal.h"

#include "amcl6d/amcl6d.h"

namespace amcl6d_node {
    amcl6d* amcl = NULL;

    void shutdown_callback(int sig) {
        amcl->clear();
        amcl->publish();

        if(amcl != NULL)
        {
            delete amcl;
            amcl = NULL;
        }
        ros::shutdown();
    }
}

int main(int argc, char** args) 
{
    ros::init(argc, args, "amcl6d");
    ros::NodeHandle nodehandle;
    signal(SIGINT, amcl6d_node::shutdown_callback);

    amcl6d_node::amcl = new amcl6d(nodehandle);

    // subscribe to mesh
    ros::Subscriber mesh_subscriber = nodehandle.subscribe("map_mesh", 1000, &amcl6d::mesh_callback, amcl6d_node::amcl);
    ros::Subscriber move_subscriber = nodehandle.subscribe("man_cur_pose", 1000, &amcl6d::move_callback, amcl6d_node::amcl);

    ros::Rate loop(100);
    while(ros::ok())
    {   
        amcl6d_node::amcl->publish();

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
