#include "ros/ros.h"
#include "signal.h"

#include "amcl6d/amcl6d.h"
#include "amcl6d_tools/Mesh.h"
#include "amcl6d_tools/Logger.h"

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
    ros::Subscriber    mesh_subscriber = nodehandle.subscribe("map_mesh", 1000, &amcl6d::mesh_callback, amcl6d_node::amcl);
    ros::ServiceClient mesh_client     = nodehandle.serviceClient<amcl6d_tools::RequestMap>("request_map");

    Logger::instance()->log("[AMCL Node] Waiting for map...");

    amcl6d_tools::RequestMap srv;
    while(!amcl6d_node::amcl->has_mesh() && ros::ok())
    {
        if(mesh_client.call(srv))
        {
            amcl6d_tools::Mesh map = srv.response.map;
            amcl6d_node::amcl->set_mesh(map); 
        }
        ros::spinOnce();
    }
    Logger::instance()->log(ros::ok()? "[AMCL Node] Map received." : "[AMCL Node] Interrupted while waiting for map.");

    ros::Subscriber move_subscriber = nodehandle.subscribe("man_cur_pose", 1000, &amcl6d::move_callback, amcl6d_node::amcl);
    
    Logger::instance()->log("[AMCL Node] Ready.");

    ros::Rate loop(100);
    while(ros::ok())
    {   
//        amcl6d_node::amcl();
        amcl6d_node::amcl->publish();

        ros::spinOnce();
        loop.sleep();
    }

    Logger::instance()->log("[AMCL Node] Shut down.");

    return 0;
}
