/**
 * mesh_publisher_node.cpp
 * 
 * This is a node to publish mesh data in the lvr format on
 * a given topic.
 *
 * Additionally it publishes the vertices of the mash as a
 * point cloud for easy visualization in rqt or rviz on the
 * topic /mesh_point_cloud .
 *
 * This node also broadcasts a default transformation from
 * world to mesh. If you need another transformation please
 * redirect it or add the frame parameter and provide a 
 * custom transformation.
 * 
 * Created: 2014-08-15
 * Author: Sebastian Höffner
 * Last modified: 2014-08-24
 * Author: Sebastian Höffner
 */
#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"

#include "amcl6d_tools/mesh_publisher.h"
#include "amcl6d_tools/RequestMap.h"

#include "dynamic_reconfigure/server.h"

namespace mesh_publisher_service 
{
    mesh_publisher* mp;

    bool request_map(amcl6d_tools::RequestMap::Request&  req,
                     amcl6d_tools::RequestMap::Response& res)
    {
        if(mp != NULL)
        {
            res.map = mp->get_message();
            return true;
        }
        else
        {
            return false;
        }
    }
}

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "RequestMap");
    ros::NodeHandle nh;
    
    // initialize mesh publisher (loads model automatically)
    mesh_publisher_service::mp = new mesh_publisher();
    
    // set reconfiguration callbacks and load mesh
    dynamic_reconfigure::Server<amcl6d_tools::mesh_publisherConfig> reconf_srv;
    dynamic_reconfigure::Server<amcl6d_tools::mesh_publisherConfig>::CallbackType
                    reconf_cbfun;
    reconf_cbfun = boost::bind(&mesh_publisher::reconfigure_callback, 
                               mesh_publisher_service::mp, _1, _2);
    reconf_srv.setCallback(reconf_cbfun);
    
    ros::ServiceServer service = nh.advertiseService("request_map", mesh_publisher_service::request_map);

    ROS_INFO("Map Request ready: request_map.");
    
    ros::spin();

    // clean up
    delete mesh_publisher_service::mp;

    return 0;
}
