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
 * Last modified: 2014-08-18
 * Author: Sebastian Höffner
 */
#include "ros/ros.h"

#include <stdlib.h>
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_broadcaster.h"

#include "amcl6d_tools/mesh_publisher.h"

#include "dynamic_reconfigure/server.h"

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "mesh_publisher");
    ros::NodeHandle nh;
    
    if(!nh.hasParam("topic"))
    {
        ROS_INFO("Parameter 'topic' was not set, publishing to /mesh.");
    }
    // load topic
    std::string topic;
    nh.param<std::string>("topic", topic, "mesh");
    Logger::instance()->logX("ss", "Topic:", topic.c_str());

    // initialize mesh publisher (loads model automatically)
    mesh_publisher* mp = new mesh_publisher();
    
    // set reconfiguration callbacks and load mesh
    dynamic_reconfigure::Server<amcl6d_tools::mesh_publisherConfig> reconf_srv;
    dynamic_reconfigure::Server<amcl6d_tools::mesh_publisherConfig>::CallbackType
                    reconf_cbfun;
    reconf_cbfun = boost::bind(&mesh_publisher::reconfigure_callback, 
                               mp, _1, _2);
    reconf_srv.setCallback(reconf_cbfun);
    
    // initialize publishers
    ros::Publisher publisher = nh.advertise<amcl6d_tools::Mesh>(
                               topic, 1000, true);
    ros::Publisher pcl_pub   = nh.advertise<sensor_msgs::PointCloud>(
                               "mesh_point_cloud", 1000, true);

    // provide transformations
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
       
    // publish and broadcast  
    ros::Rate loop_rate(1000/30); // 30 fps
    while(ros::ok())
    {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
                                              "world", mp->get_frame()));
        publisher.publish(mp->get_message());
        pcl_pub.publish(mp->get_pointcloud());

        ros::spinOnce();
        loop_rate.sleep();
    }

    // clean up
    delete mp;

    return 0;
}
