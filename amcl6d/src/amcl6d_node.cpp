#include "ros/ros.h"

#include "visualization_msgs/MarkerArray.h"
#include "amcl6d/pose_factory.h"
#include "amcl6d_tools/Mesh.h"
#include "geometry_msgs/PoseArray.h"
#include "signal.h"

namespace amcl6d {
    int number = 10000;

    pose_factory* factory = NULL;

    geometry_msgs::PoseArray poses;

    ros::Publisher pose_array_publisher;
    ros::Subscriber mesh_subscriber;

    amcl6d_tools::Mesh current_mesh;

    void generate_poses(int number)
    {
        for(int i = 0; i < number; ++i)
        {
            poses.poses.push_back(factory->generate_random_pose());
        }
    }

    void mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message)
    {
        if(message->mesh.vertices.size() == current_mesh.mesh.vertices.size())
        {
            return;
        }

        current_mesh = amcl6d_tools::Mesh(*message.get());
        factory->set_bounds(current_mesh);
    }

    void shutdown_callback(int sig) {
        poses.poses.clear();
        pose_array_publisher.publish(poses);

        if(factory != NULL)
        {
            delete factory;
            factory = NULL;
        }
        ros::shutdown();
    }

}

int main(int argc, char** args) 
{
    ros::init(argc, args, "amcl6d");
    ros::NodeHandle nodehandle;
    signal(SIGINT, amcl6d::shutdown_callback);

    // subscribe to mesh
    amcl6d::mesh_subscriber = nodehandle.subscribe("map_mesh", 1000, amcl6d::mesh_callback);

    // publisher for poses
    amcl6d::pose_array_publisher = nodehandle.advertise<geometry_msgs::PoseArray>("pose_samples", 1000);
    amcl6d::poses.header.frame_id = "world";

    // pose factory
    amcl6d::factory = new pose_factory();

    // generate poses
    amcl6d::generate_poses(amcl6d::number);

    ros::Rate loop(100);
    while(ros::ok())
    {   
        // TODO this is debugging
        amcl6d::poses.poses.clear();
        amcl6d::generate_poses(amcl6d::number);
        // publish poses
        amcl6d::pose_array_publisher.publish(amcl6d::poses);
        ros::spinOnce();
        loop.sleep();
    }


    return 0;
}
