#include "ros/ros.h"

#include "amcl6d_tools/Logger.h"

#include "cgal_raytracer/RaytraceAtPose.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud.h"

#include "dynamic_reconfigure/server.h"
#include "amcl6d_tools/raytracer_clientConfig.h"

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>  

#include <iostream>

namespace amcl6d_tools
{
    // bit masks for easier comparison in callback
    const uint32_t NO_CHANGE =   0;
    const uint32_t POS_X     =   1;
    const uint32_t POS_Y     =   2;
    const uint32_t POS_Z     =   4;
    const uint32_t YAW       =   8;
    const uint32_t PITCH     =  16;
    const uint32_t ROLL      =  32;
    const uint32_t RAYTRACE  = 512;

    double angle_conversion = M_PI / 180;

    double pos_x = 0;
    double pos_y = 0;
    double pos_z = 0;
    double yaw   = 0;
    double pitch = 0;
    double roll  = 0;

    geometry_msgs::PoseStamped current_pose;

    geometry_msgs::PoseStamped last_pose;
    sensor_msgs::PointCloud last_result;

    ros::ServiceClient service_client;
    ros::Publisher cloud_pub;
    ros::Publisher cur_pose_pub;
    ros::Publisher last_pose_pub;

    bool issue_raytrace()
    {
        return true;
    }

    void reconfigure_callback(amcl6d_tools::raytracer_clientConfig &config,
                              uint32_t level)
    {
        if(level == NO_CHANGE) 
        {
            return;
        }
        
        pos_x = (level & POS_X) != 0 || level == -1? config.x : pos_x;
        pos_y = (level & POS_Y) != 0 || level == -1? config.y : pos_y;
        pos_z = (level & POS_Z) != 0 || level == -1? config.z : pos_z;
        
        yaw   = (level & YAW)   != 0 || level == -1? 
                    config.yaw   * angle_conversion : yaw;
        pitch = (level & PITCH) != 0 || level == -1?
                    config.pitch * angle_conversion : pitch;
        roll  = (level & ROLL)  != 0 || level == -1?
                    config.roll  * angle_conversion : roll;

        Eigen::Matrix3d orientation_matrix;
        orientation_matrix = Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
        Eigen::Affine3d n_pose = Eigen::Translation3d(pos_x, pos_y, pos_z) 
                               * orientation_matrix;
       
        /**
        std::cout << orientation_matrix(0, 0) << ", " << orientation_matrix(1, 0) << ", "
                  << orientation_matrix(2, 0) << std::endl
                  << orientation_matrix(0, 1) << ", " << orientation_matrix(1, 1) << ", "
                  << orientation_matrix(2, 1) << std::endl
                  << orientation_matrix(0, 2) << ", " << orientation_matrix(1, 2) << ", "
                  << orientation_matrix(2, 2) << std::endl;
                 /**/

        tf::poseEigenToMsg(n_pose, current_pose.pose);
        
        if(level == -1)
        {
            last_pose = current_pose;
            return;
        }

        // raytrace toggle used
        if((level & RAYTRACE) != 0)
        {
            Logger::instance()->log("[Raytrace client] Raytrace issued.");
            cgal_raytracer::RaytraceAtPose srv;
            srv.request.pose = current_pose.pose;

            if(service_client.call(srv))
            {
                Logger::instance()->log("[Raytrace client] Raytrace successful.");
                last_pose.pose = srv.request.pose;
                last_result = srv.response.raytrace;
            }
            else
            {
                Logger::instance()->log("[Raytrace client] Raytrace not succesfull.");
            }
        }
    }
}

int main(int argc, char** argv)
{
    Logger::instance()->log("[Raytrace client] Initializing.");

    ros::init(argc, argv, "raytracer_client");
    ros::NodeHandle nh;

    // bind reconfigure callback 
    dynamic_reconfigure::Server<amcl6d_tools::raytracer_clientConfig> server;
    dynamic_reconfigure::Server<amcl6d_tools::raytracer_clientConfig>::CallbackType f;
    f = boost::bind(&amcl6d_tools::reconfigure_callback, _1, _2);
    server.setCallback(f);

    // client for requesting raytraces
    amcl6d_tools::service_client = nh.serviceClient<cgal_raytracer::RaytraceAtPose>("raytrace_at_pose");

    // publishers for visualizing request and response
    amcl6d_tools::cloud_pub = nh.advertise<sensor_msgs::PointCloud>("man_rt_result", 1000, true);
    amcl6d_tools::last_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("man_rt_request", 1000, true);
    amcl6d_tools::cur_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("man_cur_pose", 1000, true);

    // initialize last_pose with current_pose pose and last_result with 0 points
    amcl6d_tools::last_result.header.frame_id = "rt_frame";
    amcl6d_tools::last_result.points.resize(0);

    amcl6d_tools::current_pose.header.frame_id = "poses";
    amcl6d_tools::last_pose.header.frame_id = "poses";
    
    Logger::instance()->log("[Raytrace client] Initialized.");

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        amcl6d_tools::cloud_pub.publish(amcl6d_tools::last_result);
        amcl6d_tools::last_pose_pub.publish(amcl6d_tools::last_pose);
        amcl6d_tools::cur_pose_pub.publish(amcl6d_tools::current_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    Logger::instance()->log("[Raytrace client] Shut down.");

    return 0;
}
