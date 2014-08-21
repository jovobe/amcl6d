#include "ros/ros.h"

#include "cgal_raytracer/RaytraceAtPose.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud.h"

#include "dynamic_reconfigure/server.h"
#include "amcl6d_tools/raytracer_clientConfig.h"

namespace amcl6d_tools
{
    // bit masks for easier comparison in callback
    const uint32_t NO_CHANGE        =   0;
    const uint32_t POS_X            =   1;
    const uint32_t POS_Y            =   2;
    const uint32_t POS_Z            =   4;
    const uint32_t ORIENT_X         =   8;
    const uint32_t ORIENT_Y         =  16;
    const uint32_t ORIENT_Z         =  32;
    const uint32_t ORIENT_W         =  64;
    const uint32_t ORIENTATION_TYPE = 128;
    const uint32_t ANGLE_TYPE       = 256;
    const uint32_t RAYTRACE         = 512;

    const int EULER      = 0;
    const int QUATERNION = 1;
    int orientation_type = 0;

    const int RAD = 0;
    const int DEG = 1;
    int angle_type = 0;
    double angle_conversion = 1;

    double current_yaw;
    double current_pitch;
    double current_roll;

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
        current_pose.pose.position.x = (level & POS_X) != 0 || level == -1?
                                  config.position_x : current_pose.pose.position.x;
        current_pose.pose.position.y = (level & POS_Y) != 0 || level == -1?
                                  config.position_y : current_pose.pose.position.y;
        current_pose.pose.position.z = (level & POS_Z) != 0 || level == -1?
                                  config.position_z : current_pose.pose.position.z;
        
        if((level & ORIENTATION_TYPE) != 0 || level == -1)
        {
            orientation_type = config.type;
        }

        if((level & ANGLE_TYPE) != 0 || level == -1)
        {
            angle_type       = config.angle;
            angle_conversion = angle_type == DEG? M_PI / 180 : 1;
        }

        // use euler angles or quaternions
        if(orientation_type == EULER)
        {
            // update yaw (x) / pitch (y) / roll (z)  from current quaternion or config
            double q0 = current_pose.pose.orientation.x, 
                   q1 = current_pose.pose.orientation.y,
                   q2 = current_pose.pose.orientation.z, 
                   q3 = current_pose.pose.orientation.w;
            double yaw   = (level & ORIENT_X) != 0 || level == -1?
                              config.orientation_x * angle_conversion
                            : atan((2*q0*q1+q2*q3)/(1-2*(q1*q1+q2*q2)));
            double pitch = (level & ORIENT_Y) != 0 || level == -1?
                              config.orientation_y * angle_conversion
                            : asin(2*(q0*q2-q3*q1));
            double roll  = (level & ORIENT_Z) != 0 || level == -1?
                              config.orientation_z * angle_conversion
                            : atan((2*q0*q3+q1*q2)/(1-2*(q2*q2+q3*q3)));

            // using x = yaw, y = pitch, z = roll, rotation in that order
            ROS_INFO("using euler");
            double cy = cos(yaw / 2);
            double cp = cos(pitch / 2);
            double cr = cos(roll / 2);
            double sy = sin(yaw / 2);
            double sp = sin(pitch / 2);
            double sr = sin(roll / 2);

            current_pose.pose.orientation.x = sy * sp * cr + cy * cp * sr;
            current_pose.pose.orientation.y = sy * cp * cr + cy * sp * sr;
            current_pose.pose.orientation.z = cy * sp * cr - sy * cp * sr;
            current_pose.pose.orientation.w = cy * cp * cr - sy * sp * sr;
        }
        else
        {
            ROS_INFO("using quaternions");
            current_pose.pose.orientation.x = (level & ORIENT_X) != 0 || level == -1?
                                       config.orientation_x * angle_conversion
                                     : current_pose.pose.orientation.x;
            current_pose.pose.orientation.y = (level & ORIENT_Y) != 0 || level == -1?
                                       config.orientation_y * angle_conversion
                                     : current_pose.pose.orientation.x;
            current_pose.pose.orientation.z = (level & ORIENT_Z) != 0 || level == -1?
                                       config.orientation_z * angle_conversion
                                     : current_pose.pose.orientation.x;
            current_pose.pose.orientation.w = (level & ORIENT_W) != 0 || level == -1?
                                       config.orientation_w * angle_conversion
                                     : current_pose.pose.orientation.w;
        }
        
        if(level == -1)
        {
            last_pose = current_pose;
            return;
        }

        // raytrace toggle used
        if((level & RAYTRACE) != 0)
        {
            ROS_INFO("raytrace issued");
            cgal_raytracer::RaytraceAtPose srv;
            srv.request.pose = current_pose.pose;

            if(service_client.call(srv))
            {
                ROS_INFO("Raytrace successful.");
                last_pose.pose = srv.request.pose;
                last_result = srv.response.raytrace;
            }
            else
            {
                ROS_INFO("Raytrace not succesfull.");
            }
        }
    }
}

int main(int argc, char** argv)
{
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
    amcl6d_tools::last_result.header.frame_id = "mesh";
    amcl6d_tools::last_result.points.resize(0);

    amcl6d_tools::current_pose.header.frame_id = "world";
    amcl6d_tools::last_pose.header.frame_id = "world";
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        amcl6d_tools::cloud_pub.publish(amcl6d_tools::last_result);
        amcl6d_tools::last_pose_pub.publish(amcl6d_tools::last_pose);
        amcl6d_tools::cur_pose_pub.publish(amcl6d_tools::current_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
