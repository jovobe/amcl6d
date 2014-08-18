/**
 * raytracer_service.cpp
 *
 *  Created: 2014-08-18
 *  Author: Sebastian Höffner
 */

#include "cgal_raytracer/raytracer_service.h"

raytracer_service::raytracer_service() 
{
    m_raytracer = new CGALRaytracer();
    m_frame = "world";
    m_mesh_received = false;
}

raytracer_service::~raytracer_service() 
{
    if(m_raytracer != NULL)
    {
        delete m_raytracer;
        m_raytracer = NULL;
    }
}

bool raytracer_service::raytrace(
        cgal_raytracer::RaytraceAtPose::Request  &request,
        cgal_raytracer::RaytraceAtPose::Response &response)
{
    Logger::instance()->log("Raytrace requested.");

    // prepare cam_params for the raytrace from pose
    m_cam_params.setPose(request.pose);

    // identity matrix
    double matrix[] = {1, 0, 0, 0,
                       0, 1, 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1};
    // empty double** pointer for the points, will be filled in the raytrace
    double** points;
    // corresponding number of points
    int n_points;

    // actual raytrace
    m_raytracer->simulatePointCloud(&m_cam_params, matrix, points, n_points);
 
    // prepare answer message
    response.raytrace.header.frame_id = m_frame;
    response.raytrace.points.resize(n_points);
    for(int i = 0; i < n_points; ++i)
    {
        response.raytrace.points[i].x = points[i][0];
        response.raytrace.points[i].y = points[i][1];
        response.raytrace.points[i].z = points[i][2];
    }

    Logger::instance()->logX("sis", "Raytrace completed. Found", 
                             n_points, "points.");
    return true;
}

void raytracer_service::mesh_callback(
        const amcl6d_tools::Mesh::ConstPtr &message)
{ 
    // check if there is a mesh or mesh has changed (i.e. the number of vertices
    // or faces differs, that should usually be enough to detect changes.)
    if(!has_mesh() 
       || message->mesh.vertices.size() != m_mesh.mesh.vertices.size() 
       || message->mesh.faces.size()    != m_mesh.mesh.faces.size())
    {
        Logger::instance()->log("New Mesh received.");
        // get mesh from the message
        m_mesh = amcl6d_tools::Mesh(*message.get());
        // and send it to the raytracer
        this->m_raytracer->setMap(&m_mesh);
        
        // set flag to escape waiting for the first mesh
        m_mesh_received = true;
        Logger::instance()->log("Mesh loaded.");
    }
}

void raytracer_service::reconfigure_callback(
        cgal_raytracer::CamParamConfig &config, uint32_t level)
{ 
    Logger::instance()->log("Reconfiguring CameraParameters.");
    m_cam_params.reconfigure(config, level);
}

bool raytracer_service::has_mesh()
{
    return m_mesh_received;
}

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "raytracer_service");
    ros::NodeHandle nh;
    
    // check params
    if(nh.hasParam("mesh_topic"))
    {
        Logger::instance()->log("Custom mesh_topic given.");
    }

    // load topic
    std::string topic;
    nh.param<std::string>("mesh_topic", topic, "mesh");
    Logger::instance()->logX("ss", "Topic:", topic.c_str());

    // set up a raytracer_service instance
    raytracer_service* rt_service = new raytracer_service();
    
    // set frame for rt_service
    nh.param<std::string>("raytrace_frame", rt_service->m_frame, "world");
    
    // set reconfiguration for camera parameters
    dynamic_reconfigure::Server<cgal_raytracer::CamParamConfig> reconf_srv;
    dynamic_reconfigure::Server<cgal_raytracer::CamParamConfig>::CallbackType 
                    reconf_cbfun;
    reconf_cbfun = boost::bind(&raytracer_service::reconfigure_callback,
                    rt_service, _1, _2);
    reconf_srv.setCallback(reconf_cbfun);

    // subscribe to mesh topic
    ros::Subscriber mesh_subscriber = nh.subscribe(topic, 1000, 
                    &raytracer_service::mesh_callback, rt_service);

    // wait for a mesh
    Logger::instance()->logX("sss", "Listening to", 
                             topic.c_str(), "to receive a mesh.");
    while(!rt_service->has_mesh() && ros::ok())
    {
        ros::spinOnce();
    }

    // initialize service
    ros::ServiceServer service_server = nh.advertiseService("raytrace_at_pose", 
                    &raytracer_service::raytrace, rt_service);
    Logger::instance()->log("Raytracing service ready.");
    
    // spin: listen to mesh changes and answer trace requests
    ros::spin();
    
    // clean up
    delete rt_service;
    
    return 0;
}
