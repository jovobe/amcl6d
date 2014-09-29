/**
 * raytracer_service.cpp
 *
 *  Created: 2014-08-18
 *  Author: Sebastian Höffner
 *  Last modified: 2014-08-24
 *  Author: Sebastian Höffner
 */

#include "cgal_raytracer/raytracer_service.h"

raytracer_service::raytracer_service() 
{
    Logger::instance()->log("[Raytrace service] Initializing.");
    m_raytracer = new CGALRaytracer();
    m_frame = "rt_frame";
    m_mesh_received = false;
}

raytracer_service::~raytracer_service() 
{
    if(m_raytracer != NULL)
    {
        delete m_raytracer;
        m_raytracer = NULL;
    }
    Logger::instance()->log("[Raytrace service] Shut down.");
}

bool raytracer_service::raytrace(
        cgal_raytracer::RaytraceAtPose::Request  &request,
        cgal_raytracer::RaytraceAtPose::Response &response)
{
    Logger::instance()->log("[Raytrace service] Raytrace requested.");

    // empty double** pointer for the points, will be filled in the raytrace
    double** points;
    // corresponding number of points
    int n_points;

    // actual raytrace
    m_raytracer->simulatePointCloud(&m_cam_params, request.pose, points, n_points);
 
    // prepare answer message
    response.raytrace.header.frame_id = m_frame;

    response.raytrace.points.resize(n_points);
    for(int i = 0; i < n_points; ++i)
    {
        response.raytrace.points[i].x = points[i][0];
        response.raytrace.points[i].y = points[i][1];
        response.raytrace.points[i].z = points[i][2];
    }

    Logger::instance()->logX("sis", "[Raytrace service] Raytrace completed. Found", 
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
        Logger::instance()->log("[Raytrace service] New Mesh received.");
        // get mesh from the message
        set_mesh(amcl6d_tools::Mesh(*message.get()));
    }
}

void raytracer_service::set_mesh(amcl6d_tools::Mesh mesh)
{
    m_mesh = mesh;
    
    // and send it to the raytracer
    this->m_raytracer->setMap(&m_mesh);
        
    m_mesh_received = true;

    Logger::instance()->log("[Raytrace service] Mesh loaded.");
}

void raytracer_service::reconfigure_callback(
        cgal_raytracer::CamParamConfig &config, uint32_t level)
{ 
    Logger::instance()->log("[Raytrace service] Reconfiguring CameraParameters.");
    m_cam_params.reconfigure(config, level);
    
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    m_frame = config.rt_frame.compare("")? "rt_frame" : config.rt_frame;
}

bool raytracer_service::has_mesh()
{
    return m_mesh_received;
}

int main(int argc, char** argv)
{
    Logger::instance()->log("[Raytrace service] Initializing.");

    // initialize
    ros::init(argc, argv, "raytracer_service");
    ros::NodeHandle nh;
    
    // check params
    if(nh.hasParam("mesh_topic"))
    {
        Logger::instance()->log("[Raytrace service] Custom mesh_topic given.");
    }

    // load topic
    std::string topic;
    nh.param<std::string>("mesh_topic", topic, "map_mesh");
    Logger::instance()->logX("ss", "[Raytrace service] Topic:", topic.c_str());

    // set up a raytracer_service instance
    raytracer_service* rt_service = new raytracer_service();
    
    // set reconfiguration for camera parameters
    dynamic_reconfigure::Server<cgal_raytracer::CamParamConfig> reconf_srv;
    dynamic_reconfigure::Server<cgal_raytracer::CamParamConfig>::CallbackType 
                    reconf_cbfun;
    reconf_cbfun = boost::bind(&raytracer_service::reconfigure_callback,
                    rt_service, _1, _2);
    reconf_srv.setCallback(reconf_cbfun);

    // set service client to receive the mesh
    ros::ServiceClient map_client = nh.serviceClient<amcl6d_tools::RequestMap>("request_map");

    // prepare call
    amcl6d_tools::RequestMap srv;
    
    // backup to service: subscribe to mesh topic
    ros::Subscriber mesh_subscriber = nh.subscribe(topic, 1000, 
                    &raytracer_service::mesh_callback, rt_service);

    // wait for a mesh
    Logger::instance()->logX("sss", "[Raytrace service] Listening to", 
                             topic.c_str(), "to receive a mesh.");
    while(!rt_service->has_mesh() && ros::ok())
    {
        if(map_client.call(srv))
        {
            rt_service->set_mesh(amcl6d_tools::Mesh(srv.response.map));
            break;
        }
        ros::spinOnce();
    }

    if(ros::ok())
    {
        Logger::instance()->log("[Raytrace service] Mesh received and loaded.");
        
        // initialize service
        ros::ServiceServer service_server = nh.advertiseService("raytrace_at_pose", 
                                         &raytracer_service::raytrace, rt_service);
        Logger::instance()->log("[Raytrace service] Ready.");
    
        // asynchronous spinner for raytrace
        ros::AsyncSpinner async_spinner(8);

        async_spinner.start();
        ros::waitForShutdown();
    }

    // clean up
    delete rt_service;
    
    return 0;
}
