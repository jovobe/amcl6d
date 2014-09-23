/**
 * raytracer_service.h
 *
 * This is a ros node which provides the service RaytraceAtPose. It has to 
 * receive a Mesh message to prepare the raytracer and then starts the service.
 * The mesh can change, a change gets detected whenever the new mesh differs
 * from the old mesh in either the number of vertices or the number of faces.
 * 
 * The service expects a geometry_msgs::Pose as request and replies with a 
 * raytraced sensor_msgs::PointCloud.
 * 
 * The node also holds the camera parameters and handles the corresponding 
 * callbacks for the reconfiguration, so that no other node is needed.
 *
 *  Created: 2014-08-18
 *  Author: Sebastian Höffner
 */
#ifndef RAYTRACER_SERVICE_H
#define RAYTRACER_SERVICE_H
 
#include "ros/ros.h"

#include <string>
#include <math.h>
#include <stdlib.h>

#include "amcl6d_tools/Logger.h"

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "amcl6d_tools/Mesh.h"
#include "amcl6d_tools/RequestMap.h"

#include "cgal_raytracer/CGALRaytracer.h"
#include "cgal_raytracer/CameraParameters.h"

#include "dynamic_reconfigure/server.h"
#include "cgal_raytracer/CamParamConfig.h"

#include "cgal_raytracer/RaytraceAtPose.h"

#include <boost/thread/shared_mutex.hpp>

class raytracer_service 
{
public:
    /**
     * Default constructor. Creates a raytracer instance and presets the 
     * frame to "world". If you want to use another frame please provide a 
     * transformation.
     */
    raytracer_service();

    /**
     * Destructor. Frees the raytracer instance.
     */
    ~raytracer_service();

    /**
     * The callback for the raytracer service.
     * Provides a response PointCloud for the request Pose.
     *
     * @param request The requesting Pose
     * @param reponse The responded PointCloud
     * @return true on success
     */
    bool raytrace(cgal_raytracer::RaytraceAtPose::Request  &request,
                  cgal_raytracer::RaytraceAtPose::Response &response);

    /**
     * Callback for the /mesh-topic subscription.
     * 
     * @param message the message containing the Mesh.
     */
    void mesh_callback(const amcl6d_tools::Mesh::ConstPtr &message);

    /**
     * A callback for the dynamic reconfiguration of the camera parameters.
     * It passes it on to the CameraParamaters::reconfigure function.
     * 
     * Additionally it replaces the frame of the raytraces. (Because 
     * you can not run two dynamic reconfiguration servers in one node.)
     *
     * @param config the CameraParameters to set
     * @param a value which indicates how many values were changed, this 
     *        is not used.
     */     
    void reconfigure_callback(cgal_raytracer::CamParamConfig &config, 
                              uint32_t level);
    
    /**
     * A setter that sets the mesh and also propagates it to the underlying
     * raytracer.
     *
     * It also sets the m_mesh_received flag to true, so that has_mesh()
     * should return true.
     */
    void set_mesh(amcl6d_tools::Mesh mesh);

    /**
     * Returns true as soon as the service loaded a mesh from the subscribed
     * topic. By default this is /mesh, but can be changed with the parameter
     * mesh_topic.
     *
     * Otherwise returns false.
     */
    bool has_mesh();

    /// the frame for the sensor_msgs::PointCloud
    std::string m_frame;

    bool m_subscribe_to_map;
private:

    /// the current mesh to work with, the raytracer and the camera parameters
    amcl6d_tools::Mesh m_mesh;
    bool m_mesh_received;
    CameraParameters m_cam_params;
    CGALRaytracer* m_raytracer;

    boost::shared_mutex m_mutex;
};

#endif /* RAYTRACER_SERVICE_H */
