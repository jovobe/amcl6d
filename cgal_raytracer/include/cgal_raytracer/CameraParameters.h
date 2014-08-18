/**
 * CameraParameters.h
 *
 * This is a class to represent camera parameters needed for raytracings
 * in a simple way. Together with the dynamic_reconfiguration package the
 * reconfigure method is used for a callback to dynamically configure and
 * reconfigure the parameters.
 *
 *  Created: 2010-09-29
 *  Authors: Denis Meyer, Thomas Wiemann
 *  Last modified: 2014-08-18
 *  Author: Sebastian Höffner
 */

#ifndef CAMERAPARAMETERS_H_
#define CAMERAPARAMETERS_H_

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include "amcl6d_tools/Logger.h"

#include <cmath>
#include "cgal_raytracer/MatrixMath.hpp"
#include "cgal_raytracer/CamParamConfig.h"

#include "boost/thread/shared_mutex.hpp"

class CameraParameters
{
public:
    /**
     * Default Constructor. Sets the position and orientation to 0.
     */
    CameraParameters();

    /**
     * Copy Constructor
     * @param camParams CameraParameters
     */
    CameraParameters(const CameraParameters &camParams);
    
    /**
     * Destructor
     */
    virtual ~CameraParameters();

    /**
     * Sets the pose for the raytrace.
     */
    void setPose(geometry_msgs::Pose pose);

    /**
     * A function to be used as a callback for the dynamic reconfigure (use 
     * e.g. boost::bind) or which can be used to configure the camera manually.
     * Please refer to the documentation of the dynamic reconfigure package
     * for further details on how to construct the CamParamConfig object.
     *
     * Please provide the aperture angles in degrees.
     *
     * @param config the camera configuration
     * @param level unused but needed for the callback call
     */
    void reconfigure(cgal_raytracer::CamParamConfig &config, uint32_t level);

    /// a matrix which stores the current orientation
    double m_matrixCamOrientation[16];
    
    /** 
     * Aperture angle, resolution and focal length. The angles should be
     * provided in degrees as they will be converted to radians.
     */
    double m_minAngleH;
    double m_maxAngleH;
    double m_minAngleV;
    double m_maxAngleV;
    int    m_resolutionV;
    int    m_resolutionH;
    double m_focalLength;
    
    /// the plane gets calculated on the fly
    double m_plane_minZ;
    double m_plane_maxZ;
    double m_plane_minY;
    double m_plane_maxY;
    
    /// the range of the camera
    double m_maxRange;
    double m_minRange;

private:
    boost::shared_mutex m_mutex;
};

#endif /* CAMERAPARAMETERS_H_ */
