/**
 * CameraParameters.h
 *
 *  Created on: 29.09.2010
 *  Author: Denis Meyer, Thomas Wiemann
 *  Modified on: 08.08.2014
 *  Author: Sebastian Höffner
 */

#ifndef CAMERAPARAMETERS_H_
#define CAMERAPARAMETERS_H_

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <cmath>
#include "cgal_raytracer/MatrixMath.hpp"
#include "amcl6d_tools/Logger.h"
#include "cgal_raytracer/CamParamConfig.h"

class CameraParameters
{
public:
  /**
	 * Default Constructor
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
   * sets the pose
   */
  void setPose(geometry_msgs::Pose pose);

  void reconfigure(cgal_raytracer::CamParamConfig &config);

	double m_matrixCamOrientation[16];
	double m_minAngleH;
	double m_maxAngleH;
	double m_minAngleV;
	double m_maxAngleV;
	int    m_resolutionV;
	int    m_resolutionH;
	double m_focalLength;
	double m_plane_minZ;
	double m_plane_maxZ;
	double m_plane_minY;
	double m_plane_maxY;
	double m_maxRange;
	double m_minRange;

private:
	static const bool output;
};

#endif /* CAMERAPARAMETERS_H_ */
