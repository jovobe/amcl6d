/**
 * CameraParameters.cpp
 *
 *  Created on: 29.09.2010
 *      Author: Denis Meyer, Thomas Wiemann
 *    Modified: 08.08.2014
 *      Author: Sebastian Höffner
 */

#include "cgal_raytracer/CameraParameters.h"

const bool CameraParameters::output = true;

CameraParameters::CameraParameters() 
{
  double position[3];
  double orientation[3];
  position[0]    = 0;
  position[1]    = 0;
  position[2]    = 0;
  orientation[0] = 0;
  orientation[1] = 0;
  orientation[2] = 0;
	EulerToMatrix4(position, orientation, m_matrixCamOrientation);
}

void CameraParameters::setPose(geometry_msgs::Pose pose)
{
	double position[3];
	double orientation[3];

	position[0] = pose.position.x;
	position[1] = pose.position.y;
	position[2] = pose.position.z;

  // conversion between quaternion and euler
  double q0 = pose.orientation.x;
  double q1 = pose.orientation.y;
  double q2 = pose.orientation.z;
  double q3 = pose.orientation.w;
	orientation[0] = atan(       2 * (q0 * q1 + q2 * q3) 
                        / (1 - 2 * (q1 * q1 + q2 * q2))
                   );
	orientation[1] = asin(       2 * (q0 * q2 - q3 * q1));
	orientation[2] = atan(       2 * (q0 * q3 + q1 * q2)
                        / (1 - 2 * (q2 * q2 + q3 * q3))
                   );

	// Convert to matrix
	EulerToMatrix4(position, orientation, m_matrixCamOrientation);
}

CameraParameters::CameraParameters(const CameraParameters &camParams) {
	Logger::instance()->log("CameraParameters - copy constructor");
	M4copy(camParams.m_matrixCamOrientation, m_matrixCamOrientation);

	m_minAngleH   = camParams.m_minAngleH;
	m_maxAngleH   = camParams.m_maxAngleH;
	m_minAngleV   = camParams.m_minAngleV;
	m_maxAngleV   = camParams.m_maxAngleV;
	m_resolutionV = camParams.m_resolutionV;
	m_resolutionH = camParams.m_resolutionH;
	m_focalLength = camParams.m_focalLength;
	m_plane_minZ  = camParams.m_plane_minZ;
	m_plane_maxZ  = camParams.m_plane_maxZ;
	m_plane_minY  = camParams.m_plane_minY;
	m_plane_maxY  = camParams.m_plane_maxY;
	m_maxRange    = camParams.m_maxRange;
	m_minRange    = camParams.m_minRange;
}

CameraParameters::~CameraParameters() {
	Logger::instance()->log("~CameraParameters");
}

void CameraParameters::reconfigure(cgal_raytracer::CamParamConfig &config)
{
	m_minAngleH   = config.minAngleH; 
	m_maxAngleH   = config.maxAngleH;
	m_minAngleV   = config.minAngleV;
	m_maxAngleV   = config.maxAngleV;
	m_resolutionV = config.resolutionV;
	m_resolutionH = config.resolutionH;
	m_focalLength = config.focalLength;
	m_plane_minY  = -tan(m_minAngleV * M_PI / 180) * m_focalLength;
	m_plane_maxY  =  tan(m_minAngleV * M_PI / 180) * m_focalLength;
	m_plane_minZ  = -tan(m_minAngleH * M_PI / 180) * m_focalLength;
	m_plane_maxZ  =  tan(m_maxAngleH * M_PI / 180) * m_focalLength;
	m_maxRange    = config.maxRange;
	m_minRange    = config.minRange;
}


