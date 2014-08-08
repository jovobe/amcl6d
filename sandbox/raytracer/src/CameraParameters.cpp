/*
 * CameraParameters.cpp
 *
 *  Created on: 29.09.2010
 *      Author: Denis Meyer, Thomas Wiemann
 *    Modified: 08.08.2014
 *      Author: Sebastian Höffner
 */

#include "CameraParameters.h"

const bool CameraParameters::output = true;

CameraParameters::CameraParameters() {
	Logger::instance()->setPrefix("Log ----- ");
	Logger::instance()->setPostfix("");

	Logger::instance()->log(
			"CameraParameters - constructor without config file");

	m_minAngleH = 0.0;
	m_maxAngleH = 0.0;
	m_minAngleV = 0.0;
	m_maxAngleV = 0.0;
	m_resolutionV = 0.0;
	m_resolutionH = 0.0;
	m_focalLength = 0.0;
	m_plane_minZ = 0.0;
	m_plane_maxZ = 0.0;
	m_plane_minY = 0.0;
	m_plane_maxY = 0.0;
	m_deviceType = 1;
	m_maxRange = 0.0;
	m_minRange = 0.0;

	M4identity( m_matrixCamOrientation);
}

CameraParameters::CameraParameters(ConfigFile* cf) {
	Logger::instance()->log("CameraParameters - constructor with config file");
	parseConfigFile(cf);
	// M4identity(m_matrixCamOrientation); // DEBUG ONLY
}

CameraParameters::CameraParameters(const CameraParameters &camParams) {
	Logger::instance()->log("CameraParameters - copy constructor");
	M4copy(camParams.m_matrixCamOrientation, m_matrixCamOrientation);

	m_minAngleH = camParams.m_minAngleH;
	m_maxAngleH = camParams.m_maxAngleH;
	m_minAngleV = camParams.m_minAngleV;
	m_maxAngleV = camParams.m_maxAngleV;
	m_resolutionV = camParams.m_resolutionV;
	m_resolutionH = camParams.m_resolutionH;
	m_focalLength = camParams.m_focalLength;
	m_plane_minZ = camParams.m_plane_minZ;
	m_plane_maxZ = camParams.m_plane_maxZ;
	m_plane_minY = camParams.m_plane_minY;
	m_plane_maxY = camParams.m_plane_maxY;
	m_deviceType = camParams.m_deviceType;
	m_maxRange = camParams.m_maxRange;
	m_minRange = camParams.m_minRange;
}

CameraParameters::~CameraParameters() {
	Logger::instance()->log("~CameraParameters");
}

void CameraParameters::parseConfigFile(ConfigFile* cf) {
	Logger::instance()->log("parseConfigFile - start");
	// Get camera parameters
	m_minAngleH = cf->getAsFloat("minAngleHorizontal", 80);
	m_maxAngleH = cf->getAsFloat("maxAngleHorizontal", 110);
	m_minAngleV = cf->getAsFloat("minAngleVertical", 80);
	m_maxAngleV = cf->getAsFloat("maxAngleVertical", 110);
	m_resolutionH = cf->getAsInt("resolutionHorizontal", 40);
	m_resolutionV = cf->getAsInt("resolutionVertical", 40);

	m_focalLength = cf->getAsFloat("focalLength", 0.4);
	m_deviceType = cf->getAsFloat("deviceType", 0);

	m_minRange = cf->getAsFloat("minRange", 0);
	m_maxRange = cf->getAsFloat("maxRange", 8);

	m_plane_minY = -tan(m_minAngleV * M_PI / 180) * m_focalLength;
	m_plane_maxY =  tan(m_maxAngleV * M_PI / 180) * m_focalLength;
	m_plane_minZ = -tan(m_minAngleH * M_PI / 180) * m_focalLength;
	m_plane_maxZ =  tan(m_maxAngleH * M_PI / 180) * m_focalLength;

	// Get camera position
	double pose[3];
	double orientation[3];

	pose[0] = cf->getVecAsFloat("camPosition", 0, 0.0);
	pose[1] = cf->getVecAsFloat("camPosition", 1, 0.0);
	pose[2] = cf->getVecAsFloat("camPosition", 2, 0.0);
	orientation[0] = cf->getVecAsFloat("camOrientation", 0, 0.0) * M_PI / 180;
	orientation[1] = cf->getVecAsFloat("camOrientation", 1, 0.0) * M_PI / 180;
	orientation[2] = cf->getVecAsFloat("camOrientation", 2, 0.0) * M_PI / 180;

	// Convert to matrix
	EulerToMatrix4(pose, orientation, m_matrixCamOrientation);

	Logger::instance()->logX("sd", "parseConfigFile - m_minAngleH: ", m_minAngleH);
	Logger::instance()->logX("sd", "parseConfigFile - m_maxAngleH: ", m_maxAngleH);
	Logger::instance()->logX("sd", "parseConfigFile - m_minAngleV: ", m_minAngleV);
	Logger::instance()->logX("sd", "parseConfigFile - m_maxAngleV: ", m_maxAngleV);
	Logger::instance()->logX("si", "parseConfigFile - m_resolutionH: ", m_resolutionH);
	Logger::instance()->logX("si", "parseConfigFile - m_resolutionV: ", m_resolutionV);
	Logger::instance()->logX("sd", "parseConfigFile - m_focalLength: ", m_focalLength);
	Logger::instance()->logX("si", "parseConfigFile - m_deviceType: ", m_deviceType);
	Logger::instance()->logX("sd", "parseConfigFile - m_minRange: ", m_minRange);
	Logger::instance()->logX("sd", "parseConfigFile - m_maxRange: ", m_maxRange);
	Logger::instance()->logX("sd", "parseConfigFile - m_plane_minY: ", m_plane_minY);
	Logger::instance()->logX("sd", "parseConfigFile - m_plane_maxY: ", m_plane_maxY);
	Logger::instance()->logX("sd", "parseConfigFile - m_plane_minZ: ", m_plane_minZ);
	Logger::instance()->logX("sd", "parseConfigFile - m_plane_maxZ: ", m_plane_maxZ);

	Logger::instance()->log("Matrix Cam Orientation: ");
	Logger::instance()->log(m_matrixCamOrientation, 4, 4);
	Logger::instance()->log("parseConfigFile - end");
}
